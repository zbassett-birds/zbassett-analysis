# EKF2 Estimation Module — Software Design Description

**Vehicle:** Stratospheric glider (PX4-based)
**Module:** `ekf2` (`src/modules/ekf2/`)
**Primary source files:**
- `EKF/ekf.h`, `EKF/estimator_interface.h`, `EKF/common.h`
- `EKF2.hpp`, `EKF2Selector.hpp`
- `EKF/aid_sources/` (per-source control and fusion files)
- `EKF/yaw_estimator/EKFGSF_yaw.h`
- `EKF/imu_down_sampler/imu_down_sampler.hpp`
- `EKF/output_predictor/output_predictor.h`
- `EKF/python/ekf_derivation/generated/state.h`

---

## 1. Purpose

EKF2 is the primary navigation state estimator for the glider. It runs an Extended Kalman Filter that fuses IMU inertial measurements with one or more absolute sensor observations (GPS, barometer, magnetometer, airspeed) to produce a best estimate of vehicle attitude, position, velocity, and wind. Every downstream control and guidance function — attitude control, position control, wind compensation, failsafe logic — consumes the state estimates published by EKF2. When external aiding is degraded or lost, EKF2 continues to propagate inertial dead reckoning and declares the estimates invalid only after a configurable timeout, giving higher-level logic time to respond.

---

## 2. Inputs and Outputs

### 2.1 uORB Subscriptions (sensor inputs)

The primary trigger is the IMU feed. `EKF2` registers a callback on whichever topic arrives first:

| Topic | Source | Usage |
|---|---|---|
| `vehicle_imu` | IMU driver (per-instance) | Primary IMU trigger; `delta_ang`, `delta_vel` |
| `sensor_combined` | Sensor hub | Fallback IMU trigger |
| `sensor_gps` / `vehicle_gps_position` | GNSS driver | Position, velocity, optional dual-antenna yaw |
| `vehicle_air_data` | Baro driver | Pressure altitude |
| `vehicle_magnetometer` | Mag driver | 3-axis field measurement |
| `airspeed` / `airspeed_validated` | Airspeed sensor | True airspeed (TAS) |
| `vehicle_land_detected` | Land detector | `in_air`, `at_rest` flags |
| `vehicle_status` | Commander | Fixed-wing flag |
| `parameter_update` | Parameter server | Runtime parameter reload |

Additional conditional subscriptions: `distance_sensor` (range finder), `vehicle_optical_flow` (optical flow), `vehicle_visual_odometry` (external vision), `landing_target_pose` (aux velocity).

### 2.2 uORB Publications (estimator outputs)

Primary navigation outputs (forwarded through `EKF2Selector`):

| Topic | Contents |
|---|---|
| `vehicle_attitude` | Quaternion attitude `q`, angular velocity |
| `vehicle_local_position` | NED position and velocity, heading, validity flags |
| `vehicle_global_position` | WGS-84 latitude, longitude, altitude |
| `vehicle_odometry` | Combined pose + velocity in body frame |
| `wind` | Horizontal wind velocity North/East (m/s) |

Per-instance diagnostic outputs:

| Topic | Contents |
|---|---|
| `estimator_status` | Innovation test ratios, solution validity bitmask |
| `estimator_status_flags` | `filter_control_status_u` bitmask |
| `estimator_innovations` | Raw innovations for each active source |
| `estimator_innovation_variances` | Innovation variances |
| `estimator_innovation_test_ratios` | Normalized innovation squared per source |
| `estimator_sensor_bias` | Online gyro/accel/mag bias estimates |
| `estimator_baro_bias` / `estimator_gnss_hgt_bias` | Height sensor bias status |
| `estimator_states` | Full state vector snapshot |
| `estimator_event_flags` | Single-shot events (reset notifications) |
| `yaw_estimator_status` | EKFGSF composite yaw and per-model diagnostics |
| `ekf2_timestamps` | Per-cycle sensor latency timestamps |

---

## 3. State Vector

The EKF2 state vector is defined in `EKF/python/ekf_derivation/generated/state.h` as `StateSample`. The covariance matrix `P` is 24×24 (the quaternion occupies 4 storage elements but only 3 error-state DOF).

| Index (error-state) | Symbol | Description | Units | Frame |
|---|---|---|---|---|
| 0–2 | `quat_nominal` (δθ) | Quaternion attitude (rotation body→NED); error state is 3-component rotation vector | — | Body→NED |
| 3–5 | `vel` | NED velocity (North, East, Down) | m/s | NED |
| 6–8 | `pos` | NED position relative to local origin | m | NED |
| 9–11 | `gyro_bias` | Rate gyro bias (X, Y, Z body axes) | rad/s | Body |
| 12–14 | `accel_bias` | Accelerometer bias (X, Y, Z body axes) | m/s² | Body |
| 15–17 | `mag_I` | Earth magnetic field vector (North, East, Down) | Gauss | NED |
| 18–20 | `mag_B` | Magnetometer body-frame bias (X, Y, Z) | Gauss | Body |
| 21–22 | `wind_vel` | Horizontal wind velocity (North, East) | m/s | NED |
| 23 | `terrain` | Terrain height (AGL offset from NED origin) | m | NED-down |

The magnetometer states (`mag_I`, `mag_B`) and terrain state are present only when their respective compile-time features are enabled. The 13 "core" states relevant to flight navigation are quaternion attitude (3 DOF), NED velocity (3), NED position (3), gyro bias (3), and accel bias (3).

---

## 4. Prediction Step

### 4.1 IMU Down-Sampling (`ImuDownSampler`)

IMU data arrives at the native sensor rate (typically 250 Hz or higher). The `ImuDownSampler` class (`EKF/imu_down_sampler/imu_down_sampler.hpp`) accumulates delta-angle and delta-velocity increments until the target filter update interval is reached. The target interval is set by `EKF2_PREDICT_US` (default 10 000 µs = 100 Hz via `filter_update_interval_us` in `parameters`). The down-sampler returns a single `imuSample` containing the integrated `delta_ang` (rad) and `delta_vel` (m/s) for the accumulation window, together with their integration periods `delta_ang_dt` and `delta_vel_dt`. When the accumulated period crosses the threshold, `update()` returns `true` and the filter advances one prediction step.

### 4.2 State Prediction (`predictState`)

Called once per down-sampled IMU update. The strapdown navigation equations integrate the bias-corrected delta-angle and delta-velocity in the current body-to-earth rotation `_R_to_earth` to update the quaternion, velocity, and position states. Earth rotation (`_earth_rate_NED`) is subtracted from the angular rate to maintain accuracy over long-duration flights. Covariance propagation is handled separately by `predictCovariance`.

### 4.3 Covariance Prediction (`predictCovariance`)

Equations are auto-generated by the Python symbolic derivation tool (`EKF/python/ekf_derivation/`) and compiled into `ekf_derivation/generated/predict_covariance.h`. The process noise inputs are:

- `gyro_noise` (default 1.5 × 10⁻² rad/s): angular rate white noise for attitude and velocity propagation.
- `accel_noise` (default 3.5 × 10⁻¹ m/s²): accelerometer white noise for velocity propagation.
- `gyro_bias_p_noise` (default 1.0 × 10⁻³ rad/s²): random-walk process noise on gyro bias states.
- `accel_bias_p_noise` (default 1.0 × 10⁻² m/s³): random-walk process noise on accel bias states.
- `wind_vel_nsd` (default 1.0 × 10⁻² m/s²/√Hz): wind velocity spectral density, additionally scaled by a factor of 0.5 × vertical speed (`wind_vel_nsd_scaler`).

### 4.4 Output Predictor (`OutputPredictor`)

Because the EKF update runs on a delayed time horizon (to allow sensor data to arrive and be buffered), the filter state lags the current time by up to `delay_max_ms` (default 110 ms). The `OutputPredictor` (`EKF/output_predictor/output_predictor.h`) runs a strapdown INS at the full IMU rate on the current (non-delayed) time horizon, using the latest bias-corrected gyro and accelerometer data. After each EKF correction at the delayed horizon, `correctOutputStates` applies a proportional-integral correction to the output buffer so that the predictor tracks the EKF state. The complementary filter time constants are `EKF2_TAU_VEL` and `EKF2_TAU_POS`. The predicted quaternion and NED velocity/position exposed to flight control come from the output predictor, not the delayed EKF state, which eliminates the latency from the control loop.

---

## 5. Sensor Fusion (Aid Sources)

Each aid source is controlled by a dedicated `control*Fusion()` function called from `controlFusionModes()` every filter update cycle. Each source tracks its state in an `estimator_aid_source1d_s`, `estimator_aid_source2d_s`, or `estimator_aid_source3d_s` struct containing the most recent observation, innovation, innovation variance, test ratio, and last-fuse timestamp. The gate test is `innovation² / (gate² × innovation_variance) > 1`, and any sample that fails the gate is flagged `innovation_rejected` and not fused.

### 5.1 GNSS (GPS)

**Source file:** `EKF/aid_sources/gnss/gps_control.cpp`, `gps_checks.cpp`, `gnss_height_control.cpp`

**What it measures:** 3D position (lat/lon/alt from `gnssSample`), 3D NED velocity, and optionally dual-antenna heading (`yaw`).

**Activation:** Controlled by `EKF2_GPS_CTRL` bitmask (`GnssCtrl::HPOS`, `VPOS`, `VEL`, `YAW`). Before any GPS data is used, `runGnssChecks()` must pass continuously for `_min_gps_health_time_us` (default 10 s). The health check evaluates, depending on the `EKF2_GPS_CHECK` bitmask:

- Fix type ≥ 3D (`fix_type ≥ 3`)
- Satellite count ≥ `EKF2_REQ_NSATS` (default 6)
- PDOP ≤ `EKF2_REQ_PDOP` (default 2.0)
- Horizontal accuracy ≤ `EKF2_REQ_EPH` (default 5.0 m)
- Vertical accuracy ≤ `EKF2_REQ_EPV` (default 8.0 m)
- Speed accuracy ≤ `EKF2_REQ_SACC` (default 1.0 m/s)
- Horizontal/vertical drift (stationary only), horizontal speed (stationary only)
- Spoof detection flag

Once checks have passed and GPS fusion is active, if `runGnssChecks()` fails for longer than `reset_timeout_max` (7 s), fusion stops.

**Innovation gates:** Position: `EKF2_GPS_P_GATE` (default 5 STD). Velocity: `EKF2_GPS_V_GATE` (default 5 STD).

**States corrected:** Horizontal/vertical position, NED velocity, indirectly attitude through velocity corrections.

**Height bias:** A `HeightBiasEstimator` (`_gps_hgt_b_est`) tracks the slowly-varying offset between GPS altitude and the filter's internal altitude reference, using process noise `gps_hgt_bias_nsd` (default 0.13 m/s/√Hz).

**Yaw (dual antenna):** When `GnssCtrl::YAW` is set and the GNSS receiver provides `yaw` and `yaw_acc`, `controlGnssYawFusion()` fuses the heading as a direct yaw measurement with observation noise `gnss_heading_noise` (default 0.1 rad).

### 5.2 Barometer

**Source file:** `EKF/aid_sources/barometer/` (control in `ekf.h` `controlBaroHeightFusion`)

**What it measures:** Pressure altitude above MSL (`baroSample::hgt`), used as a vertical position observation.

**Activation:** Enabled when `EKF2_BARO_CTRL` is non-zero. Barometer is the default height reference sensor (`EKF2_HGT_REF = BARO`). Fusion is paused when ground-effect compensation is active (`gnd_effect` flag) and the baro innovation is negative, within the `gnd_effect_deadzone` (default 5.0 m). Barometer fusion times out if no successful fuse occurs within `hgt_fusion_timeout_max` (5 s).

**Bias estimator:** `_baro_b_est` is a `HeightBiasEstimator` that tracks the offset between the barometric altitude and the filter altitude reference with process noise `baro_bias_nsd` (default 0.13 m/s/√Hz). The bias is removed before the innovation is computed, so the filter does not need to re-converge after a static pressure shift.

**Innovation gate:** `EKF2_BARO_GATE` (default 5 STD). Observation noise: `EKF2_BARO_NOISE` (default 2.0 m).

**States corrected:** Vertical position (`pos.z`), indirectly vertical velocity.

**Dynamic pressure compensation:** When `CONFIG_EKF2_BARO_COMPENSATION` is enabled, static pressure position error coefficients (`EKF2_PCOEF_*`) correct the baro reading for dynamic pressure effects at speed.

### 5.3 Magnetometer

**Source file:** `EKF/aid_sources/magnetometer/mag_control.cpp`, `mag_fusion.cpp`

**What it measures:** 3-axis magnetic field vector in body frame (`magSample::mag`, Gauss).

**Modes:** Selected by `EKF2_MAG_TYPE`:
- `AUTO` (0): The filter selects between simple heading fusion (`mag_hdg`) and full 3-axis fusion (`mag_3D`) based on whether the vehicle is maneuvering. Heading fusion is used when horizontal manoeuvre acceleration is below `EKF2_MAG_ACCLIM` (default 0.5 m/s²); 3D fusion engages otherwise and after in-flight alignment completes.
- `HEADING` (1): Forces simple heading-only fusion.
- `NONE` (5): Magnetometer completely disabled.
- `INIT` (6): Used only for initial heading alignment.

**Heading fusion (`mag_hdg`):** The yaw angle derived from the magnetometer is fused as a single scalar measurement. The observation noise is `EKF2_HEAD_NOISE` (default 0.3 rad). Gate size is `EKF2_HDG_GATE` (default 2.6 STD).

**3D fusion (`mag_3D`):** All three magnetic field components are fused using the full `fuseMag()` Jacobian. The earth-field model (`mag_I`) and body-frame bias (`mag_B`) states are updated. Observation noise: `EKF2_MAG_NOISE` (default 5 × 10⁻² Gauss). Gate size: `EKF2_MAG_GATE` (default 3 STD). Process noise for earth-field states: `EKF2_MAG_E_NOISE` (default 1 × 10⁻³ Gauss/s). Process noise for body-frame bias: `EKF2_MAG_B_NOISE` (default 1 × 10⁻⁴ Gauss/s). The body-frame bias limit is ±0.5 Gauss (`getMagBiasLimit()`).

**Magnetic field checks:** When `EKF2_MAG_CHECK` enables them, the measured field strength and inclination are compared against the World Magnetic Model (WMM) values obtained from the last valid GPS position. Tolerances are `EKF2_MAG_CHK_STR` (default 0.2 Gauss) and `EKF2_MAG_CHK_INC` (default 20 deg).

**In-flight alignment:** On the first transition to in-air, the mag field states are re-initialised and the `mag_aligned_in_flight` flag is set. `isYawFinalAlignComplete()` returns true only after in-flight alignment has been active for at least 1 s.

**States corrected:** Quaternion attitude (yaw primarily), earth-field vector `mag_I`, body bias `mag_B`.

### 5.4 Airspeed

**Source file:** `EKF/aid_sources/airspeed/airspeed_fusion.cpp`

**What it measures:** True airspeed (TAS, m/s) from `airspeedSample::true_airspeed`. The EAS-to-TAS factor `eas2tas` scales the observation noise: `R = (EAS_NOISE × eas2tas)²`.

**Activation:** Requires `EKF2_ARSP_THR > 0` (default 2.0 m/s). Fusion starts when the aircraft is in-air (`in_air` flag), not in fake-position mode, and the measured TAS exceeds `arsp_thr`. It deactivates if data stops for more than 1 s or if fusion fails for more than 10 s.

**Wind initialization (`resetWindUsingAirspeed`):** When airspeed fusion first starts and no valid wind estimate exists (or wind variance exceeds `initial_wind_uncertainty²` = 1.0 m²/s²), the wind states are initialised from the TAS measurement, current heading, ground velocity, and an assumed sideslip variance of (15 deg)². This is the primary mechanism for wind state bootstrap in normal flight.

**Innovation gate:** `EKF2_TAS_GATE` (default 5 STD). Observation noise parameter: `EKF2_EAS_NOISE` (default 1.4 m/s).

**States corrected:** When another source is providing horizontal aiding (`!wind_dead_reckoning`), only `wind_vel` states are corrected. During wind dead reckoning, all states can be corrected.

**EKFGSF coupling:** When airspeed fusion is active, the current TAS is forwarded to the EKFGSF yaw estimator via `_yawEstimator.setTrueAirspeed()` to enable centripetal acceleration compensation.

### 5.5 Sideslip Constraint

**Source file:** `EKF/aid_sources/sideslip/sideslip_fusion.cpp`

**What it measures:** A synthetic zero-sideslip observation (β = 0). This is a model-based constraint rather than a real sensor: it asserts that a fixed-wing aircraft flies coordinated and does not slip sideways.

**Activation:** Controlled by `EKF2_FUSE_BETA` (default disabled). Active when the flag is set, the vehicle is fixed-wing (`fixed_wing` or `fuse_aspd`), is in-air, and is not in fake-position mode. Fusion is time-triggered at intervals of `beta_avg_ft_us` (default 150 ms).

**Observation model:** `ComputeSideslipInnovAndInnovVar()` computes the innovation as the projection of aerodynamic velocity onto the body Y axis. Observation noise: `EKF2_BETA_NOISE` (default 0.3 rad). Gate: `EKF2_BETA_GATE` (default 5 STD).

**States corrected:** Wind velocity (`wind_vel`), and when in wind dead-reckoning mode, also velocity and position states. Sideslip fusion is the other source (with airspeed) that keeps wind estimation alive when GPS-based velocity is unavailable.

### 5.6 Drag Model

**Source file:** `EKF/aid_sources/drag/drag_fusion.cpp`

**What it measures:** Specific force (accelerometer measurements) along the body X and Y axes, interpreted as aerodynamic drag. This is primarily designed for multirotor wind estimation but is also available for fixed-wing drag-model use.

**Activation:** Controlled by `EKF2_DRAG_CTRL` (default 0 = disabled). When enabled and the vehicle is in-air without fake-position, the wind states are initialised if not already active.

**Drag models:** Two models can be combined:
- Ballistic (bluff-body) drag: coefficients `EKF2_BCOEF_X` and `EKF2_BCOEF_Y` (kg/m²). Active when the coefficient is > 1.0.
- Rotor momentum drag: coefficient `EKF2_MCOEF` (1/s). The momentum drag coefficient is corrected for air density at altitude: `mcoef_corrected = mcoef × sqrt(rho / rho_sea_level)`.

**States corrected:** Wind velocity (`wind_vel`).

**Note for stratospheric operations:** At high altitude, air density (ρ) is much lower than sea-level standard (1.225 kg/m³). The drag model explicitly corrects rotor momentum drag for this through the density-ratio term. For a fixed-wing glider the bluff-body coefficients would need to be characterised for the actual airframe.

### 5.7 Zero Velocity Update (ZVU)

**Source file:** `EKF/aid_sources/ZeroVelocityUpdate.cpp`

**What it measures:** A synthetic zero-velocity observation (all three NED velocity components = 0).

**Activation:** Active when `vehicle_at_rest` flag is set. Fuses at a rate of one update per 200 ms.

**Observation variance:** `(0.2 m/s)²` once tilt is aligned; `(0.001 m/s)²` during initial alignment to speed up levelling.

**States corrected:** All three velocity components directly via `fuseDirectStateMeasurement()`. The ZVU is inhibited when vertical velocity aiding is already active and the tilt is already aligned, to avoid over-constraining a filter that is already being aided.

### 5.8 Zero Gyro Update (ZGU)

**Source file:** `EKF/aid_sources/ZeroGyroUpdate.cpp`

**What it measures:** The gyro rate itself, treated as a direct observation of the gyro bias when the vehicle is stationary.

**Activation:** Active when `vehicle_at_rest`. Gyro delta-angle samples are accumulated over a 0.2 s window. The average angular rate is fused directly against the `gyro_bias` state.

**Observation variance:** `gyro_noise²`.

**States corrected:** `gyro_bias` (indices 9–11 in the state vector) directly via `fuseDirectStateMeasurement()`. This provides rapid gyro bias convergence on the ground before flight.

---

## 6. EKF2Selector

**Source file:** `EKF2Selector.hpp`, `EKF2Selector.cpp`

When multiple IMUs are available (`CONFIG_EKF2_MULTI_INSTANCE`), the system spawns one `EKF2` instance per IMU (up to `EKF2_MAX_INSTANCES`, which is 9 in unconstrained builds or 2 in memory-constrained builds). `EKF2Selector` is a separate scheduled work item that monitors all running instances and selects the healthiest one to forward to the system-wide `vehicle_attitude`, `vehicle_local_position`, `vehicle_global_position`, `wind`, and `vehicle_odometry` topics.

**Health assessment:** For each instance, `UpdateErrorScores()` computes a `combined_test_ratio` from `estimator_status` innovation test ratios and a `relative_test_ratio` comparing each instance against the current primary. The selector maintains per-instance `healthy` hysteresis (1 s to declare healthy), `warning`, `filter_fault`, and `timeout` flags. IMU raw consistency is monitored via `sensors_status_imu` (gyro inconsistency in rad/s and accel inconsistency) and accumulated into `_accumulated_gyro_error` and `_accumulated_accel_error`.

**Switching logic:** An instance switch requires the challenger's `relative_test_ratio` to exceed `_rel_err_thresh` (0.5) relative to the current primary. Switching is suppressed by the 1 s healthy hysteresis to avoid oscillating between borderline-healthy instances.

**Reset propagation:** When the selected instance changes, or when the selected instance resets its own states, `EKF2Selector` re-accumulates and re-publishes consistent reset counters for `posNE`, `posD`, `velNE`, `velD`, `quat`, and `hagl` so that downstream consumers can detect and compensate for any step change.

---

## 7. EKFGSF Yaw Estimator

**Source file:** `EKF/yaw_estimator/EKFGSF_yaw.h`, `EKFGSF_yaw.cpp`

`EKFGSF_yaw` is an independent Gaussian Sum Filter (GSF) composed of a bank of `N_MODELS_EKFGSF = 5` small EKFs, each estimating the state vector `[vel_N, vel_E, yaw]`. It is instantiated as `_yawEstimator` inside the main `Ekf` class and is conditionally compiled with `CONFIG_EKF2_GNSS`.

**Prediction:** Each sub-EKF runs an AHRS complementary filter using `delta_ang` and `delta_vel` to propagate its 3-DOF attitude. The five models are initialised with different initial yaw seeds spanning 360 deg, so the bank can converge from any unknown heading.

**GPS velocity update:** When `fuseVelocity()` is called with a NE GPS velocity observation, each model is updated independently. The normalized innovation squared (NIS) for each model drives its Gaussian weight via `gaussianDensity()`. The composite yaw estimate `_gsf_yaw` and variance `_gsf_yaw_variance` are computed as the weighted mean and variance of all model estimates.

**Airspeed:** When TAS is available, it is passed via `setTrueAirspeed()` and used inside `ahrsPredict()` to compensate for centripetal acceleration during coordinated turns, improving yaw estimate quality.

**Gyro bias seeding:** `setGyroBias()` initialises all model gyro biases from the main EKF's current `gyro_bias` estimate. This is called at the start of each GPS control cycle (except when gyro bias is inhibited).

**Emergency yaw reset:** If the main filter detects a yaw failure (`isYawFailure()`) and horizontal velocity fusion has timed out for `EKFGSF_reset_delay` (1 s) since takeoff, `tryYawEmergencyReset()` is called. If the EKFGSF composite yaw variance is below `EKFGSF_yaw_err_max` (0.262 rad ≈ 15 deg), `resetYawToEKFGSF()` resets the main EKF's quaternion yaw to the EKFGSF estimate. This mechanism allows heading recovery without a magnetometer.

**Default TAS:** If airspeed fusion is not active but the vehicle is fixed-wing and in-air, the yaw estimator is fed `EKFGSF_tas_default` (default 15.0 m/s) to allow centripetal compensation even without a sensor.

---

## 8. Bias Estimators

### 8.1 Gyro and Accel Bias (State-Augmented)

Gyro bias (`gyro_bias`, states 9–11) and accel bias (`accel_bias`, states 12–14) are part of the primary EKF state vector and are estimated continuously alongside position and velocity. Process noise values `gyro_bias_p_noise` and `accel_bias_p_noise` (see Section 4.3) drive the bias covariance growth when the filter cannot observe bias directly.

Bias learning is inhibited axis-by-axis (`_accel_bias_inhibit[]`, `_gyro_bias_inhibit[]`) when:
- The IMU acceleration magnitude exceeds `EKF2_ABL_ACCLIM` (default 25 m/s²).
- The IMU angular rate magnitude exceeds `EKF2_ABL_GYRLIM` (default 3 rad/s).
- Accelerometer clipping (`delta_vel_clipping`) is detected.

Maximum bias magnitude is clipped to `EKF2_ABL_LIM` (default 0.4 m/s²) for accel and `EKF2_GYR_B_LIM` (default 0.4 rad/s) for gyro. Minimum covariance floor is `kGyroBiasVarianceMin = 1 × 10⁻⁹` and `kAccelBiasVarianceMin = 1 × 10⁻⁹`.

In-flight calibration (`InFlightCalibration` in `EKF2.hpp`) accumulates the bias estimate while valid aiding is active and periodically saves it back to the calibration parameters.

### 8.2 Barometer Height Bias (`HeightBiasEstimator`)

`_baro_b_est` is a single-state `BiasEstimator` that tracks the slowly varying difference between the barometric altitude and the EKF altitude reference. It is modelled as a random walk with spectral density `baro_bias_nsd` (default 0.13 m/s/√Hz). The bias is subtracted from raw baro readings before they enter the EKF update, so baro drift (e.g., from temperature gradients) is absorbed without corrupting the core altitude state. A 3-sigma gate is applied to reject outliers.

### 8.3 GPS Height Bias (`HeightBiasEstimator`)

`_gps_hgt_b_est` operates identically to the baro bias estimator but for the GPS altitude channel, with `gps_hgt_bias_nsd` (default 0.13 m/s/√Hz).

### 8.4 Magnetometer Bias (State-Augmented)

The magnetometer body-frame bias `mag_B` (states 18–20) is part of the EKF state vector. It is estimated during 3D magnetometer fusion using process noise `magb_p_noise` (default 1 × 10⁻⁴ Gauss/s). The covariance floor is `kMagVarianceMin = 1 × 10⁻⁶ Gauss²`. During `mag_hdg` (heading-only) fusion, only the heading angle is updated; the bias states are uncorrected until 3D fusion re-engages.

---

## 9. Key Parameters

| Parameter | Default | Description |
|---|---|---|
| `EKF2_GPS_CTRL` | 7 (HPOS\|VPOS\|VEL) | Bitmask enabling GPS position, height, and velocity fusion |
| `EKF2_GPS_CHECK` | 21 | Bitmask of GPS pre-flight quality checks (nsats, pdop, hacc, vacc, sacc, vspeed by default) |
| `EKF2_REQ_EPH` | 5.0 m | Maximum acceptable GPS horizontal position error to allow fusion |
| `EKF2_REQ_EPV` | 8.0 m | Maximum acceptable GPS vertical position error |
| `EKF2_REQ_SACC` | 1.0 m/s | Maximum acceptable GPS speed accuracy |
| `EKF2_REQ_NSATS` | 6 | Minimum satellite count for GPS health |
| `EKF2_ARSP_THR` | 2.0 m/s | Minimum TAS to activate airspeed fusion (0 = disabled) |
| `EKF2_BARO_NOISE` | 2.0 m | Barometer observation noise (1-sigma) |
| `EKF2_ACC_NOISE` | 0.35 m/s² | Accelerometer white noise (covariance prediction) |
| `EKF2_GYR_NOISE` | 0.015 rad/s | Rate gyro white noise (covariance prediction) |
| `EKF2_MAG_NOISE` | 0.05 Gauss | 3-axis magnetometer observation noise (1-sigma) |
| `EKF2_WIND_NSD` | 0.01 m/s²/√Hz | Wind velocity process noise spectral density |
| `EKF2_FUSE_BETA` | 0 (disabled) | Enable synthetic sideslip fusion (required for fixed-wing wind estimation) |
| `EKF2_MAG_TYPE` | 0 (AUTO) | Magnetometer fusion mode (0=auto, 1=heading-only, 5=none, 6=init-only) |
| `EKF2_HGT_REF` | 0 (BARO) | Primary height reference sensor |
| `EKF2_NOAID_TOUT` | 5 000 000 µs | Dead-reckoning timeout before position estimates declared invalid |
| `EKF2_GSF_TAS` | 15.0 m/s | Default TAS for EKFGSF when airspeed sensor is absent |
| `EKF2_PREDICT_US` | 10 000 µs | Filter update interval (IMU down-sampling target) |
| `EKF2_DELAY_MAX` | 110 ms | Maximum sensor-to-fusion time delay (sets IMU buffer size) |

---

## 10. Stratospheric Considerations

### 10.1 GPS Signal Quality Above 20 km

Stratospheric GPS reception is generally stronger in terms of sky visibility (no terrain or tree masking), but the number of visible satellites may drop as the receiver approaches the edge of civil aviation receiver firmware limits. The `EKF2_GPS_CHECK` mask and threshold parameters (`EKF2_REQ_NSATS`, `EKF2_REQ_EPH`, etc.) must be validated against the specific GNSS receiver firmware behavior above 18 km (COCOM altitude limit for some receivers). If the receiver enforces a COCOM altitude cutoff, GPS fusion will stop, and the filter will enter inertial dead reckoning. The `_min_gps_health_time_us` variable (10 s default, adjustable via `set_min_required_gps_health_time()`) must elapse with consistent health before the filter re-accepts GPS data after a dropout.

Spoofed GNSS detection is evaluated in `runGnssChecks()` via `gps.spoofed`; if set, `gps_check_fail_status.flags.spoofed` trips and fusion is blocked.

### 10.2 Barometer Thermal Gradient Effects

Above the tropopause (~11 km), the temperature lapse rate changes and the International Standard Atmosphere (ISA) model underlying the altimeter equation becomes less accurate. The barometer height bias estimator (`_baro_b_est`) will absorb a slowly evolving pressure-to-altitude conversion error as a random walk, but it cannot track large or rapid bias jumps. At very high altitude, the absolute accuracy of baro-derived altitude degrades and GPS height becomes a more reliable reference. Increasing `EKF2_HGT_REF` to `GNSS` and increasing `EKF2_BARO_NOISE` to account for higher altitude uncertainty is recommended for stratospheric flight phases.

Dynamic pressure compensation (`EKF2_PCOEF_*`) is calibrated at the airframe level and may not transfer accurately across the wide dynamic pressure range from low to high altitude.

### 10.3 Drag Model at Low Air Density

The drag fusion model in `fuseDrag()` explicitly accounts for air density through the `rho` variable (`set_air_density()` in `EstimatorInterface`). The rotor momentum drag coefficient is scaled as `mcoef × sqrt(rho / rho_sea_level)`. At 20 km altitude, air density is approximately 0.089 kg/m³ (about 7% of sea level), reducing the effective momentum drag correction by a factor of ~3.7. For a fixed-wing glider, the bluff-body drag coefficients (`EKF2_BCOEF_X`, `EKF2_BCOEF_Y`) represent a force-per-unit-velocity that scales directly with density; as density falls, the drag signal becomes much weaker and potentially indistinguishable from noise, making drag-based wind estimation unreliable above approximately 15–20 km. Sideslip and airspeed fusion are more robust alternatives at altitude.

### 10.4 Wind Estimation at Altitude

Wind velocity states (`wind_vel`, states 21–22) are initialized from `resetWindUsingAirspeed()` using the TAS measurement, current heading, and ground velocity. The wind process noise `wind_vel_nsd` with the vertical-speed scalar causes wind covariance to grow faster during altitude changes — this is intended to account for wind shear. At stratospheric altitudes, horizontal wind speeds can exceed 50 m/s in the jet stream. The initial wind uncertainty of 1.0 m/s (`initial_wind_uncertainty`) and the process noise default of 0.01 m/s²/√Hz may need to be increased to allow the filter to track large, rapidly changing winds. Wind dead reckoning (`wind_dead_reckoning` control flag) is set when only airspeed and sideslip are providing horizontal aiding but no velocity-referenced source (GPS) is active.

---

## 11. Failure Modes and Dead Reckoning

### 11.1 GPS Loss — Inertial Dead Reckoning

When all horizontal velocity and position aiding sources cease producing valid fused measurements, `updateHorizontalDeadReckoningstatus()` sets `inertial_dead_reckoning = true` and sets `control_status.flags.inertial_dead_reckoning`. The filter continues to propagate position and velocity using integrated IMU measurements only. Position accuracy degrades at a rate determined by accelerometer noise, bias, and the covariance propagation.

The horizontal dead-reckoning clock is tracked via `_time_last_horizontal_aiding`. Once inertial dead reckoning begins, if `_time_last_horizontal_aiding` exceeds `valid_timeout_max` (default 5 s, set by `EKF2_NOAID_TOUT`), the `_horizontal_deadreckon_time_exceeded` flag is set and `isLocalHorizontalPositionValid()` returns `false`.

**Position validity flags:**

| Flag/Method | Meaning |
|---|---|
| `isLocalHorizontalPositionValid()` | Returns `!_horizontal_deadreckon_time_exceeded` |
| `isLocalVerticalPositionValid()` | Returns `!_vertical_position_deadreckon_time_exceeded` |
| `isLocalVerticalVelocityValid()` | Returns `!_vertical_velocity_deadreckon_time_exceeded` |
| `isGlobalHorizontalPositionValid()` | Horizontal valid AND global origin initialized |
| `isGlobalVerticalPositionValid()` | Vertical valid AND altitude origin set |
| `control_status.flags.inertial_dead_reckoning` | True when no velocity/position aiding active |
| `control_status.flags.wind_dead_reckoning` | True when only airspeed+sideslip aiding (no absolute position) |

### 11.2 Vertical Dead Reckoning

`updateVerticalDeadReckoningStatus()` tracks separate flags for vertical position and vertical velocity. If no vertical position aiding (baro, GPS height, range finder, or external vision height) has fused successfully within `valid_timeout_max`, `_vertical_position_deadreckon_time_exceeded` is set. If vertical velocity aiding also times out while vertical position is already exceeded, `_vertical_velocity_deadreckon_time_exceeded` is set.

### 11.3 Fake Position and Height Fusion

When the dead reckoning time is exceeded and no real aiding is available, the filter can optionally activate fake position fusion (`controlFakePosFusion()`) and fake height fusion (`controlFakeHgtFusion()`). Fake position fuses the last known position with high noise (`pos_noaid_noise = 10.0 m`) to prevent the covariance from growing without bound while signalling to the navigation stack that position is uncertain. `control_status.flags.fake_pos` and `control_status.flags.fake_hgt` indicate when this is active.

### 11.4 Vertical Accelerometer Fault Detection

`checkVerticalAccelerationHealth()` monitors vertical accelerometer data for clipping and consistency failures. If the vertical innovation tests fail continuously for `bad_acc_reset_delay_us` (500 ms), the filter triggers a height reset to avoid a diverging vertical state. The `fault_status.flags.bad_acc_vertical` and `bad_acc_clipping` flags signal these conditions.

### 11.5 Yaw Failure and EKFGSF Recovery

`isYawFailure()` checks whether heading fusion is failing (e.g., magnetometer fault). If the yaw fails and horizontal velocity innovations have been failing for `EKFGSF_reset_delay` (1 s) since the last ground contact, `tryYawEmergencyReset()` attempts to recover heading from the EKFGSF estimator, provided its composite yaw variance is below `EKFGSF_yaw_err_max` (0.262 rad). A successful reset sets the `yaw_aligned_to_imu_gps` information event flag.

### 11.6 Magnetometer Fault

If the magnetometer fusion is consistently rejected (`mag_fault` flag), the filter falls back to EKFGSF-based heading or, if that is unavailable, uses the last known heading until it can be recovered. The `control_status.flags.mag_fault` bit is set and `stopMagFusion()` disables further magnetometer updates until a reset occurs.
