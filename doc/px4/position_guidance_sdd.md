# Software Design Description — Position and Guidance Algorithm

**Module:** `fw_pos_control`
**Source:** `src/modules/fw_pos_control/FixedwingPositionControl.cpp/.hpp`
**Libraries:** `src/lib/npfg/`, `src/lib/tecs/`, `src/lib/l1/`
**Update Rate:** 50 Hz (triggered by `vehicle_local_position`)

---

## 1. Purpose

`fw_pos_control` is the guidance layer of the GNC stack. It sits between the mission/navigation layer (Navigator) and the attitude control layer (fw_att_control). Given a position setpoint from the mission, it determines what attitude and throttle the aircraft must command to follow the desired path and maintain the desired altitude and airspeed.

It is the most complex module in the GNC stack (~3,500 lines). All path-following math lives in the NPFG and L1 libraries; all altitude/airspeed energy management lives in TECS. `fw_pos_control` is the orchestrator that calls those libraries and manages mode transitions.

---

## 2. Inputs and Outputs

### Inputs (uORB subscriptions)

| Topic | Content | Source |
|-------|---------|--------|
| `vehicle_local_position` | Position, velocity (NED) | EKF2 — **trigger topic** |
| `vehicle_attitude` | Quaternion attitude | EKF2 |
| `position_setpoint_triplet` | Previous / current / next waypoints | Navigator |
| `airspeed_validated` | True and calibrated airspeed | airspeed_selector |
| `vehicle_air_data` | Baro altitude, air density | Barometer driver |
| `wind` | Wind velocity estimate (NED) | EKF2 |
| `vehicle_control_mode` | Armed, auto/manual mode flags | Commander |
| `vehicle_status` | Vehicle type, in_transition flags | Commander |
| `manual_control_setpoint` | Stick inputs (manual modes) | RC input |
| `release_status` | Altitude release state | altitude_release module |

### Outputs (uORB publications)

| Topic | Content | Consumer |
|-------|---------|----------|
| `vehicle_attitude_setpoint` | Roll, pitch, yaw, thrust setpoints | fw_att_control |
| `tecs_status` | TECS internal state for logging | Logger / GCS |
| `position_controller_status` | Cross-track error, nav bearing | Logger / GCS |
| `npfg_status` | NPFG internal state | Logger / GCS |
| `flaps_setpoint` | Flap deflection command | Control allocator |
| `spoilers_setpoint` | Spoiler deflection command | Control allocator |
| `esc_stow_command` | Propeller park command | ESC driver |
| `launch_detection_status` | Launch detection state | GCS / logger |

---

## 3. Architecture

The module contains two independent but coordinated sub-algorithms:

```
position_setpoint_triplet
        │
        ▼
┌───────────────────────────────────────────────────┐
│              fw_pos_control                        │
│                                                   │
│  ┌─────────────────────┐  ┌─────────────────────┐ │
│  │   Lateral Guidance  │  │  Longitudinal Ctrl  │ │
│  │                     │  │                     │ │
│  │  NPFG / L1          │  │       TECS          │ │
│  │                     │  │                     │ │
│  │  path error →       │  │  altitude + airspd  │ │
│  │  roll setpoint      │  │  error → pitch sp   │ │
│  │                     │  │  + throttle         │ │
│  └─────────────────────┘  └─────────────────────┘ │
│                                                   │
└───────────────────────────────────────────────────┘
        │                           │
        ▼                           ▼
   roll_body                  pitch_body + thrust
        └─────────────┬─────────────┘
                      ▼
           vehicle_attitude_setpoint
```

---

## 4. Lateral Guidance

### 4.1 NPFG — Nonlinear Path Following Guidance

NPFG (`src/lib/npfg/`) is the primary lateral guidance law. It is the default in this codebase (L1 is retained as a fallback/legacy).

**Concept:** Given the vehicle position and a desired path (line or arc), NPFG computes a bearing command that drives the vehicle onto the path. It explicitly accounts for wind — it commands the aircraft to point into the wind vector as needed to track the ground path. Unlike L1, NPFG's behavior is well-defined across the full airspeed envelope, making it preferable for vehicles with a wide altitude/airspeed range.

**Key computation** (`navigateLine`, `navigateWaypoints`, `navigateLoiter`):
1. Compute the closest point on the desired path to the current vehicle position
2. Compute cross-track error (signed perpendicular distance from path)
3. Compute the path tangent unit vector
4. Call `_npfg.guideToPath(vehicle_pos, ground_vel, wind_vel, unit_path_tangent, closest_point, curvature)`
5. Retrieve roll setpoint: `getCorrectedNpfgRollSetpoint()`

The NPFG output is a roll angle setpoint. Roll setpoint is scaled toward zero if NPFG reports insufficient confidence in velocity/wind estimates (`canRun()` factor).

**Path primitives supported:**
- `navigateWaypoints()` — line segment A→B with overshoot/undershoot handling
- `navigateWaypoint()` — fly directly to a single point
- `navigateLine()` — follow an infinite line through two points
- `navigateLoiter()` — orbit a center point at a given radius and direction
- `navigatePathTangent()` — follow a path given an explicit tangent vector
- `navigateBearing()` — fly a constant heading

**NPFG tuning parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `NPFG_PERIOD` | 10.0 s | Control bandwidth — lower = tighter tracking |
| `NPFG_DAMPING` | 0.7 | Damping ratio — 0.7 is critically damped |
| `NPFG_ROLL_TC` | 0.5 s | Roll response time constant used in guidance |
| `NPFG_SW_DST_MLT` | 0.32 | Waypoint switch distance multiplier |
| `NPFG_PERIOD_SF` | 1.5 | Safety factor applied to period lower bound |
| `FW_R_LIM` | 50° | Maximum commanded roll angle |

### 4.2 L1 Guidance (Legacy)

`src/lib/l1/ECL_L1_Pos_Controller.cpp` — retained as a fallback. The core equation:

```
η = η₁ + η₂
lateral_accel = K_L1 * V² / L1_distance * sin(η)
```

Where:
- `η₁` = angle from the path line to the L1 point (function of cross-track error)
- `η₂` = angle between ground velocity vector and the path line
- `L1_distance = L1_ratio * ground_speed` — scales with speed
- `K_L1 = 4 * damping²`

L1 does not account for wind explicitly; NPFG is preferred for this vehicle.

---

## 5. Longitudinal Control — TECS

**Source:** `src/lib/tecs/TECS.cpp`

### 5.1 Concept

TECS (Total Energy Control System) manages altitude and airspeed simultaneously by controlling total specific mechanical energy and the distribution of energy between kinetic and potential forms.

**Specific Total Energy (STE):**
```
E_total = E_kinetic + E_potential = ½V² + g·h
```

**Specific Energy Balance (SEB)** — how energy is split between speed and altitude:
```
E_balance = E_potential - E_kinetic
```

- **Throttle** controls the total energy rate (climb + speed together)
- **Pitch** controls the energy balance (trading speed for altitude or vice versa)

This decouples the two longitudinal axes cleanly, which is superior to independent altitude and airspeed PID loops — especially for a glider where throttle authority is intermittent.

### 5.2 TECS Internal Structure

TECS has three internal components:

**1. Airspeed Filter (`TECSAirspeedFilter`)**
A steady-state Kalman filter on EAS and EAS rate. Smooths pitot noise before it enters the energy calculations. Falls back to trim airspeed if the sensor is invalid.

**2. Altitude Reference Model (`TECSAltitudeReferenceModel`)**
A second-order reference model that generates smooth altitude and altitude-rate setpoints from the step commands issued by the guidance layer. Prevents demanding instantaneous climb rates that exceed vehicle capability.

**3. Control (`TECSControl`)**
Computes throttle and pitch setpoints from energy errors.

- **Throttle:** proportional + integral on STE rate error, with slew rate limiting
- **Pitch:** proportional + integral on SEB rate error, converted to pitch angle via climb angle

The `speed_weight` parameter (`FW_T_SPDWEIGHT`) blends control authority between altitude and airspeed:
- 0 = all priority to altitude (no airspeed sensor fallback mode)
- 1 = balanced (default)
- 2 = all priority to airspeed (e.g. underspeed protection)

### 5.3 Key TECS Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `FW_T_ALT_TC` | 5.0 s | Altitude error time constant |
| `FW_T_THR_DAMPING` | 0.05 | Throttle damping gain |
| `FW_T_THR_INTEG` | 0.02 | Throttle integrator gain |
| `FW_T_I_GAIN_PIT` | 0.1 | Pitch integrator gain |
| `FW_T_PTCH_DAMP` | 0.1 | Pitch damping gain |
| `FW_T_SPDWEIGHT` | 1.0 | Speed vs altitude priority weighting |
| `FW_T_VERT_ACC` | 7.0 m/s² | Max vertical acceleration limit |
| `FW_T_SINK_MAX` | 5.0 m/s | Maximum sink rate |
| `FW_T_RLL2THR` | 15.0 | Roll-to-throttle feedforward (compensates for banked turns) |
| `FW_T_F_ALT_ERR` | -1.0 m | Altitude error threshold for fast descent mode |
| `FW_THR_MAX` | 1.0 | Maximum throttle (set to 0 during ascent, enabled on release) |
| `FW_THR_MIN` | 0.0 | Minimum throttle |
| `FW_P_LIM_MAX` | 30° | Maximum pitch up limit |
| `FW_P_LIM_MIN` | -30° | Maximum pitch down limit |

---

## 6. Mission Mode Handling

`fw_pos_control` manages the following flight phases internally:

### 6.1 Normal Waypoint Navigation
Navigator outputs a `position_setpoint_triplet` (prev / current / next waypoints). `fw_pos_control` calls `navigateWaypoints()` for lateral guidance and sets TECS altitude/airspeed targets from the current waypoint's altitude and `cruising_speed` fields.

### 6.2 Loiter
`navigateLoiter()` generates a circular orbit around the loiter center. TECS holds loiter altitude. Orbit direction and radius come from the position setpoint.

### 6.3 Takeoff (Launch Detection)
`LaunchDetector` (`launchdetection/`) monitors IMU acceleration to detect catapult/hand launch. Before launch is detected, throttle is held at idle. After launch, TECS takes over and climbs to the initial waypoint altitude.

### 6.4 Landing
Three-phase sequence:
1. **Approach** — NPFG follows the glide slope intercept path, TECS manages descent rate
2. **Flare** — triggered at `FW_LND_FLALT` above touchdown. Pitch is held to `FW_LND_FL_PMAX`, throttle goes to idle
3. **Touchdown** — detected by ground contact; resets state

Landing abort is supported via `VEHICLE_CMD_DO_GO_AROUND` before flare initiation.

### 6.5 Figure-Eight Pattern
`figure_eight/` — executes a figure-eight holding pattern around a center point. Uses NPFG for path following.

### 6.6 Altitude Time Constant Adaptation
`updateTECSAltitudeTimeConstant()` — reduces the TECS altitude tracking time constant near the ground (`FW_T_THR_LOW_HGT`) to improve responsiveness during low-altitude operations. Transitions via a slew rate limiter to avoid step changes.

---

## 7. Glider-Specific Behavior

`fw_pos_control` has direct awareness of the `altitude_release` module via the `release_status` subscription. When the release is triggered:
- `FW_THR_MAX` is set from 0 to `REL_THR` by the `altitude_release` module
- TECS immediately begins using the newly available throttle
- `fw_pos_control` also publishes `esc_stow_command` to un-stow the propeller if commanded

This means the transition from gliding ascent (motor locked) to powered flight is driven by a parameter change that TECS picks up automatically on the next cycle — no mode switch required.

---

## 8. Data Flow Within the Module

Each execution of `Run()` at 50 Hz follows this sequence:

```
1. Poll all subscriptions (airspeed, attitude, control mode, command, status)
2. Determine current flight phase / control mode
3. Get position setpoint from position_setpoint_triplet
4. Lateral:
     navigateWaypoints() / navigateLoiter() / etc.
     → NPFG.guideToPath()
     → getCorrectedNpfgRollSetpoint() → roll_body
5. Longitudinal:
     _tecs.update(dt, altitude, airspeed, alt_sp, airspeed_sp, ...)
     → pitch_body (from TECS SEB control)
     → thrust_sp  (from TECS STE control)
6. Apply limits (FW_R_LIM, FW_P_LIM_MIN/MAX, FW_THR_MIN/MAX)
7. Publish vehicle_attitude_setpoint
8. Publish tecs_status, npfg_status, position_controller_status
```

---

## 9. Stratospheric Considerations

| Issue | Impact | Relevant Parameters |
|-------|--------|-------------------|
| High TAS at altitude | NPFG period must be increased to avoid over-aggressive turns | `NPFG_PERIOD` |
| Low dynamic pressure | Reduced control surface effectiveness — slower attitude response | `NPFG_ROLL_TC` |
| Wide airspeed range (ascent vs. cruise) | TECS speed weighting and climb/sink limits span a large envelope | `FW_T_SPDWEIGHT`, `FW_T_SINK_MAX` |
| Motor locked during ascent | `FW_THR_MAX = 0` prevents TECS from commanding throttle | Set by `altitude_release` module |
| Post-release transient | TECS integrators may have stale state; fast-descend logic handles integrator washout | `FW_T_F_ALT_ERR` |
