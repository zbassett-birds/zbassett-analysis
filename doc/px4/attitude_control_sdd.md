# Software Design Description — Attitude Control Loops

**Modules:** `fw_att_control`, `fw_rate_control`
**Source:**
- `src/modules/fw_att_control/FixedwingAttitudeControl.cpp`
- `src/modules/fw_att_control/fw_pitch_controller.cpp`
- `src/modules/fw_att_control/fw_roll_controller.cpp`
- `src/modules/fw_att_control/fw_yaw_controller.cpp`
- `src/modules/fw_rate_control/FixedwingRateControl.cpp`
**Update Rates:** Both modules run at ~250 Hz

---

## 1. Purpose

The attitude control system is a two-layer cascaded loop that converts an attitude setpoint (roll, pitch, yaw angles) into actuator torque commands.

- **`fw_att_control`** — the outer loop. Converts attitude error into angular rate setpoints.
- **`fw_rate_control`** — the inner loop. Converts rate error into torque commands sent to the control allocator.

The cascade is intentional: the outer loop (attitude) runs slowly relative to aerodynamic dynamics, while the inner loop (rate) runs fast and tight against the IMU. Separating them allows each loop to be tuned independently for its own bandwidth requirements.

---

## 2. System Architecture

```
vehicle_attitude_setpoint
  (roll_sp, pitch_sp, yaw_sp)
         │
         ▼
┌──────────────────────────────────────────┐
│           fw_att_control   ~250 Hz        │
│                                          │
│  roll error  → RollController  → p_sp    │
│  pitch error → PitchController → q_sp   │
│  yaw (coord) → YawController   → r_sp   │
│                                          │
│  Euler → body frame (Jacobian transform) │
└──────────────────────────────────────────┘
         │
         ▼  vehicle_rates_setpoint (p, q, r)
         │
         ▼
┌──────────────────────────────────────────┐
│          fw_rate_control    ~250 Hz       │
│                                          │
│  rate error → PID (roll, pitch, yaw)     │
│  + feedforward                           │
│  + airspeed scaling                      │
│  + trim scheduling                       │
│  + anti-windup                           │
└──────────────────────────────────────────┘
         │
         ▼  vehicle_torque_setpoint + vehicle_thrust_setpoint
         │
         ▼
    control_allocator → servos
```

---

## 3. Module: `fw_att_control` — Attitude Loop

**Trigger topic:** `vehicle_attitude` (EKF2 output)
**Source:** `FixedwingAttitudeControl.cpp`

### 3.1 Inputs and Outputs

| Topic | Direction | Content |
|-------|-----------|---------|
| `vehicle_attitude` | In (trigger) | Current quaternion attitude from EKF2 |
| `vehicle_attitude_setpoint` | In | Desired roll, pitch, yaw (quaternion) from fw_pos_control |
| `airspeed_validated` | In | Calibrated airspeed for yaw controller |
| `vehicle_control_mode` | In | Armed, auto/manual mode flags |
| `manual_control_setpoint` | In | Stick inputs (stabilized manual mode) |
| `vehicle_rates_setpoint` | Out | Body angular rate commands (p, q, r) |

### 3.2 Execution Flow (`Run()`)

Each cycle:
1. Copy `vehicle_attitude`, convert quaternion to Euler angles (φ, θ, ψ)
2. Poll `vehicle_attitude_setpoint` for desired (φ_sp, θ_sp, ψ_sp)
3. Run the three attitude controllers to produce Euler rate setpoints
4. Transform Euler rate setpoints to body-frame angular rates via Jacobian
5. Clamp to rate limits
6. Publish `vehicle_rates_setpoint`

### 3.3 Roll Controller (`fw_roll_controller.cpp`)

A pure proportional controller on roll error with a body-frame Jacobian transform.

```
roll_error = roll_sp - roll

euler_rate_sp = roll_error / tc_roll

body_rate_p = euler_rate_sp - sin(pitch) * yaw_euler_rate_sp
```

The `sin(pitch)` cross-coupling term accounts for the fact that Euler roll rate and body roll rate (p) diverge as pitch increases — at 90° pitch they are completely decoupled.

**Key parameter:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `FW_R_TC` | 0.4 s | Roll time constant — lower = faster response |
| `FW_R_RMAX` | 70°/s | Maximum roll rate setpoint |

### 3.4 Pitch Controller (`fw_pitch_controller.cpp`)

A proportional + integral (PI) controller on pitch error with Jacobian transform.

```
pitch_error = pitch_sp - pitch

integrator += pitch_error * ki * dt
integrator = clamp(integrator, -limit, +limit)

euler_rate_sp = pitch_error / tc_pitch + integrator

body_rate_q = cos(roll) * euler_rate_sp + cos(pitch) * sin(roll) * yaw_euler_rate_sp
```

The Jacobian accounts for roll-pitch coupling. The integrator handles trim offsets (e.g. CG shift, aerodynamic asymmetry). A special case bypasses the Jacobian in steep dives (pitch < -45°) where the standard transform can produce poor results.

**Key parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `FW_P_TC` | 0.4 s | Pitch time constant |
| `FW_P_I` | 0.0 | Pitch integrator gain |
| `FW_P_IMAX` | 0.0 rad/s | Pitch integrator limit |
| `FW_P_RMAX_POS` | 60°/s | Max pitch-up rate setpoint |
| `FW_P_RMAX_NEG` | 60°/s | Max pitch-down rate setpoint |

### 3.5 Yaw Controller (`fw_yaw_controller.cpp`)

The yaw controller does **not** track a yaw angle setpoint. Instead it implements a **coordinated turn** constraint — it commands whatever yaw rate is required to keep sideslip near zero during banked turns.

```
# Coordinated turn equation (no sideslip assumption):
yaw_euler_rate_sp = tan(roll) * cos(pitch) * g / airspeed

# Transform to body frame:
body_rate_r = -sin(roll) * pitch_euler_rate_sp + cos(roll) * cos(pitch) * yaw_euler_rate_sp
```

This is purely kinematic — it computes the yaw rate that a coordinated aircraft must have given its roll angle, pitch angle, and airspeed. There is no error feedback to a yaw angle; the rudder is used solely for coordination. Inverted flight handling constrains the roll angle sign appropriately.

**Key parameter:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `FW_Y_RMAX` | 50°/s | Maximum yaw rate setpoint |

**Note on airspeed:** The yaw controller uses calibrated airspeed (IAS). A comment in the code (`FixedwingAttitudeControl.cpp:365`) notes the team considered using TAS for better coordination at altitude — this is a known open item and potentially relevant for stratospheric flight where IAS/TAS ratios are large.

### 3.6 Integrator Reset

Pitch and wheel integrators are reset when:
- `att_sp.reset_integral` is set (commanded by fw_pos_control on mode transitions)
- Vehicle is landed
- Not in fixed-wing mode

This prevents integrator windup from carrying over between flight phases.

---

## 4. Module: `fw_rate_control` — Rate Loop

**Trigger topic:** `vehicle_angular_velocity` (IMU-rate output from EKF2)
**Source:** `FixedwingRateControl.cpp`

### 4.1 Inputs and Outputs

| Topic | Direction | Content |
|-------|-----------|---------|
| `vehicle_angular_velocity` | In (trigger) | Body-frame angular rates p, q, r and derivatives |
| `vehicle_rates_setpoint` | In | Desired body rates from fw_att_control |
| `airspeed_validated` | In | Calibrated and true airspeed for scaling |
| `control_allocator_status` | In | Saturation feedback for anti-windup |
| `vehicle_torque_setpoint` | Out | Normalized torque commands [-1, 1] for each axis |
| `vehicle_thrust_setpoint` | Out | Normalized throttle command [0, 1] |

### 4.2 PID Rate Controller (`lib/rate_control/`)

A full PID controller runs on each axis simultaneously:

```
rate_error = rate_sp - rate_measured

torque = Kp * rate_error
       + Ki * integral(rate_error)
       + Kd * d/dt(rate_error)          # uses angular_acceleration from IMU
       + Kff * rate_sp                  # feedforward
```

The derivative term uses the angular acceleration reported directly by the IMU (not a numerical derivative of rate), which avoids amplifying quantization noise.

**Rate control PID parameters:**

| Parameter | Default | Axis | Description |
|-----------|---------|------|-------------|
| `FW_RR_P` | 0.05 | Roll | Roll rate proportional gain |
| `FW_RR_I` | 0.1 | Roll | Roll rate integral gain |
| `FW_RR_D` | 0.0 | Roll | Roll rate derivative gain |
| `FW_RR_IMAX` | 0.2 | Roll | Roll rate integrator limit |
| `FW_RR_FF` | 0.5 | Roll | Roll rate feedforward gain |
| `FW_PR_P` | 0.08 | Pitch | Pitch rate proportional gain |
| `FW_PR_I` | 0.1 | Pitch | Pitch rate integral gain |
| `FW_PR_D` | 0.0 | Pitch | Pitch rate derivative gain |
| `FW_PR_IMAX` | 0.4 | Pitch | Pitch rate integrator limit |
| `FW_PR_FF` | 0.5 | Pitch | Pitch rate feedforward gain |
| `FW_YR_P` | 0.05 | Yaw | Yaw rate proportional gain |
| `FW_YR_I` | 0.1 | Yaw | Yaw rate integral gain |
| `FW_YR_D` | 0.0 | Yaw | Yaw rate derivative gain |
| `FW_YR_IMAX` | 0.2 | Yaw | Yaw rate integrator limit |
| `FW_YR_FF` | 0.3 | Yaw | Yaw rate feedforward gain |

### 4.3 Airspeed Scaling

Control surface effectiveness scales with dynamic pressure (∝ V²). To maintain consistent closed-loop response across the airspeed envelope, the rate controller applies two scaling factors:

**IAS scaling** (`_airspeed_scaling`):
```
airspeed_scaling = trim_airspeed / IAS        (clamped to min 0.7)
```
Applied to the PID output: `control_u = angular_acceleration_sp * scaling²`

The squared scaling reflects the V² dependence of aerodynamic moment. At low airspeed, gains are increased to compensate for reduced surface effectiveness.

**TAS scaling** (`_TAS_scaling`):
```
TAS_scaling = IAS / TAS        (clamped 0.1–1.0)
```
Applied to feedforward gains only. This accounts for the density altitude effect — at high altitude, the same IAS corresponds to much higher TAS, but the aerodynamic moments depend on dynamic pressure (IAS-driven), not TAS directly. The feedforward is therefore reduced at altitude to prevent over-command.

This dual scaling is a custom modification to the stock PX4 rate controller — the comments in the code make this explicit. It is directly relevant to stratospheric operation.

**Enabled by:** `FW_ARSP_SCALE_EN` (default: 1)

### 4.4 Trim Scheduling

Aerodynamic trim changes with airspeed (e.g. elevator trim varies with dynamic pressure). The rate controller applies a bilinear interpolation of trim offsets across the airspeed range:

- Below trim airspeed: interpolate between `FW_AIRSPD_MIN` and `FW_AIRSPD_TRIM` using `FW_DTRIM_*_VMIN` offsets
- Above trim airspeed: interpolate between `FW_AIRSPD_TRIM` and `FW_AIRSPD_MAX` using `FW_DTRIM_*_VMAX` offsets

| Parameter | Default | Description |
|-----------|---------|-------------|
| `FW_DTRIM_R_VMIN` | 0.0 | Roll trim delta at min airspeed |
| `FW_DTRIM_R_VMAX` | 0.0 | Roll trim delta at max airspeed |
| `FW_DTRIM_P_VMIN` | 0.0 | Pitch trim delta at min airspeed |
| `FW_DTRIM_P_VMAX` | 0.0 | Pitch trim delta at max airspeed |
| `FW_DTRIM_Y_VMIN` | 0.0 | Yaw trim delta at min airspeed |
| `FW_DTRIM_Y_VMAX` | 0.0 | Yaw trim delta at max airspeed |

### 4.5 Anti-Windup

The rate controller receives saturation feedback from `control_allocator_status`. When the control allocator reports that a torque demand is unallocatable (actuator saturated), the corresponding integrator is prevented from winding further in the saturating direction. This is per-axis and per-direction.

### 4.6 Roll-to-Yaw Feedforward

An optional cross-axis feedforward from roll torque output to yaw command:

```
yaw_cmd += FW_RLL_TO_YAW_FF * roll_cmd
```

Used to proactively counteract adverse yaw during roll inputs. Default is 0 (disabled).

### 4.7 Battery Compensation

If `FW_BAT_SCALE_EN` is set, throttle output is scaled by the battery voltage compensation factor from `battery_status`. This maintains roughly consistent thrust as battery voltage drops.

---

## 5. Euler-to-Body Frame Jacobian

Both the roll and pitch controllers convert their Euler rate setpoints to body-frame rates. This is necessary because the attitude controller operates in Euler angles (which have intuitive physical meaning) but the rate controller and IMU operate in body-frame angular rates (p, q, r).

The general kinematic relationship is:

```
[p]   [1    0        -sin(θ)    ] [φ̇]
[q] = [0    cos(φ)   cos(θ)sin(φ)] [θ̇]
[r]   [0   -sin(φ)   cos(θ)cos(φ)] [ψ̇]
```

The controllers use simplified forms of this that cross-couple only the dominant terms:

- **Roll:** `p_sp = φ̇_sp - sin(θ) * ψ̇_sp`
- **Pitch:** `q_sp = cos(φ) * θ̇_sp + cos(θ)sin(φ) * ψ̇_sp`
- **Yaw:** `r_sp = -sin(φ) * θ̇_sp + cos(φ)cos(θ) * ψ̇_sp`

This matters most at high roll or pitch angles where Euler rates and body rates diverge significantly.

---

## 6. Stratospheric Considerations

| Issue | Impact | Relevant Parameters |
|-------|--------|---------------------|
| Low dynamic pressure | Reduced control surface effectiveness at altitude — PID output for same rate error produces less moment | IAS scaling compensates (`FW_ARSP_SCALE_EN`) |
| Large IAS/TAS ratio | Feedforward becomes too aggressive if based on TAS — TAS scaling reduces FF gains at altitude | `_TAS_scaling` applied to `FW_*R_FF` |
| Yaw coordination at altitude | Coordinated turn equation uses IAS; at high altitude and high TAS the rudder authority is proportionally less — coordination may degrade | Consider switching yaw controller to TAS (noted as open item in code) |
| Thin air aerodynamics | Stall margins are tighter; rate limits may need revisiting | `FW_R_RMAX`, `FW_P_RMAX_POS/NEG`, `FW_Y_RMAX` |
| Post-release transient | Motor start causes thrust asymmetry and pitch disturbance — rate integrators must not be saturated entering this phase | Integrators reset on mode change via `reset_integral` flag |
