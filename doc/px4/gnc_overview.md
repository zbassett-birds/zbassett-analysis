# GNC Overview — Glider Flight Software

This document provides a high-level overview of the estimation, guidance, control, and custom code in the glider flight software stack (`/home/zack/projects/glider`). The stack is based on PX4 Autopilot with custom Icarus extensions.

---

## Architecture Summary

The GNC stack follows a standard cascaded architecture running on NuttX RTOS with uORB pub/sub middleware:

```
Navigator (mission sequencing)
    ↓ position setpoints
fw_pos_control (L1 guidance + TECS)
    ↓ attitude setpoints
fw_att_control (PID attitude loops)
    ↓ rate setpoints
fw_rate_control (PID rate loops)
    ↓ actuator commands
control_allocator (actuator mixing)
    ↓
Servos / ESC
```

---

## Data Flow — GNC Pipeline

Modules do not call each other directly. Each runs as an independent task and communicates exclusively through uORB pub/sub messages stored in shared memory. A module wakes up when its trigger topic is published, reads its inputs, does its math, and publishes its output.

```
IMU / GPS Drivers
    │ publishes: vehicle_imu, vehicle_gps_position
    ▼
EKF2                              ~250 Hz | triggered by: vehicle_imu
    │ publishes: vehicle_attitude
    │            vehicle_local_position
    │            vehicle_global_position
    │            vehicle_angular_velocity
    │            wind
    ▼
Navigator                         ~10 Hz  | triggered by: vehicle_global_position
    │ publishes: position_setpoint_triplet
    ▼
fw_pos_control                    ~50 Hz  | triggered by: vehicle_local_position
    │ reads:     vehicle_attitude, airspeed_validated, position_setpoint_triplet
    │ runs:      L1 / NPFG lateral guidance → roll setpoint
    │            TECS altitude+airspeed → pitch setpoint + throttle
    │ publishes: vehicle_attitude_setpoint
    ▼
fw_att_control                    ~250 Hz | triggered by: vehicle_attitude
    │ reads:     vehicle_attitude_setpoint, vehicle_angular_velocity
    │ runs:      PID roll / pitch / yaw attitude loops
    │ publishes: vehicle_rates_setpoint
    ▼
fw_rate_control                   ~250 Hz | triggered by: vehicle_angular_velocity
    │ reads:     vehicle_rates_setpoint
    │ runs:      PID roll / pitch / yaw rate loops
    │ publishes: vehicle_torque_setpoint
    │            vehicle_thrust_setpoint
    ▼
control_allocator                 ~250 Hz | triggered by: vehicle_torque_setpoint
    │ runs:      effectiveness matrix mixing
    │ publishes: actuator_motors, actuator_servos
    ▼
Servo / ESC drivers               → hardware
```

**Key points:**
- Each arrow is a uORB message write — not a function call
- Latency accumulates hop by hop through the chain
- Any layer can be driven from an external source (e.g. offboard computer publishing `vehicle_attitude_setpoint` directly to bypass guidance)
- `fw_pos_control` also publishes `esc_stow_command` directly — the prop stow logic is wired into the position controller, not just the CLI tool

---

## Estimation

**Primary: EKF2** — `src/modules/ekf2/`

A 13-state Extended Kalman Filter. The primary state estimator for all flight.

| States | Description |
|--------|-------------|
| Position | NED frame |
| Velocity | NED frame |
| Attitude | Quaternion |
| Gyro bias | Body frame |
| Accel bias | Body frame |
| Wind | Horizontal components |

**Sensor fusion (aid sources):**
- IMU — always active; propagates the state
- GPS — position, velocity, and yaw fusion with integrity checks
- Barometer — altitude aiding
- Magnetometer — yaw and bias estimation
- Airspeed — true airspeed fusion
- Sideslip constraint — fixed-wing specific, limits lateral slip
- Drag model — body-frame drag for velocity estimation
- Optical flow, range finder, external vision — optional/situational

**Notable features:**
- **EKF2Selector** — runs multiple EKF instances in parallel and selects the healthiest one. Important for stratospheric ops where GPS reliability varies.
- **EKFGSF** — yaw-only filter that can bootstrap heading without magnetometer. Useful if mag is disturbed at altitude.
- **Output predictor** — propagates state forward between update steps to minimize control latency.
- **Online bias estimation** — gyro, accel, and baro biases are estimated continuously.

**Fallback: attitude_estimator_q** — `src/modules/attitude_estimator_q/`
Lightweight complementary quaternion filter. Used if EKF2 is declared unhealthy.

**Stratospheric considerations:**
- Low air density at altitude affects drag model fusion — will need tuning
- GPS signal quality changes above ~20 km
- Thermal gradients affect baro accuracy

---

## Guidance

**L1 Lateral Guidance** — `src/lib/l1/`, executed in `fw_pos_control/`

Computes lateral acceleration commands to null cross-track error. Single tuning parameter: L1 distance (60 m in current sim config). Smaller L1 = tighter path following, more aggressive turns. At stratospheric altitudes with high TAS and low dynamic pressure, this will need re-tuning.

**NPFG (Nonlinear Path Following Guidance)** — `src/lib/npfg/`

More modern alternative to L1. Explicitly handles wide airspeed variation — relevant given the large altitude range of the mission profile. Available as an alternative to L1 in `fw_pos_control`.

**TECS (Total Energy Control System)** — `src/lib/tecs/`

Manages altitude and airspeed by controlling total specific energy (kinetic + potential) and its distribution between throttle and pitch. Preferred over independent altitude/airspeed loops for fixed-wing with limited throttle authority — well suited to glider ops.

**Navigator** — `src/modules/navigator/`

Mission execution layer. Sequences through waypoints, manages loiter orbits, RTL, and geofence. Outputs `position_setpoint_triplet` uORB messages down to `fw_pos_control`. Handles: takeoff, cruise, loiter, landing, RTL, geofence breach response.

---

## Control

Three cascaded loops, outer to inner:

### 1. Position / Guidance Layer
**`src/modules/fw_pos_control/FixedwingPositionControl.cpp`**

- Consumes position setpoints from Navigator
- Runs L1 (or NPFG) for lateral guidance → roll setpoint
- Runs TECS for altitude/airspeed → pitch setpoint + throttle
- Handles launch detection, runway takeoff, landing flare, figure-eight patterns
- Largest and most complex control file (~138 KB)

### 2. Attitude Layer
**`src/modules/fw_att_control/`**

| File | Function |
|------|----------|
| `fw_pitch_controller.cpp` | Pitch error → pitch rate setpoint (elevator path) |
| `fw_roll_controller.cpp` | Roll error → roll rate setpoint (aileron path) |
| `fw_yaw_controller.cpp` | Yaw coordination → yaw rate setpoint (rudder path) |
| `fw_wheel_controller.cpp` | Nose wheel steering (ground ops only) |

Each loop is a PID controller. Outputs angular rate setpoints.

### 3. Rate Layer
**`src/modules/fw_rate_control/FixedwingRateControl.cpp`**

- Consumes angular rate setpoints
- Converts to actuator deflection commands (elevator, aileron, rudder)
- Closest layer to hardware — most sensitive to latency and tuning

### Control Allocator
**`src/modules/control_allocator/`**

Generalized mixer. Maps moment commands to physical actuator outputs via an effectiveness matrix. Reconfigurable for different surface layouts without changing control law code.

---

## Custom Icarus Modules

Located in `icarus_modules/src/modules/`. All four modules are mission-operations focused — they handle the deployment sequence, not flight control.

### `altitude_release`
**The most mission-critical custom module.**

Monitors GPS altitude and fires a two-actuator release mechanism when a target altitude is reached.

| Actuator | Function | Values |
|----------|----------|--------|
| 301 | Mechanical latch | 1.0 = locked, -1.0 = released |
| 302 | Hot wire cutter | Timed pulse (default 30 s), then auto-shutoff |

**Release sequence:**
1. `altitude_release` armed via `release_command` uORB topic with a target altitude
2. Module polls `vehicle_global_position` at 10 Hz
3. When altitude ≥ target: fires actuator 301 (latch open) and actuator 302 (hot wire on)
4. Simultaneously sets `FW_THR_MAX` parameter to `REL_THR` — this enables the motor post-drop (it is locked at 0 during ascent)
5. Hot wire auto-shuts off after `HOT_TIME` ms to prevent damage

**Also supports:** manual release override, rearm after release, CLI trigger (`altitude_release release`).

### `esc_stow`
One-shot CLI tool. Publishes an `esc_stow_command` uORB message to park the propeller blade. Supports per-index ESC targeting (`-i <index>`). No persistent loop — fires and exits.

```
esc_stow stow      # park propeller
esc_stow unstow    # release
esc_stow stow -i 1 # target specific ESC
```

### `power_channel`
One-shot CLI tool. Toggles a UAVCAN power bus channel on or off. Only channel 1 is currently implemented.

```
power_channel on -c 1
power_channel off -c 1
```

### `mavlink_debug_sender`
Development/debug utility. Publishes ADC raw data and incrementing test values via MAVLink debug topics at 2 Hz. Currently scaffolding-level code — useful for verifying MAVLink debug message pipeline.

---

## Key Takeaways

- **Flight-critical GNC is stock PX4.** Estimation (EKF2), guidance (L1/TECS), and control (cascaded PID) are all upstream PX4 code. Primary work is tuning, configuration, and extension.
- **Custom code is mission-ops only.** `icarus_modules` handles the deployment sequence: ascent locked, release at altitude, motor enable, prop stow.
- **`altitude_release` is the most important custom module** — it is the integration point between balloon/tow phase and free-flight phase.
- **Simulation uses idealized state.** The MATLAB/Simulink HITL sim has no EKF in the loop. Sim-to-flight gaps will appear in estimation lag, sensor noise sensitivity, and wind estimation transients.
