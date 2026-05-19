# Software Design Description — Control Allocator

**Module:** `control_allocator`
**Source:**
- `src/modules/control_allocator/ControlAllocator.cpp`
- `src/modules/control_allocator/ControlAllocator.hpp`
- `src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessFixedWing.cpp`
- `src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessFixedWing.hpp`
- `src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessControlSurfaces.cpp`
- `src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessControlSurfaces.hpp`
- `src/lib/control_allocation/control_allocation/ControlAllocation.hpp`
- `src/lib/control_allocation/actuator_effectiveness/ActuatorEffectiveness.hpp`
**Work queue:** `rate_ctrl`
**Triggers:** `vehicle_torque_setpoint` (callback), `vehicle_thrust_setpoint` (callback)

---

## 1. Purpose

`control_allocator` is the final stage in the GNC chain. It receives abstract body-frame torque and thrust demands from `fw_rate_control` and maps them to concrete actuator positions: servo deflections (ailerons, elevator, rudder) and motor throttle commands.

The module decouples the rate controller from actuator geometry. `fw_rate_control` reasons purely in body-frame moments (roll, pitch, yaw) and body-frame thrust; it has no knowledge of how many surfaces exist, how they are arranged, or their individual limits. `control_allocator` owns all of that geometry through the effectiveness matrix and applies actuator constraints before commands ever reach the mixer or PWM outputs.

For this vehicle (`CA_AIRFRAME = FIXED_WING`) there is no motor during gliding phases, so the allocation problem is purely a control-surface decomposition problem.

---

## 2. System Architecture

```
fw_rate_control
  vehicle_torque_setpoint  (roll, pitch, yaw moments)
  vehicle_thrust_setpoint  (x, y, z thrust)
           │
           ▼
┌─────────────────────────────────────────────────────┐
│               control_allocator                      │
│                                                     │
│  ActuatorEffectivenessFixedWing                     │
│    ├── ActuatorEffectivenessRotors  (motors)        │
│    └── ActuatorEffectivenessControlSurfaces         │
│            ├── build effectiveness matrix B         │
│            └── per-surface trim / flap / spoiler    │
│                                                     │
│  ControlAllocation (PSEUDO_INVERSE or              │
│                     SEQUENTIAL_DESATURATION)        │
│    └── solve u = B⁺ · τ                            │
│                                                     │
│  allocateAuxilaryControls()                         │
│    ├── flaps_setpoint  → applyFlaps()               │
│    └── spoilers_setpoint → applySpoilers()          │
│                                                     │
│  applySlewRateLimit()  (if CA_R*_SLEW / CA_SV*_SLEW │
│                          non-zero)                  │
│  clipActuatorSetpoint()                             │
└─────────────────────────────────────────────────────┘
           │
           ├── actuator_motors    (throttle per motor)
           ├── actuator_servos    (deflection per surface)
           ├── actuator_servos_trim
           └── control_allocator_status
                 (unallocated_torque / unallocated_thrust
                  → anti-windup feedback to fw_rate_control)
```

---

## 3. Inputs and Outputs

### 3.1 Primary inputs (uORB subscriptions)

| Topic | Fields consumed | Notes |
|---|---|---|
| `vehicle_torque_setpoint` | `xyz[3]` | Body-frame roll/pitch/yaw moment demands. Callback-driven; triggers allocation. |
| `vehicle_thrust_setpoint` | `xyz[3]` | Body-frame thrust demands. Triggers allocation only if torque setpoint has not been received within the last 5 ms. |
| `flaps_setpoint` | `normalized_setpoint` | Unsigned normalized [0, 1] flap demand. Consumed inside `allocateAuxilaryControls()`. |
| `spoilers_setpoint` | `normalized_setpoint` | Unsigned normalized [0, 1] spoiler demand. Consumed inside `allocateAuxilaryControls()`. |
| `vehicle_status` | `arming_state`, `vehicle_type`, `is_vtol`, `in_transition_mode` | Arm state and flight phase; determines which motors are allowed to run. |
| `vehicle_control_mode` | `flag_control_allocation_enabled` | Gates actuator publishing. |
| `failure_detector_status` | `fd_motor`, `motor_failure_mask` | Optional motor failure handling (zeroes failed motor column in B). |
| `parameter_update` | — | Triggers full parameter reload and matrix rebuild. |

### 3.2 Outputs (uORB publications)

| Topic | Content |
|---|---|
| `actuator_motors` | Per-motor throttle commands, normalized [-1, 1] (or [0, 1] for non-reversible motors). NaN for stopped/failed motors. |
| `actuator_servos` | Per-servo deflection commands, normalized [-1, 1]. |
| `actuator_servos_trim` | Trim offsets for each servo, derived from `CA_SV_CS*_TRIM` at geometry load time. |
| `control_allocator_status` | Unallocated torque (x, y, z) and thrust (x, y, z), saturation flags per actuator, motor failure mask. Published at a maximum rate of 200 Hz (every 5 ms). |

---

## 4. Effectiveness Matrix

### 4.1 Concept

The effectiveness matrix **B** (shape `NUM_AXES × NUM_ACTUATORS`, i.e. 6 × 16) encodes how each actuator contributes to each body-frame axis. The six rows correspond to:

```
row 0  →  roll   moment
row 1  →  pitch  moment
row 2  →  yaw    moment
row 3  →  thrust X
row 4  →  thrust Y
row 5  →  thrust Z
```

Each column corresponds to one actuator (motors first, then servos, in the order they were registered). Column entries are the partial derivatives of body-frame forces/moments with respect to that actuator's normalized command.

The control allocation problem is:

```
τ = B · u
```

where τ ∈ ℝ⁶ is the desired torque+thrust vector and u ∈ ℝⁿ is the actuator command vector. Because the system is generally over- or under-actuated, the inversion is approximate.

### 4.2 Weak-axis suppression

After the geometry is loaded, `update_effectiveness_matrix_if_needed()` scans every row of B. If all entries in a row have magnitude ≤ 0.05 the entire row is zeroed. This prevents the allocator from issuing actuator commands in an attempt to control an axis with only marginal authority, which would corrupt commands on axes that can actually be controlled.

### 4.3 Control surface entries

Each control surface is registered via `ActuatorEffectivenessControlSurfaces::addActuators()`, which calls `configuration.addActuator(ActuatorType::SERVOS, torque, Vector3f{})`. The `torque` vector — `(CA_SV_CS*_TRQ_R, CA_SV_CS*_TRQ_P, CA_SV_CS*_TRQ_Y)` — is placed directly into the roll, pitch, and yaw rows of the corresponding column in B. The thrust rows (3–5) are zero for all control surfaces.

Certain surface types force their torque vector to zero regardless of parameter values:

| Type | Forced torque |
|---|---|
| `LeftFlap` (9), `RightFlap` (10) | Zero |
| `Airbrake` (11) | Zero |
| `SteeringWheel` (16) | Zero |
| `LeftSpoiler` (17), `RightSpoiler` (18) | Zero |

Surfaces with a zero torque vector in all rows are zeroed out by the weak-axis suppression step and do not participate in the primary allocation. Their deflections are driven entirely by `applyFlaps()` / `applySpoilers()` after the primary solve (see Section 6).

### 4.4 Motor entries

Motors are added by `ActuatorEffectivenessRotors` with axis configuration `FixedForward`. Propeller reaction torque is disabled for fixed-wing (`_rotors.enablePropellerTorque(false)`). The motor columns therefore contribute only thrust (rows 3–5); rows 0–2 are zero.

---

## 5. Allocation Methods

The allocation method is selected by `CA_METHOD`. The module instantiates one of two concrete `ControlAllocation` subclasses.

### 5.1 PSEUDO_INVERSE (CA_METHOD = 0)

Computes the Moore-Penrose pseudoinverse B⁺ = Bᵀ(BBᵀ)⁻¹ of the effectiveness matrix and solves:

```
u = B⁺ · τ
```

The pseudoinverse is computed once when the effectiveness matrix changes and cached. Each allocation step is a single matrix-vector multiply, making this the lowest-latency option. It distributes the demand optimally in the least-squares sense but does not account for actuator limits; any saturation is handled by the subsequent `clipActuatorSetpoint()` call, with the residual reported in `control_allocator_status`.

### 5.2 SEQUENTIAL_DESATURATION (CA_METHOD = 1)

An iterative method that explicitly respects per-actuator limits. The algorithm applies the pseudoinverse solution, then iteratively detects saturated actuators, fixes them at their limits, projects the unsatisfied portion of the demand back through the reduced system, and repeats. This allows unsatisfied demand in low-priority axes to be redistributed toward higher-priority axes rather than uniformly clipping, at the cost of additional computation per cycle.

### 5.3 AUTO (CA_METHOD = 2)

Defers the choice to the effectiveness source. `ActuatorEffectivenessFixedWing` does not override `getDesiredAllocationMethod()`, so the default implementation applies: `PSEUDO_INVERSE` for all matrices. In practice `CA_METHOD = AUTO` on this vehicle behaves identically to `CA_METHOD = PSEUDO_INVERSE`.

---

## 6. Fixed-Wing Airframe (`ActuatorEffectivenessFixedWing`)

When `CA_AIRFRAME = FIXED_WING` (value 1), `update_effectiveness_source()` instantiates `ActuatorEffectivenessFixedWing`. This class composes two sub-components:

```cpp
ActuatorEffectivenessRotors _rotors{this, AxisConfiguration::FixedForward};
ActuatorEffectivenessControlSurfaces _control_surfaces{this};
```

### 6.1 Geometry construction (`getEffectivenessMatrix`)

Called whenever the configuration changes (parameter update or motor activation event, not on every cycle). The sequence is:

1. `_rotors.enablePropellerTorque(false)` — disables yaw from motor drag so that yaw authority is purely from rudder.
2. `_rotors.addActuators(configuration)` — writes motor columns into B, records `_forwards_motors_mask`.
3. `_first_control_surface_idx = configuration.num_actuators_matrix[0]` — captures the insertion index so that flap/spoiler auxiliary allocation can find the servo sub-vector later.
4. `_control_surfaces.addActuators(configuration)` — writes servo columns into B using per-surface `CA_SV_CS*_TRQ_*` parameters.

The function only rebuilds the matrix when `external_update != NO_EXTERNAL_UPDATE`. On every Run() cycle, `update_effectiveness_matrix_if_needed()` is called with `NO_EXTERNAL_UPDATE`, which the function ignores; rebuilds are rate-limited to at most once per 100 ms even for external triggers.

### 6.2 Primary allocation

`ControlAllocation::allocate()` is called on the assembled B matrix and the combined [torque; thrust] demand vector to produce the primary actuator setpoint vector u.

### 6.3 Auxiliary controls: flaps and spoilers (`allocateAuxilaryControls`)

After the primary allocation, `ActuatorEffectivenessFixedWing::allocateAuxilaryControls()` applies flap and spoiler demands as additive offsets on top of the primary servo commands:

**Flaps** (`applyFlaps`):
- Reads `flaps_setpoint.normalized_setpoint` ∈ [0, 1].
- Passes it through a slew rate limiter at `kFlapSlewRate = 0.5 [1/s]`.
- Maps [0, 1] → [-1, 1]: `slewed_value * 2 - 1`.
- Adds `(mapped_value) * CA_SV_CS*_FLAP` to each surface's actuator setpoint.

**Spoilers** (`applySpoilers`):
- Reads `spoilers_setpoint.normalized_setpoint` ∈ [0, 1].
- Passes it through a slew rate limiter at `kSpoilersSlewRate = 0.5 [1/s]`.
- Adds `slewed_value * CA_SV_CS*_SPOIL` to each surface's actuator setpoint.

Both are applied to the full servo sub-vector starting at `_first_control_surface_idx`, so they affect every configured surface according to each surface's individual scale factors. Surfaces not intended to act as flaps or spoilers should have their respective scale parameters set to 0.

### 6.4 Forward-motor stop logic (`updateSetpoint`)

After auxiliary controls are applied, `updateSetpoint()` calls `stopMaskedMotorsWithZeroThrust(_forwards_motors_mask, actuator_sp)`. Any motor in the forwards-motor mask whose allocated thrust command is zero is set to NaN (stopped), preventing windmilling commands at idle.

---

## 7. Control Surface Configuration

Up to 8 control surfaces are supported (`CA_SV_CS_COUNT` selects how many are active, max `MAX_COUNT = 8`).

Each surface is configured via a set of per-index parameters. The index runs 0–7.

### 7.1 Surface type

`CA_SV_CS*_TYPE` selects the surface type from the `Type` enum:

| Value | Type |
|---|---|
| 1 | LeftAileron |
| 2 | RightAileron |
| 3 | Elevator |
| 4 | Rudder |
| 5 | LeftElevon |
| 6 | RightElevon |
| 7 | LeftVTail |
| 8 | RightVTail |
| 9 | LeftFlap |
| 10 | RightFlap |
| 11 | Airbrake |
| 12 | Custom |
| 13 | LeftATail |
| 14 | RightATail |
| 15 | SingleChannelAileron |
| 16 | SteeringWheel |
| 17 | LeftSpoiler |
| 18 | RightSpoiler |

### 7.2 Torque effectiveness

`CA_SV_CS*_TRQ_R`, `CA_SV_CS*_TRQ_P`, `CA_SV_CS*_TRQ_Y` directly populate the roll, pitch, and yaw rows of column i in B. These values are set to zero automatically for flap, airbrake, steering, and spoiler types.

### 7.3 Trim

`CA_SV_CS*_TRIM` is a normalized [-1, 1] trim offset. It is written into `configuration.trim` and published via `actuator_servos_trim` at geometry load time. The trim is subtracted before computing allocated control in `getAllocatedControl()`, so trim deflections do not appear in the unallocated torque residual.

### 7.4 Flap and spoiler scales

`CA_SV_CS*_FLAP` and `CA_SV_CS*_SPOIL` control how much each individual surface responds to a full-scale flap or spoiler command. Setting both to 0 for a given surface excludes it from auxiliary allocation entirely.

---

## 8. Saturation Feedback and Anti-Windup

After each allocation cycle, `publish_control_allocator_status()` is called (rate-limited to 200 Hz, i.e. every 5 ms). The status message carries:

```
unallocated_torque[3]   = control_sp[0:2] - allocated_control[0:2]
unallocated_thrust[3]   = control_sp[3:5] - allocated_control[3:5]
torque_setpoint_achieved  (true if ‖unallocated_torque‖² < 1e-6)
thrust_setpoint_achieved  (true if ‖unallocated_thrust‖² < 1e-6)
actuator_saturation[16]   (UPPER / LOWER / none per actuator)
```

`allocated_control` is computed as `B · (u − trim) ⊙ scale`, where `scale` is the per-axis normalization vector maintained inside `ControlAllocation`. The difference from the requested setpoint is the torque/thrust that could not be physically delivered.

`fw_rate_control` subscribes to `control_allocator_status` and uses `unallocated_torque` for integrator anti-windup. When a surface saturates and a roll, pitch, or yaw moment cannot be fully allocated, the integrator is prevented from winding further in the direction of the unachievable demand.

---

## 9. Slew Rate Limiting

Per-actuator slew rate limits protect against abrupt commands that could cause structural loads or servo damage. Slew rates are applied inside `applySlewRateLimit(dt)` by clamping the per-cycle change of each element of the actuator setpoint vector to `slew_rate * dt`.

Slew rates are loaded from parameters into two arrays during `parameters_updated()`:

- `_params.slew_rate_motors[i]` — from `CA_R{i}_SLEW`
- `_params.slew_rate_servos[i]` — from `CA_SV{i}_SLEW`

If any slew rate is non-zero, `_has_slew_rate` is set to true and `applySlewRateLimit()` is called every cycle. If all slew rates are zero, the call is skipped entirely for efficiency.

Slew rates are in normalized units per second. A value of 1.0 means the actuator can travel from −1 to +1 in one second. A value of 0.0 disables limiting for that actuator.

Note that `CA_SV*_SLEW` (indexed by servo slot 0–7) is distinct from the internal `kFlapSlewRate` and `kSpoilersSlewRate` constants used by `ActuatorEffectivenessControlSurfaces`. The latter limit how fast the flap and spoiler normalized setpoints themselves ramp; the former limits how fast the final servo command can move after all contributions have been summed.

---

## 10. Airframe Selection

`CA_AIRFRAME` selects which `ActuatorEffectiveness` subclass is instantiated. The glider uses `FIXED_WING = 1`.

| CA_AIRFRAME value | Class instantiated |
|---|---|
| 0 | `ActuatorEffectivenessMultirotor` |
| 1 | `ActuatorEffectivenessFixedWing` |
| 2 | `ActuatorEffectivenessStandardVTOL` |
| 3 | `ActuatorEffectivenessTiltrotorVTOL` |
| 4 | `ActuatorEffectivenessTailsitterVTOL` |
| 5 | `ActuatorEffectivenessRoverAckermann` |
| 6 | *(Rover differential — bypasses this module)* |
| 7 | `ActuatorEffectivenessUUV` |
| 8 | `ActuatorEffectivenessMCTilt` |
| 9 | `ActuatorEffectivenessCustom` |
| 10 | `ActuatorEffectivenessHelicopter` (tail ESC) |
| 11 | `ActuatorEffectivenessHelicopter` (tail servo) |
| 12 | `ActuatorEffectivenessHelicopterCoaxial` |
| 13–14 | *(Spacecraft — bypasses this module)* |

If the instantiation of the selected class fails, the parameter is reverted to the previously active value and the prior effectiveness source is retained.

---

## 11. Motor Failure Handling

Controlled by `CA_FAILURE_MODE`:

- `IGNORE (0)` — no action.
- `REMOVE_FIRST_FAILING_MOTOR (1)` — when a single motor failure is reported by `failure_detector_status`, the corresponding column of B is zeroed, effectively removing that motor from the allocation. Only the first failure is handled; subsequent failures are ignored. When the failure clears, the original column is restored.

The bitmask of currently handled failures is tracked in `_handled_motor_failure_bitmask` and reflected in `control_allocator_status.handled_motor_failure_mask`.

Parameter updates are suppressed while a motor failure is active to avoid inadvertently re-including the failed motor via a geometry reload.

---

## 12. Key Parameters

| Parameter | Type | Description |
|---|---|---|
| `CA_AIRFRAME` | int | Airframe/effectiveness source selection. Use `1` (FIXED_WING) for this vehicle. |
| `CA_METHOD` | int | Allocation algorithm. `0` = PSEUDO_INVERSE, `1` = SEQUENTIAL_DESATURATION, `2` = AUTO. |
| `CA_FAILURE_MODE` | int | Motor failure response. `0` = ignore, `1` = remove first failing motor from allocation. |
| `CA_R_REV` | bitmask | Per-motor reversibility. Bit i set → motor i minimum command is −1 instead of 0. |
| `CA_R0_SLEW` … `CA_R7_SLEW` | float | Slew rate limit for motors 0–7, in normalized units per second. 0 disables limiting. |
| `CA_SV0_SLEW` … `CA_SV7_SLEW` | float | Slew rate limit for servos 0–7, in normalized units per second. 0 disables limiting. |
| `CA_SV_CS_COUNT` | int | Number of active control surfaces (0–8). |
| `CA_SV_CS*_TYPE` | int | Surface type for surface index * (see Section 7.1). |
| `CA_SV_CS*_TRQ_R` | float | Roll torque effectiveness coefficient for surface *. |
| `CA_SV_CS*_TRQ_P` | float | Pitch torque effectiveness coefficient for surface *. |
| `CA_SV_CS*_TRQ_Y` | float | Yaw torque effectiveness coefficient for surface *. |
| `CA_SV_CS*_TRIM` | float | Trim offset for surface *, normalized [-1, 1]. |
| `CA_SV_CS*_FLAP` | float | Flap scale for surface *. Fraction of flap demand added to this surface. |
| `CA_SV_CS*_SPOIL` | float | Spoiler scale for surface *. Fraction of spoiler demand added to this surface. |
