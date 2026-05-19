# Software Design Description: Icarus Custom PX4 Modules

**Project:** Icarus Stratospheric Glider  
**Document:** Icarus Custom PX4 Modules — Software Design Description  
**Source tree:** `icarus_modules/src/modules/`  
**Date:** 2026-05-19  

---

## Table of Contents

1. [Overview](#1-overview)
2. [Module: altitude_release](#2-module-altitude_release)
3. [Module: esc_stow](#3-module-esc_stow)
4. [Module: power_channel](#4-module-power_channel)
5. [Module: mavlink_debug_sender](#5-module-mavlink_debug_sender)
6. [Mission Sequence Integration](#6-mission-sequence-integration)

---

## 1. Overview

Four custom modules extend the PX4 firmware for the Icarus stratospheric glider mission. They are compiled into the firmware image alongside the standard PX4 modules and are invoked from the startup script or the MAVLink shell.

| Module | Type | Role |
|---|---|---|
| `altitude_release` | Persistent background task | Mission-critical: monitors GPS altitude, fires release mechanism, unlocks motor |
| `esc_stow` | One-shot CLI tool | Parks/unparks the propeller blade |
| `power_channel` | One-shot CLI tool | Toggles a UAVCAN power bus channel |
| `mavlink_debug_sender` | Persistent background task | Development/debug: streams ADC data and test counters over MAVLink debug topics |

All four modules use the PX4 uORB publish/subscribe middleware for inter-process communication.

---

## 2. Module: `altitude_release`

### 2.1 Purpose

`altitude_release` is the most mission-critical module on the vehicle. It monitors GPS altitude via `vehicle_global_position` and fires a two-actuator release mechanism when the vehicle reaches a configured altitude threshold. It also performs the motor-unlock step by writing to `FW_THR_MAX` at the moment of release.

### 2.2 Source Files

| File | Description |
|---|---|
| `altitude_release/altitude_release.h` | Class declaration, member variables, uORB handles |
| `altitude_release/altitude_release.cpp` | All logic: run loop, release, command processing, status publishing |

### 2.3 Architecture

`AltitudeRelease` inherits from `ModuleBase<AltitudeRelease>` and `ModuleParams`. It runs as a persistent PX4 task spawned by `px4_task_spawn_cmd` with default scheduler priority and a 1500-byte stack.

```
altitude_release_main()
    └─ AltitudeRelease::main()          [ModuleBase dispatch]
           └─ AltitudeRelease::run()    [polling loop, 10 Hz]
```

### 2.4 State Machine

The module implements a three-state machine with an additional rearm transition.

```
             arm_altitude_trigger=true
UNARMED ─────────────────────────────────► ARMED
  ▲                                          │
  │  reset_mechanism=true                    │ GPS alt >= target_altitude_m
  │  (or CLI reset command)                  │ (or cmd.manual_release=true)
  │                                          ▼
  └──────────────────────────────────── RELEASED
              rearm: arm_altitude_trigger=true
              while _released==true
              (resets actuators, FW_THR_MAX=0,
               then transitions directly to ARMED)
```

| State | `_altitude_armed` | `_released` | Meaning |
|---|---|---|---|
| UNARMED | false | false | Monitoring inactive; actuators in safe position |
| ARMED | true | false | Monitoring GPS altitude; will fire at threshold |
| RELEASED | false/true | true | Release has fired; hot wire timer running |

State variables are private members of `AltitudeRelease`:

```cpp
bool _altitude_armed{false};
bool _released{false};
bool _manual_release_triggered{false};
float _target_altitude_m{100.0f};   // default, overwritten by DROP_ALTITUDE
bool _hotwire_active{false};
hrt_abstime _hotwire_on_time{0};
```

### 2.5 Actuator Interface

Actuator commands are delivered via `MAV_CMD_DO_SET_ACTUATOR` (vehicle command ID 187), routed through the `vehicle_command` uORB topic and processed by the PX4 mixer/output layer.

The static helper `send_do_set_actuator(uint32_t func_code, float value)` constructs the command. It maintains a static advertiser handle so only a single advertisement is performed across all calls.

| Actuator | Function Code | Value 1.0 | Value -1.0 | Usage |
|---|---|---|---|---|
| Mechanical latch | 301 | Locked (param1 = 1.0) | Released (param1 = -1.0) | Holds vehicle to balloon tether hardware |
| Hot wire cutter | 302 | On (param2 = 1.0) | Off (param2 = -1.0) | Severs tether/line; timed pulse |

The function code determines which MAVLink parameter slot is populated:
- Code 302 → `cmd.param2 = value`
- All other codes (including 301) → `cmd.param1 = value`
- All unused params are set to `NAN` so the mixer only touches the targeted actuator.
- `cmd.param7 = 0.0f` selects actuator group 0 (`actuator_servos`).

**Startup initialization:** At the beginning of `run()`, both actuators are placed in their safe states:
```
send_do_set_actuator(301, 1.0f);   // latch locked
send_do_set_actuator(302, -1.0f);  // hot wire off
```

### 2.6 Hot Wire Timer

When `trigger_release()` fires, the hot wire (actuator 302) is energized (`1.0`) and `_hotwire_on_time` is captured. On every subsequent 10 Hz iteration, `run()` checks:

```
if hrt_elapsed_time(&_hotwire_on_time) > HOT_TIME * 1000 µs:
    send_do_set_actuator(302, -1.0f)   // de-energize
    _hotwire_active = false
```

`HOT_TIME` is read from the parameter store on each check (not cached), allowing the value to be changed in flight before a release. Default is 30,000 ms (30 seconds).

### 2.7 Motor Unlock Mechanism

`FW_THR_MAX` is used as a software interlock for the motor. Pre-launch the parameter is set to `0.0`, which causes TECS to command zero throttle regardless of its internal state. At the moment of release, `trigger_release()` performs:

```cpp
float thr_target = 0.4f;                 // fallback default
param_get(_param_rel_thr, &thr_target);  // read REL_THR
param_set(_param_fw_thr_max, &thr_target);
```

This is the sole mechanism by which the motor is enabled post-drop. If `_param_fw_thr_max` or `_param_rel_thr` are `PARAM_INVALID` (logged as CRITICAL errors at startup), the motor unlock will fail silently.

### 2.8 uORB Interface

| Direction | Topic | Struct | Usage |
|---|---|---|---|
| Subscribe | `parameter_update` | `parameter_update_s` | Triggers `updateParams()` via 1-second interval subscription |
| Subscribe | `release_command` | `release_command_s` | Receives arm/disarm/manual-release/reset commands from GCS or onboard logic |
| Subscribe | `vehicle_global_position` | `vehicle_global_position_s` | GPS altitude for threshold comparison |
| Publish | `release_status` | `release_status_s` | Current state broadcast at 10 Hz |

#### `release_command_s` fields processed:

| Field | Type | Action |
|---|---|---|
| `arm_altitude_trigger` | bool | Arms the module; sets `_target_altitude_m` from `target_altitude_m` field |
| `target_altitude_m` | float | Target altitude when arming |
| `manual_release` | bool | Immediately calls `trigger_release()` |
| `reset_mechanism` | bool | Full reset: disarms, re-locks actuators, sets `FW_THR_MAX = 0.0` |

Rearm logic: if `arm_altitude_trigger` is true and `_released` is already true (and it is not a reset), the module clears the released state, re-locks actuators, resets `FW_THR_MAX` to 0.0, and then re-arms for a new altitude trigger.

#### `release_status_s` fields published:

| Field | Type | Value |
|---|---|---|
| `timestamp` | uint64 | `hrt_absolute_time()` |
| `altitude_armed` | bool | `_altitude_armed` |
| `released` | bool | `_released` |
| `manual_release_triggered` | bool | `_manual_release_triggered` |
| `current_altitude_m` | float | Last received GPS altitude |
| `target_altitude_m` | float | `_target_altitude_m` |

### 2.9 Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `DROP_ALTITUDE` | float | 100.0 m | Altitude (MSL meters) at which automatic release fires. Read at startup via `param_find`/`param_get`; not re-read during flight. |
| `REL_THR` | float | 0.4 (code fallback) | Value written to `FW_THR_MAX` at the moment of release. Sets the motor's maximum throttle post-drop. |
| `HOT_TIME` | float | 30000.0 ms | Duration in milliseconds that the hot wire cutter remains energized after release. Re-read from parameter store on every iteration while the hot wire is active. |

Parameter handles are cached at startup:
```cpp
_param_fw_thr_max = param_find("FW_THR_MAX");
_param_rel_thr    = param_find("REL_THR");
_param_hot_time   = param_find("HOT_TIME");
```
If any handle is `PARAM_INVALID`, an error is logged. `DROP_ALTITUDE` is read once via an uncached `param_find` call immediately after.

### 2.10 Run Loop

`run()` executes at approximately 10 Hz (100 ms sleep per iteration):

```
run():
    parameters_update(force=true)          // initial param load
    cache FW_THR_MAX, REL_THR, HOT_TIME handles
    read DROP_ALTITUDE → _target_altitude_m
    send_do_set_actuator(301, 1.0)         // safe init
    send_do_set_actuator(302, -1.0)        // safe init

    while not should_exit():
        process_release_command()          // drain release_command subscription
        if vehicle_global_position updated:
            current_altitude = global_pos.alt
            if _altitude_armed and not _released and alt >= target:
                trigger_release()
        if _hotwire_active:
            if elapsed > HOT_TIME:
                send_do_set_actuator(302, -1.0)
                _hotwire_active = false
        publish_release_status(current_altitude)
        parameters_update()                // check for param changes
        px4_usleep(100000)                 // 100 ms
```

### 2.11 Key Code Paths

| Function | Description |
|---|---|
| `run()` | Main polling loop, 10 Hz |
| `trigger_release()` | Opens latch, starts hot wire, writes `FW_THR_MAX`. Idempotent — guarded by `if (!_released)` |
| `process_release_command()` | Drains `release_command` subscription; handles arm, disarm, manual release, reset, and rearm |
| `publish_release_status()` | Constructs and publishes `release_status_s` with current state snapshot |
| `parameters_update()` | Calls `updateParams()` when `parameter_update` topic has new data or `force=true` |
| `send_do_set_actuator()` | Static helper; builds and publishes `vehicle_command` for `MAV_CMD_DO_SET_ACTUATOR` |

### 2.12 CLI Commands

```
altitude_release start          # spawn background task
altitude_release stop           # request task exit
altitude_release status         # print armed/released/target state
altitude_release release        # manually trigger release (calls trigger_release())
altitude_release reset          # reset state, re-lock actuators, set FW_THR_MAX=0
```

The `reset` CLI command directly manipulates the instance's state members and calls `send_do_set_actuator` without going through `process_release_command`. It also calls `param_set` to restore `FW_THR_MAX` to `0.0`.

### 2.13 Task Properties

| Property | Value |
|---|---|
| Task name | `"module"` |
| Scheduler | `SCHED_DEFAULT` |
| Priority | `SCHED_PRIORITY_DEFAULT` |
| Stack size | 1500 bytes |
| Update rate | 10 Hz (100 ms sleep) |

---

## 3. Module: `esc_stow`

### 3.1 Purpose

`esc_stow` is a one-shot CLI utility that commands an ESC to park (stow) or unpark (unstow) the propeller blade. This is used at high altitude to aerodynamically stow the propeller during balloon ascent and to release it before motor start after vehicle separation.

### 3.2 Source Files

| File | Description |
|---|---|
| `esc_stow/esc_stow.cpp` | Complete implementation; no header file |

### 3.3 Architecture

`esc_stow` is not a persistent task. The entry point `esc_stow_main` parses arguments, constructs a single `esc_stow_command_s` message, advertises, publishes, unadvertises, and returns. There is no run loop and no retained state.

```
esc_stow_main(argc, argv)
    parse {stow|unstow} and optional -i <esc_index>
    build esc_stow_command_s{timestamp, esc_index, stow}
    orb_advertise(esc_stow_command, &cmd)
    orb_publish(esc_stow_command, pub, &cmd)
    orb_unadvertise(pub)
    return
```

### 3.4 uORB Interface

| Direction | Topic | Struct | Usage |
|---|---|---|---|
| Publish (one-shot) | `esc_stow_command` | `esc_stow_command_s` | Delivers stow/unstow command to ESC driver or handler |

#### `esc_stow_command_s` fields set:

| Field | Type | Value |
|---|---|---|
| `timestamp` | uint64 | `hrt_absolute_time()` |
| `esc_index` | uint8 | ESC index from `-i` argument (default 0) |
| `stow` | bool | `true` for stow, `false` for unstow |

### 3.5 ESC Index Validation

The `-i` argument is validated by `parse_esc_index()`:
- Parsed with `strtol` in base 10; rejects non-numeric input and trailing characters.
- Valid range: `0` to `esc_status_s::CONNECTED_ESC_MAX - 1`.
- On failure, an error is printed and the command is not published.

### 3.6 CLI Commands

```
esc_stow stow                  # stow ESC 0 (default)
esc_stow stow -i <index>       # stow ESC at given index
esc_stow unstow                # unstow ESC 0 (default)
esc_stow unstow -i <index>     # unstow ESC at given index
esc_stow help                  # print usage
esc_stow --help                # print usage
```

### 3.7 Error Handling

| Condition | Behavior |
|---|---|
| No subcommand | Prints usage, returns 1 |
| Unknown subcommand | `PX4_ERR`, prints usage, returns 1 |
| `-i` flag with no value | `PX4_ERR`, returns 1 |
| Invalid or out-of-range index | `PX4_ERR` with valid range message, returns 1 |
| Unknown option | `PX4_ERR`, prints usage, returns 1 |
| `orb_advertise` failure | `PX4_ERR`, returns 1 |
| `orb_publish` failure | `PX4_ERR`, returns non-zero result code |

---

## 4. Module: `power_channel`

### 4.1 Purpose

`power_channel` is a one-shot CLI utility that enables or disables a UAVCAN power bus channel by publishing a `power_channel_control` message. Currently only channel 1 is implemented and controllable.

### 4.2 Source Files

| File | Description |
|---|---|
| `power_channel/power_channel.cpp` | Complete implementation; no header file |

### 4.3 Architecture

Like `esc_stow`, `power_channel` is not a persistent task. The entry point `power_channel_main` parses arguments, publishes a single message, waits 50 ms for propagation, unadvertises, and returns.

```
power_channel_main(argc, argv)
    parse {on|off}
    parse optional -c <channel> (default 1; only 1 is valid)
    validate channel == 1
    build power_channel_control_s{timestamp, channel, enable}
    orb_advertise(power_channel_control, &control)
    orb_publish(power_channel_control, pub, &control)
    px4_usleep(50000)          // 50 ms propagation delay
    orb_unadvertise(pub)
    return
```

The 50 ms sleep before `orb_unadvertise` ensures that any subscribers (e.g., a UAVCAN node manager) have time to receive and act on the message before the topic is torn down.

### 4.4 uORB Interface

| Direction | Topic | Struct | Usage |
|---|---|---|---|
| Publish (one-shot) | `power_channel_control` | `power_channel_control_s` | Delivers enable/disable command to UAVCAN power channel handler |

#### `power_channel_control_s` fields set:

| Field | Type | Value |
|---|---|---|
| `timestamp` | uint64 | `hrt_absolute_time()` |
| `channel` | uint8 | Channel number (currently always 1) |
| `enable` | bool | `true` for on, `false` for off |

### 4.5 CLI Commands

```
power_channel on               # enable channel 1 (default)
power_channel on -c 1          # enable channel 1 (explicit)
power_channel off              # disable channel 1 (default)
power_channel off -c 1         # disable channel 1 (explicit)
```

Attempting to specify any channel other than 1 results in an error:
```
PX4_ERR("Only channel 1 is controllable")
```

### 4.6 Error Handling

| Condition | Behavior |
|---|---|
| No subcommand | Prints usage, returns 1 |
| Unknown subcommand | `PX4_ERR`, prints usage, returns 1 |
| Channel != 1 | `PX4_ERR`, returns 1 |
| `orb_advertise` returns null | `PX4_ERR`, returns 1 |
| `orb_publish` fails | `PX4_ERR` with result code, returns non-zero |

---

## 5. Module: `mavlink_debug_sender`

### 5.1 Purpose

`mavlink_debug_sender` is a development and debug utility that streams data over the four MAVLink debug topics (`NAMED_VALUE_FLOAT`, `NAMED_VALUE_INT`/`DEBUG`, `DEBUG_VECT`, `DEBUG_FLOAT_ARRAY`). It publishes ADC raw readings from `adc_report` into `debug_array`, and publishes incrementing test counters on the other three channels.

> **Note:** The values published on `debug_key_value`, `debug_value`, and `debug_vect` are test counters, not real physical quantities. The key name `"velx"` and vector name `"vel3D"` are placeholder names from scaffolding code. This module is not production flight code and should not be used as a data source for GNC algorithms.

### 5.2 Source Files

| File | Description |
|---|---|
| `mavlink_debug_sender/mavlink_debug_sender.cpp` | Complete implementation; no header file |

### 5.3 Architecture

`MavlinkDebugSender` inherits from `ModuleBase<MavlinkDebugSender>`. It runs as a persistent PX4 task with a 2000-byte stack. The run loop publishes all four debug channels at 2 Hz (500 ms sleep per iteration).

```
mavlink_debug_sender_main()
    └─ MavlinkDebugSender::main()
           └─ MavlinkDebugSender::run()
                  printf("Hello Debug!")
                  subscribe to adc_report
                  advertise debug_key_value, debug_value, debug_vect, debug_array
                  value_counter = 0
                  while not should_exit():
                      orb_check(adc_sub, &updated)
                      publish debug_key_value ("velx", value_counter)
                      publish debug_value     (ind=42, 0.5 * value_counter)
                      publish debug_vect      ("vel3D", 1x/2x/3x value_counter)
                      if adc updated:
                          copy adc_report → dbg_array.data[0..11]
                      publish debug_array
                      value_counter++
                      px4_usleep(500000)   // 500 ms
```

### 5.4 uORB Interface

| Direction | Topic | Struct | Published Content |
|---|---|---|---|
| Subscribe | `adc_report` | `adc_report_s` | ADC raw readings; polled with `orb_check` each iteration |
| Publish | `debug_key_value` | `debug_key_value_s` | Key `"velx"`, value = `value_counter` (integer counter cast to float) |
| Publish | `debug_value` | `debug_value_s` | Index 42, value = `0.5 * value_counter` |
| Publish | `debug_vect` | `debug_vect_s` | Name `"vel3D"`, x = `1.0 * value_counter`, y = `2.0 * value_counter`, z = `3.0 * value_counter` |
| Publish | `debug_array` | `debug_array_s` | ID 1, name `"dbg_array"`, data[0..11] = `adc_report.raw_data[0..11]` (12 channels) |

`debug_array` is published every iteration regardless of whether a new `adc_report` was available. If no new ADC data arrived, the array contains the values from the most recent successful copy.

### 5.5 Publication Details

All four topics are advertised once at the start of `run()` using the low-level `orb_advertise`/`orb_publish` API (not the C++ `uORB::Publication<>` wrapper). Handles are held for the lifetime of the task.

`debug_key_value.key` and `debug_vect.name` are set with `strncpy(..., 10)`, consistent with the 10-byte string fields in those structs.

### 5.6 CLI Commands

```
mavlink_debug_sender start      # spawn background task
mavlink_debug_sender stop       # request task exit
mavlink_debug_sender status     # print running status
```

No custom subcommands are defined. `custom_command` prints a warning and returns usage for any unrecognized input.

### 5.7 Task Properties

| Property | Value |
|---|---|
| Task name | `"mavlink_debug_sender"` |
| Scheduler | `SCHED_DEFAULT` |
| Priority | `SCHED_PRIORITY_DEFAULT` |
| Stack size | 2000 bytes |
| Update rate | 2 Hz (500 ms sleep) |

---

## 6. Mission Sequence Integration

The four modules cooperate across distinct phases of the stratospheric glider mission. The following describes the nominal sequence.

### 6.1 Pre-Launch Configuration

Before vehicle integration with the balloon:

1. `FW_THR_MAX` is set to `0.0` in the parameter store. This prevents TECS from commanding any throttle regardless of flight mode or airspeed, ensuring the motor cannot spin during balloon ascent.
2. `esc_stow stow` is issued to park the propeller blade in its stowed position, minimizing drag and preventing rotation during ascent.
3. `altitude_release start` is run from the startup script. On initialization the module locks actuator 301 (latch) and de-energizes actuator 302 (hot wire), placing the release hardware in its safe state.
4. A `release_command` with `arm_altitude_trigger=true` and `target_altitude_m` set to the mission `DROP_ALTITUDE` value is sent to arm the altitude trigger. Alternatively, `DROP_ALTITUDE` can be set as a parameter before launch and the module will read it at startup.

### 6.2 Ascent Under Balloon

The balloon carries the vehicle to altitude. During this phase:

- `altitude_release` polls `vehicle_global_position` at 10 Hz, comparing GPS altitude against `_target_altitude_m`. No action is taken while altitude is below the threshold.
- The motor is inert (`FW_THR_MAX = 0.0`).
- The propeller is stowed.
- `release_status` is published at 10 Hz and is available for downlink via MAVLink telemetry.
- `mavlink_debug_sender` (if running) streams ADC data and test counters to the ground station.

### 6.3 Release at Drop Altitude

When GPS altitude reaches `DROP_ALTITUDE`, `altitude_release` calls `trigger_release()`:

1. **Actuator 301 (latch)** is set to `-1.0` (released), opening the mechanical latch that connects the vehicle to the balloon tether hardware.
2. **Actuator 302 (hot wire)** is set to `1.0` (on), energizing the cutter to sever any remaining tether or line. The hot wire remains active for `HOT_TIME` milliseconds (default 30 s), then is automatically de-energized.
3. **`FW_THR_MAX`** is written with the value from the `REL_THR` parameter (code fallback 0.4), removing the throttle interlock and enabling the motor.

`release_status` is immediately updated with `released=true` and published.

### 6.4 Motor Start

With `FW_THR_MAX` now set to a non-zero value, TECS can command throttle. Depending on the active flight mode:

- TECS detects that `FW_THR_MAX` permits throttle output and begins commanding the ESC according to its energy control law.
- The motor accelerates toward the commanded throttle level.

### 6.5 Propeller Unstow

Once the motor is running or on operator command, the propeller is released from its stowed position:

```
esc_stow unstow
```

This publishes `esc_stow_command` with `stow=false` for ESC index 0 (default). The ESC driver clears the stow condition and allows the motor to drive the propeller through its full range of motion.

### 6.6 Free Flight

With the balloon separated, motor running, and propeller unstowed, the vehicle transitions to normal guided navigation. Standard PX4 GNC modules (TECS, L1/NPFG, attitude controller) manage flight. `altitude_release` continues to run and publish status, but takes no further action since `_released` is true. If a rearm is required (e.g., for testing or a second drop), a `release_command` with `arm_altitude_trigger=true` can be sent, which will reset the released state and re-arm for a new altitude threshold.

### 6.7 Module Interaction Summary

```
Pre-launch:
  altitude_release ──► actuator 301 = 1.0 (locked)
  altitude_release ──► actuator 302 = -1.0 (off)
  esc_stow stow    ──► esc_stow_command (stow=true)
  [param]            FW_THR_MAX = 0.0

Ascent:
  altitude_release  monitors vehicle_global_position.alt
  altitude_release  publishes release_status at 10 Hz

At DROP_ALTITUDE:
  altitude_release ──► actuator 301 = -1.0 (released)
  altitude_release ──► actuator 302 = 1.0 (hot wire on)
  altitude_release ──► FW_THR_MAX = REL_THR (motor unlocked)
  [HOT_TIME later]  actuator 302 = -1.0 (hot wire auto-off)

Post-drop:
  TECS            begins commanding throttle (FW_THR_MAX > 0)
  esc_stow unstow ──► esc_stow_command (stow=false)

Optional:
  power_channel on/off ──► power_channel_control (UAVCAN bus management)
  mavlink_debug_sender ──► debug_key_value / debug_value /
                           debug_vect / debug_array (ground station)
```
