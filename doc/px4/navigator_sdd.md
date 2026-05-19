# Software Design Description: Navigator Module

**Project:** PX4-Based Stratospheric Glider
**Module:** `navigator` (`src/modules/navigator/`)
**Document revision:** 2026-05-19

---

## 1. Purpose

Navigator is the autonomous-flight coordinator that sits between the Ground Control Station (GCS) / mission layer and the low-level guidance layer (`fw_pos_control`). Its responsibilities are:

- Consuming a mission that has been uploaded from the GCS and stored on the SD card via dataman.
- Translating each mission item (or failsafe action) into a `position_setpoint_triplet` that is published over uORB and consumed by `fw_pos_control` (fixed-wing position controller).
- Monitoring geofence compliance and triggering breach responses.
- Handling vehicle-command requests (reposition, speed change, mission start, etc.) from the GCS or from other autopilot modules.
- Running feasibility checks against a mission before execution begins.

From `navigator_main.cpp`:

> "Module that is responsible for autonomous flight modes. This includes missions (read from dataman), takeoff and RTL. It is also responsible for geofence violation checking."

---

## 2. Inputs and Outputs

### 2.1 uORB Subscriptions

| Topic | Source | Use in Navigator |
|---|---|---|
| `vehicle_local_position` | EKF2 / estimator | Primary poll wakeup; provides NED velocity for geofence braking-distance prediction |
| `vehicle_global_position` | EKF2 / estimator | Lat/lon/alt used by all navigation modes for position setpoint construction |
| `sensor_gps` (`_gps_pos`) | GPS driver | Alternate position source for geofence checks when `GF_SOURCE = 1` (GPS) |
| `mission` | Commander / GCS upload | Provides `mission_dataman_id`, `count`, `current_seq`, `geofence_id`, `safe_points_id` |
| `vehicle_status` | Commander | `nav_state`, `arming_state`, `vehicle_type`, `is_vtol` — drives mode selection switch |
| `vehicle_command` | Commander / MAVLink | DO_REPOSITION, DO_CHANGE_ALTITUDE, DO_ORBIT, MISSION_START, DO_CHANGE_SPEED, NAV_TAKEOFF, etc. |
| `home_position` | Commander | Home lat/lon/alt used by RTL and geofence max-distance checks |
| `vehicle_land_detected` | Land detector | Discriminates between landed and in-air states for mode logic |
| `position_controller_status` | `fw_pos_control` | Acceptance radius reported by the controller; used to widen `NAV_ACC_RAD` for FW |
| `position_controller_landing_status` | `fw_pos_control` | Landing abort flag; triggers go-around logic |
| `parameter_update` | Parameter server | Triggers `params_update()` on any parameter change |

The subscriptions above are established in `Navigator::Navigator()` (legacy `orb_subscribe`) and in `Navigator::run()` (uORB C++ API via `SubscriptionData`/`SubscriptionInterval`).

### 2.2 uORB Publications

| Topic | Consumers | Content |
|---|---|---|
| `position_setpoint_triplet` | `fw_pos_control` | Previous / current / next position setpoints |
| `mission_result` | Commander, GCS | Current sequence number, validity flag, do-jump state |
| `geofence_result` | Commander | Which fence constraints are active and which are triggered |
| `vehicle_roi` | Gimbal manager | Region-of-interest commands forwarded from mission items |
| `vehicle_command` | Various | Commands forwarded on behalf of mission items (camera, VTOL transition, etc.) |
| `vehicle_command_ack` | Commander / MAVLink | Acknowledgement of handled vehicle commands |
| `navigator_status` | GCS | Current nav-state and failure code, published at least every 500 ms |
| `mode_completed` | Commander | Signals that an autonomous mode has finished |
| `distance_sensor_mode_change_request` | Distance sensor driver | Enables rangefinder during landing phases |
| `rtl_time_estimate` / `rtl_status` | GCS | RTL timing estimate and destination type (published by RTL sub-module) |

---

## 3. Update Rate

The main loop is driven by a `px4_poll` on three file descriptors:

```
fds[0]  vehicle_local_position   (primary wakeup)
fds[1]  vehicle_status
fds[2]  mission
```

`vehicle_local_position` is rate-limited to **50 ms** via `orb_set_interval`, making the nominal loop rate **20 Hz**. The poll timeout is 1000 ms; if no data arrives within that window the loop still executes (timeouts do not trigger a `continue`).

All other subscriptions (`vehicle_global_position`, `sensor_gps`, `parameter_update`, `vehicle_land_detected`, `home_position`, `vehicle_command`) are checked inside the loop using the uORB `updated()` / `copy()` / `update()` pattern; they are processed opportunistically on every iteration rather than being polled with a separate file descriptor.

---

## 4. Navigation Modes

Each navigation mode is a separate class that inherits from `NavigatorMode` (or `MissionBlock` / `MissionBase`, which are themselves `NavigatorMode` subclasses). The array `_navigation_mode_array[]` holds pointers to all instantiated modes; on each loop iteration `run(active)` is called for every mode. Only the mode currently pointed to by `_navigation_mode` receives `active == true`.

### 4.1 Mode Selection

The `nav_state` field of `vehicle_status` drives the switch in `Navigator::run()`:

| `nav_state` value | Selected mode object |
|---|---|
| `NAVIGATION_STATE_AUTO_MISSION` | `_mission` |
| `NAVIGATION_STATE_AUTO_LOITER` | `_loiter` |
| `NAVIGATION_STATE_AUTO_RTL` | `_rtl` (unless already in mission landing, in which case `_mission` is kept) |
| `NAVIGATION_STATE_AUTO_TAKEOFF` | `_takeoff` |
| `NAVIGATION_STATE_AUTO_LAND` | `_land` |
| `NAVIGATION_STATE_AUTO_PRECLAND` | `_precland` (PrecLandMode::Required) |
| All manual / stabilised states | `nullptr` (Navigator publishes an invalid triplet once) |

No mode executes while the vehicle is disarmed: if `arming_state != ARMING_STATE_ARMED` the selected mode pointer is forced to `nullptr`.

When the active mode changes, `reset_triplets()` is called unless the transition is from Takeoff to Loiter (preserving the takeoff altitude) or a valid loiter setpoint already exists and the new mode is Loiter.

### 4.2 Mission (`mission.cpp`)

- **Trigger:** Commander sets `nav_state = NAVIGATION_STATE_AUTO_MISSION`, which happens when the operator arms and starts a mission from the GCS.
- **Behavior:** Reads mission items sequentially from dataman. Calls `setActiveMissionItems()` to build the `position_setpoint_triplet` from the current, next, and (where feasible) following items.
- **Position setpoints produced:** Depends on the current `nav_cmd` of the mission item. Waypoints produce `SETPOINT_TYPE_POSITION`; loiter items produce `SETPOINT_TYPE_LOITER`; takeoff items produce `SETPOINT_TYPE_TAKEOFF`; land items produce `SETPOINT_TYPE_LAND`.
- **Gate items:** When the current item is a gate (no position), the next position-bearing item is promoted to `current` and the one after it becomes `next`.
- **Hold-time / autocontinue:** If `autocontinue` is false or `time_inside > 0` (or a timeout is configured), the `next` setpoint is set invalid to signal the position controller to decelerate and hold at the waypoint. Normal waypoints with `autocontinue = true` populate all three triplet slots.
- **Landing detection:** `Mission::isLanding()` is checked by Navigator to enable the distance-sensor rangefinder and to avoid switching away from Mission mode when RTL is commanded during a landing approach.
- **Mission save:** The current sequence number is written back to dataman (`DM_KEY_MISSION_STATE`) when the vehicle disarms, preserving resume state.

### 4.3 Loiter (`loiter.cpp`)

- **Trigger:** `nav_state = NAVIGATION_STATE_AUTO_LOITER`, typically via Hold mode switch or after a DO_REPOSITION command.
- **Behavior on activation:** If a reposition triplet was set within the last 500 ms (i.e., a DO_REPOSITION command just arrived), `reposition()` copies that triplet directly to the position setpoint. Otherwise `set_loiter_position()` establishes a loiter at the current location.
- **Loiter point selection in `set_loiter_position()`:**
  - If landed and disarmed: sets `SETPOINT_TYPE_IDLE` and returns without publishing a loiter point.
  - If already established on a loiter circle (distance from current setpoint center is within acceptance radius plus loiter radius): reuses the existing loiter center via `setLoiterItemFromCurrentPositionSetpoint`.
  - Fixed-wing (and non-multirotor): `setLoiterItemFromCurrentPosition` — uses the current global position.
- **Setpoint type:** `SETPOINT_TYPE_LOITER` with `loiter_radius` taken from `NAV_LOITER_RAD`.
- **Speed reset:** `reset_cruising_speed()` is called on activation; `cruising_speed` is set to `-1.f` (use firmware default) on a fresh reposition unless already in Loiter mode.
- **Active loop:** Checks for a new reposition triplet (< 500 ms old) and applies it each iteration, allowing the GCS to update the loiter point while the mode is active.

### 4.4 Takeoff (`takeoff.cpp`)

- **Trigger:** `nav_state = NAVIGATION_STATE_AUTO_TAKEOFF` or a `VEHICLE_CMD_NAV_TAKEOFF` command.
- **Position setpoints produced:** `SETPOINT_TYPE_TAKEOFF`. Target altitude is the commanded altitude in the takeoff command (param7), or `MIS_TAKEOFF_ALT` above home if not specified.
- **Previous setpoint:** Set to the current global position / heading when available and home position is valid.
- **Yaw:** Set to NaN; yaw-setpoint generation is delegated to FlightTaskAuto.

### 4.5 Land (`land.cpp`)

- **Trigger:** `nav_state = NAVIGATION_STATE_AUTO_LAND`.
- **Position setpoints produced:** `SETPOINT_TYPE_LAND` at the home position or the position specified by the mission landing item.

### 4.6 RTL (`rtl.cpp`, `rtl.h`)

See Section 7 for the full RTL description.

### 4.7 PrecLand (`precland.cpp`)

- **Trigger:** `nav_state = NAVIGATION_STATE_AUTO_PRECLAND`. Navigator calls `_precland.set_mode(PrecLandMode::Required)` before each iteration.
- **Behavior:** Uses a precision landing sensor (IR/visual) to refine the landing position. Produces a `SETPOINT_TYPE_LAND` setpoint with fine-grained position corrections fed by the sensor subsystem.

---

## 5. The `position_setpoint_triplet` Message

`position_setpoint_triplet_s` contains three `position_setpoint_s` slots:

| Slot | Meaning |
|---|---|
| `previous` | The waypoint just departed; used by `fw_pos_control` / NPFG to construct the current track line |
| `current` | The active navigation target; the controller steers toward this point |
| `next` | The upcoming waypoint; used for early course correction and smooth turn-anticipation |

### 5.1 Fields of `position_setpoint_s`

| Field | Type | Description |
|---|---|---|
| `valid` | bool | When false the controller ignores this slot |
| `type` | enum | See setpoint types below |
| `lat` / `lon` | double | Geodetic target (WGS-84 degrees) |
| `alt` | float | Target altitude AMSL (meters) |
| `yaw` | float | Commanded yaw (rad); NaN means "don't constrain yaw" |
| `cruising_speed` | float | Requested airspeed (m/s); `-1.f` means use firmware default |
| `cruising_throttle` | float | Requested normalised throttle [0,1]; NaN means use firmware default. Set to `0` for glide segments |
| `acceptance_radius` | float | Horizontal switching distance (m); overrides `NAV_ACC_RAD` if > 0 |
| `alt_acceptance_radius` | float | Vertical switching distance (m) |
| `loiter_radius` | float | Orbit radius for loiter setpoints (m) |
| `loiter_minor_radius` | float | Semi-minor axis for figure-eight loiter patterns |
| `loiter_direction_counter_clockwise` | bool | Direction flag |
| `loiter_orientation` | float | Orientation angle for figure-eight patterns |
| `loiter_pattern` | uint8 | `LOITER_TYPE_ORBIT` (standard circle) or `LOITER_TYPE_FIGUREEIGHT` |
| `timestamp` | hrt_abstime | Set on every update |

### 5.2 Setpoint Types

| Symbol | Value | Used for |
|---|---|---|
| `SETPOINT_TYPE_POSITION` | — | Normal en-route waypoint; controller tracks the track line from previous to current |
| `SETPOINT_TYPE_LOITER` | — | Hold / orbit; controller maintains a circle of radius `loiter_radius` around `lat/lon/alt` |
| `SETPOINT_TYPE_TAKEOFF` | — | Takeoff phase; controller climbs to `alt` before switching to normal guidance |
| `SETPOINT_TYPE_LAND` | — | Landing approach; controller descends to the ground at `lat/lon` |
| `SETPOINT_TYPE_IDLE` | — | Published when disarmed and landed; controller idles |

### 5.3 Default Initialization

`Navigator::reset_position_setpoint()` initialises every slot to:
- `lat = lon = NaN`, `yaw = NaN`
- `loiter_radius = NAV_LOITER_RAD`
- `acceptance_radius = NAV_ACC_RAD`
- `cruising_speed = get_cruising_speed()`
- `cruising_throttle = get_cruising_throttle()`
- `valid = false`, `type = SETPOINT_TYPE_IDLE`
- `loiter_direction_counter_clockwise = false`

---

## 6. Mission Execution

### 6.1 Storage and Loading

Mission items are stored on the SD card through the dataman service. The `mission` uORB topic carries:
- `mission_dataman_id`: identifies the dataman storage partition (alternates between two banks to allow atomic updates)
- `count`: total number of items in the mission
- `current_seq`: the index of the item to execute next
- `mission_id`: a monotonic counter that increments on each new upload

Navigator's `Mission` class (which inherits `MissionBase`) loads items via `DatamanClient` (asynchronous) and a `DatamanCache` with a default cache size of 10 items (`DEFAULT_MISSION_CACHE_SIZE = 10`). The cache is used by `setActiveMissionItems()` to look ahead up to two items beyond the current index so the `next` setpoint can be populated.

Loading is performed through `_dataman_cache.loadWait(mission_dataman_id, index, ...)` with `MAX_DATAMAN_LOAD_WAIT` as the timeout.

### 6.2 Sequence Advancement

`current_seq` advances inside `MissionBase` when the position controller signals that the current setpoint has been reached (acceptance radius crossed). `Mission::set_current_mission_index()` allows the GCS to jump directly to any index by issuing `VEHICLE_CMD_MISSION_START` with param1 set to the target index.

When the mission cache is reset from item 0 (`_mission.current_seq == 0`), `resetItemCache()` is called to flush stale entries.

### 6.3 Waypoint Switching and Acceptance Radius

The horizontal acceptance radius that triggers waypoint switching is determined in `Navigator::get_acceptance_radius()`:

1. Start with `NAV_ACC_RAD` (default 10 m).
2. For fixed-wing and rovers: take `max(NAV_ACC_RAD, pos_ctrl_status.acceptance_radius)` — the position controller (NPFG) reports its own switch distance and Navigator uses whichever is larger.
3. The mission item itself can override via `mission_item.acceptance_radius`; this is translated into `position_setpoint_s::acceptance_radius` by `mission_item_to_position_setpoint()`.

Vertical acceptance uses separate parameters:
- `NAV_FW_ALT_RAD` (default 10 m) for fixed-wing en-route.
- `NAV_FW_ALTL_RAD` (default 5 m) for the waypoint immediately before a fixed-wing landing approach — tighter to ensure a clean altitude at the start of final.
- `NAV_MC_ALT_RAD` (default 0.8 m) for multirotor.

When a mission item is on the landing sequence (`fw_on_mission_landing`), `alt_acceptance_radius` is set to `FLT_MAX` to prevent the FW lateral guidance from loitering at intermediate waypoints because altitude has not yet been reached.

### 6.4 Supported MAVLink Mission Item Types (`nav_cmd`)

The following `NAV_CMD_*` values are explicitly handled in `mission.cpp`, `mission_base.cpp`, or passed through `mission_item_to_position_setpoint()`:

| MAVLink Command | Description |
|---|---|
| `NAV_CMD_WAYPOINT` | Fly to lat/lon/alt; default `SETPOINT_TYPE_POSITION` |
| `NAV_CMD_LOITER_UNLIM` | Loiter indefinitely at lat/lon/alt |
| `NAV_CMD_LOITER_TURNS` | Loiter for N complete orbits |
| `NAV_CMD_LOITER_TIME` | Loiter for a specified duration (`time_inside`) |
| `NAV_CMD_LOITER_TO_ALT` | Loiter while climbing/descending to target altitude |
| `NAV_CMD_RETURN_TO_LAUNCH` | Triggers RTL mode via Commander |
| `NAV_CMD_LAND` | Fixed-wing or multirotor landing; `SETPOINT_TYPE_LAND` |
| `NAV_CMD_TAKEOFF` | Takeoff to altitude; `SETPOINT_TYPE_TAKEOFF` |
| `NAV_CMD_VTOL_TAKEOFF` | VTOL takeoff with multi-stage climb/transition work items |
| `NAV_CMD_DO_VTOL_TRANSITION` | Triggers MC-to-FW or FW-to-MC transition; includes heading-align work item |
| `NAV_CMD_DO_LAND_START` | Marker; sets the land-start index so RTL can jump directly to landing |
| `NAV_CMD_DO_JUMP` | Jump to another mission item index |
| `NAV_CMD_DO_CHANGE_SPEED` | Sets `cruising_speed` / `cruising_throttle` in Navigator and the triplet |
| `NAV_CMD_IMAGE_START_CAPTURE` / `NAV_CMD_IMAGE_STOP_CAPTURE` | Camera commands forwarded via `vehicle_command` |
| `NAV_CMD_VIDEO_START_CAPTURE` / `NAV_CMD_VIDEO_STOP_CAPTURE` | Camera video commands forwarded |
| `NAV_CMD_SET_CAMERA_MODE` / `NAV_CMD_SET_CAMERA_SOURCE` | Camera mode/source commands forwarded |
| `NAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW` | Gimbal control forwarded |

Non-position commands (`NAV_CMD_DO_*`) are issued via `issue_command()` which calls `publish_vehicle_cmd()`, then the mission advances to the next item through the normal `autocontinue` path (or after a command timeout set by `MIS_COMMAND_TOUT`).

---

## 7. RTL Modes

### 7.1 RTL Type Selection

The RTL class (`rtl.h`, `rtl.cpp`) selects among four concrete strategies via the `RTL_TYPE` parameter and runtime checks. The available types are defined in the `RtlType` enum:

| `RtlType` | Strategy |
|---|---|
| `RTL_DIRECT` | Fly directly from the current position to the home position (or nearest safe point), then descend and land |
| `RTL_DIRECT_MISSION_LAND` | Fly directly to home, then execute the mission's planned landing sequence (requires a `NAV_CMD_DO_LAND_START` marker in the mission) |
| `RTL_MISSION_FAST` | Execute the mission forward to its landing sequence without reversing |
| `RTL_MISSION_FAST_REVERSE` | Reverse through the mission back toward the launch/takeoff point |

The actual RTL logic is implemented in four separate classes: `RtlDirect`, `RtlDirectMissionLand`, `RtlMissionFast`, and `RtlMissionFastReverse`, all inheriting from `RtlBase`. The `RTL` orchestrator selects and delegates to the appropriate one via `_rtl_mission_type_handle`.

### 7.2 Destination Selection

`RTL::findRtlDestination()` evaluates three destination types:

- **Home (`DESTINATION_TYPE_HOME`):** The home position, always available if GPS lock has been acquired.
- **Mission land start (`DESTINATION_TYPE_MISSION_LAND`):** The position of the `NAV_CMD_DO_LAND_START` marker if `hasMissionLandStart()` returns true.
- **Safe point / rally point (`DESTINATION_TYPE_SAFE_POINT`):** Safe points uploaded via MAVLink and stored in dataman. A safe point is only considered valid if it is more than `MAX_DIST_FROM_HOME_FOR_LAND_APPROACHES` (10 m) from home.

Safe points are loaded asynchronously through a `DatamanCache` (`_dataman_cache_safepoint`) with a capacity of four items. The cache is refreshed whenever `safe_points_id` changes in the `mission` topic, which Navigator detects in the main poll loop and propagates via `RTL::updateSafePoints()`.

### 7.3 Return Altitude

The return altitude is derived from `RTL_RETURN_ALT` (parameter) and optionally modified by a cone geometry controlled by `RTL_CONE_ANG`. Close to home, a reduced return altitude is accepted; farther away, the full `RTL_RETURN_ALT` is enforced. `set_return_alt_min(true)` (called on ADSB traffic conflict) forces the full return altitude unconditionally.

### 7.4 VTOL Landing Approaches

For VTOL aircraft, `RTL` reads land-approach structures from dataman (`readVtolLandApproaches`). If approaches are defined, `chooseBestLandingApproach()` selects among them based on wind direction (via `_wind_sub`). The `RTL_APPR_FORCE` parameter controls whether a VTOL approach is mandatory.

---

## 8. Geofence

### 8.1 Loading

At startup, `Navigator::run()` calls `_geofence.loadFromFile(GEOFENCE_FILENAME)` if the file `/fs/microsd/etc/geofence.txt` exists. The fence can also be updated at runtime via MAVLink: when the `geofence_id` field of the `mission` topic changes, `_geofence.updateFence()` is called to reload from dataman.

### 8.2 Fence Types

The geofence supports:
- Maximum horizontal distance from home (`GF_MAX_HOR_DIST`, default 0 = disabled)
- Maximum vertical distance from home (`GF_MAX_VER_DIST`, default 0 = disabled)
- Custom polygon or circle fences loaded from file or MAVLink

Position source for fence checks is selectable via `GF_SOURCE`: `0` = `vehicle_global_position`, `1` = raw `sensor_gps`. Using GPS (`GF_SOURCE = 1`) removes dependency on the position estimator.

### 8.3 Breach Detection — `geofence_breach_check()`

Called every iteration, but gated by `GEOFENCE_CHECK_INTERVAL_US`. The check computes a forward test point:

- **Fixed-wing:** test point distance = `2 * NAV_LOITER_RAD`; bearing taken from `position_controller_status.nav_bearing` (< 100 ms old) or from velocity vector. Vertical test distance = 5 m.
- **Multirotor:** test point distance = braking distance computed by `GeofenceBreachAvoidance` from current horizontal velocity and MPC jerk/acceleration parameters. Vertical distance from climb-rate braking model.

When `GF_PREDICT = 1`, the predicted future position is checked instead of the current position. The breach flags (`geofence_max_dist_triggered`, `geofence_max_alt_triggered`, `geofence_custom_fence_triggered`) are sticky during an active loiter-response: once triggered they can only be set true, not cleared, until the loiter state ends.

### 8.4 Breach Response

The action is controlled by `GF_ACTION`:

| `GF_ACTION` | Value | Response |
|---|---|---|
| None | 0 | No action |
| Warning | 1 | Log/mavlink warning only |
| Hold mode | 2 | (default) Compute an avoidance loiter point via `GeofenceBreachAvoidance`, publish a reposition triplet with `SETPOINT_TYPE_LOITER`, set `_time_loitering_after_gf_breach` |
| Return mode | 3 | Commander-level RTL is triggered |
| Terminate | 4 | Flight termination |
| Land mode | 5 | Land in place |

For the **Hold** response, `GeofenceBreachAvoidance` computes the loiter center:
- Multirotor: `generateLoiterPointForMultirotor()` — places loiter at or near current position.
- Fixed-wing: `generateLoiterPointForFixedWing()` — places loiter ahead on the current track at a safe distance inside the fence boundary.

The avoidance loiter altitude is similarly computed by `generateLoiterAltitudeForMulticopter()` or `generateLoiterAltitudeForFixedWing()`.

`geofence_allows_position()` is also called before processing any DO_REPOSITION, DO_CHANGE_ALTITUDE, DO_ORBIT, or DO_FIGUREEIGHT command to reject positions outside the fence before they are applied.

---

## 9. Vehicle Command Handling

Vehicle commands are processed inside `Navigator::run()` before the navigation-mode switch. Up to `vehicle_command_s::ORB_QUEUE_LENGTH` commands are drained per iteration. Processing is paused if `_wait_for_vehicle_status_timestamp != 0` (waiting for a fresh `vehicle_status` after a mode-switching command).

| Command | Handler behaviour |
|---|---|
| `VEHICLE_CMD_DO_GO_AROUND` | Acknowledged accepted; actual go-around logic remains in the position controller (noted as a TODO to move to Navigator) |
| `VEHICLE_CMD_DO_REPOSITION` | Builds a reposition triplet (`SETPOINT_TYPE_LOITER`) from param5/6/7 (lat/lon/alt). Falls back to current global position for any NaN fields. Checks geofence before applying. Sets `_wait_for_vehicle_status_timestamp`. Acknowledged by Commander |
| `VEHICLE_CMD_DO_CHANGE_ALTITUDE` | Same logic as DO_REPOSITION with only altitude populated |
| `VEHICLE_CMD_DO_ORBIT` | Builds a loiter triplet with explicit radius (param1; negative = counter-clockwise). Fixed-wing only (multirotor handles orbit in FlightTask) |
| `VEHICLE_CMD_DO_FIGUREEIGHT` | Builds a figure-eight loiter triplet. Semi-minor radius from param2; major radius auto-scaled to 2.5x semi-minor, then clamped to at least 2x semi-minor. Fixed-wing only, requires `CONFIG_FIGURE_OF_EIGHT` |
| `VEHICLE_CMD_NAV_TAKEOFF` | Builds a takeoff triplet (`SETPOINT_TYPE_TAKEOFF`); yaw set to NaN |
| `VEHICLE_CMD_DO_LAND_START` | Looks up `_mission.get_land_start_index()` and re-publishes a `VEHICLE_CMD_MISSION_START` to jump to that index |
| `VEHICLE_CMD_MISSION_START` | Calls `_mission.set_current_mission_index(param1)` to seek to a specific item |
| `VEHICLE_CMD_DO_CHANGE_SPEED` | param2 > 0: sets `cruising_speed`. param3 / 100 sets throttle. Special-case: if param3 < -0.9, sets throttle to 0 (glide mode, logged as a warning) |
| `VEHICLE_CMD_DO_SET_ROI` / `_ROI_LOCATION` / `_ROI_WPNEXT_OFFSET` / `_ROI_NONE` | Builds and publishes `vehicle_roi` |
| `VEHICLE_CMD_DO_VTOL_TRANSITION` | Resets cruising speed and throttle to defaults (VTOL Takeoff handles its own transition separately) |

---

## 10. Mission Feasibility Checking

Before a mission is allowed to execute, `MissionFeasibilityChecker::checkMissionFeasible()` is called (from `Mission::on_activation()` via `check_mission_valid(true)`).

`MissionFeasibilityChecker` owns a `FeasibilityChecker` object (from `MissionFeasibility/FeasibilityChecker.hpp`) and a reference to the `DatamanClient` from the mission's base class. It validates:

- **Geofence compliance:** `checkMissionAgainstGeofence()` iterates all mission items with positions and calls `_geofence.checkPointAgainstAllGeofences()` for each. Any item outside the active fence fails the check.
- **Terrain clearance:** The `FeasibilityChecker` component checks waypoint altitudes against terrain data where available to ensure minimum clearance.
- **Landing approach angles:** Validates that the final approach path for land items meets angle and geometry constraints acceptable to the airframe.
- **Takeoff requirements:** If `MIS_TKO_LAND_REQ` demands a takeoff item, the checker verifies one is present and valid.

If the check fails, a MAVLink critical message is emitted and `_mission_result.valid` is set false. The mission will not proceed and the GCS will display the failure reason.

---

## 11. Key Parameters

| Parameter | Default | Description |
|---|---|---|
| `NAV_ACC_RAD` | 10.0 m | Default horizontal acceptance radius for waypoint switching. For fixed-wing, the NPFG switch distance (`position_controller_status.acceptance_radius`) is used if it is larger |
| `NAV_LOITER_RAD` | 80.0 m | Default loiter orbit radius (fixed-wing). Minimum 25 m, maximum 1000 m |
| `NAV_FW_ALT_RAD` | 10.0 m | Fixed-wing vertical acceptance radius for en-route waypoints |
| `NAV_FW_ALTL_RAD` | 5.0 m | Fixed-wing vertical acceptance radius for the last waypoint before a landing approach |
| `NAV_MC_ALT_RAD` | 0.8 m | Multirotor vertical acceptance radius |
| `NAV_MIN_LTR_ALT` | -1.0 m | Minimum loiter altitude above home (disabled when < 0) |
| `NAV_MIN_GND_DIST` | -1.0 m | Minimum height above ground during Mission and RTL; requires a distance sensor (disabled when < 0) |
| `MIS_TAKEOFF_ALT` | 2.5 m | Default relative takeoff altitude if not specified in the mission |
| `MIS_TKO_LAND_REQ` | 0 | Mission takeoff/landing requirement (0 = none, 1 = takeoff, 2 = landing, 3 = both) |
| `MIS_DIST_1WP` | 10000 m | Warning threshold for horizontal distance from home to first waypoint |
| `MIS_YAW_ERR` | 12 deg | Maximum yaw error accepted for forced-heading waypoint |
| `MIS_YAW_TMT` | -1.0 s | Timeout for reaching a forced heading; -1 = disabled |
| `MIS_LND_ABRT_ALT` | 30 m | Minimum climb altitude after a landing abort (fixed-wing only) |
| `MIS_COMMAND_TOUT` | 0.0 s | Timeout for DO commands (gripper, winch, gimbal) before autocontinue |
| `RTL_TYPE` | — | RTL strategy: 0 = RTL_DIRECT, 1 = RTL_MISSION_FAST, 2 = RTL_MISSION_FAST_REVERSE, 3 = RTL_DIRECT_MISSION_LAND |
| `RTL_RETURN_ALT` | — | Minimum return altitude AMSL for RTL |
| `RTL_CONE_ANG` | — | Half-angle of the altitude cone; reduces return altitude near home |
| `RTL_MIN_DIST` | — | Minimum distance from home before attempting to climb to return altitude |
| `RTL_APPR_FORCE` | — | Force use of VTOL land approach if defined |
| `GF_ACTION` | 2 | Geofence breach response: 0=none, 1=warn, 2=hold, 3=RTL, 4=terminate, 5=land |
| `GF_SOURCE` | 0 | Position source for geofence: 0=global position, 1=raw GPS |
| `GF_MAX_HOR_DIST` | 0 m | Maximum horizontal distance from home (0 = disabled) |
| `GF_MAX_VER_DIST` | 0 m | Maximum vertical distance from home (0 = disabled) |
| `GF_PREDICT` | 0 | Pre-emptive geofence: predict vehicle trajectory and trigger early (experimental) |
| `NAV_FORCE_VT` | 1 | Force VTOL mode for takeoff and landing |
| `NAV_TRAFF_AVOID` | 1 | ADSB traffic avoidance: 0=off, 1=warn, 2=RTL, 3=land, 4=hold |

---

## 12. Source File Index

| File | Role |
|---|---|
| `navigator_main.cpp` | Main loop, mode switch, command handling, geofence breach check, publication |
| `navigator.h` | Navigator class declaration, all member variables and uORB handles |
| `navigator_params.c` | `NAV_*` parameter definitions |
| `mission.cpp` / `mission.h` | Mission mode; item loading, setpoint construction, takeoff/landing/VTOL work-item state machine |
| `mission_base.cpp` / `mission_base.h` | Common mission sequencing, dataman interface, `getNextPositionItems()` |
| `mission_block.cpp` / `mission_block.h` | `mission_item_to_position_setpoint()`, helper conversions |
| `mission_params.c` | `MIS_*` parameter definitions |
| `mission_feasibility_checker.cpp` / `.h` | Entry point for pre-flight feasibility check |
| `MissionFeasibility/FeasibilityChecker.hpp` | Terrain clearance, approach angle, geofence-vs-mission checks |
| `loiter.cpp` / `loiter.h` | Loiter mode |
| `rtl.cpp` / `rtl.h` | RTL orchestrator: destination selection, type selection, VTOL approach |
| `rtl_direct.cpp` / `.h` | RTL_DIRECT strategy |
| `rtl_direct_mission_land.cpp` / `.h` | RTL_DIRECT_MISSION_LAND strategy |
| `rtl_mission_fast.cpp` / `.h` | RTL_MISSION_FAST strategy |
| `rtl_mission_fast_reverse.cpp` / `.h` | RTL_MISSION_FAST_REVERSE strategy |
| `rtl_base.h` | Abstract base class for RTL strategies |
| `land.cpp` / `land.h` | Land mode |
| `precland.cpp` / `precland.h` | Precision landing mode |
| `precland_params.c` | Precision landing parameters |
| `geofence.cpp` / `geofence.h` | Fence geometry, file loading, violation tests |
| `geofence_params.c` | `GF_*` parameter definitions |
| `GeofenceBreachAvoidance/geofence_breach_avoidance.h` | Avoidance setpoint computation for FW and MC |
| `navigator_mode.cpp` / `navigator_mode.h` | Abstract `NavigatorMode` base class (`on_activation`, `on_inactive`, `on_active`, `run`) |
| `navigation.h` | `mission_item_s`, `NAV_CMD_*` enumerations |
