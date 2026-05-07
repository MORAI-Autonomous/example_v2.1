# TCP API Reference

> Auto-generated from `transport/message_schema.py`. Do not edit manually.

## Common Header

Every TCP packet uses this 16-byte header before the payload described below.

| Offset | Type | Field | Description |
|--------|------|-------|-------------|
| `+0` | `uint8` | `magic` | Fixed magic byte `0x4D` (`'M'`) |
| `+1` | `uint8` | `msg_class` | `0x01` = request, `0x02` = response |
| `+2` | `uint32` | `msg_type` | Command / response type such as `0x1102` |
| `+6` | `uint32` | `payload_size` | Payload size in bytes, excluding the 16-byte header |
| `+10` | `uint32` | `request_id` | Request / response correlation id |
| `+14` | `uint16` | `flag` | Reserved, currently `0` |

- Header format: `proto.HEADER_FMT = <BBIIIH`
- Header size: `16 bytes`
- Payload sizes shown in this document do not include the 16-byte header.

## Summary

| Msg Type | Name | Request Payload | Response Payload |
|----------|------|-----------------|------------------|
| `0x1101` | `GetSimulationTimeStatus` | `0 bytes` | `44 bytes (mode = 1) / 48 bytes (mode = 2)` |
| `0x1102` | `SetSimulationTimeModeCommand` | `16 bytes (mode = 1) / 20 bytes (mode = 2)` | `20 bytes` |
| `0x1201` | `FixedStep` | `4 bytes` | `8 bytes` |
| `0x1202` | `SaveData` | `0 bytes` | `8 bytes` |
| `0x1301` | `CreateObject` | `36 bytes` | `>= 12 bytes` |
| `0x1302` | `ManualControlById` | `>= 28 bytes` | `8 bytes` |
| `0x1303` | `TransformControlById` | `>= 40 bytes` | `8 bytes` |
| `0x1304` | `SetTrajectory` | `>= 16 bytes + 32 bytes * item_count` | `8 bytes` |
| `0x1401` | `ActiveSuiteStatus` | `0 bytes` | `>= 20 bytes + variable bytes * item_count` |
| `0x1402` | `LoadSuite` | `>= 4 bytes` | `8 bytes` |
| `0x1504` | `ScenarioStatus` | `0 bytes` | `12 bytes` |
| `0x1505` | `ScenarioControl` | `>= 8 bytes` | `8 bytes` |

## Requests

## `0x1101` GetSimulationTimeStatus

- Direction: `request`
- Payload: `0 bytes`
- Builder: `tcp.send_get_status()`

Query current simulation time mode and timing state.

Wire layout: variant-specific

This message has no payload.

Notes:
- No payload.

## `0x1102` SetSimulationTimeModeCommand

- Direction: `request`
- Payload: `16 bytes (mode = 1) / 20 bytes (mode = 2)`
- Builder: `tcp.send_simulation_time_mode_command()`

Set simulation time mode using mode-specific payload layouts.

Wire layout: variant-specific

Variants:

### Variable Mode

- Selector: `mode = 1`

Wire layout: `i i i f`

| Field | Type | Description |
|------|------|-------------|
| `mode` | `int32` | 1 = TIME_MODE_VARIABLE |
| `target_fps` | `int32` | Target FPS (10~200) |
| `physics_delta_time` | `int32` | Physics substep delta time in ms |
| `simulation_speed` | `float32` | Simulation speed multiplier |

### Fixed Mode

- Selector: `mode = 2`

Wire layout: `i i i i i`

| Field | Type | Description |
|------|------|-------------|
| `mode` | `int32` | 2 = TIME_MODE_FIXED |
| `simulation_delta_time` | `int32` | Simulation tick delta time in ms |
| `physics_delta_time` | `int32` | Physics substep delta time in ms |
| `rtf` | `int32` | Real-Time Factor (1~20) |
| `user_control` | `int32` | 0 = auto, 1 = step-by-step |

## `0x1201` FixedStep

- Direction: `request`
- Payload: `4 bytes`
- Builder: `tcp.send_fixed_step()`

Advance the simulator by a fixed number of steps.

Wire layout: `I`

| Field | Type | Description |
|------|------|-------------|
| `step_count` | `uint32` | Number of simulation steps to execute |

## `0x1202` SaveData

- Direction: `request`
- Payload: `0 bytes`
- Builder: `tcp.send_save_data()`

Trigger simulator-side data capture.

Wire layout: variant-specific

This message has no payload.

Notes:
- No payload.

## `0x1301` CreateObject

- Direction: `request`
- Payload: `36 bytes`
- Builder: `tcp.send_create_object()`

Create an entity with initial transform and vehicle configuration.

Wire layout: `i f f f f f f i i`

| Field | Type | Description |
|------|------|-------------|
| `entity_type` | `int32` | - |
| `pos_x` | `float32` | - |
| `pos_y` | `float32` | - |
| `pos_z` | `float32` | - |
| `rot_x` | `float32` | - |
| `rot_y` | `float32` | - |
| `rot_z` | `float32` | - |
| `driving_mode` | `int32` | - |
| `ground_vehicle_model` | `int32` | - |

## `0x1302` ManualControlById

- Direction: `request`
- Payload: `>= 28 bytes`
- Builder: `tcp.send_manual_control_by_id()`

Send manual throttle, brake, and steering-wheel angle to a target entity.

Wire layout: `[uint32 len][bytes] d d d`

| Field | Type | Description |
|------|------|-------------|
| `entity_id` | `uint32 length + utf-8 bytes` | - |
| `throttle` | `float64` | - |
| `brake` | `float64` | - |
| `steer_angle` | `float64` | - |

## `0x1303` TransformControlById

- Direction: `request`
- Payload: `>= 40 bytes`
- Builder: `tcp.send_transform_control_by_id()`

Set target transform, steer angle, and speed for a target entity.

Wire layout: `[uint32 len][bytes] f f f f f f f d`

| Field | Type | Description |
|------|------|-------------|
| `entity_id` | `uint32 length + utf-8 bytes` | - |
| `pos_x` | `float32` | - |
| `pos_y` | `float32` | - |
| `pos_z` | `float32` | - |
| `rot_x` | `float32` | - |
| `rot_y` | `float32` | - |
| `rot_z` | `float32` | - |
| `steer_angle` | `float32` | - |
| `speed` | `float64` | Currently derived from Vehicle Info local velocity in m/s |

## `0x1304` SetTrajectory

- Direction: `request`
- Payload: `>= 16 bytes + 32 bytes * item_count`
- Builder: `tcp.send_set_trajectory()`

Send a named trajectory and follow mode to a target entity.

Wire layout: `[uint32 len][bytes] i [uint32 len][bytes] I`

| Field | Type | Description |
|------|------|-------------|
| `entity_id` | `uint32 length + utf-8 bytes` | - |
| `follow_mode` | `int32` | - |
| `trajectory_name` | `uint32 length + utf-8 bytes` | - |
| `point_count` | `uint32` | - |

Repeat layout:

| Field | Type | Description |
|------|------|-------------|
| `points[].x` | `float64` | - |
| `points[].y` | `float64` | - |
| `points[].z` | `float64` | - |
| `points[].time` | `float64` | - |

Notes:
- Each trajectory point is serialized as four float64 values.

## `0x1401` ActiveSuiteStatus

- Direction: `request`
- Payload: `0 bytes`
- Builder: `tcp.send_active_suite_status()`

Query the active suite and scenario list.

Wire layout: variant-specific

This message has no payload.

Notes:
- No payload.

## `0x1402` LoadSuite

- Direction: `request`
- Payload: `>= 4 bytes`
- Builder: `tcp.send_load_suite()`

Load a MORAI suite from a path string.

Wire layout: `[uint32 len][bytes]`

| Field | Type | Description |
|------|------|-------------|
| `suite_path` | `uint32 length + utf-8 bytes` | - |

## `0x1504` ScenarioStatus

- Direction: `request`
- Payload: `0 bytes`
- Builder: `tcp.send_scenario_status()`

Query current scenario execution state.

Wire layout: variant-specific

This message has no payload.

Notes:
- No payload.

## `0x1505` ScenarioControl

- Direction: `request`
- Payload: `>= 8 bytes`
- Builder: `tcp.send_scenario_control()`

Control scenario playback state and optional target scenario name.

Wire layout: `I [uint32 len][bytes]`

| Field | Type | Description |
|------|------|-------------|
| `command` | `uint32` | 1=Play, 2=Pause, 3=Stop, 4=Prev, 5=Next |
| `scenario_name` | `uint32 length + utf-8 bytes` | - |

## Responses

## `0x1101` GetSimulationTimeStatus

- Direction: `response`
- Payload: `44 bytes (mode = 1) / 48 bytes (mode = 2)`
- Parser: `tcp.parse_get_status_payload()`

Return current simulation time mode and current simulation clock state using mode-specific layouts.

Wire layout: variant-specific

Variants:

### Variable Mode

- Selector: `mode = 1`

Wire layout: `I I I i i f Q q i`

| Field | Type | Description |
|------|------|-------------|
| `result_code` | `uint32` | - |
| `detail_code` | `uint32` | - |
| `mode` | `uint32` | 1 = TIME_MODE_VARIABLE |
| `target_fps` | `int32` | Target FPS |
| `physics_delta_time` | `int32` | Physics substep delta time in ms |
| `simulation_speed` | `float32` | Simulation speed multiplier |
| `step_index` | `uint64` | Accumulated step count |
| `seconds` | `int64` | Simulation time seconds |
| `nanos` | `int32` | Simulation time nanoseconds remainder |

### Fixed Mode

- Selector: `mode = 2`

Wire layout: `I I I i i i i Q q i`

| Field | Type | Description |
|------|------|-------------|
| `result_code` | `uint32` | - |
| `detail_code` | `uint32` | - |
| `mode` | `uint32` | 2 = TIME_MODE_FIXED |
| `simulation_delta_time` | `int32` | Simulation tick delta time in ms |
| `physics_delta_time` | `int32` | Physics substep delta time in ms |
| `rtf` | `int32` | Real-Time Factor (1~20) |
| `user_control` | `int32` | 0 = auto, 1 = step-by-step |
| `step_index` | `uint64` | Accumulated step count |
| `seconds` | `int64` | Simulation time seconds |
| `nanos` | `int32` | Simulation time nanoseconds remainder |

## `0x1102` SetSimulationTimeModeCommand

- Direction: `response`
- Payload: `20 bytes`
- Parser: `tcp.parse_set_simulation_time_mode_payload()`

Return result code and the applied simulation time settings.

Wire layout: `I I I f f`

| Field | Type | Description |
|------|------|-------------|
| `result_code` | `uint32` | - |
| `detail_code` | `uint32` | - |
| `mode` | `uint32` | - |
| `fixed_delta` | `float32` | - |
| `simulation_speed` | `float32` | - |

## `0x1201` FixedStep

- Direction: `response`
- Payload: `8 bytes`
- Parser: `tcp.parse_result_code()`

Return result code for a fixed-step request.

Wire layout: `I I`

| Field | Type | Description |
|------|------|-------------|
| `result_code` | `uint32` | - |
| `detail_code` | `uint32` | - |

## `0x1202` SaveData

- Direction: `response`
- Payload: `8 bytes`
- Parser: `tcp.parse_result_code()`

Return result code for a save-data request.

Wire layout: `I I`

| Field | Type | Description |
|------|------|-------------|
| `result_code` | `uint32` | - |
| `detail_code` | `uint32` | - |

## `0x1301` CreateObject

- Direction: `response`
- Payload: `>= 12 bytes`
- Parser: `tcp.parse_create_object_payload()`

Return result code and the created object identifier.

Wire layout: `I I [uint32 len][bytes]`

| Field | Type | Description |
|------|------|-------------|
| `result_code` | `uint32` | - |
| `detail_code` | `uint32` | - |
| `object_id` | `uint32 length + utf-8 bytes` | - |

## `0x1302` ManualControlById

- Direction: `response`
- Payload: `8 bytes`
- Parser: `tcp.parse_result_code()`

Return result code for a manual-control request.

Wire layout: `I I`

| Field | Type | Description |
|------|------|-------------|
| `result_code` | `uint32` | - |
| `detail_code` | `uint32` | - |

## `0x1303` TransformControlById

- Direction: `response`
- Payload: `8 bytes`
- Parser: `tcp.parse_result_code()`

Return result code for a transform-control request.

Wire layout: `I I`

| Field | Type | Description |
|------|------|-------------|
| `result_code` | `uint32` | - |
| `detail_code` | `uint32` | - |

## `0x1304` SetTrajectory

- Direction: `response`
- Payload: `8 bytes`
- Parser: `tcp.parse_result_code()`

Return result code for a set-trajectory request.

Wire layout: `I I`

| Field | Type | Description |
|------|------|-------------|
| `result_code` | `uint32` | - |
| `detail_code` | `uint32` | - |

## `0x1401` ActiveSuiteStatus

- Direction: `response`
- Payload: `>= 20 bytes + variable bytes * item_count`
- Parser: `tcp.parse_active_suite_status_payload()`

Return the active suite, active scenario, and scenario name list.

Wire layout: `I I [uint32 len][bytes] [uint32 len][bytes] I`

| Field | Type | Description |
|------|------|-------------|
| `result_code` | `uint32` | - |
| `detail_code` | `uint32` | - |
| `active_suite_name` | `uint32 length + utf-8 bytes` | - |
| `active_scenario_name` | `uint32 length + utf-8 bytes` | - |
| `scenario_list_size` | `uint32` | - |

Repeat layout:

| Field | Type | Description |
|------|------|-------------|
| `scenario_list[].name` | `uint32 length + utf-8 bytes` | - |

## `0x1402` LoadSuite

- Direction: `response`
- Payload: `8 bytes`
- Parser: `tcp.parse_result_code()`

Return result code for a load-suite request.

Wire layout: `I I`

| Field | Type | Description |
|------|------|-------------|
| `result_code` | `uint32` | - |
| `detail_code` | `uint32` | - |

## `0x1504` ScenarioStatus

- Direction: `response`
- Payload: `12 bytes`
- Parser: `tcp.parse_scenario_status_payload()`

Return result code and current scenario execution state.

Wire layout: `I I I`

| Field | Type | Description |
|------|------|-------------|
| `result_code` | `uint32` | - |
| `detail_code` | `uint32` | - |
| `state` | `uint32` | 1=Play, 2=Pause, 3=Stop |

## `0x1505` ScenarioControl

- Direction: `response`
- Payload: `8 bytes`
- Parser: `tcp.parse_result_code()`

Return result code for a scenario-control request.

Wire layout: `I I`

| Field | Type | Description |
|------|------|-------------|
| `result_code` | `uint32` | - |
| `detail_code` | `uint32` | - |
