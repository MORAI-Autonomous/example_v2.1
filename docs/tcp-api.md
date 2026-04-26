# TCP API Reference

> Auto-generated from `transport/message_schema.py`. Do not edit manually.

## Summary

| Msg Type | Direction | Name | Payload |
|----------|-----------|------|---------|
| `0x1101` | `request` | `GetSimulationTimeStatus` | `0 bytes` |
| `0x1102` | `request` | `SetSimulationTimeModeCommand` | `12 bytes` |
| `0x1201` | `request` | `FixedStep` | `4 bytes` |
| `0x1202` | `request` | `SaveData` | `0 bytes` |
| `0x1301` | `request` | `CreateObject` | `36 bytes` |
| `0x1302` | `request` | `ManualControlById` | `>= 28 bytes` |
| `0x1303` | `request` | `TransformControlById` | `>= 40 bytes` |
| `0x1304` | `request` | `SetTrajectory` | `>= 16 bytes + 32 bytes * item_count` |
| `0x1401` | `request` | `ActiveSuiteStatus` | `0 bytes` |
| `0x1402` | `request` | `LoadSuite` | `>= 4 bytes` |
| `0x1504` | `request` | `ScenarioStatus` | `0 bytes` |
| `0x1505` | `request` | `ScenarioControl` | `>= 8 bytes` |
| `0x1101` | `response` | `GetSimulationTimeStatus` | `40 bytes` |
| `0x1102` | `response` | `SetSimulationTimeModeCommand` | `20 bytes` |
| `0x1201` | `response` | `FixedStep` | `8 bytes` |
| `0x1202` | `response` | `SaveData` | `8 bytes` |
| `0x1301` | `response` | `CreateObject` | `>= 12 bytes` |
| `0x1302` | `response` | `ManualControlById` | `8 bytes` |
| `0x1303` | `response` | `TransformControlById` | `8 bytes` |
| `0x1304` | `response` | `SetTrajectory` | `8 bytes` |
| `0x1401` | `response` | `ActiveSuiteStatus` | `>= 20 bytes + variable bytes * item_count` |
| `0x1402` | `response` | `LoadSuite` | `8 bytes` |
| `0x1504` | `response` | `ScenarioStatus` | `12 bytes` |
| `0x1505` | `response` | `ScenarioControl` | `8 bytes` |

## Requests

## `0x1101` GetSimulationTimeStatus

- Direction: `request`
- Payload: `0 bytes`
- Builder: `tcp.send_get_status()`

Query current simulation time mode and timing state.

Wire layout: `(no payload)`

This message has no payload.

Notes:
- No payload.

## `0x1102` SetSimulationTimeModeCommand

- Direction: `request`
- Payload: `12 bytes`
- Builder: `tcp.send_simulation_time_mode_command()`

Set simulation time mode, fixed delta, and variable-mode speed.

Wire layout: `i f f`

| Field | Type | Description |
|------|------|-------------|
| `mode` | `int32` | 1=Variable, 2=Fixed Delta, 3=Fixed Step |
| `fixed_delta` | `float32` | Milliseconds per simulation tick |
| `simulation_speed` | `float32` | Playback speed multiplier for variable mode |

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

Wire layout: `(no payload)`

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

Wire layout: `(no payload)`

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

Wire layout: `(no payload)`

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
- Payload: `40 bytes`
- Parser: `tcp.parse_get_status_payload()`

Return current simulation time mode and current simulation clock state.

Wire layout: `I I I f f Q q I`

| Field | Type | Description |
|------|------|-------------|
| `result_code` | `uint32` | - |
| `detail_code` | `uint32` | - |
| `mode` | `uint32` | 1=Variable, 2=Fixed Delta, 3=Fixed Step |
| `fixed_delta` | `float32` | Milliseconds per simulation tick |
| `simulation_speed` | `float32` | Playback speed multiplier for variable mode |
| `step_index` | `uint64` | Current fixed-step index |
| `seconds` | `int64` | Simulation clock seconds |
| `nanos` | `uint32` | Simulation clock nanoseconds |

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
