from __future__ import annotations

from dataclasses import dataclass
import struct
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple


@dataclass(frozen=True)
class FieldSpec:
    name: str
    field_type: str
    description: str = ""


@dataclass(frozen=True)
class MessageSpec:
    msg_type: int
    name: str
    direction: str
    summary: str
    fields: tuple[FieldSpec, ...] = ()
    repeat_fields: tuple[FieldSpec, ...] = ()
    handler: str = ""
    parser: str = ""
    notes: tuple[str, ...] = ()

    @property
    def has_payload(self) -> bool:
        return bool(self.fields)


TYPE_LABELS: Dict[str, str] = {
    "int32": "int32",
    "uint32": "uint32",
    "int64": "int64",
    "uint64": "uint64",
    "float32": "float32",
    "float64": "float64",
    "string_u32": "uint32 length + utf-8 bytes",
}

TYPE_SIZES: Dict[str, Optional[int]] = {
    "int32": 4,
    "uint32": 4,
    "int64": 8,
    "uint64": 8,
    "float32": 4,
    "float64": 8,
    "string_u32": None,
}

STRUCT_FORMAT_CHARS: Dict[str, str] = {
    "int32": "i",
    "uint32": "I",
    "int64": "q",
    "uint64": "Q",
    "float32": "f",
    "float64": "d",
}


MESSAGES: tuple[MessageSpec, ...] = (
    MessageSpec(
        msg_type=0x1101,
        name="GetSimulationTimeStatus",
        direction="request",
        summary="Query current simulation time mode and timing state.",
        handler="tcp.send_get_status()",
        notes=("No payload.",),
    ),
    MessageSpec(
        msg_type=0x1102,
        name="SetSimulationTimeModeCommand",
        direction="request",
        summary="Set simulation time mode, fixed delta, and variable-mode speed.",
        handler="tcp.send_simulation_time_mode_command()",
        fields=(
            FieldSpec("mode", "int32", "1=Variable, 2=Fixed Delta, 3=Fixed Step"),
            FieldSpec("fixed_delta", "float32", "Milliseconds per simulation tick"),
            FieldSpec("simulation_speed", "float32", "Playback speed multiplier for variable mode"),
        ),
    ),
    MessageSpec(
        msg_type=0x1201,
        name="FixedStep",
        direction="request",
        summary="Advance the simulator by a fixed number of steps.",
        handler="tcp.send_fixed_step()",
        fields=(FieldSpec("step_count", "uint32", "Number of simulation steps to execute"),),
    ),
    MessageSpec(
        msg_type=0x1202,
        name="SaveData",
        direction="request",
        summary="Trigger simulator-side data capture.",
        handler="tcp.send_save_data()",
        notes=("No payload.",),
    ),
    MessageSpec(
        msg_type=0x1301,
        name="CreateObject",
        direction="request",
        summary="Create an entity with initial transform and vehicle configuration.",
        handler="tcp.send_create_object()",
        fields=(
            FieldSpec("entity_type", "int32"),
            FieldSpec("pos_x", "float32"),
            FieldSpec("pos_y", "float32"),
            FieldSpec("pos_z", "float32"),
            FieldSpec("rot_x", "float32"),
            FieldSpec("rot_y", "float32"),
            FieldSpec("rot_z", "float32"),
            FieldSpec("driving_mode", "int32"),
            FieldSpec("ground_vehicle_model", "int32"),
        ),
    ),
    MessageSpec(
        msg_type=0x1302,
        name="ManualControlById",
        direction="request",
        summary="Send manual throttle, brake, and steering-wheel angle to a target entity.",
        handler="tcp.send_manual_control_by_id()",
        fields=(
            FieldSpec("entity_id", "string_u32"),
            FieldSpec("throttle", "float64"),
            FieldSpec("brake", "float64"),
            FieldSpec("steer_angle", "float64"),
        ),
    ),
    MessageSpec(
        msg_type=0x1303,
        name="TransformControlById",
        direction="request",
        summary="Set target transform, steer angle, and speed for a target entity.",
        handler="tcp.send_transform_control_by_id()",
        fields=(
            FieldSpec("entity_id", "string_u32"),
            FieldSpec("pos_x", "float32"),
            FieldSpec("pos_y", "float32"),
            FieldSpec("pos_z", "float32"),
            FieldSpec("rot_x", "float32"),
            FieldSpec("rot_y", "float32"),
            FieldSpec("rot_z", "float32"),
            FieldSpec("steer_angle", "float32"),
            FieldSpec("speed", "float64", "Currently derived from Vehicle Info local velocity in m/s"),
        ),
    ),
    MessageSpec(
        msg_type=0x1304,
        name="SetTrajectory",
        direction="request",
        summary="Send a named trajectory and follow mode to a target entity.",
        handler="tcp.send_set_trajectory()",
        fields=(
            FieldSpec("entity_id", "string_u32"),
            FieldSpec("follow_mode", "int32"),
            FieldSpec("trajectory_name", "string_u32"),
            FieldSpec("point_count", "uint32"),
        ),
        repeat_fields=(
            FieldSpec("points[].x", "float64"),
            FieldSpec("points[].y", "float64"),
            FieldSpec("points[].z", "float64"),
            FieldSpec("points[].time", "float64"),
        ),
        notes=("Each trajectory point is serialized as four float64 values.",),
    ),
    MessageSpec(
        msg_type=0x1401,
        name="ActiveSuiteStatus",
        direction="request",
        summary="Query the active suite and scenario list.",
        handler="tcp.send_active_suite_status()",
        notes=("No payload.",),
    ),
    MessageSpec(
        msg_type=0x1402,
        name="LoadSuite",
        direction="request",
        summary="Load a MORAI suite from a path string.",
        handler="tcp.send_load_suite()",
        fields=(FieldSpec("suite_path", "string_u32"),),
    ),
    MessageSpec(
        msg_type=0x1504,
        name="ScenarioStatus",
        direction="request",
        summary="Query current scenario execution state.",
        handler="tcp.send_scenario_status()",
        notes=("No payload.",),
    ),
    MessageSpec(
        msg_type=0x1505,
        name="ScenarioControl",
        direction="request",
        summary="Control scenario playback state and optional target scenario name.",
        handler="tcp.send_scenario_control()",
        fields=(
            FieldSpec("command", "uint32", "1=Play, 2=Pause, 3=Stop, 4=Prev, 5=Next"),
            FieldSpec("scenario_name", "string_u32"),
        ),
    ),
)


RESPONSE_MESSAGES: tuple[MessageSpec, ...] = (
    MessageSpec(
        msg_type=0x1101,
        name="GetSimulationTimeStatus",
        direction="response",
        summary="Return current simulation time mode and current simulation clock state.",
        parser="tcp.parse_get_status_payload()",
        fields=(
            FieldSpec("result_code", "uint32"),
            FieldSpec("detail_code", "uint32"),
            FieldSpec("mode", "uint32", "1=Variable, 2=Fixed Delta, 3=Fixed Step"),
            FieldSpec("fixed_delta", "float32", "Milliseconds per simulation tick"),
            FieldSpec("simulation_speed", "float32", "Playback speed multiplier for variable mode"),
            FieldSpec("step_index", "uint64", "Current fixed-step index"),
            FieldSpec("seconds", "int64", "Simulation clock seconds"),
            FieldSpec("nanos", "uint32", "Simulation clock nanoseconds"),
        ),
    ),
    MessageSpec(
        msg_type=0x1102,
        name="SetSimulationTimeModeCommand",
        direction="response",
        summary="Return result code and the applied simulation time settings.",
        parser="tcp.parse_set_simulation_time_mode_payload()",
        fields=(
            FieldSpec("result_code", "uint32"),
            FieldSpec("detail_code", "uint32"),
            FieldSpec("mode", "uint32"),
            FieldSpec("fixed_delta", "float32"),
            FieldSpec("simulation_speed", "float32"),
        ),
    ),
    MessageSpec(
        msg_type=0x1201,
        name="FixedStep",
        direction="response",
        summary="Return result code for a fixed-step request.",
        parser="tcp.parse_result_code()",
        fields=(
            FieldSpec("result_code", "uint32"),
            FieldSpec("detail_code", "uint32"),
        ),
    ),
    MessageSpec(
        msg_type=0x1202,
        name="SaveData",
        direction="response",
        summary="Return result code for a save-data request.",
        parser="tcp.parse_result_code()",
        fields=(
            FieldSpec("result_code", "uint32"),
            FieldSpec("detail_code", "uint32"),
        ),
    ),
    MessageSpec(
        msg_type=0x1301,
        name="CreateObject",
        direction="response",
        summary="Return result code and the created object identifier.",
        parser="tcp.parse_create_object_payload()",
        fields=(
            FieldSpec("result_code", "uint32"),
            FieldSpec("detail_code", "uint32"),
            FieldSpec("object_id", "string_u32"),
        ),
    ),
    MessageSpec(
        msg_type=0x1302,
        name="ManualControlById",
        direction="response",
        summary="Return result code for a manual-control request.",
        parser="tcp.parse_result_code()",
        fields=(
            FieldSpec("result_code", "uint32"),
            FieldSpec("detail_code", "uint32"),
        ),
    ),
    MessageSpec(
        msg_type=0x1303,
        name="TransformControlById",
        direction="response",
        summary="Return result code for a transform-control request.",
        parser="tcp.parse_result_code()",
        fields=(
            FieldSpec("result_code", "uint32"),
            FieldSpec("detail_code", "uint32"),
        ),
    ),
    MessageSpec(
        msg_type=0x1304,
        name="SetTrajectory",
        direction="response",
        summary="Return result code for a set-trajectory request.",
        parser="tcp.parse_result_code()",
        fields=(
            FieldSpec("result_code", "uint32"),
            FieldSpec("detail_code", "uint32"),
        ),
    ),
    MessageSpec(
        msg_type=0x1401,
        name="ActiveSuiteStatus",
        direction="response",
        summary="Return the active suite, active scenario, and scenario name list.",
        parser="tcp.parse_active_suite_status_payload()",
        fields=(
            FieldSpec("result_code", "uint32"),
            FieldSpec("detail_code", "uint32"),
            FieldSpec("active_suite_name", "string_u32"),
            FieldSpec("active_scenario_name", "string_u32"),
            FieldSpec("scenario_list_size", "uint32"),
        ),
        repeat_fields=(FieldSpec("scenario_list[].name", "string_u32"),),
    ),
    MessageSpec(
        msg_type=0x1402,
        name="LoadSuite",
        direction="response",
        summary="Return result code for a load-suite request.",
        parser="tcp.parse_result_code()",
        fields=(
            FieldSpec("result_code", "uint32"),
            FieldSpec("detail_code", "uint32"),
        ),
    ),
    MessageSpec(
        msg_type=0x1504,
        name="ScenarioStatus",
        direction="response",
        summary="Return result code and current scenario execution state.",
        parser="tcp.parse_scenario_status_payload()",
        fields=(
            FieldSpec("result_code", "uint32"),
            FieldSpec("detail_code", "uint32"),
            FieldSpec("state", "uint32", "1=Play, 2=Pause, 3=Stop"),
        ),
    ),
    MessageSpec(
        msg_type=0x1505,
        name="ScenarioControl",
        direction="response",
        summary="Return result code for a scenario-control request.",
        parser="tcp.parse_result_code()",
        fields=(
            FieldSpec("result_code", "uint32"),
            FieldSpec("detail_code", "uint32"),
        ),
    ),
)


def iter_messages() -> Iterable[MessageSpec]:
    return MESSAGES


def iter_response_messages() -> Iterable[MessageSpec]:
    return RESPONSE_MESSAGES


def get_message(msg_type: int) -> MessageSpec:
    for message in MESSAGES:
        if message.msg_type == msg_type:
            return message
    raise KeyError(msg_type)


def get_response_message(msg_type: int) -> MessageSpec:
    for message in RESPONSE_MESSAGES:
        if message.msg_type == msg_type:
            return message
    raise KeyError(msg_type)


def get_static_payload_size(message: MessageSpec) -> Optional[int]:
    total = 0
    for field in message.fields:
        size = TYPE_SIZES[field.field_type]
        if size is None:
            return None
        total += size
    return total


def get_min_payload_size(message: MessageSpec) -> int:
    total = 0
    for field in message.fields:
        size = TYPE_SIZES[field.field_type]
        if size is None:
            total += 4
        else:
            total += size
    return total


def describe_payload_size(message: MessageSpec) -> str:
    static_size = get_static_payload_size(message)
    if static_size is not None:
        return f"{static_size} bytes"
    base = f">= {get_min_payload_size(message)} bytes"
    if message.repeat_fields:
        per_item_sizes = [TYPE_SIZES[field.field_type] for field in message.repeat_fields]
        if all(size is not None for size in per_item_sizes):
            per_item = sum(size or 0 for size in per_item_sizes)
            return f"{base} + {per_item} bytes * item_count"
        return f"{base} + variable bytes * item_count"
    return base


def render_wire_type(field_type: str) -> str:
    return TYPE_LABELS[field_type]


def render_struct_format(fields: Iterable[FieldSpec]) -> str:
    fmt_parts: List[str] = []
    for field in fields:
        if field.field_type == "string_u32":
            fmt_parts.append("[uint32 len][bytes]")
        else:
            fmt_parts.append(STRUCT_FORMAT_CHARS[field.field_type])
    return " ".join(fmt_parts) if fmt_parts else "(no payload)"


def fixed_fields(fields: Iterable[FieldSpec]) -> tuple[FieldSpec, ...]:
    return tuple(field for field in fields if field.field_type != "string_u32")


def prefixed_string_fields(fields: Iterable[FieldSpec]) -> tuple[FieldSpec, ...]:
    return tuple(field for field in fields if field.field_type == "string_u32")


def build_struct_format(fields: Sequence[FieldSpec], endian: str = "<") -> str:
    return endian + "".join(STRUCT_FORMAT_CHARS[field.field_type] for field in fields)


def pack_value(field_type: str, value: Any) -> bytes:
    if field_type == "string_u32":
        encoded = str(value).encode("utf-8")
        return struct.pack("<I", len(encoded)) + encoded
    return struct.pack("<" + STRUCT_FORMAT_CHARS[field_type], value)


def pack_fields(fields: Sequence[FieldSpec], values: Mapping[str, Any]) -> bytes:
    payload_parts: List[bytes] = []
    for field in fields:
        if field.name not in values:
            raise KeyError(field.name)
        payload_parts.append(pack_value(field.field_type, values[field.name]))
    return b"".join(payload_parts)


def pack_repeated_fields(
    fields: Sequence[FieldSpec],
    items: Sequence[Mapping[str, Any]],
) -> bytes:
    payload_parts: List[bytes] = []
    for item in items:
        payload_parts.append(pack_fields(fields, item))
    return b"".join(payload_parts)


def pack_message_payload(
    msg_type: int,
    values: Mapping[str, Any],
    repeated_items: Optional[Sequence[Mapping[str, Any]]] = None,
) -> bytes:
    message = get_message(msg_type)
    payload = pack_fields(message.fields, values)
    if message.repeat_fields:
        if repeated_items is None:
            raise ValueError(f"message 0x{msg_type:04X} requires repeated_items")
        payload += pack_repeated_fields(message.repeat_fields, repeated_items)
    elif repeated_items:
        raise ValueError(f"message 0x{msg_type:04X} does not support repeated_items")
    return payload


def unpack_value(field_type: str, payload: bytes, offset: int = 0) -> Tuple[Any, int]:
    if field_type == "string_u32":
        if offset + 4 > len(payload):
            raise ValueError(f"not enough bytes for string length at offset {offset}")
        (str_len,) = struct.unpack_from("<I", payload, offset)
        offset += 4
        end = offset + str_len
        if end > len(payload):
            raise ValueError(f"not enough bytes for string value at offset {offset}")
        return payload[offset:end].decode("utf-8", errors="replace"), end

    size = TYPE_SIZES[field_type]
    if size is None or offset + size > len(payload):
        raise ValueError(f"not enough bytes for {field_type} at offset {offset}")
    value = struct.unpack_from("<" + STRUCT_FORMAT_CHARS[field_type], payload, offset)[0]
    return value, offset + size


def unpack_fields(
    fields: Sequence[FieldSpec],
    payload: bytes,
    offset: int = 0,
) -> Tuple[Dict[str, Any], int]:
    values: Dict[str, Any] = {}
    for field in fields:
        value, offset = unpack_value(field.field_type, payload, offset)
        values[field.name] = value
    return values, offset


def unpack_repeated_fields(
    fields: Sequence[FieldSpec],
    payload: bytes,
    count: int,
    offset: int = 0,
) -> Tuple[List[Dict[str, Any]], int]:
    items: List[Dict[str, Any]] = []
    for _ in range(count):
        item, offset = unpack_fields(fields, payload, offset)
        items.append(item)
    return items, offset


def unpack_message_payload(
    msg_type: int,
    payload: bytes,
    direction: str = "request",
    repeated_count_field: Optional[str] = None,
) -> Tuple[Dict[str, Any], List[Dict[str, Any]], int]:
    message = get_message(msg_type) if direction == "request" else get_response_message(msg_type)
    values, offset = unpack_fields(message.fields, payload, 0)

    repeated_items: List[Dict[str, Any]] = []
    if message.repeat_fields:
        if repeated_count_field is None:
            raise ValueError(f"message 0x{msg_type:04X} requires repeated_count_field")
        count = values.get(repeated_count_field)
        if not isinstance(count, int):
            raise ValueError(f"field {repeated_count_field} must be decoded before repeated fields")
        repeated_items, offset = unpack_repeated_fields(message.repeat_fields, payload, count, offset)

    return values, repeated_items, offset
