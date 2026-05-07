from __future__ import annotations

import socket
import struct
from typing import Any, Dict, List, Optional, Tuple

from transport.message_schema import (
    get_response_message,
    pack_message_payload,
    unpack_fields,
    unpack_message_payload,
)
import transport.protocol_defs as proto


# ============================================================
# Low-level recv / send helpers
# ============================================================

def recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("Socket closed by peer")
        buf.extend(chunk)
    return bytes(buf)


def recv_header_synced(sock: socket.socket) -> bytes:
    """Read a valid TCP header after syncing on the MAGIC byte."""
    while True:
        b = recv_exact(sock, 1)
        if b[0] != proto.MAGIC:
            continue

        rest = recv_exact(sock, proto.HEADER_SIZE - 1)
        header_bytes = b + rest

        _, msg_class, msg_type, payload_size, _, _ = struct.unpack(proto.HEADER_FMT, header_bytes)

        if msg_class not in proto.VALID_MSG_CLASSES:
            continue
        if msg_type not in proto.VALID_MSG_TYPES:
            continue
        if payload_size > 1024 * 1024:
            continue

        return header_bytes


def recv_packet(sock: socket.socket) -> Tuple[int, int, int, int, int, bytes]:
    """Return `(msg_class, msg_type, payload_size, request_id, flag, payload)`."""
    header_bytes = recv_header_synced(sock)
    _, msg_class, msg_type, payload_size, request_id, flag = struct.unpack(
        proto.HEADER_FMT, header_bytes
    )
    if payload_size < 0 or payload_size > 1024 * 1024:
        raise ValueError(f"Invalid payload_size: {payload_size}")

    payload = recv_exact(sock, payload_size) if payload_size > 0 else b""
    return msg_class, msg_type, payload_size, request_id, flag, payload


def build_header(
    msg_class: int,
    msg_type: int,
    payload_size: int,
    request_id: int,
    flag: int = 0,
) -> bytes:
    return struct.pack(
        proto.HEADER_FMT,
        proto.MAGIC, msg_class, msg_type, payload_size, request_id, flag,
    )


def _send_packet(
    sock: socket.socket,
    request_id: int,
    msg_type: int,
    payload: bytes,
    log: str = "",
) -> None:
    """Build the header, send the packet, and emit optional send log."""
    header = build_header(proto.MSG_CLASS_REQ, msg_type, len(payload), request_id, proto.FLAG)
    sock.sendall(header + payload)
    if log:
        print(f"[SEND][TCP] {log} rid={request_id}")


# ============================================================
# Payload builders
# ============================================================

def build_manual_control_by_id_payload(
    entity_id: str,
    throttle: float,
    brake: float,
    steer_angle: float,
) -> bytes:
    return pack_message_payload(
        proto.MSG_TYPE_MANUAL_CONTROL_BY_ID_COMMAND,
        {
            "entity_id": entity_id,
            "throttle": throttle,
            "brake": brake,
            "steer_angle": steer_angle,
        },
    )


def build_transform_control_by_id_payload(
    entity_id: str,
    pos_x: float, pos_y: float, pos_z: float,
    rot_x: float, rot_y: float, rot_z: float,
    steer_angle: float,
    speed: float,
) -> bytes:
    return pack_message_payload(
        proto.MSG_TYPE_TRANSFORM_CONTROL_BY_ID_COMMAND,
        {
            "entity_id": entity_id,
            "pos_x": pos_x,
            "pos_y": pos_y,
            "pos_z": pos_z,
            "rot_x": rot_x,
            "rot_y": rot_y,
            "rot_z": rot_z,
            "steer_angle": steer_angle,
            "speed": speed,
        },
    )


def build_set_trajectory_payload(
    entity_id: str,
    follow_mode: int,
    trajectory_name: str,
    points: List[Tuple[float, float, float, float]],
) -> bytes:
    return pack_message_payload(
        proto.MSG_TYPE_SET_TRAJECTORY_COMMAND,
        {
            "entity_id": entity_id,
            "follow_mode": follow_mode,
            "trajectory_name": trajectory_name,
            "point_count": len(points),
        },
        repeated_items=[
            {
                "points[].x": x,
                "points[].y": y,
                "points[].z": z,
                "points[].time": t,
            }
            for x, y, z, t in points
        ],
    )


# ============================================================
# Send commands
# ============================================================

def send_get_status(sock: socket.socket, request_id: int) -> None:
    _send_packet(sock, request_id, proto.MSG_TYPE_GET_SIMULATION_TIME_STATUS, b"",
                 "GetStatus(0x1101)")


def send_simulation_time_mode_command(
    sock: socket.socket,
    request_id: int,
    mode: int,
    target_fps: int = 60,
    physics_delta_time: int = 10,
    simulation_speed: float = 1.0,
    simulation_delta_time: int = 16,
    rtf: int = 1,
    user_control: int = 0,
) -> None:
    """mode: 1=variable, 2=fixed."""
    if mode == proto.TIME_MODE_VARIABLE:
        payload_values = {
            "mode": mode,
            "target_fps": int(target_fps),
            "physics_delta_time": int(physics_delta_time),
            "simulation_speed": float(simulation_speed),
        }
        log_text = (
            "SetSimulationTimeModeCommand(0x1102) "
            f"mode={mode} target_fps={target_fps} physics_delta_time={physics_delta_time} "
            f"simulation_speed={simulation_speed}"
        )
    elif mode == proto.TIME_MODE_FIXED:
        payload_values = {
            "mode": mode,
            "simulation_delta_time": int(simulation_delta_time),
            "physics_delta_time": int(physics_delta_time),
            "rtf": int(rtf),
            "user_control": int(user_control),
        }
        log_text = (
            "SetSimulationTimeModeCommand(0x1102) "
            f"mode={mode} simulation_delta_time={simulation_delta_time} "
            f"physics_delta_time={physics_delta_time} rtf={rtf} user_control={user_control}"
        )
    else:
        raise ValueError(f"Unsupported simulation time mode: {mode}")

    payload = pack_message_payload(
        proto.MSG_TYPE_SET_SIMULATION_TIME_MODE_COMMAND,
        payload_values,
    )
    _send_packet(
        sock,
        request_id,
        proto.MSG_TYPE_SET_SIMULATION_TIME_MODE_COMMAND,
        payload,
        log_text,
    )


def send_fixed_step(sock: socket.socket, request_id: int, step_count: int) -> None:
    payload = pack_message_payload(
        proto.MSG_TYPE_FIXED_STEP,
        {"step_count": step_count},
    )
    _send_packet(sock, request_id, proto.MSG_TYPE_FIXED_STEP, payload)


def send_save_data(sock: socket.socket, request_id: int) -> None:
    _send_packet(sock, request_id, proto.MSG_TYPE_SAVE_DATA, b"")


def send_create_object(
    sock: socket.socket,
    request_id: int,
    entity_type: int,
    pos_x: float, pos_y: float, pos_z: float,
    rot_x: float, rot_y: float, rot_z: float,
    driving_mode: int,
    ground_vehicle_model: int,
) -> None:
    payload = pack_message_payload(
        proto.MSG_TYPE_CREATE_OBJECT,
        {
            "entity_type": entity_type,
            "pos_x": pos_x,
            "pos_y": pos_y,
            "pos_z": pos_z,
            "rot_x": rot_x,
            "rot_y": rot_y,
            "rot_z": rot_z,
            "driving_mode": driving_mode,
            "ground_vehicle_model": ground_vehicle_model,
        },
    )
    _send_packet(sock, request_id, proto.MSG_TYPE_CREATE_OBJECT, payload,
                 "CreateObject(0x1301)")


def send_manual_control_by_id(
    sock: socket.socket,
    request_id: int,
    entity_id: str,
    throttle: float,
    brake: float,
    steer_angle: float,
) -> None:
    payload = build_manual_control_by_id_payload(entity_id, throttle, brake, steer_angle)
    _send_packet(sock, request_id, proto.MSG_TYPE_MANUAL_CONTROL_BY_ID_COMMAND, payload)


def send_transform_control_by_id(
    sock: socket.socket,
    request_id: int,
    entity_id: str,
    pos_x: float, pos_y: float, pos_z: float,
    rot_x: float, rot_y: float, rot_z: float,
    steer_angle: float,
    speed: float,
) -> None:
    payload = build_transform_control_by_id_payload(
        entity_id, pos_x, pos_y, pos_z, rot_x, rot_y, rot_z, steer_angle, speed,
    )
    _send_packet(sock, request_id, proto.MSG_TYPE_TRANSFORM_CONTROL_BY_ID_COMMAND, payload)


def send_set_trajectory(
    sock: socket.socket,
    request_id: int,
    entity_id: str,
    follow_mode: int,
    trajectory_name: str,
    points: List[Tuple[float, float, float, float]],
) -> None:
    payload = build_set_trajectory_payload(entity_id, follow_mode, trajectory_name, points)
    _send_packet(sock, request_id, proto.MSG_TYPE_SET_TRAJECTORY_COMMAND, payload,
                 f"SetTrajectory(0x1304) id={entity_id} points={len(points)}")


def send_load_suite(sock: socket.socket, request_id: int, suite_path: str) -> None:
    payload = pack_message_payload(
        proto.MSG_TYPE_LOAD_SUITE,
        {"suite_path": suite_path},
    )
    _send_packet(sock, request_id, proto.MSG_TYPE_LOAD_SUITE, payload,
                 f"LoadSuite(0x1402) suite_path={suite_path}")


def send_scenario_status(sock: socket.socket, request_id: int) -> None:
    _send_packet(sock, request_id, proto.MSG_TYPE_SCENARIO_STATUS, b"",
                 "ScenarioStatus(0x1504)")


def send_scenario_control(
    sock: socket.socket,
    request_id: int,
    command: int,
    scenario_name: str = "",
) -> None:
    payload = pack_message_payload(
        proto.MSG_TYPE_SCENARIO_CONTROL,
        {
            "command": command,
            "scenario_name": scenario_name,
        },
    )
    _send_packet(sock, request_id, proto.MSG_TYPE_SCENARIO_CONTROL, payload,
                 f"ScenarioControl(0x1505) command={command} scenario_name={scenario_name!r}")


def send_active_suite_status(sock: socket.socket, request_id: int) -> None:
    _send_packet(sock, request_id, proto.MSG_TYPE_ACTIVE_SUITE_STATUS, b"",
                 "ActiveSuiteStatus(0x1401)")


# ============================================================
# Response parsers
# ============================================================

def parse_result_code(payload: bytes) -> Optional[Tuple[int, int]]:
    if len(payload) != proto.RESULT_SIZE:
        return None
    try:
        values, offset = unpack_fields(get_response_message(0x1201).fields, payload)
    except ValueError:
        return None
    if offset != len(payload):
        return None
    return values["result_code"], values["detail_code"]


def parse_get_status_payload(payload: bytes) -> Optional[Dict[str, Any]]:
    if len(payload) < proto.RESULT_SIZE + 4:
        return None
    try:
        mode = struct.unpack_from("<I", payload, offset=proto.RESULT_SIZE)[0]
    except struct.error:
        return None

    if mode == proto.TIME_MODE_VARIABLE:
        expected_size = proto.GET_STATUS_VARIABLE_SIZE
    elif mode == proto.TIME_MODE_FIXED:
        expected_size = proto.GET_STATUS_FIXED_SIZE
    else:
        return None

    if len(payload) != expected_size:
        return None

    try:
        values, _, offset = unpack_message_payload(0x1101, payload, direction="response")
    except ValueError:
        return None
    return values if offset == len(payload) else None


def parse_set_simulation_time_mode_payload(payload: bytes) -> Optional[Dict[str, Any]]:
    if len(payload) != proto.SET_SIM_TIME_MODE_RESP_SIZE:
        return None
    try:
        values, _, offset = unpack_message_payload(0x1102, payload, direction="response")
    except ValueError:
        return None
    return values if offset == len(payload) else None


def parse_create_object_payload(payload: bytes) -> Optional[Dict[str, Any]]:
    if len(payload) < proto.RESULT_SIZE + 4:
        return None
    try:
        values, _, offset = unpack_message_payload(0x1301, payload, direction="response")
    except ValueError:
        return None
    if offset != len(payload):
        return None
    values["object_id_length"] = len(values["object_id"].encode("utf-8"))
    return values


def parse_active_suite_status_payload(payload: bytes) -> Optional[Dict[str, Any]]:
    if len(payload) < proto.RESULT_SIZE + proto.ACTIVE_SUITE_STATUS_RESP_MIN_SIZE:
        return None

    try:
        values, repeated_items, offset = unpack_message_payload(
            0x1401,
            payload,
            direction="response",
            repeated_count_field="scenario_list_size",
        )
    except ValueError as e:
        print(f"[PARSE][ActiveSuiteStatus] {e}")
        return None

    if offset != len(payload):
        return None
    if values["result_code"] != 0:
        print(
            f"[PARSE][ActiveSuiteStatus] Server error: "
            f"result_code={values['result_code']} detail_code={values['detail_code']}"
        )
        return None

    return {
        "result_code": values["result_code"],
        "detail_code": values["detail_code"],
        "active_suite_name": values["active_suite_name"],
        "active_scenario_name": values["active_scenario_name"],
        "scenario_list_size": values["scenario_list_size"],
        "scenario_list": [item["scenario_list[].name"] for item in repeated_items],
    }


def parse_scenario_status_payload(payload: bytes) -> Optional[Dict[str, Any]]:
    try:
        values, _, offset = unpack_message_payload(0x1504, payload, direction="response")
    except ValueError as e:
        print(f"[PARSE][ScenarioStatus] {e}")
        return None
    return values if offset == len(payload) else None
