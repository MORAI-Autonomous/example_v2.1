# tcp_transport.py
import socket
import struct
from typing import Any, Dict, List, Optional, Tuple

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
    """Stream에서 MAGIC 바이트로 동기화 후 유효한 헤더를 반환."""
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
    """(msg_class, msg_type, payload_size, request_id, flag, payload)"""
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
):
    """헤더 빌드 → sendall → 로그 출력을 한 곳에서 처리."""
    header = build_header(proto.MSG_CLASS_REQ, msg_type, len(payload), request_id, proto.FLAG)
    sock.sendall(header + payload)
    if log:
        print(f"[SEND][TCP] {log} rid={request_id}")


def _pack_str(s: str) -> bytes:
    """길이-접두 UTF-8 인코딩 (uint32 len + bytes)."""
    b = s.encode("utf-8")
    return struct.pack("<I", len(b)) + b


def _unpack_str(payload: bytes, offset: int) -> Tuple[str, int]:
    """offset 위치에서 uint32 길이 + UTF-8 문자열을 읽고 (문자열, 다음 offset)을 반환."""
    if offset + proto.ACTIVE_SUITE_STATUS_STR_LEN_SIZE > len(payload):
        raise ValueError(f"_unpack_str: length 필드 읽기 실패 (offset={offset}, buf={len(payload)})")
    (str_len,) = struct.unpack_from(proto.ACTIVE_SUITE_STATUS_STR_LEN_FMT, payload, offset)
    offset += proto.ACTIVE_SUITE_STATUS_STR_LEN_SIZE

    end = offset + str_len
    if end > len(payload):
        raise ValueError(f"_unpack_str: 문자열 데이터 부족 (need={str_len}, remain={len(payload)-offset})")
    text = payload[offset:end].decode("utf-8", errors="replace")
    return text, end


# ============================================================
# Payload builders
# ============================================================

def build_manual_control_by_id_payload(
    entity_id: str,
    throttle: float,
    brake: float,
    steer_angle: float,
) -> bytes:
    return (
        _pack_str(entity_id)
        + struct.pack(proto.MANUAL_CONTROL_BY_ID_VALUES_FMT, throttle, brake, steer_angle)
    )


def build_transform_control_by_id_payload(
    entity_id: str,
    pos_x: float, pos_y: float, pos_z: float,
    rot_x: float, rot_y: float, rot_z: float,
    steer_angle: float,
) -> bytes:
    return (
        _pack_str(entity_id)
        + struct.pack(
            proto.TRANSFORM_CONTROL_BY_ID_VALUES_FMT,
            pos_x, pos_y, pos_z,
            rot_x, rot_y, rot_z,
            steer_angle,
        )
    )


def build_set_trajectory_payload(
    entity_id: str,
    follow_mode: int,
    trajectory_name: str,
    points: List[Tuple[float, float, float, float]],  # (x, y, z, time)
) -> bytes:
    point_data = b"".join(struct.pack("<dddd", x, y, z, t) for x, y, z, t in points)
    return (
        _pack_str(entity_id)
        + struct.pack("<i", follow_mode)
        + _pack_str(trajectory_name)
        + struct.pack("<I", len(points))
        + point_data
    )


# ============================================================
# Send commands
# ============================================================

def send_get_status(sock: socket.socket, request_id: int):
    _send_packet(sock, request_id, proto.MSG_TYPE_GET_SIMULATION_TIME_STATUS, b"",
                 "GetStatus(0x1101)")


def send_simulation_time_mode_command(
    sock: socket.socket,
    request_id: int,
    mode: int,
    fixed_delta: float,
    simulation_speed: float = 1.0,
):
    """mode: 1=variable, 2=fixed_delta, 3=fixed_step"""
    payload = struct.pack(proto.SET_SIM_TIME_MODE_REQ_FMT, mode, fixed_delta, simulation_speed)
    _send_packet(sock, request_id, proto.MSG_TYPE_SET_SIMULATION_TIME_MODE_COMMAND, payload,
                 f"SetSimulationTimeModeCommand(0x1102) mode={mode} fixed_delta={fixed_delta} speed={simulation_speed}")


def send_fixed_step(sock: socket.socket, request_id: int, step_count: int):
    payload = struct.pack("<I", step_count)
    _send_packet(sock, request_id, proto.MSG_TYPE_FIXED_STEP, payload)


def send_save_data(sock: socket.socket, request_id: int):
    _send_packet(sock, request_id, proto.MSG_TYPE_SAVE_DATA, b"")


def send_create_object(
    sock: socket.socket,
    request_id: int,
    entity_type: int,
    pos_x: float, pos_y: float, pos_z: float,
    rot_x: float, rot_y: float, rot_z: float,
    driving_mode: int,
    ground_vehicle_model: int,
):
    """payload: int32 entity_type, float pos×3, float rot×3, int32 driving_mode, int32 model"""
    payload = struct.pack(
        "<i fff fff ii",
        entity_type,
        pos_x, pos_y, pos_z,
        rot_x, rot_y, rot_z,
        driving_mode, ground_vehicle_model,
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
):
    payload = build_manual_control_by_id_payload(entity_id, throttle, brake, steer_angle)
    _send_packet(sock, request_id, proto.MSG_TYPE_MANUAL_CONTROL_BY_ID_COMMAND, payload,
                 f"ManualControlById(0x1302) id={entity_id} throttle={throttle} brake={brake} steer={steer_angle}")


def send_transform_control_by_id(
    sock: socket.socket,
    request_id: int,
    entity_id: str,
    pos_x: float, pos_y: float, pos_z: float,
    rot_x: float, rot_y: float, rot_z: float,
    steer_angle: float,
):
    payload = build_transform_control_by_id_payload(
        entity_id, pos_x, pos_y, pos_z, rot_x, rot_y, rot_z, steer_angle,
    )
    _send_packet(sock, request_id, proto.MSG_TYPE_TRANSFORM_CONTROL_BY_ID_COMMAND, payload,
                 f"TransformControlById(0x1303) id={entity_id} "
                 f"pos=({pos_x:.2f},{pos_y:.2f},{pos_z:.2f}) "
                 f"rot=({rot_x:.2f},{rot_y:.2f},{rot_z:.2f}) steer={steer_angle:.2f}")


def send_set_trajectory(
    sock: socket.socket,
    request_id: int,
    entity_id: str,
    follow_mode: int,
    trajectory_name: str,
    points: List[Tuple[float, float, float, float]],
):
    payload = build_set_trajectory_payload(entity_id, follow_mode, trajectory_name, points)
    _send_packet(sock, request_id, proto.MSG_TYPE_SET_TRAJECTORY_COMMAND, payload,
                 f"SetTrajectory(0x1304) id={entity_id} points={len(points)}")


def send_load_suite(sock: socket.socket, request_id: int, suite_path: str):
    payload = _pack_str(suite_path)
    _send_packet(sock, request_id, proto.MSG_TYPE_LOAD_SUITE, payload,
                 f"LoadSuite(0x1402) suite_path={suite_path}")


def send_scenario_status(sock: socket.socket, request_id: int):
    _send_packet(sock, request_id, proto.MSG_TYPE_SCENARIO_STATUS, b"",
                 "ScenarioStatus(0x1504)")


def send_scenario_control(sock: socket.socket, request_id: int, command: int, scenario_name: str = ""):
    payload = struct.pack("<I", command) + _pack_str(scenario_name)
    _send_packet(sock, request_id, proto.MSG_TYPE_SCENARIO_CONTROL, payload,
                 f"ScenarioControl(0x1505) command={command} scenario_name={scenario_name!r}")


def send_active_suite_status(sock: socket.socket, request_id: int):
    """payload 없이 헤더만 전송."""
    _send_packet(sock, request_id, proto.MSG_TYPE_ACTIVE_SUITE_STATUS, b"",
                 "ActiveSuiteStatus(0x1401)")


# ============================================================
# Response parsers
# ============================================================

def parse_result_code(payload: bytes) -> Optional[Tuple[int, int]]:
    if len(payload) != proto.RESULT_SIZE:
        return None
    return struct.unpack(proto.RESULT_FMT, payload)


def parse_get_status_payload(payload: bytes) -> Optional[Dict[str, Any]]:
    if len(payload) < proto.GET_STATUS_PAYLOAD_SIZE:
        return None
    result_code, detail_code = struct.unpack_from(proto.RESULT_FMT, payload, 0)
    mode, fixed_delta, simulation_speed, step_index, seconds, nanos = struct.unpack_from(
        proto.STATUS_FMT, payload, proto.RESULT_SIZE
    )
    return {
        "result_code": result_code, "detail_code": detail_code,
        "mode": mode, "fixed_delta": fixed_delta,
        "simulation_speed": simulation_speed,
        "step_index": step_index, "seconds": seconds, "nanos": nanos,
    }


def parse_set_simulation_time_mode_payload(payload: bytes) -> Optional[Dict[str, Any]]:
    if len(payload) < proto.SET_SIM_TIME_MODE_RESP_SIZE:
        return None
    result_code, detail_code, mode, fixed_delta, simulation_speed = struct.unpack(
        proto.SET_SIM_TIME_MODE_RESP_FMT,
        payload[:proto.SET_SIM_TIME_MODE_RESP_SIZE],
    )
    return {
        "result_code": result_code, "detail_code": detail_code,
        "mode": mode, "fixed_delta": fixed_delta,
        "simulation_speed": simulation_speed,
    }


def parse_create_object_payload(payload: bytes) -> Optional[Dict[str, Any]]:
    """ResultCode + uint32 object_id_length + bytes object_id (utf-8)"""
    if len(payload) < proto.RESULT_SIZE + 4:
        return None
    result_code, detail_code = struct.unpack_from(proto.RESULT_FMT, payload, 0)
    (object_id_length,) = struct.unpack_from("<I", payload, proto.RESULT_SIZE)

    expected = proto.RESULT_SIZE + 4 + object_id_length
    if len(payload) != expected:
        return None

    object_id = payload[proto.RESULT_SIZE + 4:expected].decode("utf-8", errors="replace")
    return {
        "result_code": result_code, "detail_code": detail_code,
        "object_id_length": object_id_length, "object_id": object_id,
    }


def parse_active_suite_status_payload(payload: bytes) -> Optional[Dict[str, Any]]:
    """
    Response layout:
        uint32  active_suite_name_size
        bytes   active_suite_name
        uint32  active_scenario_name_size
        bytes   active_scenario_name
        uint32  scenario_list_size
        repeat scenario_list_size × {
            uint32  name_size
            bytes   name
        }

    Returns:
        {
            "active_suite_name":    str,
            "active_scenario_name": str,
            "scenario_list":        List[str],
        }
        or None if payload is malformed.
    """
    # ResultCode (result_code: uint32 + detail_code: uint32) 를 먼저 건너뜀
    if len(payload) < proto.RESULT_SIZE + proto.ACTIVE_SUITE_STATUS_RESP_MIN_SIZE:
        return None

    result_code, detail_code = struct.unpack_from(proto.RESULT_FMT, payload, 0)
    if result_code != 0:
        print(f"[PARSE][ActiveSuiteStatus] Server error: result_code={result_code} detail_code={detail_code}")
        return None

    try:
        offset = proto.RESULT_SIZE  # 8바이트 skip

        # 1) active_suite_name
        active_suite_name, offset = _unpack_str(payload, offset)

        # 2) active_scenario_name
        active_scenario_name, offset = _unpack_str(payload, offset)

        # 3) scenario_list_size
        if offset + proto.ACTIVE_SUITE_STATUS_LIST_COUNT_SIZE > len(payload):
            return None
        (scenario_list_size,) = struct.unpack_from(
            proto.ACTIVE_SUITE_STATUS_LIST_COUNT_FMT, payload, offset
        )
        offset += proto.ACTIVE_SUITE_STATUS_LIST_COUNT_SIZE

        # 4) scenario_list 항목 순회
        scenario_list: List[str] = []
        for i in range(scenario_list_size):
            name, offset = _unpack_str(payload, offset)
            scenario_list.append(name)

    except ValueError as e:
        print(f"[PARSE][ActiveSuiteStatus] 파싱 오류: {e}")
        return None

    return {
        "result_code":          result_code,
        "detail_code":          detail_code,
        "active_suite_name":    active_suite_name,
        "active_scenario_name": active_scenario_name,
        "scenario_list":        scenario_list,
    }

def parse_scenario_status_payload(payload: bytes) -> dict | None:
    try:
        # payload = [16]~[27], 총 12바이트
        # result_code(4) + detail_code(4) + state(4)
        result_code  = struct.unpack_from("<I", payload, offset=0)[0]
        detail_code  = struct.unpack_from("<I", payload, offset=4)[0]
        state        = struct.unpack_from("<I", payload, offset=8)[0]
        return {
            "result_code": result_code,
            "detail_code": detail_code,
            "state": state,
        }
    except Exception as e:
        print(f"[PARSE][ScenarioStatus] {e}")
        return None