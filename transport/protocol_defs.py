from __future__ import annotations

import struct

from transport.message_schema import build_struct_format, fixed_fields, get_message

# ============================================================
# Network Config
# ============================================================

TCP_SERVER_IP   = "127.0.0.1"
TCP_SERVER_PORT = 20000

UDP_IP          = "127.0.0.1"
UDP_PORT        = 9090

UDP_IP_TR       = "127.0.0.1"
UDP_PORT_TR     = 9094


# ============================================================
# Protocol Constants
# ============================================================

LITTLE_ENDIAN = "<"
MAGIC         = 0x4D  # 'M'
FLAG          = 0

MSG_CLASS_REQ  = 0x01
MSG_CLASS_RESP = 0x02

VALID_MSG_CLASSES = {MSG_CLASS_REQ, MSG_CLASS_RESP}


# ============================================================
# Message Types
# ============================================================

# Simulation Time
MSG_TYPE_GET_SIMULATION_TIME_STATUS    = 0x1101
MSG_TYPE_SET_SIMULATION_TIME_MODE_COMMAND = 0x1102

# Fixed Step
MSG_TYPE_FIXED_STEP                    = 0x1201
MSG_TYPE_SAVE_DATA                     = 0x1202

# Object Control
MSG_TYPE_CREATE_OBJECT                 = 0x1301
MSG_TYPE_MANUAL_CONTROL_BY_ID_COMMAND  = 0x1302
MSG_TYPE_TRANSFORM_CONTROL_BY_ID_COMMAND = 0x1303
MSG_TYPE_SET_TRAJECTORY_COMMAND        = 0x1304

# Suite / Scenario
MSG_TYPE_ACTIVE_SUITE_STATUS           = 0x1401
MSG_TYPE_LOAD_SUITE                    = 0x1402
MSG_TYPE_SCENARIO_STATUS               = 0x1504
MSG_TYPE_SCENARIO_CONTROL              = 0x1505

VALID_MSG_TYPES = {
    MSG_TYPE_GET_SIMULATION_TIME_STATUS,
    MSG_TYPE_SET_SIMULATION_TIME_MODE_COMMAND,
    MSG_TYPE_FIXED_STEP,
    MSG_TYPE_SAVE_DATA,
    MSG_TYPE_CREATE_OBJECT,
    MSG_TYPE_MANUAL_CONTROL_BY_ID_COMMAND,
    MSG_TYPE_TRANSFORM_CONTROL_BY_ID_COMMAND,
    MSG_TYPE_SET_TRAJECTORY_COMMAND,
    MSG_TYPE_LOAD_SUITE,
    MSG_TYPE_SCENARIO_STATUS,
    MSG_TYPE_SCENARIO_CONTROL,
    MSG_TYPE_ACTIVE_SUITE_STATUS,
}


# ============================================================
# Simulation Time Mode
# ============================================================

TIME_MODE_VARIABLE    = 1
TIME_MODE_FIXED_DELTA = 2
TIME_MODE_FIXED_STEP  = 3

RESULT_CODE_MAP = {
    0: "OK",
    101: "Invalid State",
    102: "Invalid Param",
    200: "Failed",
    201: "Timeout",
    202: "Not Supported",
}

# ============================================================
# Packet Formats & Sizes
# ============================================================

_MSG_1102 = get_message(0x1102)
_MSG_1302 = get_message(0x1302)
_MSG_1303 = get_message(0x1303)
_MSG_1304 = get_message(0x1304)

# --- Header ---
HEADER_FMT  = "<BBIIIH"
HEADER_SIZE = struct.calcsize(HEADER_FMT)   # 16

# --- Common Result ---
RESULT_FMT  = "<II"                         # uint32 result_code, uint32 detail_code
RESULT_SIZE = struct.calcsize(RESULT_FMT)   # 8

# --- Simulation Status ---
STATUS_FMT  = "<IffQqI"                     # mode, speed, fixed_delta, step_index, seconds, nanos
STATUS_SIZE = struct.calcsize(STATUS_FMT)   # 32

GET_STATUS_PAYLOAD_SIZE = RESULT_SIZE + STATUS_SIZE  # 40

# --- Set Simulation Time Mode ---
SET_SIM_TIME_MODE_REQ_FMT   = build_struct_format(fixed_fields(_MSG_1102.fields), LITTLE_ENDIAN)
SET_SIM_TIME_MODE_REQ_SIZE  = struct.calcsize(SET_SIM_TIME_MODE_REQ_FMT)   # 12

SET_SIM_TIME_MODE_RESP_FMT  = LITTLE_ENDIAN + "IIIff"
SET_SIM_TIME_MODE_RESP_SIZE = struct.calcsize(SET_SIM_TIME_MODE_RESP_FMT)  # 20

# --- Manual Command (UDP, legacy) ---
MANUAL_FMT  = build_struct_format(fixed_fields(_MSG_1302.fields[1:]), LITTLE_ENDIAN)  # throttle, brake, steer
MANUAL_SIZE = struct.calcsize(MANUAL_FMT)   # 24

# --- Manual Control By ID ---
MANUAL_CONTROL_BY_ID_PREFIX_FMT  = LITTLE_ENDIAN + "I"
MANUAL_CONTROL_BY_ID_PREFIX_SIZE = struct.calcsize(MANUAL_CONTROL_BY_ID_PREFIX_FMT)  # 4

MANUAL_CONTROL_BY_ID_VALUES_FMT  = build_struct_format(fixed_fields(_MSG_1302.fields[1:]), LITTLE_ENDIAN)
MANUAL_CONTROL_BY_ID_VALUES_SIZE = struct.calcsize(MANUAL_CONTROL_BY_ID_VALUES_FMT)  # 24

MANUAL_CONTROL_BY_ID_MIN_SIZE = (
    MANUAL_CONTROL_BY_ID_PREFIX_SIZE +
    MANUAL_CONTROL_BY_ID_VALUES_SIZE
)  # 28

# --- Transform Control (legacy, no ID) ---
TRANSFORM_CONTROL_FMT  = build_struct_format(fixed_fields(_MSG_1303.fields[1:]), LITTLE_ENDIAN)
TRANSFORM_CONTROL_SIZE = struct.calcsize(TRANSFORM_CONTROL_FMT)  # 36

# --- Transform Control By ID ---
TRANSFORM_CONTROL_BY_ID_PREFIX_FMT  = LITTLE_ENDIAN + "I"
TRANSFORM_CONTROL_BY_ID_PREFIX_SIZE = struct.calcsize(TRANSFORM_CONTROL_BY_ID_PREFIX_FMT)  # 4

TRANSFORM_CONTROL_BY_ID_VALUES_FMT  = build_struct_format(fixed_fields(_MSG_1303.fields[1:]), LITTLE_ENDIAN)
TRANSFORM_CONTROL_BY_ID_VALUES_SIZE = struct.calcsize(TRANSFORM_CONTROL_BY_ID_VALUES_FMT)  # 36

TRANSFORM_CONTROL_BY_ID_MIN_SIZE = (
    TRANSFORM_CONTROL_BY_ID_PREFIX_SIZE +
    TRANSFORM_CONTROL_BY_ID_VALUES_SIZE
)  # 40

# --- Set Trajectory ---
#
# Layout:
#   int32  entity_id_size
#   bytes  entity_id (utf-8)
#   int32  follow_mode
#   int32  trajectory_name_size
#   bytes  trajectory_name (utf-8)
#   int32  point_count
#   repeat point_count × { double x, y, z, time }
#
SET_TRAJECTORY_PREFIX_FMT     = LITTLE_ENDIAN + "I"
SET_TRAJECTORY_PREFIX_SIZE    = struct.calcsize(SET_TRAJECTORY_PREFIX_FMT)      # 4

SET_TRAJECTORY_FOLLOW_MODE_FMT  = build_struct_format((_MSG_1304.fields[1],), LITTLE_ENDIAN)
SET_TRAJECTORY_FOLLOW_MODE_SIZE = struct.calcsize(SET_TRAJECTORY_FOLLOW_MODE_FMT)  # 4

SET_TRAJECTORY_NAME_SIZE_FMT  = LITTLE_ENDIAN + "I"
SET_TRAJECTORY_NAME_SIZE_SIZE = struct.calcsize(SET_TRAJECTORY_NAME_SIZE_FMT)   # 4

SET_TRAJECTORY_POINT_COUNT_FMT  = build_struct_format((_MSG_1304.fields[3],), LITTLE_ENDIAN)
SET_TRAJECTORY_POINT_COUNT_SIZE = struct.calcsize(SET_TRAJECTORY_POINT_COUNT_FMT)  # 4

SET_TRAJECTORY_POINT_FMT  = build_struct_format(_MSG_1304.repeat_fields, LITTLE_ENDIAN)
SET_TRAJECTORY_POINT_SIZE = struct.calcsize(SET_TRAJECTORY_POINT_FMT)           # 32

SET_TRAJECTORY_MIN_SIZE = (
    SET_TRAJECTORY_PREFIX_SIZE +
    SET_TRAJECTORY_FOLLOW_MODE_SIZE +
    SET_TRAJECTORY_NAME_SIZE_SIZE +
    SET_TRAJECTORY_POINT_COUNT_SIZE
)  # 16

# --- Active Suite Status ---
#
# Request  : payload 없음 (Header만 전송)
#
# Response layout:
#   uint32  active_suite_name_size
#   bytes   active_suite_name          (utf-8, active_suite_name_size bytes)
#   uint32  active_scenario_name_size
#   bytes   active_scenario_name       (utf-8, active_scenario_name_size bytes)
#   uint32  scenario_list_size         (리스트 항목 수)
#   repeat scenario_list_size × {
#       uint32  name_size
#       bytes   name                   (utf-8, name_size bytes)
#   }
#
# 고정 크기 필드만 정의 (string/list는 가변이므로 fmt 없음)
#
ACTIVE_SUITE_STATUS_STR_LEN_FMT  = LITTLE_ENDIAN + "I"   # uint32 length prefix 공용
ACTIVE_SUITE_STATUS_STR_LEN_SIZE = struct.calcsize(ACTIVE_SUITE_STATUS_STR_LEN_FMT)  # 4

ACTIVE_SUITE_STATUS_LIST_COUNT_FMT  = LITTLE_ENDIAN + "I"
ACTIVE_SUITE_STATUS_LIST_COUNT_SIZE = struct.calcsize(ACTIVE_SUITE_STATUS_LIST_COUNT_FMT)  # 4

# 최소 수신 크기:
#   active_suite_name_size(4) + active_scenario_name_size(4) + scenario_list_size(4)
#   = 12  (각 문자열 길이가 0이고 리스트가 비어 있을 때)
ACTIVE_SUITE_STATUS_RESP_MIN_SIZE = (
    ACTIVE_SUITE_STATUS_STR_LEN_SIZE +   # active_suite_name_size
    ACTIVE_SUITE_STATUS_STR_LEN_SIZE +   # active_scenario_name_size
    ACTIVE_SUITE_STATUS_LIST_COUNT_SIZE  # scenario_list_size
)  # 12


# ============================================================
# AutoCall Defaults
# ============================================================

MAX_CALL_NUM              = 1000
AUTO_TIMEOUT_SEC          = 2.0
AUTO_DELAY_BETWEEN_CMDS_SEC = 0.0  # 필요시 0.01 등으로 조절
