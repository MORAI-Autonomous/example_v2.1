import struct

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
SET_SIM_TIME_MODE_REQ_FMT   = LITTLE_ENDIAN + "Iff"
SET_SIM_TIME_MODE_REQ_SIZE  = struct.calcsize(SET_SIM_TIME_MODE_REQ_FMT)   # 12

SET_SIM_TIME_MODE_RESP_FMT  = LITTLE_ENDIAN + "IIIff"
SET_SIM_TIME_MODE_RESP_SIZE = struct.calcsize(SET_SIM_TIME_MODE_RESP_FMT)  # 20

# --- Manual Command (UDP, legacy) ---
MANUAL_FMT  = "<ddd"                        # throttle, brake, steer (float64 x3)
MANUAL_SIZE = struct.calcsize(MANUAL_FMT)   # 24

# --- Manual Control By ID ---
MANUAL_CONTROL_BY_ID_PREFIX_FMT  = LITTLE_ENDIAN + "I"
MANUAL_CONTROL_BY_ID_PREFIX_SIZE = struct.calcsize(MANUAL_CONTROL_BY_ID_PREFIX_FMT)  # 4

MANUAL_CONTROL_BY_ID_VALUES_FMT  = LITTLE_ENDIAN + "ddd"
MANUAL_CONTROL_BY_ID_VALUES_SIZE = struct.calcsize(MANUAL_CONTROL_BY_ID_VALUES_FMT)  # 24

MANUAL_CONTROL_BY_ID_MIN_SIZE = (
    MANUAL_CONTROL_BY_ID_PREFIX_SIZE +
    MANUAL_CONTROL_BY_ID_VALUES_SIZE
)  # 28

# --- Transform Control (legacy, no ID) ---
TRANSFORM_CONTROL_FMT  = "<fffffff"         # pos(x,y,z), rot(x,y,z), steer_angle
TRANSFORM_CONTROL_SIZE = struct.calcsize(TRANSFORM_CONTROL_FMT)  # 28

# --- Transform Control By ID ---
TRANSFORM_CONTROL_BY_ID_PREFIX_FMT  = LITTLE_ENDIAN + "I"
TRANSFORM_CONTROL_BY_ID_PREFIX_SIZE = struct.calcsize(TRANSFORM_CONTROL_BY_ID_PREFIX_FMT)  # 4

TRANSFORM_CONTROL_BY_ID_VALUES_FMT  = LITTLE_ENDIAN + "fffffff"
TRANSFORM_CONTROL_BY_ID_VALUES_SIZE = struct.calcsize(TRANSFORM_CONTROL_BY_ID_VALUES_FMT)  # 28

TRANSFORM_CONTROL_BY_ID_MIN_SIZE = (
    TRANSFORM_CONTROL_BY_ID_PREFIX_SIZE +
    TRANSFORM_CONTROL_BY_ID_VALUES_SIZE
)  # 32

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

SET_TRAJECTORY_FOLLOW_MODE_FMT  = LITTLE_ENDIAN + "i"
SET_TRAJECTORY_FOLLOW_MODE_SIZE = struct.calcsize(SET_TRAJECTORY_FOLLOW_MODE_FMT)  # 4

SET_TRAJECTORY_NAME_SIZE_FMT  = LITTLE_ENDIAN + "I"
SET_TRAJECTORY_NAME_SIZE_SIZE = struct.calcsize(SET_TRAJECTORY_NAME_SIZE_FMT)   # 4

SET_TRAJECTORY_POINT_COUNT_FMT  = LITTLE_ENDIAN + "I"
SET_TRAJECTORY_POINT_COUNT_SIZE = struct.calcsize(SET_TRAJECTORY_POINT_COUNT_FMT)  # 4

SET_TRAJECTORY_POINT_FMT  = LITTLE_ENDIAN + "dddd"
SET_TRAJECTORY_POINT_SIZE = struct.calcsize(SET_TRAJECTORY_POINT_FMT)           # 32

SET_TRAJECTORY_MIN_SIZE = (
    SET_TRAJECTORY_PREFIX_SIZE +
    SET_TRAJECTORY_FOLLOW_MODE_SIZE +
    SET_TRAJECTORY_NAME_SIZE_SIZE +
    SET_TRAJECTORY_POINT_COUNT_SIZE
)  # 16


# ============================================================
# AutoCall Defaults
# ============================================================

MAX_CALL_NUM              = 5000
AUTO_TIMEOUT_SEC          = 2.0
AUTO_DELAY_BETWEEN_CMDS_SEC = 0.0  # 필요시 0.01 등으로 조절