# protocol_defs.py
import struct

# =========================
# TCP Server Config Fixed Step Mode Control
# =========================
TCP_SERVER_IP = "127.0.0.1"
TCP_SERVER_PORT = 9093

# =========================
# UDP Sender Config (Manual Command)
# =========================
UDP_IP = "127.0.0.1"
UDP_PORT = 9090

UDP_IP_TR = "127.0.0.1"
UDP_PORT_TR = 9094

# ManualCommand payload: throttle, brake, steer (float64 x3) = 24 bytes
MANUAL_FMT = "<ddd"
MANUAL_SIZE = struct.calcsize(MANUAL_FMT)

# =========================
# Protocol (TCP header matches <BBIIIH)
# =========================
LITTLE_ENDIAN = "<"
MAGIC = 0x4D  # 'M'

MSG_CLASS_REQ = 0x01
MSG_CLASS_RESP = 0x02

MSG_TYPE_SAVE_DATA = 0x1101
MSG_TYPE_CREATE_OBJECT = 0x1103
MSG_TYPE_MANUAL_CONTROL_BY_ID_COMMAND = 0x1104
MSG_TYPE_TRANSFORM_CONTROL_BY_ID_COMMAND = 0x1105

MSG_TYPE_FIXED_STEP = 0x1200
MSG_TYPE_GET_STATUS = 0x1201

FLAG = 0

HEADER_FMT = "<BBIIIH"
HEADER_SIZE = struct.calcsize(HEADER_FMT)  # 16

RESULT_FMT = "<II"  # uint32 result_code, uint32 detail_code
RESULT_SIZE = struct.calcsize(RESULT_FMT)  # 8

STATUS_FMT = "<fQqI"  # float fixed_delta, uint64 step_index, int64 seconds, uint32 nanos
STATUS_SIZE = struct.calcsize(STATUS_FMT)  # 24

GET_STATUS_PAYLOAD_SIZE = RESULT_SIZE + STATUS_SIZE  # 32

# =========================
# 0x1104 ManualControlById payload
# uint32 id_str_size
# bytes  str_id (utf-8)
# double throttle
# double brake
# double steer_angle
# =========================
MANUAL_CONTROL_BY_ID_PREFIX_FMT = LITTLE_ENDIAN + "I"
MANUAL_CONTROL_BY_ID_PREFIX_SIZE = struct.calcsize(MANUAL_CONTROL_BY_ID_PREFIX_FMT)  # 4

MANUAL_CONTROL_BY_ID_VALUES_FMT = LITTLE_ENDIAN + "ddd"
MANUAL_CONTROL_BY_ID_VALUES_SIZE = struct.calcsize(MANUAL_CONTROL_BY_ID_VALUES_FMT)  # 24

MANUAL_CONTROL_BY_ID_MIN_SIZE = (
    MANUAL_CONTROL_BY_ID_PREFIX_SIZE +
    MANUAL_CONTROL_BY_ID_VALUES_SIZE
)  # 28 (id 길이 0일 때 최소)

VALID_MSG_CLASSES = {MSG_CLASS_REQ, MSG_CLASS_RESP}
VALID_MSG_TYPES = {
    MSG_TYPE_FIXED_STEP, 
    MSG_TYPE_GET_STATUS, 
    MSG_TYPE_SAVE_DATA, 
    MSG_TYPE_CREATE_OBJECT, 
    MSG_TYPE_MANUAL_CONTROL_BY_ID_COMMAND,
    MSG_TYPE_TRANSFORM_CONTROL_BY_ID_COMMAND}

# =========================
# AutoCall defaults
# =========================
MAX_CALL_NUM = 1000
AUTO_TIMEOUT_SEC = 2.0
AUTO_DELAY_BETWEEN_CMDS_SEC = 0.0  # 0.01 등으로 조절 가능


# Transform control payload
# pos(x,y,z) rot(x,y,z) steer_angle
TRANSFORM_CONTROL_FMT = "<fffffff"
TRANSFORM_CONTROL_SIZE = struct.calcsize(TRANSFORM_CONTROL_FMT)  # 28

# =========================
# 0x1105 TransformControlById payload
# uint32 id_str_size
# bytes  str_id (utf-8)
# float pos_x, pos_y, pos_z
# float rot_x, rot_y, rot_z
# float steer_angle
# =========================
TRANSFORM_CONTROL_BY_ID_PREFIX_FMT = LITTLE_ENDIAN + "I"
TRANSFORM_CONTROL_BY_ID_PREFIX_SIZE = struct.calcsize(TRANSFORM_CONTROL_BY_ID_PREFIX_FMT)  # 4

TRANSFORM_CONTROL_BY_ID_VALUES_FMT = LITTLE_ENDIAN + "fffffff"
TRANSFORM_CONTROL_BY_ID_VALUES_SIZE = struct.calcsize(TRANSFORM_CONTROL_BY_ID_VALUES_FMT)  # 28

TRANSFORM_CONTROL_BY_ID_MIN_SIZE = (
    TRANSFORM_CONTROL_BY_ID_PREFIX_SIZE +
    TRANSFORM_CONTROL_BY_ID_VALUES_SIZE
)  # 32 (id 길이 0일 때 최소)