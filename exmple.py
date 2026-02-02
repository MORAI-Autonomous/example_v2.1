import socket
import struct
import threading
import time
import msvcrt

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

# ManualCommand payload: throttle, brake, steer (float64 x3) = 24 bytes
MANUAL_FMT = "<ddd"
MANUAL_SIZE = struct.calcsize(MANUAL_FMT)

# =========================
# Protocol (TCP header matches <BBIIIH)
# =========================
MAGIC = 0x4D

MSG_CLASS_REQ  = 0x01
MSG_CLASS_RESP = 0x02

MSG_TYPE_SAVE_DATA = 0x1101      # ✅ SaveDataCommand MsgType (서버와 동일하게 맞추기)
MSG_TYPE_FIXED_STEP = 0x1200
MSG_TYPE_GET_STATUS  = 0x1201

FLAG = 0

HEADER_FMT  = "<BBIIIH"
HEADER_SIZE = struct.calcsize(HEADER_FMT)  # 16

RESULT_FMT  = "<II"                        # uint32 result_code, uint32 detail_code
RESULT_SIZE = struct.calcsize(RESULT_FMT)  # 8

STATUS_FMT  = "<fQqI"   # float, uint64, int64, uint32
STATUS_SIZE = 24

GET_STATUS_PAYLOAD_SIZE = RESULT_SIZE + STATUS_SIZE  # 32

VALID_MSG_CLASSES = {MSG_CLASS_REQ, MSG_CLASS_RESP}
VALID_MSG_TYPES = {MSG_TYPE_FIXED_STEP, MSG_TYPE_GET_STATUS, MSG_TYPE_SAVE_DATA}


# =========================
# TCP Helpers
# =========================
def recv_exact(sock: socket.socket, n: int) -> bytes:
    """TCP stream에서 정확히 n바이트를 읽는다. 상대가 끊으면 예외."""
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("Socket closed by peer")
        buf.extend(chunk)
    return bytes(buf)

def recv_header_synced(sock: socket.socket) -> bytes:
    while True:
        b = recv_exact(sock, 1)
        if b[0] != MAGIC:
            continue

        rest = recv_exact(sock, HEADER_SIZE - 1)
        header_bytes = b + rest

        # 헤더 검증
        header_type, msg_class, msg_type, payload_size, request_id, flag = struct.unpack(HEADER_FMT, header_bytes)

        if msg_class not in VALID_MSG_CLASSES:
            continue
        if msg_type not in VALID_MSG_TYPES:
            continue
        if payload_size > 1024 * 1024:
            continue
        # flag가 반드시 0이라면:
        # if flag != 0: continue

        return header_bytes       


def recv_packet(sock: socket.socket):
    header_bytes = recv_header_synced(sock)

    #print(f"[DEBUG] header_raw: {header_bytes.hex()}")

    header_type, msg_class, msg_type, payload_size, request_id, flag = struct.unpack(
        HEADER_FMT, header_bytes
    )

    # payload_size sanity check (폭주 방지)
    if payload_size < 0 or payload_size > 1024 * 1024:
        raise ValueError(f"Invalid payload_size: {payload_size}")

    payload = recv_exact(sock, payload_size) if payload_size > 0 else b""
    return msg_class, msg_type, payload_size, request_id, flag, payload


def build_header(msg_class: int, msg_type: int, payload_size: int, request_id: int, flag: int = 0) -> bytes:
    return struct.pack(
        HEADER_FMT,
        MAGIC,
        msg_class,
        msg_type,
        payload_size,
        request_id,
        flag
    )


# =========================
# FixedStepCommand (0x1200)
# request payload: uint32 step_count
# response payload: ResultCode (uint32 result_code, uint32 detail_code)
# =========================
def send_fixed_step(sock: socket.socket, request_id: int, step_count: int):
    payload = struct.pack("<I", step_count)  # uint32
    header = build_header(MSG_CLASS_REQ, MSG_TYPE_FIXED_STEP, len(payload), request_id, FLAG)
    sock.sendall(header + payload)
    print(f"[SEND][TCP] FixedStepCommand(0x1200) request_id={request_id}, step_count={step_count}")

def send_get_status(sock: socket.socket, request_id: int):
    payload = b""  # No payload
    header = build_header(MSG_CLASS_REQ, MSG_TYPE_GET_STATUS, len(payload), request_id, FLAG)
    sock.sendall(header + payload)
    print(f"[SEND][TCP] GetStatusCommand(0x1201) request_id={request_id}")

def send_save_data(sock: socket.socket, request_id: int):
    payload = b""  # No payload
    header = build_header(MSG_CLASS_REQ, MSG_TYPE_SAVE_DATA, len(payload), request_id, FLAG)
    sock.sendall(header + payload)
    print(f"[SEND][TCP] SaveDataCommand(0x1101) request_id={request_id}")

def parse_result_code(payload: bytes):
    if len(payload) != RESULT_SIZE:
        return None
    return struct.unpack(RESULT_FMT, payload)


# =========================
# ManualCommand (UDP, no header)
# payload: float throttle, float brake, float steer
# =========================
def send_manual_udp(udp_sock: socket.socket, throttle: float, brake: float, steer: float):
    payload = struct.pack(MANUAL_FMT, throttle, brake, steer)

    if len(payload) != MANUAL_SIZE:
        raise RuntimeError(f"Manual payload size mismatch: {len(payload)} (expected {MANUAL_SIZE})")

    udp_sock.sendto(payload, (UDP_IP, UDP_PORT))
    print(f"[SEND][UDP] ManualCommand -> {UDP_IP}:{UDP_PORT} "
          f"(throttle={throttle:.3f}, brake={brake:.3f}, steer={steer:.3f}) "
          f"size={len(payload)}B")
    

def parse_get_status_payload(payload: bytes):
    if len(payload) != GET_STATUS_PAYLOAD_SIZE:
        return None

    # ResultCode (8B)
    result_code, detail_code = struct.unpack_from(RESULT_FMT, payload, 0)

    # Status (24B)
    fixed_delta, step_index, seconds, nanos = struct.unpack_from(STATUS_FMT, payload, RESULT_SIZE)

    return {
        "result_code": result_code,
        "detail_code": detail_code,
        "fixed_delta": fixed_delta,
        "step_index": step_index,
        "seconds": seconds,
        "nanos": nanos,
    }


# =========================
# Receiver Thread (TCP)
# =========================
class Receiver(threading.Thread):
    def __init__(self, sock: socket.socket, pending: dict, lock: threading.Lock):
        super().__init__(daemon=True)
        self.sock = sock
        self.pending = pending
        self.lock = lock
        self.running = True

    def stop(self):
        self.running = False

    def run(self):
        try:
            while self.running:
                msg_class, msg_type, payload_size, request_id, flag, payload = recv_packet(self.sock)

                print(f"[RECV][TCP] msg_type=0x{msg_type:04X} "
                      f"request_id={request_id} payload_size={payload_size}")
                
                if msg_class == MSG_CLASS_RESP and msg_type == MSG_TYPE_SAVE_DATA:
                    rc = parse_result_code(payload)
                    if rc is None:
                        print(f"           SaveData ResultCode parse failed. payload_len={len(payload)} (expected {RESULT_SIZE})")
                    else:
                        result_code, detail_code = rc
                        print(f"           SaveData ResultCode: result_code={result_code} detail_code={detail_code}")

                # 0x1200 응답 처리
                if msg_class == MSG_CLASS_RESP and msg_type == MSG_TYPE_FIXED_STEP:
                    rc = parse_result_code(payload)
                    if rc is None:
                        print(f"           ResultCode parse failed. payload_len={len(payload)}")
                    else:
                        result_code, detail_code = rc
                        print(f"           ResultCode: result_code={result_code} detail_code={detail_code}")

                
                # 0x1201 응답 처리 (ResultCode + Status)
                if msg_class == MSG_CLASS_RESP and msg_type == MSG_TYPE_GET_STATUS:
                    parsed = parse_get_status_payload(payload)
                    if parsed is None:
                        print(f"           GetStatus parse failed. payload_len={len(payload)} "
                              f"(expected {GET_STATUS_PAYLOAD_SIZE})")
                    else:
                        print(f"           ResultCode: result_code={parsed['result_code']} detail_code={parsed['detail_code']}")
                        print(f"           Status: fixed_delta={parsed['fixed_delta']:.6f} "
                              f"step_index={parsed['step_index']} "
                              f"sim_time={parsed['seconds']}s {parsed['nanos']}ns")

                # pending 정리 (요청/응답 매칭)
                with self.lock:
                    if request_id in self.pending:
                        sent_time = self.pending.pop(request_id)
                        elapsed_ms = (time.time() - sent_time) * 1000.0
                        print(f"           Matched pending request_id={request_id} (RTT={elapsed_ms:.1f} ms)")
                    else:
                        print("           (no pending entry for this request_id)")

                print("")

        except Exception as e:
            print(f"[RECV-THREAD] stopped: {e}")


# =========================
# Main (Windows)
# =========================
def main():
    # TCP connect (Fixed Step Mode Control)
    tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_sock.connect((TCP_SERVER_IP, TCP_SERVER_PORT))

    # UDP socket (Manual Command)
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    pending = {}
    lock = threading.Lock()

    receiver = Receiver(tcp_sock, pending, lock)
    receiver.start()

    request_id = 1

    print("Connected.")
    print("Press [1] : Send FixedStepCommand(0x1200) via TCP")
    print("Press [2] : Send ManualCommand (24 bytes, no header) via UDP")
    print("Press [3] : Send Get Fixed Mode Status Command(0x1201) via TCP")
    print("Press [4] : Send Save Data Command(0x1101) via TCP")
    print("Press [Q] : Quit\n")

    # 기본 Manual 값 (원하면 여기만 바꿔도 됨)
    manual_throttle = 1.0
    manual_brake = 0.0
    manual_steer = 0.0

    try:
        while True:
            if msvcrt.kbhit():
                key = msvcrt.getch().decode(errors="ignore").lower()

                if key == "q":
                    break

                if key == "1":
                    step_count = 10
                    with lock:
                        # if len(pending) > 0:
                        #     print("[INFO] Pending request exists. Wait for response before sending next FixedStepCommand.")
                        #     continue
                        pending[request_id] = time.time()
                    send_fixed_step(tcp_sock, request_id, step_count)
                    request_id += 1                

                elif key == "2":
                    # UDP는 request_id/pending/응답처리 없음
                    send_manual_udp(udp_sock, manual_throttle, manual_brake, manual_steer)

                elif key == "3":
                    with lock:
                        pending[request_id] = time.time()
                    send_get_status(tcp_sock, request_id)
                    request_id += 1
                elif key == "4":
                    with lock:
                        pending[request_id] = time.time()
                    send_save_data(tcp_sock, request_id)
                    request_id += 1

            time.sleep(0.01)

    finally:
        receiver.stop()
        try:
            tcp_sock.shutdown(socket.SHUT_RDWR)
        except Exception:
            pass
        tcp_sock.close()
        udp_sock.close()
        print("Disconnected.")


if __name__ == "__main__":
    main()
