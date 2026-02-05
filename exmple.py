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

# =========================
# UDP Receiver Config (Vehicle Info)
# =========================
VEHICLE_INFO_IP = "0.0.0.0"
VEHICLE_INFO_PORT = 9092

# Vehicle Info payload (no header)
# int64 seconds, int32 nanos, char[24] id, float32 x18
VEHICLE_INFO_FMT = "<qi24s18f"
VEHICLE_INFO_SIZE = struct.calcsize(VEHICLE_INFO_FMT)  # 108 bytes


# ManualCommand payload: throttle, brake, steer (float64 x3) = 24 bytes
MANUAL_FMT = "<ddd"
MANUAL_SIZE = struct.calcsize(MANUAL_FMT)

# =========================
# Protocol (TCP header matches <BBIIIH)
# =========================
MAGIC = 0x4D        # M

MSG_CLASS_REQ = 0x01
MSG_CLASS_RESP = 0x02

MSG_TYPE_SAVE_DATA = 0x1101
MSG_TYPE_FIXED_STEP = 0x1200
MSG_TYPE_GET_STATUS = 0x1201

FLAG = 0

HEADER_FMT = "<BBIIIH"
HEADER_SIZE = struct.calcsize(HEADER_FMT)  # 16

RESULT_FMT = "<II"  # uint32 result_code, uint32 detail_code
RESULT_SIZE = struct.calcsize(RESULT_FMT)  # 8

STATUS_FMT = "<fQqI"  # float, uint64, int64, uint32
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

        header_type, msg_class, msg_type, payload_size, request_id, flag = struct.unpack(
            HEADER_FMT, header_bytes
        )

        if msg_class not in VALID_MSG_CLASSES:
            continue
        if msg_type not in VALID_MSG_TYPES:
            continue
        if payload_size > 1024 * 1024:
            continue

        return header_bytes


def recv_packet(sock: socket.socket):
    header_bytes = recv_header_synced(sock)

    header_type, msg_class, msg_type, payload_size, request_id, flag = struct.unpack(
        HEADER_FMT, header_bytes
    )

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
# =========================
def send_fixed_step(sock: socket.socket, request_id: int, step_count: int):
    payload = struct.pack("<I", step_count)  # uint32
    header = build_header(MSG_CLASS_REQ, MSG_TYPE_FIXED_STEP, len(payload), request_id, FLAG)
    sock.sendall(header + payload)
    print(f"[SEND][TCP] FixedStepCommand(0x1200) request_id={request_id}, step_count={step_count}")


def send_get_status(sock: socket.socket, request_id: int):
    payload = b""
    header = build_header(MSG_CLASS_REQ, MSG_TYPE_GET_STATUS, len(payload), request_id, FLAG)
    sock.sendall(header + payload)
    print(f"[SEND][TCP] GetStatusCommand(0x1201) request_id={request_id}")


def send_save_data(sock: socket.socket, request_id: int):
    payload = b""
    header = build_header(MSG_CLASS_REQ, MSG_TYPE_SAVE_DATA, len(payload), request_id, FLAG)
    sock.sendall(header + payload)
    print(f"[SEND][TCP] SaveDataCommand(0x1101) request_id={request_id}")


def parse_result_code(payload: bytes):
    if len(payload) != RESULT_SIZE:
        return None
    return struct.unpack(RESULT_FMT, payload)


# =========================
# ManualCommand (UDP, no header)
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

    result_code, detail_code = struct.unpack_from(RESULT_FMT, payload, 0)
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
# Vehicle Info (UDP Receiver, no header)
# =========================
def parse_vehicle_info_payload(data: bytes):
    if len(data) < VEHICLE_INFO_SIZE:
        return None

    seconds, nanos, raw_id, *floats = struct.unpack(VEHICLE_INFO_FMT, data[:VEHICLE_INFO_SIZE])

    vehicle_id = raw_id.split(b"\x00", 1)[0].decode("utf-8", errors="ignore")

    # floats[0:3] location, [3:6] rotation, [6:9] vel, [9:12] accel, [12:15] ang_vel, [15:18] control
    loc = floats[0:3]
    rot = floats[3:6]
    vel = floats[6:9]
    acc = floats[9:12]
    ang = floats[12:15]
    ctrl = floats[15:18]

    return {
        "seconds": seconds,
        "nanos": nanos,
        "id": vehicle_id,
        "location": {"x": loc[0], "y": loc[1], "z": loc[2]},
        "rotation": {"x": rot[0], "y": rot[1], "z": rot[2]},
        "local_velocity": {"x": vel[0], "y": vel[1], "z": vel[2]},
        "local_acceleration": {"x": acc[0], "y": acc[1], "z": acc[2]},
        "angular_velocity": {"x": ang[0], "y": ang[1], "z": ang[2]},
        "control": {"throttle": ctrl[0], "brake": ctrl[1], "steer_angle": ctrl[2]},
        "raw_size": len(data),
    }


class VehicleInfoReceiver(threading.Thread):
    """
    UDP 수신 전용(9092).
    - 너무 자주 찍히면 보기 힘드니 rate-limit(기본 0.2s)
    - 패킷이 들어올 때만 출력
    """
    def __init__(self, udp_sock: socket.socket, print_interval_sec: float = 0.2):
        super().__init__(daemon=True)
        self.udp_sock = udp_sock
        self.running = True
        self.print_interval_sec = print_interval_sec
        self._last_print_t = 0.0

    def stop(self):
        self.running = False

    def run(self):
        while self.running:
            try:
                data, addr = self.udp_sock.recvfrom(2048)  # 108B이므로 여유 있게
                parsed = parse_vehicle_info_payload(data)

                now = time.time()
                if parsed is None:
                    # 크기/구조가 다르면 디버그 용으로 최소 정보만
                    if now - self._last_print_t >= self.print_interval_sec:
                        self._last_print_t = now
                        print(f"[RECV][UDP][VehicleInfo] from={addr} invalid_size={len(data)} "
                              f"(expected>={VEHICLE_INFO_SIZE})")
                    continue

                # rate-limit 출력
                if now - self._last_print_t < self.print_interval_sec:
                    continue
                self._last_print_t = now

                loc = parsed["location"]
                vel = parsed["local_velocity"]
                ctrl = parsed["control"]

                print(f"[RECV][UDP][VehicleInfo:{VEHICLE_INFO_PORT}] id='{parsed['id']}' "
                      f"time={parsed['seconds']}s {parsed['nanos']}ns size={parsed['raw_size']}B")
                print(f"    loc=({loc['x']:.3f}, {loc['y']:.3f}, {loc['z']:.3f}) "
                      f"vel=({vel['x']:.3f}, {vel['y']:.3f}, {vel['z']:.3f}) "
                      f"ctrl=(thr={ctrl['throttle']:.3f}, brk={ctrl['brake']:.3f}, steer={ctrl['steer_angle']:.3f})")
                print("")

            except OSError as e:
                if self.running:
                    print(f"[UDP-VEHICLE-THREAD] stopped: {e}")
                break


def print_key_bindings():
    print("Press [1] : Send ManualCommand (24 bytes, no header) via UDP")
    print("Press [2] : Send Get Fixed Mode Status Command(0x1201) via TCP")
    print("Press [3] : Send FixedStepCommand(0x1200) via TCP")        
    print("Press [4] : Send Save Data Command(0x1101) via TCP")
    print(f"(UDP Vehicle Info Receiver running on port {VEHICLE_INFO_PORT}, no key needed)")
    print("Press [Q] : Quit\n")


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

                if msg_class == MSG_CLASS_RESP and msg_type == MSG_TYPE_FIXED_STEP:
                    rc = parse_result_code(payload)
                    if rc is None:
                        print(f"           ResultCode parse failed. payload_len={len(payload)}")
                    else:
                        result_code, detail_code = rc
                        print(f"           ResultCode: result_code={result_code} detail_code={detail_code}")

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

                with self.lock:
                    if request_id in self.pending:
                        sent_time = self.pending.pop(request_id)
                        elapsed_ms = (time.time() - sent_time) * 1000.0
                        print(f"           Matched pending request_id={request_id} (RTT={elapsed_ms:.1f} ms)")
                    else:
                        print("           (no pending entry for this request_id)")

                print("")

        except (ConnectionError, OSError) as e:
            print(f"[RECV-THREAD] stopped: {e}")
            self.running = False

            global tcp_sock, receiver
            tcp_sock.close()
            tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            tcp_sock, receiver = reconnect(tcp_sock, receiver, self.pending, self.lock)


def reconnect(tcp_sock, receiver, pending, lock):
    while True:
        try:
            print("[INFO] Attempting to reconnect...")
            time.sleep(5)
            tcp_sock.connect((TCP_SERVER_IP, TCP_SERVER_PORT))
            print("[INFO] Reconnected to the server.")

            receiver = Receiver(tcp_sock, pending, lock)
            receiver.start()

            print_key_bindings()
            return tcp_sock, receiver

        except ConnectionRefusedError as e:
            print(f"[ERROR] Reconnection failed: {e}. Retrying in 5 seconds...")
        except Exception as e:
            print(f"[ERROR] Unexpected error during reconnection: {e}. Retrying in 5 seconds...")


# =========================
# Main (Windows)
# =========================
global tcp_sock, receiver


def main():
    global tcp_sock, receiver

    tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    udp_send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Vehicle Info UDP recv socket
    udp_vehicle_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_vehicle_sock.bind((VEHICLE_INFO_IP, VEHICLE_INFO_PORT))

    pending = {}
    lock = threading.Lock()

    # start UDP vehicle receiver
    vehicle_receiver = VehicleInfoReceiver(udp_vehicle_sock, print_interval_sec=0.2)
    vehicle_receiver.start()

    try:
        tcp_sock.connect((TCP_SERVER_IP, TCP_SERVER_PORT))
        receiver = Receiver(tcp_sock, pending, lock)
        receiver.start()
        print("Connected.")
    except Exception as e:
        print(f"[ERROR] Initial connection failed: {e}")
        tcp_sock, receiver = reconnect(tcp_sock, None, pending, lock)

    request_id = 1

    print_key_bindings()

    manual_throttle = 1.0
    manual_brake = 0.0
    manual_steer = 0.0

    try:
        while True:
            if msvcrt.kbhit():
                key = msvcrt.getch().decode(errors="ignore").lower()

                if key == "q":
                    break

                try:
                    if key == "1":
                        send_manual_udp(udp_send_sock, manual_throttle, manual_brake, manual_steer)

                    elif key == "2":
                        with lock:
                            pending[request_id] = time.time()
                        send_get_status(tcp_sock, request_id)
                        request_id += 1

                    if key == "3":
                        step_count = 1
                        with lock:
                            pending[request_id] = time.time()
                        send_fixed_step(tcp_sock, request_id, step_count)
                        request_id += 1

                    elif key == "4":
                        with lock:
                            pending[request_id] = time.time()
                        send_save_data(tcp_sock, request_id)
                        request_id += 1

                except (ConnectionError, OSError):
                    print("[ERROR] Connection lost. Attempting to reconnect...")
                    tcp_sock.close()
                    tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    tcp_sock, receiver = reconnect(tcp_sock, receiver, pending, lock)

            time.sleep(0.01)

    finally:
        # stop threads
        try:
            receiver.stop()
        except Exception:
            pass

        try:
            vehicle_receiver.stop()
        except Exception:
            pass

        # close sockets
        try:
            tcp_sock.shutdown(socket.SHUT_RDWR)
        except Exception:
            pass
        try:
            tcp_sock.close()
        except Exception:
            pass

        try:
            udp_send_sock.close()
        except Exception:
            pass

        try:
            udp_vehicle_sock.close()
        except Exception:
            pass

        print("Disconnected.")


if __name__ == "__main__":
    main()
