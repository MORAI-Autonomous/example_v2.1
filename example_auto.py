import socket
import struct
import threading
import time
import sys

# =========================
# TCP Server Config Fixed Step Mode Control
# =========================
TCP_SERVER_IP = "127.0.0.1"
TCP_SERVER_PORT = 9091

# =========================
# UDP Sender Config (Manual Command)
# =========================
UDP_IP = "127.0.0.1"
UDP_PORT = 9090

# =========================
# UDP Receiver Config (Vehicle Info)
# =========================
VEHICLE_INFO_IP = "0.0.0.0"
VEHICLE_INFO_PORT = 9098

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
MAGIC = 0x4D  # 'M'

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

MAX_CALL_NUM = 500

# =========================
# Key Input
# =========================
if sys.platform == "win32":
    import msvcrt

    def get_key() -> str:
        ch = msvcrt.getch()
        try:
            return ch.decode("utf-8", errors="ignore").lower()
        except Exception:
            return ""
else:
    import termios
    import tty

    def get_key() -> str:
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
            return ch.lower()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)


# =========================
# TCP Helpers
# =========================
def recv_exact(sock: socket.socket, n: int) -> bytes:
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
# Commands
# =========================
def send_fixed_step(sock: socket.socket, request_id: int, step_count: int):
    payload = struct.pack("<I", step_count)  # uint32
    header = build_header(MSG_CLASS_REQ, MSG_TYPE_FIXED_STEP, len(payload), request_id, FLAG)
    sock.sendall(header + payload)
    #print(f"[SEND][TCP] FixedStep(0x1200) rid={request_id} step_count={step_count}")


def send_get_status(sock: socket.socket, request_id: int):
    payload = b""
    header = build_header(MSG_CLASS_REQ, MSG_TYPE_GET_STATUS, len(payload), request_id, FLAG)
    sock.sendall(header + payload)
    print(f"[SEND][TCP] GetStatus(0x1201) rid={request_id}")


def send_save_data(sock: socket.socket, request_id: int):
    payload = b""
    header = build_header(MSG_CLASS_REQ, MSG_TYPE_SAVE_DATA, len(payload), request_id, FLAG)
    sock.sendall(header + payload)
    #print(f"[SEND][TCP] SaveData(0x1101) rid={request_id}")


def parse_result_code(payload: bytes):
    if len(payload) != RESULT_SIZE:
        return None
    return struct.unpack(RESULT_FMT, payload)


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
# ManualCommand (UDP)
# =========================
def send_manual_udp(udp_sock: socket.socket, throttle: float, brake: float, steer: float):
    payload = struct.pack(MANUAL_FMT, throttle, brake, steer)
    if len(payload) != MANUAL_SIZE:
        raise RuntimeError(f"Manual payload size mismatch: {len(payload)} (expected {MANUAL_SIZE})")

    udp_sock.sendto(payload, (UDP_IP, UDP_PORT))
    print(f"[SEND][UDP] ManualCommand -> {UDP_IP}:{UDP_PORT} "
          f"(thr={throttle:.3f}, brk={brake:.3f}, steer={steer:.3f}) size={len(payload)}B")


# =========================
# Vehicle Info (UDP Receiver)
# =========================
def parse_vehicle_info_payload(data: bytes):
    if len(data) < VEHICLE_INFO_SIZE:
        return None

    seconds, nanos, raw_id, *floats = struct.unpack(VEHICLE_INFO_FMT, data[:VEHICLE_INFO_SIZE])
    vehicle_id = raw_id.split(b"\x00", 1)[0].decode("utf-8", errors="ignore")

    loc = floats[0:3]
    vel = floats[6:9]
    ctrl = floats[15:18]

    return {
        "seconds": seconds,
        "nanos": nanos,
        "id": vehicle_id,
        "location": {"x": loc[0], "y": loc[1], "z": loc[2]},
        "local_velocity": {"x": vel[0], "y": vel[1], "z": vel[2]},
        "control": {"throttle": ctrl[0], "brake": ctrl[1], "steer_angle": ctrl[2]},
        "raw_size": len(data),
    }


class VehicleInfoReceiver(threading.Thread):
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
                data, addr = self.udp_sock.recvfrom(2048)
                parsed = parse_vehicle_info_payload(data)

                now = time.time()
                if parsed is None:
                    if now - self._last_print_t >= self.print_interval_sec:
                        self._last_print_t = now
                        print(f"[RECV][UDP][VehicleInfo] from={addr} invalid_size={len(data)} (expected>={VEHICLE_INFO_SIZE})")
                    continue

                if now - self._last_print_t < self.print_interval_sec:
                    continue
                self._last_print_t = now

                loc = parsed["location"]
                vel = parsed["local_velocity"]
                ctrl = parsed["control"]

                # print(f"[RECV][UDP][VehicleInfo:{VEHICLE_INFO_PORT}] id='{parsed['id']}' "
                #       f"time={parsed['seconds']}s {parsed['nanos']}ns size={parsed['raw_size']}B")
                # print(f"    loc=({loc['x']:.3f}, {loc['y']:.3f}, {loc['z']:.3f}) "
                #       f"vel=({vel['x']:.3f}, {vel['y']:.3f}, {vel['z']:.3f}) "
                #       f"ctrl=(thr={ctrl['throttle']:.3f}, brk={ctrl['brake']:.3f}, steer={ctrl['steer_angle']:.3f})")
                # print("")
            except OSError as e:
                if self.running:
                    print(f"[UDP-VEHICLE-THREAD] stopped: {e}")
                break


# =========================
# Pending (request sync)
# key: (request_id, msg_type) -> {"t": float, "ev": Event}
# =========================
def pending_add(pending: dict, lock: threading.Lock, request_id: int, msg_type: int) -> threading.Event:
    ev = threading.Event()
    with lock:
        pending[(request_id, msg_type)] = {"t": time.time(), "ev": ev}
    return ev


def pending_pop(pending: dict, lock: threading.Lock, request_id: int, msg_type: int):
    with lock:
        pending.pop((request_id, msg_type), None)


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

                #print(f"[RECV][TCP] msg_type=0x{msg_type:04X} rid={request_id} payload_size={payload_size}")

                # parse
                if msg_class == MSG_CLASS_RESP and msg_type == MSG_TYPE_SAVE_DATA:
                    rc = parse_result_code(payload)
                    # if rc is None:
                    #     print(f"           SaveData parse failed. payload_len={len(payload)} (expected {RESULT_SIZE})")
                    # else:
                    #     result_code, detail_code = rc
                    #     #print(f"           SaveData ResultCode: result_code={result_code} detail_code={detail_code}")

                elif msg_class == MSG_CLASS_RESP and msg_type == MSG_TYPE_FIXED_STEP:
                    rc = parse_result_code(payload)
                    # if rc is None:
                    #     print(f"           FixedStep parse failed. payload_len={len(payload)} (expected {RESULT_SIZE})")
                    # else:
                    #     result_code, detail_code = rc
                    #     #print(f"           FixedStep ResultCode: result_code={result_code} detail_code={detail_code}")

                elif msg_class == MSG_CLASS_RESP and msg_type == MSG_TYPE_GET_STATUS:
                    parsed = parse_get_status_payload(payload)
                    # if parsed is None:
                    #     print(f"           GetStatus parse failed. payload_len={len(payload)} (expected {GET_STATUS_PAYLOAD_SIZE})")
                    # else:
                    #     print(f"           ResultCode: result_code={parsed['result_code']} detail_code={parsed['detail_code']}")
                    #     print(f"           Status: fixed_delta={parsed['fixed_delta']:.6f} step_index={parsed['step_index']} "
                    #           f"sim_time={parsed['seconds']}s {parsed['nanos']}ns")

                # sync signal
                if msg_class == MSG_CLASS_RESP:
                    key = (request_id, msg_type)
                    with self.lock:
                        item = self.pending.get(key)
                        if item is not None:
                            rtt_ms = (time.time() - item["t"]) * 1000.0
                            #print(f"           Matched pending key={key} (RTT={rtt_ms:.1f} ms)")
                            item["ev"].set()
                        #else:
                            #print(f"           (no pending entry for key={key})")

                #print("")

        except (ConnectionError, OSError) as e:
            print(f"[RECV-THREAD] stopped: {e}")
            self.running = False


# =========================
# AutoCaller Thread (FixedStep <-> SaveData)
# =========================
class AutoCaller(threading.Thread):
    def __init__(
        self,
        tcp_sock: socket.socket,
        pending: dict,
        lock: threading.Lock,
        request_id_ref: dict,
        max_calls: int,
        step_count: int = 1,
        timeout_sec: float = 2.0,
    ):
        super().__init__(daemon=True)
        self.tcp_sock = tcp_sock
        self.pending = pending
        self.lock = lock
        self.request_id_ref = request_id_ref  # {"value": int}
        self.max_calls = max_calls
        self.step_count = step_count
        self.timeout_sec = timeout_sec
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    def _next_rid(self) -> int:
        with self.lock:
            rid = self.request_id_ref["value"]
            self.request_id_ref["value"] += 1
        return rid

    def run(self):
        target_steps = MAX_CALL_NUM  # '스텝 횟수' 기준
        print("[AUTO] started. target_steps=", target_steps)

        for i in range(target_steps):
            if self._stop.is_set():
                break

            # FixedStep
            rid_step = self._next_rid()
            ev_step = pending_add(self.pending, self.lock, rid_step, MSG_TYPE_FIXED_STEP)
            send_fixed_step(self.tcp_sock, rid_step, step_count=self.step_count)
            if not ev_step.wait(self.timeout_sec):
                #print(f"[AUTO][TIMEOUT] FixedStep resp timeout. i={i} rid={rid_step}")
                pending_pop(self.pending, self.lock, rid_step, MSG_TYPE_FIXED_STEP)
                break
            pending_pop(self.pending, self.lock, rid_step, MSG_TYPE_FIXED_STEP)

            if self._stop.is_set():
                break

            # SaveData
            rid_save = self._next_rid()
            ev_save = pending_add(self.pending, self.lock, rid_save, MSG_TYPE_SAVE_DATA)
            send_save_data(self.tcp_sock, rid_save)
            if not ev_save.wait(self.timeout_sec):
                #print(f"[AUTO][TIMEOUT] SaveData resp timeout. i={i} rid={rid_save}")
                pending_pop(self.pending, self.lock, rid_save, MSG_TYPE_SAVE_DATA)
                break
            pending_pop(self.pending, self.lock, rid_save, MSG_TYPE_SAVE_DATA)

            # 진행 로그 (원하면 50마다)
            if (i + 1) % 50 == 0:
                print(f"[AUTO] steps done: {i+1}/{target_steps}")

        print("[AUTO] stopped.")


# =========================
# UI
# =========================
def print_key_bindings():
    print("Press [1] : Send ManualCommand (UDP)")
    print("Press [2] : Send GetStatus (TCP 0x1201)")
    print("Press [3] : Send FixedStep (TCP 0x1200)")
    print("Press [4] : Send SaveData (TCP 0x1101)")
    print("Press [5] : Toggle AutoCall (FixedStep <-> SaveData with response sync)")
    print(f"(UDP Vehicle Info Receiver running on port {VEHICLE_INFO_PORT})")
    print("Press [Q] : Quit\n")


# =========================
# Main
# =========================
def main():
    tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    udp_send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    udp_vehicle_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_vehicle_sock.bind((VEHICLE_INFO_IP, VEHICLE_INFO_PORT))

    pending = {}
    lock = threading.Lock()
    request_id_ref = {"value": 1}

    vehicle_receiver = VehicleInfoReceiver(udp_vehicle_sock, print_interval_sec=0.2)
    vehicle_receiver.start()

    receiver = None
    auto = None

    try:
        tcp_sock.connect((TCP_SERVER_IP, TCP_SERVER_PORT))
        receiver = Receiver(tcp_sock, pending, lock)
        receiver.start()
        print("Connected.")
    except Exception as e:
        print(f"[ERROR] Initial connection failed: {e}")
        return

    print_key_bindings()

    manual_throttle = 1
    manual_brake = 0.0
    manual_steer = 0.0

    try:
        while True:
            key = get_key()
            if key == "q":
                break

            try:
                if key == "1":
                    send_manual_udp(udp_send_sock, manual_throttle, manual_brake, manual_steer)

                elif key == "2":
                    rid = None
                    with lock:
                        rid = request_id_ref["value"]
                        request_id_ref["value"] += 1
                    pending_add(pending, lock, rid, MSG_TYPE_GET_STATUS)
                    send_get_status(tcp_sock, rid)

                elif key == "3":
                    with lock:
                        rid = request_id_ref["value"]
                        request_id_ref["value"] += 1
                    pending_add(pending, lock, rid, MSG_TYPE_FIXED_STEP)
                    send_fixed_step(tcp_sock, rid, step_count=1)

                elif key == "4":
                    with lock:
                        rid = request_id_ref["value"]
                        request_id_ref["value"] += 1
                    pending_add(pending, lock, rid, MSG_TYPE_SAVE_DATA)
                    send_save_data(tcp_sock, rid)

                elif key == "5":
                    # toggle auto
                    if auto is None or not auto.is_alive():
                        auto = AutoCaller(
                            tcp_sock=tcp_sock,
                            pending=pending,
                            lock=lock,
                            request_id_ref=request_id_ref,
                            max_calls=MAX_CALL_NUM,
                            step_count=1,
                            timeout_sec=2.0,
                        )
                        auto.start()
                    else:
                        auto.stop()
                        auto = None

            except (ConnectionError, OSError) as e:
                print(f"[ERROR] Connection lost: {e}")
                # stop auto on disconnect
                if auto is not None and auto.is_alive():
                    auto.stop()
                    auto = None
                break

            time.sleep(0.001)

    finally:
        if auto is not None and auto.is_alive():
            auto.stop()

        if receiver is not None:
            try:
                receiver.stop()
            except Exception:
                pass

        try:
            vehicle_receiver.stop()
        except Exception:
            pass

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