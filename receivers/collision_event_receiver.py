# collision_event_receiver.py
import socket
import struct
import threading
import time

# =========================
# Config
# =========================
COLLISION_EVENT_IP = "127.0.0.1"
COLLISION_EVENT_PORT = 9094  # UE 송신 포트와 맞추세요

# 출력 옵션
PRINT_FULL_ITEM = True      # True: 모든 필드 출력 / False: 요약만
PRINT_INTERVAL_SEC = 0.0    # 0이면 rate-limit 없이 수신마다 출력, 0.05 등으로 제한 가능

# =========================
# Packet Format
# =========================
# Base: entity_id (char[24]) + collision_object_count (uint32)
COLLISION_BASE_FMT = "<24sI"
COLLISION_BASE_SIZE = struct.calcsize(COLLISION_BASE_FMT)  # 28

# Repeat:
# collision_object_id (char[24])
# object_type (ENUM -> uint32)  # 필요 시 I -> i
# seconds (int64), nanos (int32)
# floats x18
COLLISION_REPEAT_FMT = "<24sIqi18f"
COLLISION_REPEAT_SIZE = struct.calcsize(COLLISION_REPEAT_FMT)  # 112


def _decode_cstr24(raw: bytes) -> str:
    return raw.split(b"\x00", 1)[0].decode("utf-8", errors="ignore")


def _fmt_vec3(v: dict, prec: int = 3) -> str:
    return f"({v['x']:.{prec}f}, {v['y']:.{prec}f}, {v['z']:.{prec}f})"


def parse_collision_event_payload(data: bytes):
    if len(data) < COLLISION_BASE_SIZE:
        return None

    entity_raw, count = struct.unpack_from(COLLISION_BASE_FMT, data, 0)
    entity_id = _decode_cstr24(entity_raw)

    expected_min = COLLISION_BASE_SIZE + (count * COLLISION_REPEAT_SIZE)
    if len(data) < expected_min:
        return {
            "error": "size_mismatch",
            "entity_id": entity_id,
            "count": count,
            "raw_size": len(data),
            "expected_min": expected_min,
        }

    items = []
    offset = COLLISION_BASE_SIZE

    for _ in range(count):
        tup = struct.unpack_from(COLLISION_REPEAT_FMT, data, offset)
        offset += COLLISION_REPEAT_SIZE

        collision_object_raw = tup[0]
        object_type = tup[1]
        seconds = tup[2]
        nanos = tup[3]
        floats = tup[4:]  # 18 floats

        loc = floats[0:3]
        rot = floats[3:6]
        dim = floats[6:9]
        vel = floats[9:12]
        acc = floats[12:15]
        spec = floats[15:18]  # front, rear, wheel_base

        items.append({
            "collision_object_id": _decode_cstr24(collision_object_raw),
            "object_type": object_type,
            "collision_time": {"seconds": seconds, "nanos": nanos},
            "transform": {
                "location": {"x": loc[0], "y": loc[1], "z": loc[2]},
                "rotation": {"x": rot[0], "y": rot[1], "z": rot[2]},
            },
            "dimensions": {"length": dim[0], "width": dim[1], "height": dim[2]},
            "vehicle_state": {
                "velocity": {"x": vel[0], "y": vel[1], "z": vel[2]},
                "acceleration": {"x": acc[0], "y": acc[1], "z": acc[2]},
            },
            "vehicle_spec": {"overhang_front": spec[0], "overhang_rear": spec[1], "wheel_base": spec[2]},
        })

    return {
        "entity_id": entity_id,
        "count": count,
        "items": items,
        "raw_size": len(data),
    }


def print_collision_event(parsed: dict, addr):
    print(f"[RECV][CollisionEvent:{COLLISION_EVENT_PORT}] entity='{parsed['entity_id']}' "
          f"count={parsed['count']} size={parsed['raw_size']}B from={addr}")

    for i, it in enumerate(parsed["items"]):
        t = it["collision_time"]
        loc = it["transform"]["location"]
        rot = it["transform"]["rotation"]
        dim = it["dimensions"]
        vel = it["vehicle_state"]["velocity"]
        acc = it["vehicle_state"]["acceleration"]
        spec = it["vehicle_spec"]

        # 1) 기본 한 줄 요약
        print(f"  [{i}] obj='{it['collision_object_id']}' type={it['object_type']} "
              f"time={t['seconds']}s {t['nanos']}ns loc={_fmt_vec3(loc, 2)}")

        if not PRINT_FULL_ITEM:
            continue

        # 2) 상세 출력
        print(f"       rotation={_fmt_vec3(rot, 2)}")
        print(f"       dimensions=(L={dim['length']:.2f}, W={dim['width']:.2f}, H={dim['height']:.2f})")
        print(f"       velocity={_fmt_vec3(vel, 3)}")
        print(f"       acceleration={_fmt_vec3(acc, 3)}")
        print(f"       spec=(front={spec['overhang_front']:.2f}, rear={spec['overhang_rear']:.2f}, "
              f"wheel_base={spec['wheel_base']:.2f})")

    print("")


class CollisionEventReceiver(threading.Thread):
    def __init__(self, udp_sock: socket.socket, print_interval_sec: float):
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
                data, addr = self.udp_sock.recvfrom(65535)
                parsed = parse_collision_event_payload(data)

                if parsed is None:
                    print(f"[RECV][CollisionEvent:{COLLISION_EVENT_PORT}] invalid_size={len(data)} from={addr}")
                    continue

                if "error" in parsed:
                    print(f"[RECV][CollisionEvent:{COLLISION_EVENT_PORT}] SIZE_MISMATCH "
                          f"entity='{parsed['entity_id']}' count={parsed['count']} "
                          f"raw={parsed['raw_size']}B expected>={parsed['expected_min']}B from={addr}")
                    continue

                if self.print_interval_sec > 0.0:
                    now = time.time()
                    if now - self._last_print_t < self.print_interval_sec:
                        continue
                    self._last_print_t = now

                print_collision_event(parsed, addr)

            except OSError as e:
                if self.running:
                    print(f"[CollisionReceiver] stopped: {e}")
                break


def main():
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_sock.bind((COLLISION_EVENT_IP, COLLISION_EVENT_PORT))

    print(f"[INFO] Listening CollisionEvent UDP on {COLLISION_EVENT_IP}:{COLLISION_EVENT_PORT}")
    print(f"[INFO] Base={COLLISION_BASE_SIZE}B Repeat={COLLISION_REPEAT_SIZE}B "
          f"(RepeatFmt='{COLLISION_REPEAT_FMT}')")
    print(f"[INFO] PRINT_FULL_ITEM={PRINT_FULL_ITEM}, PRINT_INTERVAL_SEC={PRINT_INTERVAL_SEC}")
    print("[INFO] Ctrl+C to quit\n")

    receiver = CollisionEventReceiver(udp_sock, print_interval_sec=PRINT_INTERVAL_SEC)
    receiver.start()

    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[INFO] Stopping...")
    finally:
        receiver.stop()
        try:
            udp_sock.close()
        except Exception:
            pass
        print("[INFO] Closed.")


if __name__ == "__main__":
    main()