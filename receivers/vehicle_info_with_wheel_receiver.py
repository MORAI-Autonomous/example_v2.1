# vehicle_info_receiver.py
import socket
import struct
import threading
import time

# =========================
# UDP Receiver Config (Vehicle Info)
# =========================
VEHICLE_INFO_IP = "127.0.0.1"
VEHICLE_INFO_PORT = 9091

# Vehicle Info base payload (no header)
# int64 seconds, int32 nanos, char[24] id, float32 x18
VEHICLE_INFO_FMT = "<qi24s18f"
VEHICLE_INFO_SIZE = struct.calcsize(VEHICLE_INFO_FMT)  # 108 bytes

# Extra tail payload:
# int32 wheel_count, then wheel_count * (float32 x,y,z)
WHEEL_COUNT_FMT = "<i"
WHEEL_COUNT_SIZE = struct.calcsize(WHEEL_COUNT_FMT)  # 4 bytes
WHEEL_VEC3_FMT = "<3f"
WHEEL_VEC3_SIZE = struct.calcsize(WHEEL_VEC3_FMT)    # 12 bytes

# 안전장치: 비정상 패킷이 wheel_count를 크게 찍는 경우 방어
MAX_WHEEL_COUNT = 32

PRINT_INTERVAL_SEC = 0.2  # 출력 rate-limit (0이면 매 패킷 출력)


def _decode_cstr24(raw: bytes) -> str:
    return raw.split(b"\x00", 1)[0].decode("utf-8", errors="ignore")


def parse_vehicle_info_payload(data: bytes):
    # 1) base payload 검사
    if len(data) < VEHICLE_INFO_SIZE:
        return None

    # 2) base payload 파싱
    seconds, nanos, raw_id, *floats = struct.unpack_from(VEHICLE_INFO_FMT, data, 0)
    vehicle_id = _decode_cstr24(raw_id)

    # floats mapping (원본 example.py 기준)
    # [0:3] location, [3:6] rotation, [6:9] vel, [9:12] accel, [12:15] ang_vel, [15:18] control
    loc = floats[0:3]
    rot = floats[3:6]
    vel = floats[6:9]
    acc = floats[9:12]
    ang = floats[12:15]
    ctrl = floats[15:18]

    out = {
        "seconds": seconds,
        "nanos": nanos,
        "id": vehicle_id,
        "location": {"x": loc[0], "y": loc[1], "z": loc[2]},
        "rotation": {"x": rot[0], "y": rot[1], "z": rot[2]},
        "local_velocity": {"x": vel[0], "y": vel[1], "z": vel[2]},
        "local_acceleration": {"x": acc[0], "y": acc[1], "z": acc[2]},
        "angular_velocity": {"x": ang[0], "y": ang[1], "z": ang[2]},
        "control": {"throttle": ctrl[0], "brake": ctrl[1], "steer_angle": ctrl[2]},
        "wheels": [],  # [{x,y,z}, ...]
        "wheel_count": 0,
        "raw_size": len(data),
    }

    # 3) wheel tail이 없을 수도 있으니, 남은 바이트 체크
    offset = VEHICLE_INFO_SIZE
    if len(data) < offset + WHEEL_COUNT_SIZE:
        # wheel 정보가 없는 패킷으로 간주 (기존 호환)
        return out

    # 4) wheel_count 파싱
    (wheel_count,) = struct.unpack_from(WHEEL_COUNT_FMT, data, offset)
    offset += WHEEL_COUNT_SIZE

    # 방어 로직
    if wheel_count < 0 or wheel_count > MAX_WHEEL_COUNT:
        # wheel_count가 비정상 -> 이 패킷은 wheel 파싱을 포기하고 base만 반환
        # (원하면 None 리턴으로 바꿔도 됨)
        out["wheel_count"] = wheel_count
        return out

    need_bytes = wheel_count * WHEEL_VEC3_SIZE
    if len(data) < offset + need_bytes:
        # wheel_count는 있는데 실제 데이터가 부족한 경우
        # base + wheel_count까지만 신뢰하고 반환
        out["wheel_count"] = wheel_count
        return out

    # 5) wheel vec3 배열 파싱
    wheels = []
    for i in range(wheel_count):
        x, y, z = struct.unpack_from(WHEEL_VEC3_FMT, data, offset)
        offset += WHEEL_VEC3_SIZE
        wheels.append({"x": x, "y": y, "z": z})

    out["wheel_count"] = wheel_count
    out["wheels"] = wheels
    return out


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
                # wheel 데이터까지 고려해서 버퍼를 더 크게 잡는 게 안전
                data, addr = self.udp_sock.recvfrom(4096)
                parsed = parse_vehicle_info_payload(data)

                now = time.time()
                if parsed is None:
                    if self.print_interval_sec <= 0.0 or (now - self._last_print_t >= self.print_interval_sec):
                        self._last_print_t = now
                        print(
                            f"[RECV][UDP][VehicleInfo:{VEHICLE_INFO_PORT}] invalid_size={len(data)} "
                            f"(expected>={VEHICLE_INFO_SIZE}) from={addr}"
                        )
                    continue

                if self.print_interval_sec > 0.0 and (now - self._last_print_t < self.print_interval_sec):
                    continue
                self._last_print_t = now

                loc = parsed["location"]
                rot = parsed["rotation"]
                vel = parsed["local_velocity"]
                acc = parsed["local_acceleration"]
                ang = parsed["angular_velocity"]
                ctrl = parsed["control"]

                print(
                    f"[RECV][UDP][VehicleInfo:{VEHICLE_INFO_PORT}] id='{parsed['id']}' "
                    f"time={parsed['seconds']}s {parsed['nanos']}ns size={parsed['raw_size']}B from={addr}"
                )
                print(
                    f"    loc=({loc['x']:.3f}, {loc['y']:.3f}, {loc['z']:.3f}) "
                    f"rot=({rot['x']:.3f}, {rot['y']:.3f}, {rot['z']:.3f})"
                )
                print(
                    f"    vel=({vel['x']:.3f}, {vel['y']:.3f}, {vel['z']:.3f}) "
                    f"acc=({acc['x']:.3f}, {acc['y']:.3f}, {acc['z']:.3f}) "
                    f"ang=({ang['x']:.3f}, {ang['y']:.3f}, {ang['z']:.3f})"
                )
                print(
                    f"    ctrl=(thr={ctrl['throttle']:.3f}, brk={ctrl['brake']:.3f}, steer={ctrl['steer_angle']:.3f})"
                )

                wc = parsed.get("wheel_count", 0)
                wheels = parsed.get("wheels", [])
                print(f"    wheels: count={wc} parsed={len(wheels)}")
                for i, w in enumerate(wheels):
                    print(f"        [{i}] world_loc=({w['x']:.3f}, {w['y']:.3f}, {w['z']:.3f})")

                print("")

            except OSError as e:
                if self.running:
                    print(f"[VehicleInfoReceiver] stopped: {e}")
                break


def main():
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_sock.bind((VEHICLE_INFO_IP, VEHICLE_INFO_PORT))

    print(f"[INFO] Listening VehicleInfo UDP on {VEHICLE_INFO_IP}:{VEHICLE_INFO_PORT}")
    print(f"[INFO] VEHICLE_INFO_SIZE={VEHICLE_INFO_SIZE}B fmt='{VEHICLE_INFO_FMT}'")
    print("[INFO] tail: int32 wheel_count + wheel_count*(float32 x,y,z)")
    print(f"[INFO] PRINT_INTERVAL_SEC={PRINT_INTERVAL_SEC} (0=print every packet)")
    print("[INFO] Ctrl+C to quit\n")

    receiver = VehicleInfoReceiver(udp_sock, print_interval_sec=PRINT_INTERVAL_SEC)
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