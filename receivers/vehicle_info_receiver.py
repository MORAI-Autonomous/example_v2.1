from __future__ import annotations

# vehicle_info_receiver.py
import socket
import struct
import threading
import time

# =========================
# UDP Receiver Config (Vehicle Info)
# =========================
VEHICLE_INFO_IP = "127.0.0.1"
VEHICLE_INFO_PORT = 9097

# Vehicle Info payload (no header)
# int64 seconds, int32 nanos, char[24] id, float32 x18
VEHICLE_INFO_FMT = "<qi24s18f"
VEHICLE_INFO_SIZE = struct.calcsize(VEHICLE_INFO_FMT)  # 108 bytes

PRINT_INTERVAL_SEC = 0.2  # 출력 rate-limit (0이면 매 패킷 출력)


def _decode_cstr24(raw: bytes) -> str:
    return raw.split(b"\x00", 1)[0].decode("utf-8", errors="ignore")


def parse_vehicle_info_payload(data: bytes):
    if len(data) < VEHICLE_INFO_SIZE:
        return None

    seconds, nanos, raw_id, *floats = struct.unpack(VEHICLE_INFO_FMT, data[:VEHICLE_INFO_SIZE])
    vehicle_id = _decode_cstr24(raw_id)

    # floats mapping (원본 example.py 기준)
    # [0:3] location, [3:6] rotation, [6:9] vel, [9:12] accel, [12:15] ang_vel, [15:18] control
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
