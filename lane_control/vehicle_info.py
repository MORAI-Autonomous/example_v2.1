from __future__ import annotations
# lane_control/vehicle_info.py
#
# VehicleInfoThread — Vehicle Info with Wheel UDP 수신 스레드
#   포트 9091(기본)에서 바이너리 패킷을 수신해 속도(m/s)를 저장한다.
#   data_cb(parsed: dict) 콜백으로 파싱 결과를 외부에 전달한다.

import socket
import threading

from receivers.vehicle_info_with_wheel_receiver import parse_vehicle_info_payload


class VehicleInfoThread(threading.Thread):
    """
    Vehicle Info with Wheel UDP 수신 → 최신 속도(m/s) 저장
    포트: 9091 (vehicle_info_with_wheel_receiver)
    """

    def __init__(self, ip: str = "127.0.0.1", port: int = 9091,
                 log_fn=None, data_cb=None):
        super().__init__(daemon=True, name="vi-recv")
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.settimeout(1.0)
        self._sock.bind((ip, port))
        self._running     = False
        self._lock        = threading.Lock()
        self._speed_mps:  float = 0.0
        self._speed_valid: bool = False   # 한 번이라도 수신됐는지
        self._log     = log_fn or (lambda msg, level="INFO": print(f"[VI] {msg}"))
        self._data_cb = data_cb           # fn(parsed: dict) — 파싱 결과 콜백

    def get_speed_mps(self) -> float:
        with self._lock:
            return self._speed_mps

    def is_valid(self) -> bool:
        with self._lock:
            return self._speed_valid

    def stop(self):
        self._running = False
        try:
            self._sock.close()
        except OSError:
            pass

    def run(self):
        self._running = True
        self._log(f"Vehicle Info 수신 시작 {self._sock.getsockname()}")
        while self._running:
            try:
                data, _ = self._sock.recvfrom(4096)
                parsed  = parse_vehicle_info_payload(data)
                if parsed:
                    v   = parsed["local_velocity"]
                    spd = (v["x"]**2 + v["y"]**2 + v["z"]**2) ** 0.5
                    with self._lock:
                        self._speed_mps   = spd
                        self._speed_valid = True
                    if self._data_cb:
                        try:
                            self._data_cb(parsed)
                        except Exception:
                            pass
            except socket.timeout:
                continue
            except OSError:
                break
        self._log("Vehicle Info 수신 종료")
