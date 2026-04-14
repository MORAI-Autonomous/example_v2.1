# ad_runner.py
#
# autonomous_driving 모듈 기반 단독 자율주행 실행 스크립트
# 제어 명령: TCP ManualControlById (0x1302)
#
# 실행:
#   python ad_runner.py
#   python ad_runner.py --ego-port 9091 --tcp-ip 127.0.0.1 --tcp-port 20000
#
# 종료: Ctrl+C

import argparse
import itertools
import socket
import threading
import time
import numpy as np

import transport.tcp_transport as tcp
import transport.protocol_defs as proto
from receivers.vehicle_info_with_wheel_receiver import parse_vehicle_info_payload
from autonomous_driving.autonomous_driving import AutonomousDriving
from autonomous_driving.vehicle_state import VehicleState


# ─── 조향각 최대값 (rad) — 노말라이즈 기준 ───────────────────────────
MAX_STEER_RAD = 0.5   # pure pursuit 출력을 -1~1 로 변환할 때 사용

# ─── Request ID 카운터 ────────────────────────────────────────────
_rid_iter = itertools.count(1)

def _next_rid() -> int:
    return next(_rid_iter)


# ─── Runner ──────────────────────────────────────────────────────
class AdRunner:
    def __init__(
        self,
        tcp_sock:  socket.socket,
        entity_id: str,
        vi_ip:     str,
        vi_port:   int,
        path_file: str = 'path_link.csv',
        log_fn=None,
    ):
        # UDP 수신 소켓 (Vehicle Info)
        self._recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._recv_sock.settimeout(2.0)
        self._recv_sock.bind((vi_ip, vi_port))

        # TCP 소켓 (제어 명령 송신)
        self._tcp_sock  = tcp_sock
        self._entity_id = entity_id

        # 자율주행 모듈
        self._ad = AutonomousDriving(path_file)

        self._running = False
        self._lock    = threading.Lock()
        self._latest  = None
        self._log     = log_fn or (lambda msg, level="INFO": print(f"[AD] {msg}"))

        self._log(f"Vehicle Info 수신 : {vi_ip}:{vi_port}")
        self._log(f"TCP 제어          : entity_id={entity_id}")

    def start(self):
        """논블로킹 시작 — 수신/제어 루프를 각각 데몬 스레드로 실행."""
        self._running = True
        threading.Thread(target=self._recv_loop,    daemon=True).start()
        threading.Thread(target=self._control_loop, daemon=True).start()

    def stop(self):
        self._running = False
        try:
            self._recv_sock.close()
        except Exception:
            pass

    # ── UDP 수신 스레드 ──────────────────────────────────────────
    def _recv_loop(self):
        while self._running:
            try:
                data, _ = self._recv_sock.recvfrom(65535)
                parsed = parse_vehicle_info_payload(data)
                if parsed:
                    with self._lock:
                        self._latest = parsed
            except socket.timeout:
                continue
            except OSError:
                break

    # ── 제어 루프 (30Hz) ─────────────────────────────────────────
    def _control_loop(self):
        sampling_time = 1.0 / 30.0
        self._log("주행 시작")

        while self._running:
            t_start = time.perf_counter()

            with self._lock:
                parsed = self._latest

            if parsed:
                vehicle_state = VehicleState(
                    x        = parsed["location"]["x"],
                    y        = parsed["location"]["y"],
                    yaw      = np.deg2rad(parsed["rotation"]["z"]),
                    velocity = parsed["local_velocity"]["x"] / 3.6,
                )

                try:
                    control_input, _ = self._ad.execute(vehicle_state)
                    steer_norm = float(np.clip(
                        control_input.steering / MAX_STEER_RAD, -1.0, 1.0
                    ))

                    tcp.send_manual_control_by_id(
                        self._tcp_sock,
                        _next_rid(),
                        entity_id   = self._entity_id,
                        throttle    = control_input.accel,
                        brake       = control_input.brake,
                        steer_angle = steer_norm,
                    )

                    self._log(
                        f"pos=({vehicle_state.position.x:.1f}, {vehicle_state.position.y:.1f})  "
                        f"vel={vehicle_state.velocity*3.6:.1f}km/h  "
                        f"accel={control_input.accel:.3f}  brake={control_input.brake:.3f}  "
                        f"steer={steer_norm:.3f}"
                    )
                except Exception as e:
                    self._log(f"ERROR: {e}", "ERROR")
            else:
                self._log("차량 상태 대기 중...", "INFO")

            elapsed = time.perf_counter() - t_start
            sleep_t = sampling_time - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

        self._log("주행 종료")


# ─── 진입점 ──────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="autonomous_driving 단독 실행 스크립트")
    parser.add_argument("--tcp-ip",    default=proto.TCP_SERVER_IP,
                        help=f"시뮬레이터 TCP IP (기본: {proto.TCP_SERVER_IP})")
    parser.add_argument("--tcp-port",  type=int, default=proto.TCP_SERVER_PORT,
                        help=f"시뮬레이터 TCP 포트 (기본: {proto.TCP_SERVER_PORT})")
    parser.add_argument("--ego-ip",    default="127.0.0.1",
                        help="EgoInfo UDP 수신 IP (기본: 127.0.0.1)")
    parser.add_argument("--ego-port",  type=int, default=9091,
                        help="EgoInfo UDP 수신 포트 (기본: 9091)")
    parser.add_argument("--entity-id", default="Car_1",
                        help="제어할 차량 entity_id (기본: Car_1)")
    args = parser.parse_args()

    # TCP 연결
    print(f"[AD] TCP 연결 중 → {args.tcp_ip}:{args.tcp_port} ...")
    tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    try:
        tcp_sock.connect((args.tcp_ip, args.tcp_port))
    except OSError as e:
        print(f"[AD] TCP 연결 실패: {e}")
        return
    print("[AD] TCP 연결 성공")

    runner = AdRunner(
        tcp_sock  = tcp_sock,
        entity_id = args.entity_id,
        ego_ip    = args.ego_ip,
        ego_port  = args.ego_port,
    )

    try:
        runner.start()
    except KeyboardInterrupt:
        print("\n[AD] 종료 중...")
    finally:
        runner.stop()
        tcp_sock.close()
        print("[AD] 종료 완료")


if __name__ == "__main__":
    main()
