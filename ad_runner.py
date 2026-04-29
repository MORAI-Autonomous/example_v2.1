from __future__ import annotations

# ad_runner.py
#
# autonomous_driving 모듈 기반 단독 자율주행 실행 스크립트
# 제어 명령: TCP ManualControlById (0x1302)
#
# 충돌 모드(is_chaser=True) 시:
#   - AutonomousDriving 대신 target 방향 추적 + 고정 스로틀
#   - target 위치는 모듈 수준 _shared_positions 레지스트리 경유

import argparse
import itertools
import socket
import threading
import time
import numpy as np

import transport.tcp_transport as tcp
import transport.protocol_defs as proto
from receivers.vehicle_info_receiver import parse_vehicle_info_payload
from autonomous_driving.autonomous_driving import AutonomousDriving
from autonomous_driving.vehicle_state import VehicleState


# ─── 조향각 최대값 (rad) ────────────────────────────────────────
MAX_STEER_RAD = 0.5

# ─── Request ID 카운터 ──────────────────────────────────────────
_rid_iter = itertools.count(1)

def _next_rid() -> int:
    return next(_rid_iter)


# ─── 속도 비례 제어 ──────────────────────────────────────────────
_SPEED_GAIN = 0.1   # throttle·brake per kph error

def _speed_ctrl(current_kph: float, target_kph: float):
    """현재 속도와 목표 속도 차이로 throttle / brake 계산."""
    err = target_kph - current_kph
    if err > 0:
        return float(np.clip(err * _SPEED_GAIN, 0.0, 1.0)), 0.0
    else:
        return 0.0, float(np.clip(-err * _SPEED_GAIN, 0.0, 0.5))


_CHASE_LFD_MIN = 3.0
_CHASE_LFD_MAX = 15.0
_CHASE_STEER_GAIN = 1.35

def _calc_chase_steer_norm(parsed: dict, target_x: float, target_y: float, wheelbase: float) -> float:
    """타겟 현재 위치를 직접 look-ahead point 로 두고 공격적으로 조향한다."""
    dx = target_x - parsed["location"]["x"]
    dy = target_y - parsed["location"]["y"]
    distance = float(np.hypot(dx, dy))
    if distance < 1e-3:
        return 0.0

    yaw = np.deg2rad(parsed["rotation"]["z"])
    local_x = np.cos(-yaw) * dx - np.sin(-yaw) * dy
    local_y = np.sin(-yaw) * dx + np.cos(-yaw) * dy
    theta = float(np.arctan2(local_y, local_x))
    lfd = float(np.clip(distance, _CHASE_LFD_MIN, _CHASE_LFD_MAX))
    steer_rad = np.arctan2(2.0 * wheelbase * np.sin(theta), lfd) * _CHASE_STEER_GAIN
    return float(np.clip(steer_rad / MAX_STEER_RAD, -1.0, 1.0))


# ─── 공유 위치 레지스트리 (충돌 모드: runner 간 위치 공유) ─────────
_shared_positions: dict = {}   # entity_id → {"x": float, "y": float, "speed_kph": float}
_shared_pos_lock  = threading.Lock()

def _update_shared_pos(entity_id: str, x: float, y: float, speed_kph: float) -> None:
    with _shared_pos_lock:
        _shared_positions[entity_id] = {"x": x, "y": y, "speed_kph": speed_kph}

def _get_shared_pos(entity_id: str) -> dict:
    with _shared_pos_lock:
        return dict(_shared_positions.get(entity_id, {}))

def clear_shared_positions() -> None:
    with _shared_pos_lock:
        _shared_positions.clear()


# ─── Runner ─────────────────────────────────────────────────────
class AdRunner:
    def __init__(
        self,
        tcp_sock:          socket.socket,
        entity_id:         str,
        vi_ip:             str,
        vi_port:           int,
        path_file:         str   = "path_link.csv",
        map_name:          str   = None,
        log_fn=None,
        status_cb=None,
        # 충돌 모드 파라미터
        is_chaser:            bool  = False,
        is_collision_target:  bool  = False,
        target_entity_id:     str   = None,
        speed_kph:            float = 60.0,   # target 정속 / chaser = ×1.2
        trigger_kph:          float = 5.0,
        max_speed_kph:        float = None,
    ):
        # UDP 수신 소켓
        self._recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._recv_sock.settimeout(2.0)
        self._recv_sock.bind((vi_ip, vi_port))

        self._tcp_sock              = tcp_sock
        self._entity_id             = entity_id
        self._is_chaser             = is_chaser
        self._is_collision_target   = is_collision_target
        self._target_entity_id      = target_entity_id
        self._target_speed_kph      = speed_kph if not is_chaser else speed_kph * 1.2
        self._trigger_kph           = trigger_kph
        self._max_speed_kph         = max_speed_kph

        self._ad = AutonomousDriving(path_file, map_name=map_name, max_speed_kph=max_speed_kph)

        self._running   = False
        self._lock      = threading.Lock()
        self._latest    = None
        self._log       = log_fn or (lambda msg, level="INFO": print(f"[AD] {msg}"))
        self._status_cb = status_cb or (lambda *a: None)

        role = "Chaser" if is_chaser else "PathFollow"
        self._log(f"Vehicle Info 수신 : {vi_ip}:{vi_port}")
        self._log(f"TCP 제어          : entity_id={entity_id} ({role})")

    def start(self) -> None:
        self._running = True
        threading.Thread(target=self._recv_loop,    daemon=True).start()
        threading.Thread(target=self._control_loop, daemon=True).start()

    def stop(self) -> None:
        self._running = False
        try:
            self._recv_sock.close()
        except Exception:
            pass

    def update_max_speed_kph(self, max_speed_kph: float) -> None:
        self._max_speed_kph = float(max_speed_kph)
        self._ad.set_max_speed_kph(self._max_speed_kph)

    # ── UDP 수신 ────────────────────────────────────────────────
    def _recv_loop(self) -> None:
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

    # ── 제어 루프 (30Hz) ────────────────────────────────────────
    def _control_loop(self) -> None:
        sampling_time = 1.0 / 30.0
        self._log("주행 시작")

        while self._running:
            t_start = time.perf_counter()

            with self._lock:
                parsed = self._latest

            if parsed:
                # 항상 공유 레지스트리에 현재 위치/속도 기록
                speed_kph = abs(parsed["local_velocity"]["x"]) * 3.6
                _update_shared_pos(
                    self._entity_id,
                    parsed["location"]["x"],
                    parsed["location"]["y"],
                    speed_kph,
                )
                if self._is_chaser:
                    self._run_chaser(parsed)
                else:
                    self._run_path_follow(parsed)
            else:
                self._log("차량 상태 대기 중...", "INFO")

            elapsed = time.perf_counter() - t_start
            sleep_t = sampling_time - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

        self._log("주행 종료")

    # ── 경로 추종 ────────────────────────────────────────────────
    def _run_path_follow(self, parsed: dict) -> None:
        vs = VehicleState(
            x        = parsed["location"]["x"],
            y        = parsed["location"]["y"],
            yaw      = np.deg2rad(parsed["rotation"]["z"]),
            velocity = parsed["local_velocity"]["x"],
        )
        try:
            ctrl, _ = self._ad.execute(vs)
            steer_norm = float(np.clip(ctrl.steering / MAX_STEER_RAD, -1.0, 1.0))

            if self._is_collision_target or self._is_chaser:
                # 충돌 모드: 조향은 Pure Pursuit, 속도는 설정값으로 고정
                # (target = speed_kph, chaser = speed_kph × 1.2)
                current_kph = abs(parsed["local_velocity"]["x"]) * 3.6
                throttle, brake = _speed_ctrl(current_kph, self._target_speed_kph)
            else:
                throttle, brake = ctrl.accel, ctrl.brake

            tcp.send_manual_control_by_id(
                self._tcp_sock, _next_rid(),
                entity_id   = self._entity_id,
                throttle    = throttle,
                brake       = brake,
                steer_angle = steer_norm,
            )
            self._status_cb(
                self._entity_id,
                vs.position.x, vs.position.y,
                vs.velocity * 3.6,
                throttle, brake, steer_norm,
            )
        except Exception as e:
            self._log(f"ERROR: {e}", "ERROR")

    # ── 충돌 추적 ────────────────────────────────────────────────
    def _run_chaser(self, parsed: dict) -> None:
        """Trigger 이후 target 현재 위치를 직접 추적해 추돌을 유도한다."""
        target = _get_shared_pos(self._target_entity_id)
        if not target:
            return   # target 위치 아직 미수신

        # trigger: target 속도가 기준 이상이어야 출발
        if target["speed_kph"] < self._trigger_kph:
            tcp.send_manual_control_by_id(
                self._tcp_sock, _next_rid(),
                entity_id=self._entity_id,
                throttle=0.0, brake=0.5, steer_angle=0.0,
            )
            return

        current_kph = abs(parsed["local_velocity"]["x"]) * 3.6
        throttle, brake = _speed_ctrl(current_kph, self._target_speed_kph)
        steer_norm = _calc_chase_steer_norm(
            parsed,
            target_x=target["x"],
            target_y=target["y"],
            wheelbase=float(self._ad.pure_pursuit.wheelbase),
        )
        tcp.send_manual_control_by_id(
            self._tcp_sock, _next_rid(),
            entity_id=self._entity_id,
            throttle=throttle, brake=brake, steer_angle=steer_norm,
        )
        self._status_cb(
            self._entity_id,
            parsed["location"]["x"], parsed["location"]["y"],
            abs(parsed["local_velocity"]["x"]) * 3.6,
            throttle, brake, steer_norm,
        )


# ─── 진입점 ─────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="autonomous_driving 단독 실행 스크립트")
    parser.add_argument("--tcp-ip",    default=proto.TCP_SERVER_IP)
    parser.add_argument("--tcp-port",  type=int, default=proto.TCP_SERVER_PORT)
    parser.add_argument("--ego-ip",    default="127.0.0.1")
    parser.add_argument("--ego-port",  type=int, default=9091)
    parser.add_argument("--entity-id", default="Car_1")
    args = parser.parse_args()

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
        vi_ip     = args.ego_ip,
        vi_port   = args.ego_port,
    )
    try:
        runner.start()
        threading.Event().wait()   # Ctrl+C 대기
    except KeyboardInterrupt:
        print("\n[AD] 종료 중...")
    finally:
        runner.stop()
        tcp_sock.close()
        print("[AD] 종료 완료")


if __name__ == "__main__":
    main()
