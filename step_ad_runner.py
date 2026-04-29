from __future__ import annotations

# step_ad_runner.py
# ad_runner.py 구조 기반 + Fixed Step 추가.
#
# 루프 순서:
#   ① 모든 차량 VI 읽기
#   ② 모든 차량 ManualControl 전송 (fire-and-forget)
#   ③ FixedStep 전송 → ACK 대기  (시뮬레이터 1틱 진행 + VI 전송)
#
# collision_cfg = {
#   "chaser_entity_id": str,   # 이 차량은 path 대신 target을 추적
#   "target_entity_id": str,   # 추적 대상
#   "throttle": float,         # chaser 고정 스로틀
#   "trigger_kph": float,      # target 이 이 속도 이상이면 chaser 출발
# }

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

MAX_STEER_RAD = 0.5

_rid_iter = itertools.count(1)

def _next_rid() -> int:
    return next(_rid_iter)


# ── 속도 비례 제어 ────────────────────────────────────────────
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


# ── 차량 컨텍스트 ─────────────────────────────────────────────

class _VehicleCtx:
    def __init__(self, entity_id: str, vi_ip: str, vi_port: int, path_file: str,
                 map_name: str = None,
                 is_chaser: bool = False,
                 is_collision_target: bool = False,
                 speed_kph: float = 60.0,
                 trigger_kph: float = 5.0,
                 max_speed_kph: float = None):
        self.entity_id           = entity_id
        self.is_chaser           = is_chaser
        self.is_collision_target = is_collision_target
        # target: speed_kph 정속 / chaser: speed_kph × 1.2 로 추돌
        self.target_speed_kph    = speed_kph if not is_chaser else speed_kph * 1.2
        self.trigger_kph         = trigger_kph
        self.ad                  = AutonomousDriving(path_file, map_name=map_name, max_speed_kph=max_speed_kph)
        self.latest         = None
        self.lock           = threading.Lock()
        self.vi_event       = threading.Event()   # FixedStep 후 VI 도착 신호

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.settimeout(2.0)
        self.sock.bind((vi_ip, vi_port))


# ── StepAdRunner ──────────────────────────────────────────────

class StepAdRunner:
    def __init__(
        self,
        tcp_sock:      socket.socket,
        vehicles:      list,           # [{ entity_id, vi_ip, vi_port, path }, ...]
        pending:       dict,
        lock:          threading.Lock,
        request_id_ref,                # RequestIdCounter (app.py 공유)
        pending_add_fn,
        pending_pop_fn,
        timeout_sec:   float = 3.0,
        log_fn=None,
        status_cb=None,
        on_done=None,
        collision_cfg: dict = None,    # 충돌 모드 설정 (없으면 일반 path follow)
        save_data:     bool = False,
        **kwargs,
    ):
        self._tcp_sock      = tcp_sock
        self._pending       = pending
        self._lock          = lock
        self._rid           = request_id_ref
        self._pending_add   = pending_add_fn
        self._pending_pop   = pending_pop_fn
        self._timeout_sec   = timeout_sec
        self._log           = log_fn or (lambda msg, level="INFO": print(f"[StepAD] {msg}"))
        self._status_cb     = status_cb or (lambda *a: None)
        self._on_done       = on_done
        self._running       = False
        self._collision_cfg = collision_cfg
        self._save_data     = save_data
        self._ctxs: list[_VehicleCtx] = []

        chaser_id = (collision_cfg or {}).get("chaser_entity_id")
        target_id = (collision_cfg or {}).get("target_entity_id")
        speed_kph = (collision_cfg or {}).get("speed_kph", 60.0)
        for v in vehicles:
            is_chaser = (chaser_id == v["entity_id"])
            is_target = bool(collision_cfg) and (v["entity_id"] == target_id)
            ctx = _VehicleCtx(
                entity_id           = v["entity_id"],
                vi_ip               = v.get("vi_ip", "0.0.0.0"),
                vi_port             = v["vi_port"],
                path_file           = v.get("path", "path_link.csv"),
                map_name            = v.get("map_name"),
                is_chaser           = is_chaser,
                is_collision_target = is_target,
                speed_kph           = speed_kph,
                trigger_kph         = (collision_cfg or {}).get("trigger_kph", 5.0),
                max_speed_kph       = v.get("max_speed_kph"),
            )
            self._ctxs.append(ctx)
            if is_chaser:
                role = f"Chaser ({speed_kph * 1.2:.0f} km/h)"
            elif is_target:
                role = f"Target ({speed_kph:.0f} km/h)"
            else:
                role = f"PathFollow (max={v.get('max_speed_kph', 0):.0f} km/h)"
            self._log(f"[{ctx.entity_id}] VI 수신 대기 → {v.get('vi_ip', '0.0.0.0')}:{v['vi_port']} ({role})")

    def update_max_speed_kph(self, entity_id: str, max_speed_kph: float) -> bool:
        for ctx in self._ctxs:
            if ctx.entity_id == entity_id:
                ctx.ad.set_max_speed_kph(float(max_speed_kph))
                return True
        return False

    # ── 공개 API ──────────────────────────────────────────────

    def start(self) -> None:
        self._running = True
        for ctx in self._ctxs:
            threading.Thread(target=self._recv_loop, args=(ctx,), daemon=True).start()
        threading.Thread(target=self._control_loop, daemon=True).start()

    def stop(self) -> None:
        self._running = False
        for ctx in self._ctxs:
            try:
                ctx.sock.close()
            except Exception:
                pass

    # ── UDP 수신 스레드 (차량별) ───────────────────────────────

    def _recv_loop(self, ctx: _VehicleCtx) -> None:
        while self._running:
            try:
                data, _ = ctx.sock.recvfrom(65535)
                parsed = parse_vehicle_info_payload(data)
                if parsed:
                    with ctx.lock:
                        ctx.latest = parsed
                    ctx.vi_event.set()   # VI 도착 신호
            except socket.timeout:
                continue
            except OSError:
                break

    # ── 차량별 제어 ───────────────────────────────────────────

    def _send_path_follow(self, ctx: _VehicleCtx, parsed: dict) -> None:
        """경로 추종 제어 (Pure Pursuit).
        충돌 모드 target 차량은 Pure Pursuit 조향을 유지하되 속도를 speed_kph로 제어."""
        vs = VehicleState(
            x        = parsed["location"]["x"],
            y        = parsed["location"]["y"],
            yaw      = np.deg2rad(parsed["rotation"]["z"]),
            velocity = parsed["local_velocity"]["x"],
        )
        try:
            ctrl, _ = ctx.ad.execute(vs)
            steer_n = float(np.clip(ctrl.steering / MAX_STEER_RAD, -1.0, 1.0))

            if ctx.is_collision_target or ctx.is_chaser:
                # 충돌 모드: 조향은 Pure Pursuit, 속도는 설정값으로 고정
                # (target = speed_kph, chaser = speed_kph × 1.2)
                current_kph = abs(parsed["local_velocity"]["x"]) * 3.6
                throttle, brake = _speed_ctrl(current_kph, ctx.target_speed_kph)
            else:
                throttle, brake = ctrl.accel, ctrl.brake

            tcp.send_manual_control_by_id(
                self._tcp_sock, _next_rid(),
                entity_id   = ctx.entity_id,
                throttle    = throttle,
                brake       = brake,
                steer_angle = steer_n,
            )
            self._status_cb(
                ctx.entity_id,
                vs.position.x, vs.position.y,
                vs.velocity * 3.6,
                throttle, brake, steer_n,
            )
        except Exception as e:
            self._log(f"[{ctx.entity_id}] 제어 오류: {e}", "ERROR")

    def _send_chaser(self, ctx: _VehicleCtx, parsed: dict) -> None:
        """Trigger 이후 target 현재 위치를 직접 추적해 추돌을 유도한다."""
        target_id = self._collision_cfg["target_entity_id"]
        target_ctx = next((c for c in self._ctxs if c.entity_id == target_id), None)
        if target_ctx is None:
            return

        with target_ctx.lock:
            target_parsed = target_ctx.latest
        if target_parsed is None:
            return

        # trigger: target 속도가 기준 이상이어야 출발
        target_kph = abs(target_parsed["local_velocity"]["x"]) * 3.6
        if target_kph < ctx.trigger_kph:
            tcp.send_manual_control_by_id(
                self._tcp_sock, _next_rid(),
                entity_id=ctx.entity_id,
                throttle=0.0, brake=0.5, steer_angle=0.0,
            )
            return

        current_kph = abs(parsed["local_velocity"]["x"]) * 3.6
        throttle, brake = _speed_ctrl(current_kph, ctx.target_speed_kph)
        steer_n = _calc_chase_steer_norm(
            parsed,
            target_x=target_parsed["location"]["x"],
            target_y=target_parsed["location"]["y"],
            wheelbase=float(ctx.ad.pure_pursuit.wheelbase),
        )
        tcp.send_manual_control_by_id(
            self._tcp_sock, _next_rid(),
            entity_id=ctx.entity_id,
            throttle=throttle, brake=brake, steer_angle=steer_n,
        )
        self._status_cb(
            ctx.entity_id,
            parsed["location"]["x"], parsed["location"]["y"],
            abs(parsed["local_velocity"]["x"]) * 3.6,
            throttle, brake, steer_n,
        )

    # ── 제어 루프 ─────────────────────────────────────────────

    _TIMING_INTERVAL = 100  # N 스텝마다 타이밍 통계 출력

    def _control_loop(self) -> None:
        self._log("주행 시작")

        _t_ack   = []   # FixedStep ACK 대기 (이전 루프에서 선제 전송된 스텝)
        _t_vi    = []   # VI 도착 대기
        _t_cmd   = []   # 제어 커맨드 전송 소요
        _t_total = []   # 전체 루프 소요

        def _send_all_cmds() -> None:
            for ctx in self._ctxs:
                with ctx.lock:
                    parsed = ctx.latest
                if parsed is None:
                    continue
                if ctx.is_chaser:
                    self._send_chaser(ctx, parsed)
                else:
                    self._send_path_follow(ctx, parsed)

        def _presend_step():
            """다음 FixedStep을 선제 전송하고 (ev, rid) 반환."""
            r = self._rid.next()
            e = self._pending_add(self._pending, self._lock, r,
                                  proto.MSG_TYPE_FIXED_STEP)
            tcp.send_fixed_step(self._tcp_sock, r, step_count=1)
            return e, r

        try:
            # ── 프라이밍: 초기 커맨드 없이 첫 스텝 전송 → save → 초기 VI 수신 ──
            for ctx in self._ctxs:
                ctx.vi_event.clear()

            ev, rid = _presend_step()
            if not ev.wait(self._timeout_sec):
                self._pending_pop(self._pending, self._lock, rid,
                                  proto.MSG_TYPE_FIXED_STEP)
                self._log("초기 FixedStep ACK timeout — 중단", "ERROR")
                return
            self._pending_pop(self._pending, self._lock, rid,
                               proto.MSG_TYPE_FIXED_STEP)
            if self._save_data:
                tcp.send_save_data(self._tcp_sock, _next_rid())
                for ctx in self._ctxs:
                    if not ctx.vi_event.wait(self._timeout_sec):
                        self._log(f"[{ctx.entity_id}] 초기 VI timeout — 중단", "ERROR")
                        return

            # 초기 커맨드 전송 후 첫 파이프라인 스텝 선제 전송
            _send_all_cmds()
            for ctx in self._ctxs:
                ctx.vi_event.clear()
            ev, rid = _presend_step()

            # ── 메인 파이프라인 루프 ──────────────────────────────
            #
            # 루프 진입 시 ev/rid 는 이미 전송된 FixedStep_N 을 가리킴.
            #
            # 순서:
            #   ① ACK_N 대기          (이전 루프 끝 또는 프라이밍에서 선제 전송)
            #   ② SaveData_N 전송
            #   ③ FixedStep_N+1 선제 전송  ← VI 대기 동안 RTT 오버랩
            #   ④ VI_N 대기
            #   ⑤ cmd_N+1 전송        (다음 스텝 이후에 서버에 도착 → 1-step control lag)
            #
            while self._running:
                t0 = time.perf_counter()

                # ① ACK 대기
                if not ev.wait(self._timeout_sec):
                    self._pending_pop(self._pending, self._lock, rid,
                                      proto.MSG_TYPE_FIXED_STEP)
                    self._log(
                        f"FixedStep ACK timeout ({self._timeout_sec}s) — 중단. "
                        "시나리오가 Fixed Step 모드인지 확인하세요.",
                        "ERROR"
                    )
                    break
                self._pending_pop(self._pending, self._lock, rid,
                                   proto.MSG_TYPE_FIXED_STEP)

                t1 = time.perf_counter()

                # ② SaveData 전송
                if self._save_data:
                    try:
                        tcp.send_save_data(self._tcp_sock, _next_rid())
                    except OSError as e:
                        self._log(f"SaveData 전송 오류: {e}", "ERROR")
                        break

                # ③ 다음 FixedStep 선제 전송 (VI 대기 동안 RTT 진행)
                for ctx in self._ctxs:
                    ctx.vi_event.clear()
                try:
                    ev, rid = _presend_step()
                except OSError as e:
                    self._log(f"FixedStep 전송 오류: {e}", "ERROR")
                    break

                t2 = time.perf_counter()

                # ④ VI 도착 대기 (save_data=False 이면 서버가 VI를 보내지 않으므로 생략)
                if self._save_data:
                    for ctx in self._ctxs:
                        if not ctx.vi_event.wait(self._timeout_sec):
                            self._log(
                                f"[{ctx.entity_id}] VI timeout ({self._timeout_sec}s) — 이전 상태로 계속",
                                "WARN"
                            )

                t3 = time.perf_counter()

                # ⑤ 제어 커맨드 전송
                _send_all_cmds()

                t4 = time.perf_counter()

                _t_ack.append((t1 - t0) * 1000)
                _t_vi.append((t3 - t2) * 1000)
                _t_cmd.append((t4 - t3) * 1000)
                _t_total.append((t4 - t0) * 1000)

                if len(_t_total) >= self._TIMING_INTERVAL:
                    def _stats(s):
                        return sum(s) / len(s), min(s), max(s)
                    aa, an, ax = _stats(_t_ack)
                    va, vn, vx = _stats(_t_vi)
                    ca, cn, cx = _stats(_t_cmd)
                    ta, tn, tx = _stats(_t_total)
                    self._log(
                        f"[Timing/{self._TIMING_INTERVAL}스텝] "
                        f"total={ta:.1f}ms({tn:.1f}~{tx:.1f})  "
                        f"ack_wait={aa:.1f}({an:.1f}~{ax:.1f})  "
                        f"vi_wait={va:.1f}({vn:.1f}~{vx:.1f})  "
                        f"cmd={ca:.1f}({cn:.1f}~{cx:.1f})",
                        "INFO"
                    )
                    _t_ack.clear(); _t_vi.clear()
                    _t_cmd.clear(); _t_total.clear()

        finally:
            self._running = False
            self._log("주행 종료")
            if self._on_done:
                self._on_done()
