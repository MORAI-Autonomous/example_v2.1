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


# ── 차량 컨텍스트 ─────────────────────────────────────────────

class _VehicleCtx:
    def __init__(self, entity_id: str, vi_ip: str, vi_port: int, path_file: str,
                 map_name: str = None,
                 is_chaser: bool = False,
                 is_collision_target: bool = False,
                 speed_kph: float = 60.0,
                 trigger_kph: float = 5.0):
        self.entity_id           = entity_id
        self.is_chaser           = is_chaser
        self.is_collision_target = is_collision_target
        # target: speed_kph 정속 / chaser: speed_kph × 1.2 로 추돌
        self.target_speed_kph    = speed_kph if not is_chaser else speed_kph * 1.2
        self.trigger_kph         = trigger_kph
        self.ad                  = AutonomousDriving(path_file, map_name=map_name)
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
        **kwargs,                      # save_data 등 미사용 파라미터
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
            )
            self._ctxs.append(ctx)
            if is_chaser:
                role = f"Chaser ({speed_kph * 1.2:.0f} kph)"
            elif is_target:
                role = f"Target ({speed_kph:.0f} kph)"
            else:
                role = "PathFollow"
            self._log(f"[{ctx.entity_id}] VI 수신 대기 → {v.get('vi_ip', '0.0.0.0')}:{v['vi_port']} ({role})")

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
            velocity = parsed["local_velocity"]["x"] / 3.6,
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
        """Trigger 조건 확인 후 Path Follow 로 주행 (Pure Pursuit 조향 + speed × 1.2)."""
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

        # target 출발 확인 → Pure Pursuit 경로 추종으로 추돌
        self._send_path_follow(ctx, parsed)

    # ── 제어 루프 ─────────────────────────────────────────────

    _TIMING_INTERVAL = 100  # N 스텝마다 타이밍 통계 출력

    def _control_loop(self) -> None:
        self._log("주행 시작")

        # 타이밍 누적 (ms)
        _t_cmd   = []   # ① 제어 커맨드 전송 소요
        _t_step  = []   # ② FixedStep 송신 → ACK 수신 (네트워크 RTT + 서버 처리)
        _t_save  = []   # ③ SaveData 송신 소요
        _t_vi    = []   # ④ VI 도착 대기 소요
        _t_total = []   # 전체 루프 소요

        try:
            while self._running:
                t0 = time.perf_counter()

                # ① 모든 차량 제어 명령 전송
                for ctx in self._ctxs:
                    with ctx.lock:
                        parsed = ctx.latest

                    if parsed is None:
                        self._log(f"[{ctx.entity_id}] 차량 상태 대기 중...", "INFO")
                        continue

                    if ctx.is_chaser:
                        self._send_chaser(ctx, parsed)
                    else:
                        self._send_path_follow(ctx, parsed)

                t1 = time.perf_counter()

                # ② FixedStep 전송 → ACK 대기
                for ctx in self._ctxs:
                    ctx.vi_event.clear()

                rid = self._rid.next()
                ev  = self._pending_add(self._pending, self._lock, rid,
                                        proto.MSG_TYPE_FIXED_STEP)
                try:
                    tcp.send_fixed_step(self._tcp_sock, rid, step_count=1)
                except OSError as e:
                    self._log(f"FixedStep 전송 오류: {e}", "ERROR")
                    break

                t2 = time.perf_counter()

                if not ev.wait(self._timeout_sec):
                    self._pending_pop(self._pending, self._lock, rid,
                                      proto.MSG_TYPE_FIXED_STEP)
                    self._log(
                        f"FixedStep ACK timeout ({self._timeout_sec}s) — 중단. "
                        "시나리오가 Fixed Step 모드인지 확인하세요.",
                        "ERROR"
                    )
                    break

                t3 = time.perf_counter()

                self._pending_pop(self._pending, self._lock, rid,
                                  proto.MSG_TYPE_FIXED_STEP)

                # ③ SaveData 전송 (fire-and-forget)
                try:
                    tcp.send_save_data(self._tcp_sock, _next_rid())
                except OSError as e:
                    self._log(f"SaveData 전송 오류: {e}", "ERROR")
                    break

                t4 = time.perf_counter()

                # ④ VI 도착 대기
                for ctx in self._ctxs:
                    if not ctx.vi_event.wait(self._timeout_sec):
                        self._log(
                            f"[{ctx.entity_id}] VI timeout ({self._timeout_sec}s) — 이전 상태로 계속",
                            "WARN"
                        )

                t5 = time.perf_counter()

                _t_cmd.append((t1 - t0) * 1000)
                _t_step.append((t3 - t2) * 1000)
                _t_save.append((t4 - t3) * 1000)
                _t_vi.append((t5 - t4) * 1000)
                _t_total.append((t5 - t0) * 1000)

                if len(_t_total) >= self._TIMING_INTERVAL:
                    def _stats(samples):
                        return (
                            sum(samples) / len(samples),
                            min(samples),
                            max(samples),
                        )
                    ca, cn, cx   = _stats(_t_cmd)
                    sa, sn, sx   = _stats(_t_step)
                    va, vn, vx   = _stats(_t_vi)
                    ta, tn, tx   = _stats(_t_total)
                    self._log(
                        f"[Timing/{self._TIMING_INTERVAL}스텝] "
                        f"total={ta:.1f}ms({tn:.1f}~{tx:.1f})  "
                        f"cmd={ca:.1f}({cn:.1f}~{cx:.1f})  "
                        f"step_ack={sa:.1f}({sn:.1f}~{sx:.1f})  "
                        f"vi_wait={va:.1f}({vn:.1f}~{vx:.1f})",
                        "INFO"
                    )
                    _t_cmd.clear(); _t_step.clear()
                    _t_save.clear(); _t_vi.clear(); _t_total.clear()

        finally:
            self._running = False
            self._log("주행 종료")
            if self._on_done:
                self._on_done()
