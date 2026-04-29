from __future__ import annotations

# app.py
import os
import socket
import threading
import time

import dearpygui.dearpygui as dpg

from transport.protocol_defs import *
import transport.tcp_transport as tcp
import transport.tcp_thread as tcp_thread_mod
import automation.automation as ac
import ad_runner as AdRunner_mod
from ad_runner import AdRunner
from step_ad_runner import StepAdRunner
from lane_runner import LaneRunner
import utils.ui_queue as ui_queue
import panels.log               as log_panel
import panels.monitor            as monitor_panel
import panels.commands           as cmd_panel
import panels.lane_control_panel  as lc_panel
import panels.autonomous_panel    as au_panel
import panels.file_playback_panel as fp_panel
import panels.transform_playback_panel as tfp_panel

APP_TITLE = "Sim Control Example"
_logo_tag = None   # 로고 텍스처 태그 (main()에서 로드 후 설정)

# ── 레이아웃 상수 ─────────────────────────────────────────
W_INIT, H_INIT = 1400, 1200  # 초기 뷰포트 크기
W_MIN,  H_MIN  = 900,  600   # 최소 크기
CMD_W      = 400        # 커맨드 패널 너비 (고정)
LOG_H      = 280        # 로그 패널 높이 (고정)
TITLEBAR_H = 38         # 타이틀바 + separator 높이
PAD        = 12         # 좌우/하단 여백

# 동적 크기 헬퍼 — 리사이즈 시 뷰포트 실제 크기 반환
def _vp_w():  return dpg.get_viewport_width()
def _vp_h():  return dpg.get_viewport_height()
def _top_h(): return max(_vp_h() - TITLEBAR_H - LOG_H - PAD, 100)
def _mon_w(): return max(_vp_w() - CMD_W - PAD * 3, 200)


# ============================================================
# RequestIdCounter
# ============================================================
class RequestIdCounter:
    def __init__(self, start: int = 1):
        self._lock  = threading.Lock()
        self._value = start

    def next(self) -> int:
        with self._lock:
            rid = self._value
            self._value += 1
        return rid


# ============================================================
# Pending helpers
# ============================================================
def pending_add(pending, lock, request_id, msg_type):
    ev = threading.Event()
    with lock:
        pending[(request_id, msg_type)] = {"t": time.time(), "ev": ev}
    return ev

def pending_pop(pending, lock, request_id, msg_type):
    with lock:
        pending.pop((request_id, msg_type), None)


# ============================================================
# TCP helpers
# ============================================================
def _make_tcp_socket():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    s.settimeout(5.0)   # 전송 블로킹 5s 상한 → UI 먹통 방지
    return s

def _close_socket(sock):
    for fn in (lambda: sock.shutdown(socket.SHUT_RDWR), sock.close):
        try: fn()
        except Exception: pass

def _set_conn_status(connected: bool):
    ui_queue.post(lambda c=connected: (
        dpg.configure_item("conn_label",
            color=(100, 255, 100, 255) if c else (255, 80, 80, 255)),
        dpg.set_value("conn_label", "● Connected" if c else "○ Disconnected"),
        dpg.configure_item("btn_reconnect", show=not c),
    ))


# ============================================================
# AppState
# ============================================================
class AppState:
    def __init__(self):
        self.pending     = {}
        self.lock        = threading.Lock()
        self.rid         = RequestIdCounter()
        self.tcp_sock    = None
        self.receiver    = None
        self.auto_caller = None
        self.fp_caller   = None
        self.tfp_caller  = None
        self.ad_runners:      list = []
        self.step_ad_runners: list = []
        self.lc_runner   = None
        self._connecting = False
        self._conn_lock  = threading.Lock()

    def dispatch(self, msg_type: int, send_fn):
        if self.tcp_sock is None:
            log_panel.append("Not connected.", "WARN")
            return
        def _send():
            try:
                rid = self.rid.next()
                pending_add(self.pending, self.lock, rid, msg_type)
                send_fn(rid)
            except OSError as e:
                log_panel.append(f"Send error: {e}", "ERROR")
        threading.Thread(target=_send, daemon=True).start()

    def toggle_auto(self, max_calls: int = MAX_CALL_NUM) -> bool:
        if self.auto_caller is None or not self.auto_caller.is_alive():
            self.auto_caller = ac.AutoCaller(
                tcp_sock=self.tcp_sock,
                pending=self.pending,
                lock=self.lock,
                request_id_ref=self.rid,
                max_calls=max_calls,
                pending_add_fn=pending_add,
                pending_pop_fn=pending_pop,
                step_count=1,
                timeout_sec=AUTO_TIMEOUT_SEC,
                delay_sec=AUTO_DELAY_BETWEEN_CMDS_SEC,
                progress_every=50,
            )
            def _on_done(s=self):
                s.auto_caller = None
            _patch_auto_caller(self.auto_caller, on_done=_on_done)
            self.auto_caller.start()
            return True
        else:
            self.auto_caller.stop()
            self.auto_caller = None
            cmd_panel.reset_auto_ui()
            return False

    def start_fp(self, rows: list, entity_id: str) -> None:
        if self.fp_caller is not None and self.fp_caller.is_alive():
            log_panel.append("[FP] 이미 재생 중입니다.", "WARN")
            return
        self.fp_caller = ac.AutoCaller(
            tcp_sock=self.tcp_sock,
            pending=self.pending,
            lock=self.lock,
            request_id_ref=self.rid,
            max_calls=len(rows),
            pending_add_fn=pending_add,
            pending_pop_fn=pending_pop,
            step_count=1,
            timeout_sec=AUTO_TIMEOUT_SEC,
            delay_sec=AUTO_DELAY_BETWEEN_CMDS_SEC,
            progress_every=1,
        )
        def _on_done(s=self):
            s.fp_caller = None
        _patch_fp_caller(self.fp_caller, rows, entity_id, on_done=_on_done)
        self.fp_caller.start()

    def stop_fp(self) -> None:
        if self.fp_caller and self.fp_caller.is_alive():
            self.fp_caller.stop()
            self.fp_caller = None

    def start_tfp(self, vehicles: list) -> None:
        if self.tfp_caller is not None and self.tfp_caller.is_alive():
            log_panel.append("[TFP] 이미 재생 중입니다.", "WARN")
            return
        total_rows = max((len(v["rows"]) for v in vehicles), default=0)
        if total_rows <= 0:
            log_panel.append("[TFP] 재생할 행이 없습니다.", "WARN")
            return
        self.tfp_caller = ac.AutoCaller(
            tcp_sock=self.tcp_sock,
            pending=self.pending,
            lock=self.lock,
            request_id_ref=self.rid,
            max_calls=total_rows,
            pending_add_fn=pending_add,
            pending_pop_fn=pending_pop,
            step_count=1,
            timeout_sec=AUTO_TIMEOUT_SEC,
            delay_sec=AUTO_DELAY_BETWEEN_CMDS_SEC,
            progress_every=1,
        )
        def _on_done(s=self):
            s.tfp_caller = None
        _patch_tfp_caller(self.tfp_caller, vehicles, on_done=_on_done)
        self.tfp_caller.start()

    def stop_tfp(self) -> None:
        if self.tfp_caller and self.tfp_caller.is_alive():
            self.tfp_caller.stop()
            self.tfp_caller = None

    def start_ad(self, vehicles: list, collision_cfg: dict = None) -> None:
        if self.ad_runners:
            log_panel.append("[AD] 이미 실행 중입니다.", "WARN")
            return
        AdRunner_mod.clear_shared_positions()
        chaser_id = (collision_cfg or {}).get("chaser_entity_id")
        target_id = (collision_cfg or {}).get("target_entity_id")
        speed_kph = (collision_cfg or {}).get("speed_kph", 60.0)
        for v in vehicles:
            is_chaser = (chaser_id == v["entity_id"])
            is_target = bool(collision_cfg) and (v["entity_id"] == target_id)
            try:
                runner = AdRunner(
                    tcp_sock              = self.tcp_sock,
                    entity_id             = v["entity_id"],
                    vi_ip                 = "0.0.0.0",
                    vi_port               = v["vi_port"],
                    path_file             = v.get("path", "path_link.csv"),
                    map_name              = v.get("map_name"),
                    log_fn                = lambda msg, level="INFO", eid=v["entity_id"]:
                                               log_panel.append(f"[AD:{eid}] {msg}", level),
                    status_cb             = au_panel.update_status,
                    is_chaser             = is_chaser,
                    is_collision_target   = is_target,
                    target_entity_id      = target_id,
                    speed_kph             = speed_kph,
                    trigger_kph           = (collision_cfg or {}).get("trigger_kph", 5.0),
                    max_speed_kph         = v.get("max_speed_kph"),
                )
                runner.start()
                self.ad_runners.append(runner)
                if is_chaser:
                    role = f"Chaser ({speed_kph * 1.2:.0f} km/h)"
                elif is_target:
                    role = f"Target ({speed_kph:.0f} km/h)"
                else:
                    role = f"PathFollow (max={v.get('max_speed_kph', 0):.0f} km/h)"
                log_panel.append(f"[AD:{v['entity_id']}] 시작 (port={v['vi_port']}, {role})")
            except Exception as e:
                log_panel.append(f"[AD:{v['entity_id']}] 시작 실패: {e}", "ERROR")
        if not self.ad_runners:
            au_panel.reset_ui()

    def stop_ad(self) -> None:
        for runner in self.ad_runners:
            runner.stop()
        self.ad_runners.clear()
        au_panel.reset_ui()

    def update_ad_max_speed(self, entity_id: str, max_speed_kph: float) -> None:
        updated = False
        for runner in self.ad_runners:
            if getattr(runner, "_entity_id", None) == entity_id:
                runner.update_max_speed_kph(max_speed_kph)
                updated = True
        for runner in self.step_ad_runners:
            updated = runner.update_max_speed_kph(entity_id, max_speed_kph) or updated
        if updated:
            log_panel.append(f"[AD:{entity_id}] max speed 업데이트 -> {max_speed_kph:.0f} km/h", "INFO")

    def start_step_ad(self, vehicles: list, save_data: bool = False,
                      collision_cfg: dict = None) -> None:
        if self.step_ad_runners:
            log_panel.append("[StepAD] 이미 실행 중입니다.", "WARN")
            return
        try:
            def _on_done(s=self):
                s.step_ad_runners.clear()
                au_panel.reset_ui()
            runner = StepAdRunner(
                tcp_sock       = self.tcp_sock,
                vehicles       = vehicles,
                pending        = self.pending,
                lock           = self.lock,
                request_id_ref = self.rid,
                pending_add_fn = pending_add,
                pending_pop_fn = pending_pop,
                timeout_sec    = AUTO_TIMEOUT_SEC,
                save_data      = save_data,
                log_fn         = lambda msg, level="INFO": log_panel.append(f"[StepAD] {msg}", level),
                status_cb      = au_panel.update_status,
                on_done        = _on_done,
                collision_cfg  = collision_cfg,
            )
            runner.start()
            self.step_ad_runners.append(runner)
            ids = ", ".join(v["entity_id"] for v in vehicles)
            log_panel.append(f"[StepAD] 시작 (vehicles: {ids})")
        except Exception as e:
            log_panel.append(f"[StepAD] 시작 실패: {e}", "ERROR")
            au_panel.reset_ui()

    def stop_step_ad(self) -> None:
        for runner in self.step_ad_runners:
            runner.stop()
        self.step_ad_runners.clear()
        au_panel.reset_ui()

    def start_lc(
        self,
        cam_port:    int,
        vi_port:     int,
        entity_id:   str,
        speed_ctrl:  bool,
        target_kmh:  float,
        throttle:    float,
        invert_steer: bool = True,
    ) -> None:
        if self.lc_runner is not None:
            log_panel.append("[LC] 이미 실행 중입니다.", "WARN")
            return
        try:
            self.lc_runner = LaneRunner(
                tcp_sock     = self.tcp_sock,
                entity_id    = entity_id,
                cam_ip       = "0.0.0.0",
                cam_port     = cam_port,
                vi_ip        = "0.0.0.0",
                vi_port      = vi_port,
                speed_ctrl   = speed_ctrl,
                target_kmh   = target_kmh,
                throttle     = throttle,
                invert_steer = invert_steer,
                log_fn       = lambda msg, level="INFO": log_panel.append(f"[LC] {msg}", level),
                frame_cb     = lc_panel.update_frame,
                vi_cb        = lc_panel.update_vehicle_info,
                debug_cb     = lc_panel.update_debug_frame,
            )
            self.lc_runner.start()
            lc_panel.set_runner(self.lc_runner)
        except Exception as e:
            log_panel.append(f"[LC] 시작 실패: {e}", "ERROR")
            self.lc_runner = None
            lc_panel.reset_ui()

    def stop_lc(self) -> None:
        lc_panel.set_runner(None)
        if self.lc_runner:
            self.lc_runner.stop()
            self.lc_runner = None
        lc_panel.reset_ui()

    def connect(self):
        with self._conn_lock:
            if self._connecting:
                return
            self._connecting = True

        def _run():
            if self.receiver:
                self.receiver.stop()
                self.receiver = None
            if self.tcp_sock:
                _close_socket(self.tcp_sock)
                self.tcp_sock = None

            sock = _make_tcp_socket()
            while True:
                try:
                    log_panel.append(f"Connecting {TCP_SERVER_IP}:{TCP_SERVER_PORT}...", "INFO")
                    sock.connect((TCP_SERVER_IP, TCP_SERVER_PORT))
                    log_panel.append(f"Connected {TCP_SERVER_IP}:{TCP_SERVER_PORT}", "INFO")
                    self.tcp_sock = sock
                    _set_conn_status(True)
                    self.receiver = tcp_thread_mod.Receiver(
                        sock, self.pending, self.lock,
                        on_disconnect=self._on_disconnect,
                    )
                    self.receiver.start()
                    cmd_panel.init(
                        tcp_sock=self.tcp_sock,
                        dispatch_fn=self.dispatch,
                        toggle_auto_fn=self.toggle_auto,
                    )
                    lc_panel.init(
                        start_lc_fn=self.start_lc,
                        stop_lc_fn=self.stop_lc,
                    )
                    fp_panel.init(
                        start_fp_fn=self.start_fp,
                        stop_fp_fn=self.stop_fp,
                    )
                    tfp_panel.init(
                        start_tfp_fn=self.start_tfp,
                        stop_tfp_fn=self.stop_tfp,
                    )
                    au_panel.init(
                        start_ad_fn=self.start_ad,
                        stop_ad_fn=self.stop_ad,
                        start_step_ad_fn=self.start_step_ad,
                        stop_step_ad_fn=self.stop_step_ad,
                        update_max_speed_fn=self.update_ad_max_speed,
                    )
                    break
                except Exception as e:
                    log_panel.append(f"Connect failed: {e} — retry 5s", "ERROR")
                    _set_conn_status(False)
                    _close_socket(sock)
                    time.sleep(5)
                    sock = _make_tcp_socket()

            with self._conn_lock:
                self._connecting = False

        threading.Thread(target=_run, daemon=True).start()

    def _on_disconnect(self):
        self.tcp_sock = None                                    # 즉시 null → dispatch null guard 히트
        # 대기 중인 ev.wait() 즉시 해제 — StepAdRunner 등이 timeout까지 기다리지 않도록
        with self.lock:
            for item in self.pending.values():
                item["ev"].set()
            self.pending.clear()
        if self.auto_caller and self.auto_caller.is_alive():
            self.auto_caller.stop()
        if self.fp_caller and self.fp_caller.is_alive():
            self.fp_caller.stop()
        if self.tfp_caller and self.tfp_caller.is_alive():
            self.tfp_caller.stop()
        for runner in list(self.step_ad_runners):
            runner.stop()
        self.step_ad_runners.clear()
        _set_conn_status(False)
        log_panel.append("Connection lost. Click Reconnect to retry.", "ERROR")


# ============================================================
# AutoCaller patch
# ============================================================
def _patch_auto_caller(caller: ac.AutoCaller, on_done=None):
    def patched_run():
        for i in range(caller.max_calls):
            if caller._stop.is_set():
                break

            rid = caller._next_rid()
            ev  = caller.pending_add(caller.pending, caller.lock, rid, MSG_TYPE_FIXED_STEP)
            tcp.send_fixed_step(caller.tcp_sock, rid, step_count=caller.step_count)
            if not caller._wait_or_stop(ev):
                caller.pending_pop(caller.pending, caller.lock, rid, MSG_TYPE_FIXED_STEP)
                log_panel.append(f"[AUTO][TIMEOUT] FixedStep i={i} rid={rid}", "WARN")
                break
            caller.pending_pop(caller.pending, caller.lock, rid, MSG_TYPE_FIXED_STEP)
            if caller.delay_sec > 0:
                time.sleep(caller.delay_sec)
            if caller._stop.is_set():
                break

            rid = caller._next_rid()
            ev  = caller.pending_add(caller.pending, caller.lock, rid, MSG_TYPE_SAVE_DATA)
            tcp.send_save_data(caller.tcp_sock, rid)
            if not caller._wait_or_stop(ev):
                caller.pending_pop(caller.pending, caller.lock, rid, MSG_TYPE_SAVE_DATA)
                log_panel.append(f"[AUTO][TIMEOUT] SaveData i={i} rid={rid}", "WARN")
                break
            caller.pending_pop(caller.pending, caller.lock, rid, MSG_TYPE_SAVE_DATA)
            if caller.delay_sec > 0:
                time.sleep(caller.delay_sec)

            if caller.progress_every > 0 and (i + 1) % caller.progress_every == 0:
                cmd_panel.update_auto_progress(i + 1, caller.max_calls)
                log_panel.append(f"progress {i+1}/{caller.max_calls}", "AUTO")

        cmd_panel.reset_auto_ui()
        log_panel.append("AutoCaller finished.", "AUTO")
        if on_done:
            on_done()

    caller.run = patched_run


# ============================================================
# FileCaller patch
# ============================================================
def _patch_fp_caller(caller: ac.AutoCaller, rows: list, entity_id: str, on_done=None):
    """
    AutoCaller.run 을 CSV 파일 재생 루프로 교체한다.
    각 행마다:
      1. ManualControlById 전송 (fire-and-forget)
      2. FixedStep 전송 → ACK 대기
      3. SaveData 전송 → ACK 대기
    """
    def patched_run():
        total = len(rows)
        log_panel.append(f"[FP] 시작: {total}행, entity={entity_id}", "INFO")

        for i, row in enumerate(rows):
            if caller._stop.is_set():
                break

            # ── 1. ManualControlById (no ACK) ─────────────────
            rid = caller._next_rid()
            tcp.send_manual_control_by_id(
                caller.tcp_sock, rid,
                entity_id=entity_id,
                throttle=row['throttle'],
                brake=row['brake'],
                steer_angle=row['swa'],
            )

            if caller._stop.is_set():
                break

            # ── 2. FixedStep (ACK 대기) ────────────────────────
            rid = caller._next_rid()
            ev  = caller.pending_add(caller.pending, caller.lock, rid, MSG_TYPE_FIXED_STEP)
            tcp.send_fixed_step(caller.tcp_sock, rid, step_count=caller.step_count)
            if not caller._wait_or_stop(ev):
                caller.pending_pop(caller.pending, caller.lock, rid, MSG_TYPE_FIXED_STEP)
                log_panel.append(f"[FP][TIMEOUT] FixedStep i={i} rid={rid}", "WARN")
                break
            caller.pending_pop(caller.pending, caller.lock, rid, MSG_TYPE_FIXED_STEP)

            if caller._stop.is_set():
                break

            # ── 3. SaveData (ACK 대기) ─────────────────────────
            rid = caller._next_rid()
            ev  = caller.pending_add(caller.pending, caller.lock, rid, MSG_TYPE_SAVE_DATA)
            tcp.send_save_data(caller.tcp_sock, rid)
            if not caller._wait_or_stop(ev):
                caller.pending_pop(caller.pending, caller.lock, rid, MSG_TYPE_SAVE_DATA)
                log_panel.append(f"[FP][TIMEOUT] SaveData i={i} rid={rid}", "WARN")
                break
            caller.pending_pop(caller.pending, caller.lock, rid, MSG_TYPE_SAVE_DATA)

            if caller.delay_sec > 0:
                time.sleep(caller.delay_sec)

            # ── Progress ───────────────────────────────────────
            fp_panel.update_progress(i + 1, total)

        stopped = caller._stop.is_set()
        fp_panel.reset_ui(stopped=stopped)
        log_panel.append(f"[FP] {'중단됨' if stopped else '재생 완료'} ({total}행)", "INFO")
        if on_done:
            on_done()

    caller.run = patched_run


# ============================================================
# TransformPlayback patch
# ============================================================
def _patch_tfp_caller(caller: ac.AutoCaller, vehicles: list, on_done=None):
    """
    AutoCaller.run 을 TransformControlById CSV 재생 루프로 교체한다.
    각 step 마다:
      1. 모든 차량 TransformControlById 전송 (fire-and-forget)
      2. CSV 시간 간격만큼 대기
    """
    def patched_run():
        total = max((len(v["rows"]) for v in vehicles), default=0)
        ids = ", ".join(v["entity_id"] for v in vehicles)
        log_panel.append(f"[TFP] 시작: {total}행, vehicles={ids}", "INFO")

        def _row_time(i: int):
            for vehicle in vehicles:
                if i < len(vehicle["rows"]):
                    return vehicle["rows"][i].get("time_sec")
            return None

        for i in range(total):
            if caller._stop.is_set():
                break

            for vehicle in vehicles:
                if i >= len(vehicle["rows"]):
                    continue
                row = vehicle["rows"][i]
                rid = caller._next_rid()
                tcp.send_transform_control_by_id(
                    caller.tcp_sock, rid,
                    entity_id=vehicle["entity_id"],
                    pos_x=row["pos_x"], pos_y=row["pos_y"], pos_z=row["pos_z"],
                    rot_x=row["rot_x"], rot_y=row["rot_y"], rot_z=row["rot_z"],
                    steer_angle=row["steer_angle"],
                    speed=row["speed"],
                )

            if caller._stop.is_set():
                break

            tfp_panel.update_progress(i + 1, total)

            if i + 1 < total:
                t_cur = _row_time(i)
                t_next = _row_time(i + 1)
                if t_cur is not None and t_next is not None:
                    sleep_sec = max(0.0, min(t_next - t_cur, 0.2))
                else:
                    sleep_sec = max(caller.delay_sec, 0.02)
                if sleep_sec > 0 and caller._stop.wait(timeout=sleep_sec):
                    break

        stopped = caller._stop.is_set()
        tfp_panel.reset_ui(stopped=stopped)
        log_panel.append(f"[TFP] {'중단됨' if stopped else '재생 완료'} ({total}행)", "INFO")
        if on_done:
            on_done()

    caller.run = patched_run


# ============================================================
# UI build
# ============================================================
def build_ui(state: AppState):

    with dpg.theme() as global_theme:
        with dpg.theme_component(dpg.mvAll):
            dpg.add_theme_color(dpg.mvThemeCol_WindowBg,      (22, 22, 26))
            dpg.add_theme_color(dpg.mvThemeCol_ChildBg,       (28, 28, 33))
            dpg.add_theme_color(dpg.mvThemeCol_FrameBg,       (40, 40, 48))
            dpg.add_theme_color(dpg.mvThemeCol_FrameBgHovered,(50, 50, 60))
            dpg.add_theme_color(dpg.mvThemeCol_Button,        (50, 50, 62))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (65, 65, 82))
            dpg.add_theme_color(dpg.mvThemeCol_Header,        (50, 90, 140, 180))
            dpg.add_theme_color(dpg.mvThemeCol_HeaderHovered, (60, 110, 170, 200))
            dpg.add_theme_color(dpg.mvThemeCol_TitleBgActive, (35, 35, 45))
            dpg.add_theme_color(dpg.mvThemeCol_Tab,           (38, 38, 50))
            dpg.add_theme_color(dpg.mvThemeCol_TabHovered,    (55, 90, 140))
            dpg.add_theme_color(dpg.mvThemeCol_TabActive,     (45, 80, 130))
            dpg.add_theme_color(dpg.mvThemeCol_Border,        (60, 60, 75))
            dpg.add_theme_color(dpg.mvThemeCol_Text,          (210, 210, 215))
            dpg.add_theme_style(dpg.mvStyleVar_WindowRounding, 6)
            dpg.add_theme_style(dpg.mvStyleVar_FrameRounding,  4)
            dpg.add_theme_style(dpg.mvStyleVar_ItemSpacing,    x=6, y=4)
            dpg.add_theme_style(dpg.mvStyleVar_WindowPadding,  x=8, y=6)
    dpg.bind_theme(global_theme)

    # ── 커스텀 탭 버튼 테마 (active / inactive) ──────────────────
    with dpg.theme(tag="theme_tab_active"):
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button,        (45, 80, 130, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (60, 100, 160, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive,  (45, 80, 130, 255))
            dpg.add_theme_color(dpg.mvThemeCol_Text,          (255, 255, 255, 255))
    with dpg.theme(tag="theme_tab_inactive"):
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button,        (38, 38, 50, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (55, 90, 140, 180))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive,  (38, 38, 50, 255))
            dpg.add_theme_color(dpg.mvThemeCol_Text,          (180, 180, 185, 255))

    def _select_tab(name: str) -> None:
        dpg.configure_item("mon_scroll", show=(name == "udp"))
        dpg.configure_item("lc_scroll",  show=(name == "lc"))
        dpg.configure_item("au_scroll",  show=(name == "au"))
        dpg.configure_item("fp_scroll",  show=(name == "fp"))
        dpg.configure_item("tfp_scroll", show=(name == "tfp"))
        for tag, key in [("tab_btn_udp", "udp"), ("tab_btn_lc", "lc"),
                         ("tab_btn_au", "au"), ("tab_btn_fp", "fp"),
                         ("tab_btn_tfp", "tfp")]:
            dpg.bind_item_theme(tag, "theme_tab_active" if name == key else "theme_tab_inactive")

    with dpg.window(tag="app_info_modal", label="App Info", modal=True,
                    show=False, no_resize=True, width=420, height=180):
        dpg.add_text(APP_TITLE)
        dpg.add_spacer(height=6)
        dpg.add_text("Python example client for MORAI simulator TCP/UDP control.")
        dpg.add_text("This menu bar is a scaffold for future app info and settings.")
        dpg.add_spacer(height=12)
        dpg.add_button(label="Close", callback=lambda: dpg.configure_item("app_info_modal", show=False))

    with dpg.window(tag="main_window", no_title_bar=True,
                    no_resize=True, no_move=True,
                    no_scrollbar=True, no_scroll_with_mouse=True):

        # ── 타이틀바 ──────────────────────────────────────
        def _apply_conn(s=None, a=None):
            global TCP_SERVER_IP, TCP_SERVER_PORT
            new_ip   = dpg.get_value("tb_ip_input").strip()
            new_port = dpg.get_value("tb_port_input")
            if new_ip:
                TCP_SERVER_IP   = new_ip
            TCP_SERVER_PORT = new_port
            state.connect()

        with dpg.menu_bar():
            with dpg.menu(label="App"):
                dpg.add_menu_item(
                    label="App Info",
                    callback=lambda: dpg.configure_item("app_info_modal", show=True),
                )
                dpg.add_menu_item(
                    label="Reconnect",
                    callback=lambda: state.connect(),
                )
            with dpg.menu(label="Settings"):
                dpg.add_menu_item(label="Preferences (Coming Soon)", enabled=False)

        with dpg.group(horizontal=True):
            if _logo_tag:
                dpg.add_image(_logo_tag, width=28, height=28)
                dpg.add_spacer(width=6)
            dpg.add_text(APP_TITLE, color=(160, 160, 170))
            dpg.add_spacer(width=16)
            dpg.add_text("IP:", color=(160, 160, 170))
            dpg.add_input_text(tag="tb_ip_input",
                               default_value=TCP_SERVER_IP,
                               width=120, on_enter=True,
                               callback=_apply_conn)
            dpg.add_text("PORT:", color=(160, 160, 170))
            dpg.add_input_int(tag="tb_port_input",
                              default_value=TCP_SERVER_PORT,
                              width=100, on_enter=True,
                              min_value=1, max_value=65535,
                              step=0,
                              callback=_apply_conn)
            dpg.add_text("● Connected", tag="conn_label", color=(140, 200, 140))
            dpg.add_spacer(width=8)
            dpg.add_button(label="Reconnect", tag="btn_reconnect",
                           callback=lambda: state.connect(), show=False)
        dpg.add_separator()

        # ── 상단: 커맨드(좌) | 모니터(우) ────────────────
        with dpg.group(horizontal=True):
            # 커맨드 패널 — 내부 컨텐츠가 길므로 세로 스크롤만 허용
            with dpg.child_window(tag="cmd_window",
                                  width=CMD_W, height=_top_h(),
                                  border=True,
                                  no_scrollbar=False):
                cmd_panel.build(parent="cmd_window")

            # 모니터 탭 — tab_bar 는 고정, 콘텐츠 스크롤은 탭 내부 child_window 가 담당
            with dpg.child_window(tag="mon_window",
                                  width=_mon_w(), height=_top_h(),
                                  border=True,
                                  no_scrollbar=True, no_scroll_with_mouse=True):
                # ── 커스텀 탭 버튼 행 ──────────────────────────
                with dpg.group(horizontal=True):
                    dpg.add_button(label=" UDP Monitor ", tag="tab_btn_udp",
                                   callback=lambda: _select_tab("udp"))
                    dpg.add_button(label=" Lane Control ", tag="tab_btn_lc",
                                   callback=lambda: _select_tab("lc"))
                    dpg.add_button(label=" Path Follow ", tag="tab_btn_au",
                                   callback=lambda: _select_tab("au"))
                    dpg.add_button(label=" File Playback ", tag="tab_btn_fp",
                                   callback=lambda: _select_tab("fp"))
                    dpg.add_button(label=" Transform Playback ", tag="tab_btn_tfp",
                                   callback=lambda: _select_tab("tfp"))
                dpg.add_separator()

                # ── 탭 콘텐츠 (한 번에 하나만 표시) ───────────
                with dpg.child_window(tag="mon_scroll",
                                      width=-1, height=-1,
                                      border=False, show=True):
                    monitor_panel.build(parent="mon_scroll")

                with dpg.child_window(tag="lc_scroll",
                                      width=-1, height=-1,
                                      border=False, show=False):
                    lc_panel.build(parent="lc_scroll")

                with dpg.child_window(tag="au_scroll",
                                      width=-1, height=-1,
                                      border=False, show=False):
                    au_panel.build(parent="au_scroll")

                with dpg.child_window(tag="fp_scroll",
                                      width=-1, height=-1,
                                      border=False, show=False):
                    fp_panel.build(parent="fp_scroll")

                with dpg.child_window(tag="tfp_scroll",
                                      width=-1, height=-1,
                                      border=False, show=False):
                    tfp_panel.build(parent="tfp_scroll")

                # 초기 버튼 테마 적용
                dpg.bind_item_theme("tab_btn_udp", "theme_tab_active")
                dpg.bind_item_theme("tab_btn_lc",  "theme_tab_inactive")
                dpg.bind_item_theme("tab_btn_au",  "theme_tab_inactive")
                dpg.bind_item_theme("tab_btn_fp",  "theme_tab_inactive")
                dpg.bind_item_theme("tab_btn_tfp", "theme_tab_inactive")

        # ── 하단: 로그 ────────────────────────────────────
        # no_scrollbar=True: log_child 가 자체 스크롤 담당
        with dpg.child_window(tag="log_window",
                              width=_vp_w() - PAD, height=-1,
                              border=True,
                              no_scrollbar=True, no_scroll_with_mouse=True):
            log_panel.build(parent="log_window")

    # viewport resize callback 에서 직접 DPG 레이아웃을 건드리지 않고
    # 메인 루프에서만 반영해 hit-test / layout 꼬임 가능성을 줄인다.
    _layout_state = {
        "dirty": True,
        "last_size": (-1, -1),
    }

    def _mark_layout_dirty():
        _layout_state["dirty"] = True

    def _apply_layout(force: bool = False) -> bool:
        vp_w = _vp_w()
        vp_h = _vp_h()
        size = (vp_w, vp_h)
        if not force and not _layout_state["dirty"] and size == _layout_state["last_size"]:
            return False

        _layout_state["dirty"] = False
        _layout_state["last_size"] = size

        if dpg.does_item_exist("main_window"):
            dpg.configure_item("main_window", pos=(0, 0), width=vp_w, height=vp_h)

        top_h = max(vp_h - TITLEBAR_H - LOG_H - PAD, 100)
        mon_w = max(vp_w - CMD_W - PAD * 3, 200)

        if dpg.does_item_exist("cmd_window"):
            dpg.configure_item("cmd_window", height=top_h)
        if dpg.does_item_exist("mon_window"):
            dpg.configure_item("mon_window", width=mon_w, height=top_h)
        if dpg.does_item_exist("log_window"):
            dpg.configure_item("log_window", width=vp_w - PAD)
        return True

    dpg.set_viewport_resize_callback(_mark_layout_dirty)
    return _apply_layout


# ============================================================
# Main
# ============================================================
def main():
    state = AppState()

    dpg.create_context()

    # ── 텍스처 로드 ───────────────────────────────────────────
    global _logo_tag
    _ASSET_DIR  = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets")
    _LOGO_PATH  = os.path.join(_ASSET_DIR, "Logo_SIM_V2_1_Black_80X80.PNG")
    _FOLDER_PATH = os.path.join(_ASSET_DIR, "folder_icon.png")

    with dpg.texture_registry():
        if os.path.exists(_LOGO_PATH):
            _w, _h, _ch, _data = dpg.load_image(_LOGO_PATH)
            dpg.add_static_texture(_w, _h, _data, tag="app_logo")
            _logo_tag = "app_logo"
        if os.path.exists(_FOLDER_PATH):
            _fw, _fh, _fch, _fdata = dpg.load_image(_FOLDER_PATH)
            dpg.add_static_texture(_fw, _fh, _fdata, tag="folder_icon")

    # 한글·유니코드 폰트 로드 (기본 폰트는 ASCII만 지원)
    _FONT_CANDIDATES = [
        "C:/Windows/Fonts/malgun.ttf",    # 맑은 고딕 (Windows 기본)
        "C:/Windows/Fonts/gulim.ttc",     # 굴림
    ]
    for _fp in _FONT_CANDIDATES:
        if os.path.exists(_fp):
            with dpg.font_registry():
                with dpg.font(_fp, 17) as _font:
                    dpg.add_font_range_hint(dpg.mvFontRangeHint_Default)
                    dpg.add_font_range_hint(dpg.mvFontRangeHint_Korean)
                    dpg.add_font_range(0x2000, 0x27FF)  # General Punctuation ~ Dingbats (—,→,■,▶,✗ 등)
            dpg.bind_font(_font)
            break

    # PNG → ICO 변환 후 뷰포트 아이콘 설정
    _ICO_PATH = os.path.join(os.path.dirname(__file__), "assets", "logo.ico")
    if os.path.exists(_LOGO_PATH) and not os.path.exists(_ICO_PATH):
        try:
            from PIL import Image
            Image.open(_LOGO_PATH).save(_ICO_PATH, format="ICO", sizes=[(32,32),(48,48),(64,64)])
        except Exception:
            pass

    dpg.create_viewport(
        title=APP_TITLE,
        width=W_INIT, height=H_INIT,
        min_width=W_MIN, min_height=H_MIN,
        resizable=True,
        small_icon=_ICO_PATH if os.path.exists(_ICO_PATH) else "",
        large_icon=_ICO_PATH if os.path.exists(_ICO_PATH) else "",
    )
    dpg.setup_dearpygui()

    try:
        apply_layout = build_ui(state)
    except Exception:
        import traceback
        traceback.print_exc()
        input("UI build failed. Press Enter to exit.")
        return

    dpg.set_primary_window("main_window", True)
    dpg.show_viewport()
    apply_layout(force=True)

    state.connect()

    _frame_ts = [time.monotonic()]   # 리스트로 감싸 워치독 스레드와 공유

    def _watchdog():
        while True:
            time.sleep(3)
            age = time.monotonic() - _frame_ts[0]
            if age > 2.0:
                print(f"[WATCHDOG] 메인 루프 {age:.1f}s 미실행 — freeze 감지")

    threading.Thread(target=_watchdog, daemon=True).start()

    # ── 프레임 성능 통계 ──────────────────────────────────────
    _FRAME_WARN_MS  = 150.0
    _TARGET_FPS     = 60                    # idle 시 최대 fps 제한
    _TARGET_FRAME_S = 1.0 / _TARGET_FPS
    _STAT_INTERVAL  = 5.0                   # 통계 출력 주기(초)
    _stat_t         = time.monotonic()
    _stat_render_ms = 0.0
    _stat_drain_n   = 0
    _stat_frames    = 0

    # viewport 위치/크기 추적 (window drag 진단)
    _last_vp_pos  = list(dpg.get_viewport_pos())
    _last_vp_size = [dpg.get_viewport_width(), dpg.get_viewport_height()]

    while dpg.is_dearpygui_running():
        frame_start = time.perf_counter()
        _frame_ts[0] = time.monotonic()

        # ── viewport/layout 동기화 ──────────────────────────
        apply_layout()

        # ── log flush: pending 라인 → set_value 최대 1회 ─────
        log_panel.flush()

        # ── ui_queue drain ───────────────────────────────────
        t0 = time.perf_counter()
        n_drained = ui_queue.drain()
        t1 = time.perf_counter()
        drain_ms = (t1 - t0) * 1000.0
        if drain_ms > _FRAME_WARN_MS:
            print(f"[PERF] drain() {drain_ms:.1f}ms  items={n_drained}")

        # ── DPG render ───────────────────────────────────────
        # render 직전 viewport 위치/크기 기록 (window drag 감지)
        pre_pos  = list(dpg.get_viewport_pos())
        pre_size = [dpg.get_viewport_width(), dpg.get_viewport_height()]

        dpg.render_dearpygui_frame()

        t2 = time.perf_counter()
        render_ms = (t2 - t1) * 1000.0

        # render 직후 위치/크기 변화 확인 → window drag/resize 감지
        post_pos  = list(dpg.get_viewport_pos())
        post_size = [dpg.get_viewport_width(), dpg.get_viewport_height()]
        vp_moved   = (post_pos  != pre_pos)
        vp_resized = (post_size != pre_size)

        if render_ms > _FRAME_WARN_MS:
            diag = []
            if vp_moved:
                diag.append(f"window_moved {pre_pos}→{post_pos}")
            if vp_resized:
                diag.append(f"window_resized {pre_size}→{post_size}")
            diag_str = "  " + "  ".join(diag) if diag else ""
            print(f"[PERF] render() {render_ms:.1f}ms  drain_items={n_drained}{diag_str}")

        # viewport 위치/크기 변화 별도 로그 (느린 프레임이 아니어도)
        if post_pos != _last_vp_pos:
            print(f"[DIAG] window moved  {_last_vp_pos} → {post_pos}")
            _last_vp_pos = post_pos
        if post_size != _last_vp_size:
            print(f"[DIAG] window resized {_last_vp_size} → {post_size}")
            _last_vp_size = post_size

        # ── idle fps 제한 (60fps 상한) ──────────────────────
        # render가 빠를 때 5000fps로 GPU 혹사 방지
        elapsed = t2 - frame_start
        sleep_t = _TARGET_FRAME_S - elapsed
        if sleep_t > 0.001:
            time.sleep(sleep_t)

        # ── 5초 평균 통계 ────────────────────────────────────
        _stat_render_ms += render_ms
        _stat_drain_n   += n_drained
        _stat_frames    += 1
        now = time.monotonic()
        if now - _stat_t >= _STAT_INTERVAL:
            f = max(_stat_frames, 1)
            print(
                f"[STAT] frames={_stat_frames}"
                f"  fps={_stat_frames/_STAT_INTERVAL:.1f}"
                f"  avg_render={_stat_render_ms/f:.1f}ms"
                f"  avg_drain_items={_stat_drain_n/f:.1f}"
            )
            _stat_t         = now
            _stat_render_ms = 0.0
            _stat_drain_n   = 0
            _stat_frames    = 0

    if state.auto_caller and state.auto_caller.is_alive():
        state.auto_caller.stop()
    if state.fp_caller and state.fp_caller.is_alive():
        state.fp_caller.stop()
    if state.tfp_caller and state.tfp_caller.is_alive():
        state.tfp_caller.stop()
    for _runner in state.ad_runners:
        _runner.stop()
    for _runner in state.step_ad_runners:
        _runner.stop()
    if state.lc_runner:
        state.lc_runner.stop()
    if state.receiver:
        state.receiver.stop()
    if state.tcp_sock:
        _close_socket(state.tcp_sock)
    dpg.destroy_context()


if __name__ == "__main__":
    main()
