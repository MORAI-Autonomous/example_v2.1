# panels/commands.py
from __future__ import annotations
from typing import Callable, Optional
import json
import os
import threading

import dearpygui.dearpygui as dpg
import utils.ui_queue as ui_queue
import transport.protocol_defs as proto
import transport.tcp_transport as tcp
import panels.log as log

_STATE_FILE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "config", "commands_state.json"
)

_tcp_sock                           = None
_dispatch:         Optional[Callable] = None
_toggle_auto:      Optional[Callable] = None
_timer_cancel:     threading.Event    = threading.Event()
_timer_thread:     Optional[threading.Thread] = None
_elapsed_cancel:   threading.Event    = threading.Event()
_elapsed_thread:   Optional[threading.Thread] = None

def init(tcp_sock, dispatch_fn: Callable, toggle_auto_fn: Callable) -> None:
    global _tcp_sock, _dispatch, _toggle_auto
    _tcp_sock    = tcp_sock
    _dispatch    = dispatch_fn
    _toggle_auto = toggle_auto_fn


def build(parent: int | str) -> None:
    with dpg.child_window(parent=parent, width=-1, height=-1, border=False):

        # ── Suite ──────────────────────────────────────────
        _section("SUITE")

        # Status : [Get]
        with dpg.group(horizontal=True):
            dpg.add_text("Status    :", color=(180, 180, 180, 255))
            dpg.add_button(label="Get",
                callback=lambda: _dispatch(
                    proto.MSG_TYPE_ACTIVE_SUITE_STATUS,
                    lambda rid: tcp.send_active_suite_status(_tcp_sock, rid)))

        # Browse : [파일 선택]
        # Path   : [경로 표시]
        # Load   : [Load]
        with dpg.group(horizontal=True):
            dpg.add_text("Browse    :", color=(180, 180, 180, 255))
            _folder_btn(callback=_browse_suite)
        with dpg.group(horizontal=True):
            dpg.add_text("Path      :", color=(180, 180, 180, 255))
            dpg.add_input_text(tag="suite_path", width=-1, hint="suite file path")
        with dpg.group(horizontal=True):
            dpg.add_text("Load      :", color=(180, 180, 180, 255))
            dpg.add_button(label="Load", callback=_load_suite)

        # ── Simulation Time ────────────────────────────────
        _section("SIMULATION TIME")

        # Sim Status : [Get]
        with dpg.group(horizontal=True):
            dpg.add_text("Sim Status :", color=(180, 180, 180, 255))
            dpg.add_button(label="Get",
                callback=lambda: _dispatch(
                    proto.MSG_TYPE_GET_SIMULATION_TIME_STATUS,
                    lambda rid: tcp.send_get_status(_tcp_sock, rid)))

        # Mode : [combo] [Hz / speed input] [Set]
        _MODE_ITEMS = ["Variable", "Fixed Delta", "Fixed Step"]
        with dpg.group(horizontal=True):
            dpg.add_text("Mode       :", color=(180, 180, 180, 255))
            dpg.add_combo(tag="sim_mode_combo", items=_MODE_ITEMS,
                          default_value="Fixed Step", width=105,
                          callback=_on_sim_mode_combo)
            dpg.add_input_float(tag="sim_hz", default_value=60.0,
                                min_value=1.0, max_value=1000.0,
                                format="%.1f", step=0, width=65, show=True)
            dpg.add_text("Hz", tag="sim_hz_label",
                         color=(160, 160, 160, 255), show=True)
            dpg.add_input_int(tag="sim_speed", default_value=1,
                              min_value=1, max_value=100,
                              step=0, width=55, show=False)
            dpg.add_text("x", tag="sim_speed_label",
                         color=(160, 160, 160, 255), show=False)
            dpg.add_button(label="Set", callback=_on_set_sim_mode)

        # ── Scenario ───────────────────────────────────────
        _section("SCENARIO")

        # Name : [scenario name input]
        with dpg.group(horizontal=True):
            dpg.add_text("Name      :", color=(180, 180, 180, 255))
            dpg.add_input_text(tag="sc_name", default_value="",
                               width=-1, hint="scenario name")

        # Auto Stop : [checkbox] [min] m [sec] s
        with dpg.group(horizontal=True):
            dpg.add_text("Auto Stop :", color=(180, 180, 180, 255))
            dpg.add_checkbox(tag="sc_timer_enabled", default_value=True)
            dpg.add_input_int(tag="sc_timer_min", default_value=1,
                              min_value=0, max_value=99, step=0, width=45)
            dpg.add_text("m", color=(160, 160, 160, 255))
            dpg.add_input_int(tag="sc_timer_sec", default_value=0,
                              min_value=0, max_value=59, step=0, width=45)
            dpg.add_text("s", color=(160, 160, 160, 255))

        # Elapsed
        with dpg.group(horizontal=True):
            dpg.add_text("Elapsed   :", color=(180, 180, 180, 255))
            dpg.add_text("0:00", tag="sc_elapsed_text", color=(160, 200, 160, 255))

        # Control : [Prev] [Stop] [Play] [Pause] [Next]
        _SC_OTHERS = {"◀◀": 4, "II": 2, "▶▶": 5}
        with dpg.group(horizontal=True):
            dpg.add_text("Control   :", color=(180, 180, 180, 255))
            dpg.add_button(label="◀◀", user_data=4,
                callback=lambda s, a, u: _dispatch(
                    proto.MSG_TYPE_SCENARIO_CONTROL,
                    lambda rid, cc=u: tcp.send_scenario_control(
                        _tcp_sock, rid, command=cc,
                        scenario_name=dpg.get_value("sc_name"))))
            dpg.add_button(label="■", callback=_on_sc_stop)
            dpg.add_button(label="▶", callback=_on_sc_play)
            dpg.add_button(label="II", user_data=2,
                callback=lambda s, a, u: _dispatch(
                    proto.MSG_TYPE_SCENARIO_CONTROL,
                    lambda rid, cc=u: tcp.send_scenario_control(
                        _tcp_sock, rid, command=cc,
                        scenario_name=dpg.get_value("sc_name"))))
            dpg.add_button(label="▶▶", user_data=5,
                callback=lambda s, a, u: _dispatch(
                    proto.MSG_TYPE_SCENARIO_CONTROL,
                    lambda rid, cc=u: tcp.send_scenario_control(
                        _tcp_sock, rid, command=cc,
                        scenario_name=dpg.get_value("sc_name"))))

        # Status : [Get]
        with dpg.group(horizontal=True):
            dpg.add_text("Status    :", color=(180, 180, 180, 255))
            dpg.add_button(label="Get",
                callback=lambda: _dispatch(
                    proto.MSG_TYPE_SCENARIO_STATUS,
                    lambda rid: tcp.send_scenario_status(_tcp_sock, rid)))

        # ── Object Control ─────────────────────────────────
        _section("OBJECT CONTROL")

        # ID : [entity_id input]
        with dpg.group(horizontal=True):
            dpg.add_text("ID        :", color=(180, 180, 180, 255))
            dpg.add_input_text(tag="obj_entity_id",
                               default_value="Car_1", width=140)

        # ── Manual Control (collapsing) ────────────────────
        with dpg.collapsing_header(label="Manual Control", default_open=True):
            dpg.add_spacer(height=2)
            with dpg.group(horizontal=True):
                for tag, label, default in [
                    ("mc_thr",   "Throttle",    0.4),
                    ("mc_brk",   "Brake",       0.0),
                    ("mc_steer", "Steer Angle", 0.0),
                ]:
                    dpg.add_text(label, color=(160, 160, 160, 255))
                    dpg.add_input_float(tag=tag, default_value=default,
                                        min_value=-1.0, max_value=1.0,
                                        step=0, width=60, format="%.2f")
            dpg.add_spacer(height=2)
            dpg.add_button(label="Send",
                callback=lambda: _dispatch(
                    proto.MSG_TYPE_MANUAL_CONTROL_BY_ID_COMMAND,
                    lambda rid: tcp.send_manual_control_by_id(
                        _tcp_sock, rid,
                        entity_id=dpg.get_value("obj_entity_id"),
                        throttle=dpg.get_value("mc_thr"),
                        brake=dpg.get_value("mc_brk"),
                        steer_angle=dpg.get_value("mc_steer"))))

        # ── Transform Control (collapsing) ─────────────────
        with dpg.collapsing_header(label="Transform Control", default_open=True):
            dpg.add_spacer(height=2)
            with dpg.group(horizontal=True):
                for tag, lbl in [("tc_px","px"),("tc_py","py"),("tc_pz","pz")]:
                    dpg.add_text(lbl, color=(160, 160, 160, 255))
                    dpg.add_input_float(tag=tag, default_value=0.0, step=0, width=80)
            with dpg.group(horizontal=True):
                for tag, lbl in [("tc_rx","rx"),("tc_ry","ry"),("tc_rz","rz")]:
                    dpg.add_text(lbl, color=(160, 160, 160, 255))
                    dpg.add_input_float(tag=tag, default_value=0.0, step=0, width=80)
            with dpg.group(horizontal=True):
                dpg.add_text("steer", color=(160, 160, 160, 255))
                dpg.add_input_float(tag="tc_steer", default_value=0.0, step=0, width=80)
                dpg.add_text("speed", color=(160, 160, 160, 255))
                dpg.add_input_float(tag="tc_speed", default_value=0.0, step=0, width=80)
            dpg.add_spacer(height=2)
            dpg.add_button(label="Send",
                callback=lambda: _dispatch(
                    proto.MSG_TYPE_TRANSFORM_CONTROL_BY_ID_COMMAND,
                    lambda rid: tcp.send_transform_control_by_id(
                        _tcp_sock, rid,
                        entity_id=dpg.get_value("obj_entity_id"),
                        pos_x=dpg.get_value("tc_px"), pos_y=dpg.get_value("tc_py"),
                        pos_z=dpg.get_value("tc_pz"), rot_x=dpg.get_value("tc_rx"),
                        rot_y=dpg.get_value("tc_ry"), rot_z=dpg.get_value("tc_rz"),
                        steer_angle=dpg.get_value("tc_steer"),
                        speed=dpg.get_value("tc_speed"))))

        # ── Fixed Step ─────────────────────────────────────
        _section("FIXED STEP")

        # Step : count [input] [FixedStep]
        with dpg.group(horizontal=True):
            dpg.add_text("Step      :", color=(180, 180, 180, 255))
            dpg.add_text("count", color=(160, 160, 160, 255))
            dpg.add_input_int(tag="fs_step_count", default_value=1,
                              min_value=1, max_value=9999, width=60, step=0)
            dpg.add_button(label="▶ FixedStep",
                callback=lambda: _dispatch(
                    proto.MSG_TYPE_FIXED_STEP,
                    lambda rid: tcp.send_fixed_step(
                        _tcp_sock, rid,
                        step_count=dpg.get_value("fs_step_count"))))

        # SaveData : [Save]
        with dpg.group(horizontal=True):
            dpg.add_text("SaveData  :", color=(180, 180, 180, 255))
            dpg.add_button(label="Save",
                callback=lambda: _dispatch(
                    proto.MSG_TYPE_SAVE_DATA,
                    lambda rid: tcp.send_save_data(_tcp_sock, rid)))

        # Auto : [max_calls] [▶▶ AutoCaller]
        with dpg.group(horizontal=True):
            dpg.add_text("Auto      :", color=(180, 180, 180, 255))
            dpg.add_input_int(tag="auto_max_calls",
                              default_value=proto.MAX_CALL_NUM,
                              min_value=1, max_value=999999,
                              step=0, width=80)
            dpg.add_button(label="▶▶ AutoCaller", tag="btn_auto",
                           callback=_on_auto_toggle)

        # Progress bar
        with dpg.group(horizontal=True):
            dpg.add_text("          ", color=(180, 180, 180, 255))
            dpg.add_text("0", tag="auto_progress_text")
            dpg.add_text("/", color=(160, 160, 160, 255))
            dpg.add_text(str(proto.MAX_CALL_NUM), tag="auto_total_text",
                         color=(160, 160, 160, 255))
        dpg.add_progress_bar(tag="auto_progress_bar",
                             default_value=0.0, width=-1, overlay="")

        _load_state()


def update_auto_progress(current: int, total: int) -> None:
    def _apply(c=current, t=total):
        if not dpg.does_item_exist("auto_progress_bar"):
            return
        ratio = c / t if t > 0 else 0.0
        dpg.set_value("auto_progress_bar", ratio)
        dpg.configure_item("auto_progress_bar", overlay=f"{c}/{t}")
        dpg.set_value("auto_progress_text", str(c))
        dpg.set_value("auto_total_text", str(t))
    ui_queue.post(_apply)


def reset_auto_ui() -> None:
    def _apply():
        if not dpg.does_item_exist("btn_auto"):
            return
        dpg.configure_item("btn_auto", label="▶▶ AutoCaller")
        dpg.set_value("auto_progress_bar", 0.0)
        dpg.configure_item("auto_progress_bar", overlay="")
        dpg.set_value("auto_progress_text", "0")
    ui_queue.post(_apply)


def _on_auto_toggle() -> None:
    if _toggle_auto is None:
        return
    max_calls = dpg.get_value("auto_max_calls")
    dpg.set_value("auto_total_text", str(max_calls))
    running = _toggle_auto(max_calls)
    label = "■ Stop" if running else "▶▶ AutoCaller"
    dpg.configure_item("btn_auto", label=label)


def _on_sc_play() -> None:
    _save_state()
    _timer_cancel.set()
    _elapsed_cancel.set()
    _start_elapsed_counter()
    _start_sc_timer()
    _dispatch(
        proto.MSG_TYPE_SCENARIO_CONTROL,
        lambda rid: tcp.send_scenario_control(
            _tcp_sock, rid, command=1,
            scenario_name=dpg.get_value("sc_name")))


def _on_sc_stop() -> None:
    _timer_cancel.set()
    _elapsed_cancel.set()
    ui_queue.post(lambda: dpg.does_item_exist("sc_elapsed_text") and
                  dpg.set_value("sc_elapsed_text", "0:00"))
    _dispatch(
        proto.MSG_TYPE_SCENARIO_CONTROL,
        lambda rid: tcp.send_scenario_control(
            _tcp_sock, rid, command=3,
            scenario_name=dpg.get_value("sc_name")))


def _start_sc_timer() -> None:
    global _timer_thread, _timer_cancel
    if not dpg.get_value("sc_timer_enabled"):
        return
    total_sec = dpg.get_value("sc_timer_min") * 60 + dpg.get_value("sc_timer_sec")
    if total_sec <= 0:
        return
    _timer_cancel = threading.Event()
    cancel      = _timer_cancel
    elapsed_ev  = _elapsed_cancel   # 생성 시점의 이벤트를 캡처
    sc_name     = dpg.get_value("sc_name")

    def _run() -> None:
        timed_out = not cancel.wait(timeout=total_sec)
        # 타임아웃 후라도 수동 Stop으로 cancel이 set됐으면 중복 전송 방지
        if timed_out and not cancel.is_set():
            elapsed_ev.set()        # 캡처한 이벤트만 조작
            log.append(f"[Scenario] {total_sec}초 경과 — 자동 정지")
            _dispatch(
                proto.MSG_TYPE_SCENARIO_CONTROL,
                lambda rid: tcp.send_scenario_control(
                    _tcp_sock, rid, command=3, scenario_name=sc_name))

    _timer_thread = threading.Thread(target=_run, daemon=True)
    _timer_thread.start()


def _start_elapsed_counter() -> None:
    global _elapsed_thread, _elapsed_cancel
    auto_stop = dpg.get_value("sc_timer_enabled")
    total_sec = dpg.get_value("sc_timer_min") * 60 + dpg.get_value("sc_timer_sec")
    _elapsed_cancel = threading.Event()
    cancel = _elapsed_cancel

    def _fmt(s: int) -> str:
        return f"{s // 60}:{s % 60:02d}"

    def _run() -> None:
        elapsed = 0
        while True:
            text = f"{_fmt(elapsed)} / {_fmt(total_sec)}" if (auto_stop and total_sec > 0) else _fmt(elapsed)
            ui_queue.post(lambda t=text: dpg.does_item_exist("sc_elapsed_text") and
                          dpg.set_value("sc_elapsed_text", t))
            if cancel.wait(timeout=1.0):
                break
            elapsed += 1

    _elapsed_thread = threading.Thread(target=_run, daemon=True)
    _elapsed_thread.start()


def _browse_suite() -> None:
    def _open_dialog():
        import tkinter as tk
        from tkinter import filedialog
        root = tk.Tk()
        root.withdraw()
        root.attributes("-topmost", True)
        path = filedialog.askopenfilename(
            title="Select Suite File",
            filetypes=[("MORAI Suite", "*.msuite"), ("All files", "*.*")],
        )
        root.destroy()
        if path:
            ui_queue.post(lambda p=path: (dpg.set_value("suite_path", p), _save_state()))
    threading.Thread(target=_open_dialog, daemon=True).start()


def _load_suite() -> None:
    path = dpg.get_value("suite_path").strip()
    if not path:
        log.append("[Suite] 파일 경로가 없습니다. Browse로 파일을 선택해 주세요.", level="WARN")
        return
    _save_state()
    _dispatch(
        proto.MSG_TYPE_LOAD_SUITE,
        lambda rid: tcp.send_load_suite(_tcp_sock, rid, suite_path=path),
    )


def _on_sim_mode_combo(sender, app_data) -> None:
    is_variable = (app_data == "Variable")
    dpg.configure_item("sim_hz",          show=not is_variable)
    dpg.configure_item("sim_hz_label",    show=not is_variable)
    dpg.configure_item("sim_speed",       show=is_variable)
    dpg.configure_item("sim_speed_label", show=is_variable)


def _on_set_sim_mode() -> None:
    _MODE_MAP = {
        "Variable":    proto.TIME_MODE_VARIABLE,
        "Fixed Delta": proto.TIME_MODE_FIXED_DELTA,
        "Fixed Step":  proto.TIME_MODE_FIXED_STEP,
    }
    mode_str = dpg.get_value("sim_mode_combo")
    mode     = _MODE_MAP[mode_str]

    if mode == proto.TIME_MODE_VARIABLE:
        speed       = float(dpg.get_value("sim_speed"))
        fixed_delta = 0.0
    else:
        speed       = 1.0
        fixed_delta = 1000.0 / max(dpg.get_value("sim_hz"), 1.0)

    _dispatch(
        proto.MSG_TYPE_SET_SIMULATION_TIME_MODE_COMMAND,
        lambda rid, m=mode, fd=fixed_delta, sp=speed:
            tcp.send_simulation_time_mode_command(
                _tcp_sock, rid, mode=m, fixed_delta=fd, simulation_speed=sp),
    )


def _folder_btn(callback) -> None:
    """폴더 아이콘 버튼 — 텍스처가 없으면 텍스트 버튼으로 폴백."""
    import dearpygui.dearpygui as _dpg
    if _dpg.does_alias_exist("folder_icon"):
        _dpg.add_image_button("folder_icon", width=22, height=22, callback=callback)
    else:
        _dpg.add_button(label="...", callback=callback)


def _save_state() -> None:
    try:
        os.makedirs(os.path.dirname(_STATE_FILE), exist_ok=True)
        data = {
            "suite_path":        dpg.get_value("suite_path"),
            "sc_timer_enabled":  dpg.get_value("sc_timer_enabled"),
            "sc_timer_min":      dpg.get_value("sc_timer_min"),
            "sc_timer_sec":      dpg.get_value("sc_timer_sec"),
        }
        with open(_STATE_FILE, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
    except Exception as e:
        print(f"[Commands] save state error: {e}")


def _load_state() -> None:
    if not os.path.isfile(_STATE_FILE):
        return
    try:
        with open(_STATE_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
        if data.get("suite_path") and dpg.does_item_exist("suite_path"):
            dpg.set_value("suite_path", data["suite_path"])
        if dpg.does_item_exist("sc_timer_enabled"):
            dpg.set_value("sc_timer_enabled", data.get("sc_timer_enabled", True))
        if dpg.does_item_exist("sc_timer_min"):
            dpg.set_value("sc_timer_min", data.get("sc_timer_min", 1))
        if dpg.does_item_exist("sc_timer_sec"):
            dpg.set_value("sc_timer_sec", data.get("sc_timer_sec", 0))
    except Exception as e:
        print(f"[Commands] load state error: {e}")


def _section(label: str) -> None:
    dpg.add_spacer(height=6)
    dpg.add_text(label, color=(200, 200, 100, 255))
    dpg.add_separator()
    dpg.add_spacer(height=2)



