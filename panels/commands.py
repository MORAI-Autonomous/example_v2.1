# panels/commands.py
from __future__ import annotations
from typing import Callable, Optional
import threading

import dearpygui.dearpygui as dpg
import utils.ui_queue as ui_queue
import transport.protocol_defs as proto
import transport.tcp_transport as tcp

_tcp_sock                     = None
_dispatch: Optional[Callable] = None
_toggle_auto: Optional[Callable] = None


def init(tcp_sock, dispatch_fn: Callable, toggle_auto_fn: Callable) -> None:
    global _tcp_sock, _dispatch, _toggle_auto
    _tcp_sock    = tcp_sock
    _dispatch    = dispatch_fn
    _toggle_auto = toggle_auto_fn


def build(parent: int | str) -> None:
    with dpg.child_window(parent=parent, width=-1, height=-1, border=False):

        # ── Simulation Time ────────────────────────────────
        _section("SIMULATION TIME")
        dpg.add_button(label="GetStatus",
            callback=lambda: _dispatch(
                proto.MSG_TYPE_GET_SIMULATION_TIME_STATUS,
                lambda rid: tcp.send_get_status(_tcp_sock, rid)))
        with dpg.group(horizontal=True):
            dpg.add_text("Hz", color=(180, 180, 180, 255))
            dpg.add_input_float(
                tag="sim_hz",
                default_value=20.0,
                min_value=1.0, max_value=1000.0,
                format="%.1f",
                width=75,
            )
            dpg.add_button(label="SetMode: FixedStep",
                callback=_on_set_fixed_step)

        # ── Fixed Step ─────────────────────────────────────
        _section("FIXED STEP")
        with dpg.group(horizontal=True):
            dpg.add_text("step_count")
            dpg.add_input_int(tag="fs_step_count", default_value=1,
                              min_value=1, max_value=9999, width=80)
        with dpg.group(horizontal=True):
            dpg.add_button(label="FixedStep",
                callback=lambda: _dispatch(
                    proto.MSG_TYPE_FIXED_STEP,
                    lambda rid: tcp.send_fixed_step(
                        _tcp_sock, rid,
                        step_count=dpg.get_value("fs_step_count"))))
            dpg.add_button(label="SaveData",
                callback=lambda: _dispatch(
                    proto.MSG_TYPE_SAVE_DATA,
                    lambda rid: tcp.send_save_data(_tcp_sock, rid)))

        dpg.add_spacer(height=4)
        with dpg.group(horizontal=True):
            dpg.add_button(label=">> AutoCaller", tag="btn_auto",
                           callback=_on_auto_toggle)
            dpg.add_text("0", tag="auto_progress_text")
            dpg.add_text(f"/ {proto.MAX_CALL_NUM}")
        dpg.add_progress_bar(tag="auto_progress_bar",
                             default_value=0.0, width=-1, overlay="")

        # ── Object Control ─────────────────────────────────
        _section("OBJECT CONTROL")
        with dpg.group(horizontal=True):
            dpg.add_text("entity_id")
            dpg.add_input_text(tag="obj_entity_id",
                               default_value="Car_1", width=120)

        dpg.add_text("Manual", color=(180, 180, 180, 255))
        with dpg.group(horizontal=True):
            for tag, label, default in [
                ("mc_thr",   "thr", 0.4),
                ("mc_brk",   "brk", 0.0),
                ("mc_steer", "str", 0.0),
            ]:
                dpg.add_text(label)
                dpg.add_input_float(tag=tag, default_value=default,
                                    min_value=-1.0, max_value=1.0, width=65)
        dpg.add_button(label="ManualControlById",
            callback=lambda: _dispatch(
                proto.MSG_TYPE_MANUAL_CONTROL_BY_ID_COMMAND,
                lambda rid: tcp.send_manual_control_by_id(
                    _tcp_sock, rid,
                    entity_id=dpg.get_value("obj_entity_id"),
                    throttle=dpg.get_value("mc_thr"),
                    brake=dpg.get_value("mc_brk"),
                    steer_angle=dpg.get_value("mc_steer"))))

        dpg.add_spacer(height=4)
        dpg.add_text("Transform", color=(180, 180, 180, 255))
        with dpg.group(horizontal=True):
            for tag, lbl in [("tc_px","px"),("tc_py","py"),("tc_pz","pz"),
                              ("tc_rx","rx"),("tc_ry","ry"),("tc_rz","rz")]:
                dpg.add_text(lbl)
                dpg.add_input_float(tag=tag, default_value=0.0, width=58)
        with dpg.group(horizontal=True):
            dpg.add_text("steer")
            dpg.add_input_float(tag="tc_steer", default_value=0.0, width=65)
        dpg.add_button(label="TransformControlById",
            callback=lambda: _dispatch(
                proto.MSG_TYPE_TRANSFORM_CONTROL_BY_ID_COMMAND,
                lambda rid: tcp.send_transform_control_by_id(
                    _tcp_sock, rid,
                    entity_id=dpg.get_value("obj_entity_id"),
                    pos_x=dpg.get_value("tc_px"), pos_y=dpg.get_value("tc_py"),
                    pos_z=dpg.get_value("tc_pz"), rot_x=dpg.get_value("tc_rx"),
                    rot_y=dpg.get_value("tc_ry"), rot_z=dpg.get_value("tc_rz"),
                    steer_angle=dpg.get_value("tc_steer"))))

        # ── Scenario ───────────────────────────────────────
        _section("SCENARIO")
        _CMDS = {"PLAY": 1, "PAUSE": 2, "STOP": 3, "PREV": 4, "NEXT": 5}
        with dpg.group(horizontal=True):
            dpg.add_combo(tag="sc_cmd", items=list(_CMDS.keys()),
                          default_value="PLAY", width=90)
            dpg.add_input_text(tag="sc_name", default_value="",
                               width=120, hint="scenario name")
            dpg.add_button(label="Send",
                callback=lambda: _dispatch(
                    proto.MSG_TYPE_SCENARIO_CONTROL,
                    lambda rid: tcp.send_scenario_control(
                        _tcp_sock, rid,
                        command=_CMDS[dpg.get_value("sc_cmd")],
                        scenario_name=dpg.get_value("sc_name"))))
        dpg.add_button(label="ScenarioStatus",
            callback=lambda: _dispatch(
                proto.MSG_TYPE_SCENARIO_STATUS,
                lambda rid: tcp.send_scenario_status(_tcp_sock, rid)))

        # ── Suite ──────────────────────────────────────────
        _section("SUITE")
        dpg.add_button(label="ActiveSuiteStatus",
            callback=lambda: _dispatch(
                proto.MSG_TYPE_ACTIVE_SUITE_STATUS,
                lambda rid: tcp.send_active_suite_status(_tcp_sock, rid)))
        with dpg.group(horizontal=True):
            dpg.add_input_text(tag="suite_path", width=210, hint="suite file path")
            dpg.add_button(label="Browse", callback=_browse_suite)
        dpg.add_button(label="LoadSuite", callback=_load_suite)


def update_auto_progress(current: int, total: int) -> None:
    def _apply(c=current, t=total):
        if not dpg.does_item_exist("auto_progress_bar"):
            return
        ratio = c / t if t > 0 else 0.0
        dpg.set_value("auto_progress_bar", ratio)
        dpg.configure_item("auto_progress_bar", overlay=f"{c}/{t}")
        dpg.set_value("auto_progress_text", str(c))
    ui_queue.post(_apply)


def reset_auto_ui() -> None:
    def _apply():
        if not dpg.does_item_exist("btn_auto"):
            return
        dpg.configure_item("btn_auto", label=">> AutoCaller")
        dpg.set_value("auto_progress_bar", 0.0)
        dpg.configure_item("auto_progress_bar", overlay="")
        dpg.set_value("auto_progress_text", "0")
    ui_queue.post(_apply)


def _on_auto_toggle() -> None:
    if _toggle_auto is None:
        return
    running = _toggle_auto()
    label = "[] Stop" if running else ">> AutoCaller"
    dpg.configure_item("btn_auto", label=label)


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
            ui_queue.post(lambda p=path: dpg.set_value("suite_path", p))
    threading.Thread(target=_open_dialog, daemon=True).start()


def _load_suite() -> None:
    path = dpg.get_value("suite_path").strip()
    if not path:
        return
    _dispatch(
        proto.MSG_TYPE_LOAD_SUITE,
        lambda rid: tcp.send_load_suite(_tcp_sock, rid, suite_path=path),
    )


def _on_set_fixed_step() -> None:
    hz         = max(dpg.get_value("sim_hz"), 1.0)
    fixed_delta = 1000.0 / hz          # Hz → ms
    _dispatch(
        proto.MSG_TYPE_SET_SIMULATION_TIME_MODE_COMMAND,
        lambda rid, fd=fixed_delta: tcp.send_simulation_time_mode_command(
            _tcp_sock, rid,
            mode=proto.TIME_MODE_FIXED_STEP,
            fixed_delta=fd,
        ),
    )


def _section(label: str) -> None:
    dpg.add_spacer(height=6)
    dpg.add_text(label, color=(200, 200, 100, 255))
    dpg.add_separator()
    dpg.add_spacer(height=2)