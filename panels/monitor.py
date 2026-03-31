# panels/monitor.py
import math
import socket
import threading

import dearpygui.dearpygui as dpg

import ui_queue
from vehicle_info_receiver import parse_vehicle_info_payload as _parse_vi
from vehicle_info_with_wheel_receiver import parse_vehicle_info_payload as _parse_vi_wheel
from collision_event_receiver import parse_collision_event_payload as _parse_col

# ─── Receiver registry ──────────────────────────────────────────
_MAX_SPEED = 50.0

_NAMES = [
    "vehicle_info_receiver",
    "vehicle_info_with_wheel_receiver",
    "collision_event_receiver",
]

_DEFAULTS = {
    "vehicle_info_receiver":            ("127.0.0.1", 9097),
    "vehicle_info_with_wheel_receiver": ("127.0.0.1", 9091),
    "collision_event_receiver":         ("127.0.0.1", 9094),
}

_PARSE = {
    "vehicle_info_receiver":            _parse_vi,
    "vehicle_info_with_wheel_receiver": _parse_vi_wheel,
    "collision_event_receiver":         _parse_col,
}

# ─── Per-receiver persistent state ──────────────────────────────
_recv_state = {
    name: {
        "ip":      _DEFAULTS[name][0],
        "port":    _DEFAULTS[name][1],
        "running": False,
        "thread":  None,
        "sock":    None,
    }
    for name in _NAMES
}

_current: str = _NAMES[0]

# ─── UI tag map ─────────────────────────────────────────────────
_T = {
    # Control bar
    "combo":        "mon_combo",
    "ip":           "mon_ip",
    "port":         "mon_port",
    "btn_start":    "mon_btn_start",
    "btn_stop":     "mon_btn_stop",
    "status":       "mon_status",
    # Vehicle Info display group
    "vi_group":     "mon_vi_group",
    "vi_id":        "mon_vi_id",
    "vi_time":      "mon_vi_time",
    "vi_loc_x":     "mon_vi_loc_x",  "vi_loc_y": "mon_vi_loc_y",  "vi_loc_z": "mon_vi_loc_z",
    "vi_rot_x":     "mon_vi_rot_x",  "vi_rot_y": "mon_vi_rot_y",  "vi_rot_z": "mon_vi_rot_z",
    "vi_vel_x":     "mon_vi_vel_x",  "vi_vel_y": "mon_vi_vel_y",  "vi_vel_z": "mon_vi_vel_z",
    "vi_acc_x":     "mon_vi_acc_x",  "vi_acc_y": "mon_vi_acc_y",  "vi_acc_z": "mon_vi_acc_z",
    "vi_ang_x":     "mon_vi_ang_x",  "vi_ang_y": "mon_vi_ang_y",  "vi_ang_z": "mon_vi_ang_z",
    "vi_thr":       "mon_vi_thr",
    "vi_brk":       "mon_vi_brk",
    "vi_steer":     "mon_vi_steer",
    "vi_speed_bar": "mon_vi_speed_bar",
    "vi_wheel_grp": "mon_vi_wheel_grp",   # shown only for vi_wheel receiver
    "vi_wheel_items": "mon_vi_wheel_items",
    # Collision display group
    "col_group":    "mon_col_group",
    "col_entity":   "mon_col_entity",
    "col_count":    "mon_col_count",
    "col_items":    "mon_col_items",
}


# ─── Generic UDP receiver thread ────────────────────────────────
class _UDPThread(threading.Thread):
    def __init__(self, sock: socket.socket, parse_fn, on_data, on_error):
        super().__init__(daemon=True)
        self.sock     = sock
        self.parse_fn = parse_fn
        self.on_data  = on_data
        self.on_error = on_error
        self.running  = True

    def stop(self) -> None:
        self.running = False

    def run(self) -> None:
        while self.running:
            try:
                data, addr = self.sock.recvfrom(65535)
                parsed = self.parse_fn(data)
                if parsed is not None and "error" not in parsed:
                    self.on_data(parsed, addr)
            except OSError:
                if self.running:
                    ui_queue.post(self.on_error)
                break


# ─── Build ──────────────────────────────────────────────────────
def build(parent: int | str) -> None:
    global _current
    _current = _NAMES[0]

    # ── 수신기 선택 콤보 ──────────────────────────────────────
    dpg.add_text("UDP Receiver", color=(200, 200, 100, 255), parent=parent)
    dpg.add_separator(parent=parent)

    dpg.add_combo(
        tag=_T["combo"],
        items=_NAMES,
        default_value=_current,
        width=-1,
        callback=_on_combo,
        parent=parent,
    )
    dpg.add_spacer(height=4, parent=parent)

    # ── IP / Port ─────────────────────────────────────────────
    with dpg.group(horizontal=True, parent=parent):
        dpg.add_text("IP:", color=(160, 160, 170))
        dpg.add_input_text(
            tag=_T["ip"],
            default_value=_recv_state[_current]["ip"],
            width=130,
        )
        dpg.add_spacer(width=8)
        dpg.add_text("Port:", color=(160, 160, 170))
        dpg.add_input_int(
            tag=_T["port"],
            default_value=_recv_state[_current]["port"],
            width=80,
            step=0,
            min_value=1,
            max_value=65535,
        )

    # ── Start / Stop / Status ─────────────────────────────────
    with dpg.group(horizontal=True, parent=parent):
        dpg.add_button(label="Start", tag=_T["btn_start"],
                       callback=_on_start, width=70)
        dpg.add_button(label="Stop",  tag=_T["btn_stop"],
                       callback=_on_stop,  width=70)
        dpg.add_text("● Stopped", tag=_T["status"], color=(180, 80, 80, 255))

    dpg.add_separator(parent=parent)

    # ── Vehicle Info display ──────────────────────────────────
    with dpg.group(tag=_T["vi_group"], parent=parent, show=True):
        dpg.add_text("Vehicle Info", color=(200, 200, 100, 255))
        dpg.add_separator()

        with dpg.group(horizontal=True):
            dpg.add_text("ID :")
            dpg.add_text("-", tag=_T["vi_id"])
            dpg.add_spacer(width=20)
            dpg.add_text("Time :")
            dpg.add_text("-", tag=_T["vi_time"])

        dpg.add_spacer(height=6)
        _vi_section(_T["vi_group"], "Location (m)",   "vi_loc")
        _vi_section(_T["vi_group"], "Rotation (deg)", "vi_rot")
        _vi_section(_T["vi_group"], "Velocity (m/s)", "vi_vel")
        _vi_section(_T["vi_group"], "Acceleration",   "vi_acc")
        _vi_section(_T["vi_group"], "Angular Vel",    "vi_ang")

        dpg.add_spacer(height=6)
        dpg.add_text("Control", color=(200, 200, 100, 255))
        dpg.add_separator()
        with dpg.table(header_row=True, borders_innerV=True, resizable=True):
            dpg.add_table_column(label="Throttle")
            dpg.add_table_column(label="Brake")
            dpg.add_table_column(label="Steer")
            with dpg.table_row():
                dpg.add_text("0.000", tag=_T["vi_thr"])
                dpg.add_text("0.000", tag=_T["vi_brk"])
                dpg.add_text("0.000", tag=_T["vi_steer"])

        dpg.add_spacer(height=4)
        dpg.add_text("Speed")
        dpg.add_progress_bar(
            tag=_T["vi_speed_bar"],
            default_value=0.0,
            overlay="0.00 m/s",
            width=-1,
        )

        # Wheel section — visible only for vi_wheel receiver
        with dpg.group(tag=_T["vi_wheel_grp"], show=False):
            dpg.add_spacer(height=6)
            dpg.add_text("Wheels", color=(200, 200, 100, 255))
            dpg.add_separator()
            dpg.add_group(tag=_T["vi_wheel_items"])

    # ── Collision display ─────────────────────────────────────
    with dpg.group(tag=_T["col_group"], parent=parent, show=False):
        dpg.add_text("Collision Events", color=(200, 200, 100, 255))
        dpg.add_separator()

        with dpg.group(horizontal=True):
            dpg.add_text("Entity:", color=(180, 180, 180, 255))
            dpg.add_text("-", tag=_T["col_entity"])
            dpg.add_spacer(width=16)
            dpg.add_text("Count:", color=(180, 180, 180, 255))
            dpg.add_text("0", tag=_T["col_count"])

        dpg.add_spacer(height=4)
        dpg.add_group(tag=_T["col_items"])

    _refresh_buttons()


# ─── Section helper ──────────────────────────────────────────────
def _vi_section(parent, label: str, prefix: str) -> None:
    dpg.add_text(label, color=(180, 180, 180, 255), parent=parent)
    with dpg.table(parent=parent, header_row=True,
                   borders_innerV=True, resizable=True):
        for axis in ["X", "Y", "Z"]:
            dpg.add_table_column(label=axis)
        with dpg.table_row():
            for axis in ["x", "y", "z"]:
                dpg.add_text("0.000", tag=_T[f"{prefix}_{axis}"])
    dpg.add_spacer(height=2, parent=parent)


# ─── Combo callback ──────────────────────────────────────────────
def _on_combo(sender, app_data) -> None:
    global _current
    # Persist currently displayed IP/Port back to old receiver
    _recv_state[_current]["ip"]   = dpg.get_value(_T["ip"])
    _recv_state[_current]["port"] = dpg.get_value(_T["port"])

    _current = app_data

    # Load saved IP/Port for the new receiver
    dpg.set_value(_T["ip"],   _recv_state[_current]["ip"])
    dpg.set_value(_T["port"], _recv_state[_current]["port"])

    # Show/hide display groups
    is_col   = (_current == "collision_event_receiver")
    is_wheel = (_current == "vehicle_info_with_wheel_receiver")
    dpg.configure_item(_T["vi_group"],    show=not is_col)
    dpg.configure_item(_T["col_group"],   show=is_col)
    dpg.configure_item(_T["vi_wheel_grp"], show=is_wheel)

    _refresh_buttons()


# ─── Start / Stop ────────────────────────────────────────────────
def _on_start(*_) -> None:
    st = _recv_state[_current]
    if st["running"]:
        return

    ip   = dpg.get_value(_T["ip"]).strip() or "0.0.0.0"
    port = dpg.get_value(_T["port"])
    st["ip"]   = ip
    st["port"] = port

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((ip, port))
    except Exception as e:
        dpg.set_value(_T["status"], f"✗ {e}")
        dpg.configure_item(_T["status"], color=(255, 80, 80, 255))
        return

    name = _current
    thread = _UDPThread(
        sock     = sock,
        parse_fn = _PARSE[name],
        on_data  = lambda p, a, n=name: _on_data(n, p, a),
        on_error = lambda n=name: _on_thread_error(n),
    )
    st["sock"]    = sock
    st["thread"]  = thread
    st["running"] = True
    thread.start()

    _refresh_buttons()


def _on_stop(*_) -> None:
    st = _recv_state[_current]
    if not st["running"]:
        return

    st["running"] = False
    if st["thread"]:
        st["thread"].stop()
        st["thread"] = None
    if st["sock"]:
        try:
            st["sock"].close()
        except Exception:
            pass
        st["sock"] = None

    _refresh_buttons()


def _on_thread_error(name: str) -> None:
    """Called on UI thread via ui_queue when a receiver thread dies unexpectedly."""
    st = _recv_state[name]
    st["running"] = False
    st["thread"]  = None
    st["sock"]    = None
    if name == _current:
        _refresh_buttons()


# ─── Button / status refresh ─────────────────────────────────────
def _refresh_buttons() -> None:
    running = _recv_state[_current]["running"]
    if running:
        dpg.set_value(_T["status"], "● Running")
        dpg.configure_item(_T["status"], color=(80, 200, 80, 255))
    else:
        dpg.set_value(_T["status"], "● Stopped")
        dpg.configure_item(_T["status"], color=(180, 80, 80, 255))


# ─── Data routing ────────────────────────────────────────────────
def _on_data(name: str, parsed: dict, addr) -> None:
    if name in ("vehicle_info_receiver", "vehicle_info_with_wheel_receiver"):
        has_wheel = (name == "vehicle_info_with_wheel_receiver")
        ui_queue.post(lambda p=parsed, w=has_wheel: _apply_vi(p, w))
    elif name == "collision_event_receiver":
        ui_queue.post(lambda p=parsed: _apply_col(p))


# ─── Vehicle Info display update ─────────────────────────────────
def _apply_vi(p: dict, has_wheel: bool) -> None:
    if not dpg.does_item_exist(_T["vi_id"]):
        return

    dpg.set_value(_T["vi_id"],   p["id"])
    dpg.set_value(_T["vi_time"], f"{p['seconds']}s {p['nanos']}ns")

    for prefix, key in [
        ("vi_loc", "location"),
        ("vi_rot", "rotation"),
        ("vi_vel", "local_velocity"),
        ("vi_acc", "local_acceleration"),
        ("vi_ang", "angular_velocity"),
    ]:
        for axis in ["x", "y", "z"]:
            dpg.set_value(_T[f"{prefix}_{axis}"], f"{p[key][axis]:.3f}")

    ctrl = p["control"]
    dpg.set_value(_T["vi_thr"],   f"{ctrl['throttle']:.3f}")
    dpg.set_value(_T["vi_brk"],   f"{ctrl['brake']:.3f}")
    dpg.set_value(_T["vi_steer"], f"{ctrl['steer_angle']:.3f}")

    vel   = p["local_velocity"]
    speed = math.sqrt(vel["x"]**2 + vel["y"]**2 + vel["z"]**2)
    dpg.set_value(_T["vi_speed_bar"], min(speed / _MAX_SPEED, 1.0))
    dpg.configure_item(_T["vi_speed_bar"], overlay=f"{speed:.2f} m/s")

    if has_wheel:
        _rebuild_wheels(p.get("wheels", []), p.get("wheel_count", 0))


def _rebuild_wheels(wheels: list, wheel_count: int) -> None:
    items_tag = _T["vi_wheel_items"]
    if not dpg.does_item_exist(items_tag):
        return
    dpg.delete_item(items_tag, children_only=True)

    dpg.add_text(f"count={wheel_count}  parsed={len(wheels)}",
                 parent=items_tag, color=(180, 180, 180, 255))
    for i, w in enumerate(wheels):
        dpg.add_text(
            f"  [{i}]  ({w['x']:.3f},  {w['y']:.3f},  {w['z']:.3f})",
            parent=items_tag, color=(200, 200, 200, 255),
        )


# ─── Collision display update ─────────────────────────────────────
def _apply_col(p: dict) -> None:
    if not dpg.does_item_exist(_T["col_entity"]):
        return

    dpg.set_value(_T["col_entity"], p["entity_id"])
    dpg.set_value(_T["col_count"],  str(p["count"]))

    items_tag = _T["col_items"]
    dpg.delete_item(items_tag, children_only=True)

    for i, it in enumerate(p.get("items", [])):
        t    = it["collision_time"]
        loc  = it["transform"]["location"]
        rot  = it["transform"]["rotation"]
        dim  = it["dimensions"]
        vel  = it["vehicle_state"]["velocity"]
        acc  = it["vehicle_state"]["acceleration"]
        spec = it["vehicle_spec"]

        dpg.add_separator(parent=items_tag)
        dpg.add_text(
            f"[{i}]  {it['collision_object_id']}   type={it['object_type']}   "
            f"time={t['seconds']}s {t['nanos']}ns",
            parent=items_tag, color=(200, 200, 100, 255),
        )
        with dpg.table(parent=items_tag, header_row=True,
                       borders_innerV=True, resizable=True):
            dpg.add_table_column(label="Location")
            dpg.add_table_column(label="Rotation")
            with dpg.table_row():
                dpg.add_text(f"({loc['x']:.2f}, {loc['y']:.2f}, {loc['z']:.2f})")
                dpg.add_text(f"({rot['x']:.2f}, {rot['y']:.2f}, {rot['z']:.2f})")

        with dpg.table(parent=items_tag, header_row=True,
                       borders_innerV=True, resizable=True):
            dpg.add_table_column(label="Velocity")
            dpg.add_table_column(label="Acceleration")
            with dpg.table_row():
                dpg.add_text(f"({vel['x']:.3f}, {vel['y']:.3f}, {vel['z']:.3f})")
                dpg.add_text(f"({acc['x']:.3f}, {acc['y']:.3f}, {acc['z']:.3f})")

        dpg.add_text(
            f"  dim=(L={dim['length']:.2f}, W={dim['width']:.2f}, H={dim['height']:.2f})   "
            f"spec=(front={spec['overhang_front']:.2f}, rear={spec['overhang_rear']:.2f}, "
            f"wb={spec['wheel_base']:.2f})",
            parent=items_tag, color=(160, 160, 160, 255),
        )
