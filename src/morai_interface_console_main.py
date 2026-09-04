from __future__ import annotations

# morai_interface_console.py
import ipaddress
import json
import os
import socket
import threading
import time
from typing import Optional

import dearpygui.dearpygui as dpg

from transport.protocol_defs import *
import transport.tcp_transport as tcp
import transport.tcp_thread as tcp_thread_mod
import runners.auto_caller as ac
import runners.ad_runner as AdRunner_mod
from runners.ad_runner import AdRunner
from runners.ros2_ad_runner import Ros2AdRunner
from runners.step_ad_runner import StepAdRunner
from runners.lane_runner import LaneRunner
import utils.ui_queue as ui_queue
import panels.log               as log_panel
import panels.monitor            as monitor_panel
import panels.commands           as cmd_panel
import panels.lane_control_panel  as lc_panel
import panels.camera_sensor_panel as cam_sensor_panel
import panels.radar_sensor_panel as radar_sensor_panel
import panels.lidar_panel as lidar_panel
import panels.autonomous_panel    as au_panel
import panels.file_playback_panel as fp_panel
import panels.transform_playback_panel as tfp_panel
import panels.udp_control_panel as udp_ctrl_panel
import panels.object_control as obj_panel
import panels.traffic_scenario as traffic_panel
import transport.commands as udp_cmd
from utils.project_paths import ROOT_DIR

INTERFACE_CONSOLE_TITLE = "MORAI Interface Console"
_logo_tag = None   # Loaded in main() when the logo texture is available.

# Layout constants
W_INIT, H_INIT = 1400, 1200  # Initial viewport size.
W_MIN,  H_MIN  = 900,  600   # Minimum viewport size.
_BASE_DIR = str(ROOT_DIR)
_INTERFACE_CONSOLE_STATE_FILE = os.path.join(
    _BASE_DIR,
    "config",
    "interface_console_state.json",
)
_DEFAULT_TAB = "udp"
_TAB_KEYS = {
    "udp", "udp_ctrl", "cam_sensor", "radar_sensor", "lidar_sensor",
    "object_control", "traffic", "lc", "au", "fp", "tfp",
}
CMD_W      = 400        # Fixed command panel width.
LOG_H      = 280        # Fixed log panel height.
TITLEBAR_H = 38         # Title bar plus separator height.
PAD        = 12         # Horizontal and bottom padding.

# Viewport size helpers used by responsive layout code.
def _vp_w():  return dpg.get_viewport_width()
def _vp_h():  return dpg.get_viewport_height()
def _top_h(): return max(_vp_h() - TITLEBAR_H - LOG_H - PAD, 100)
def _mon_w(): return max(_vp_w() - CMD_W - PAD * 3, 200)


def _load_interface_console_state() -> dict:
    try:
        with open(_INTERFACE_CONSOLE_STATE_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
        return data if isinstance(data, dict) else {}
    except Exception:
        return {}


def _save_interface_console_state(state=None) -> None:
    try:
        width = int(dpg.get_viewport_width())
        height = int(dpg.get_viewport_height())
        if width < W_MIN or height < H_MIN:
            return
        tcp_ip = TCP_SERVER_IP
        tcp_port = TCP_SERVER_PORT
        try:
            if dpg.does_item_exist("tb_ip_input"):
                value = str(dpg.get_value("tb_ip_input")).strip()
                if value:
                    tcp_ip = value
            if dpg.does_item_exist("tb_port_input"):
                tcp_port = int(dpg.get_value("tb_port_input"))
            tcp_ip, tcp_port = _validate_tcp_endpoint(tcp_ip, tcp_port)
        except Exception:
            tcp_ip = TCP_SERVER_IP
            tcp_port = TCP_SERVER_PORT
            pass

        os.makedirs(os.path.dirname(_INTERFACE_CONSOLE_STATE_FILE), exist_ok=True)
        with open(_INTERFACE_CONSOLE_STATE_FILE, "w", encoding="utf-8") as f:
            json.dump(
                {
                    "viewport": {
                        "width": width,
                        "height": height,
                    },
                    "selected_tab": _normalize_tab(
                        getattr(state, "selected_tab", _DEFAULT_TAB)
                    ),
                    "tcp": {
                        "ip": tcp_ip,
                        "port": tcp_port,
                    },
                },
                f,
                indent=2,
            )
    except Exception:
        pass


def _initial_viewport_size() -> tuple:
    state = _load_interface_console_state()
    viewport = state.get("viewport")
    if not isinstance(viewport, dict):
        return W_INIT, H_INIT

    try:
        width = max(W_MIN, int(viewport.get("width", W_INIT)))
        height = max(H_MIN, int(viewport.get("height", H_INIT)))
    except Exception:
        return W_INIT, H_INIT

    return width, height


def _normalize_tab(tab_name: str) -> str:
    return tab_name if tab_name in _TAB_KEYS else _DEFAULT_TAB


def _initial_selected_tab() -> str:
    state = _load_interface_console_state()
    return _normalize_tab(str(state.get("selected_tab", _DEFAULT_TAB)))


def _initial_tcp_endpoint() -> tuple:
    state = _load_interface_console_state()
    tcp_state = state.get("tcp")
    if not isinstance(tcp_state, dict):
        return TCP_SERVER_IP, TCP_SERVER_PORT

    try:
        return _validate_tcp_endpoint(
            tcp_state.get("ip", TCP_SERVER_IP),
            tcp_state.get("port", TCP_SERVER_PORT),
        )
    except Exception:
        return TCP_SERVER_IP, TCP_SERVER_PORT


def _apply_initial_tcp_endpoint() -> None:
    global TCP_SERVER_IP, TCP_SERVER_PORT
    TCP_SERVER_IP, TCP_SERVER_PORT = _initial_tcp_endpoint()


def _validate_tcp_endpoint(ip_value, port_value) -> tuple[str, int]:
    ip = str(ip_value).strip()
    try:
        ip_obj = ipaddress.ip_address(ip)
    except ValueError:
        raise ValueError(f"Invalid TCP IP: {ip!r}")
    if ip_obj.version != 4:
        raise ValueError(f"TCP IP must be IPv4: {ip!r}")

    try:
        port = int(port_value)
    except (TypeError, ValueError):
        raise ValueError(f"Invalid TCP port: {port_value!r}")
    if not (1 <= port <= 65535):
        raise ValueError(f"TCP port out of range: {port}")
    return str(ip_obj), port


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
    s.settimeout(5.0)   # Bound blocking sends so the UI does not hang.
    return s

def _close_socket(sock):
    for fn in (lambda: sock.shutdown(socket.SHUT_RDWR), sock.close):
        try: fn()
        except Exception: pass

def _set_conn_status(connected: bool, editing_enabled: Optional[bool] = None):
    if editing_enabled is None:
        editing_enabled = not connected

    def _apply():
        tcp_ip = TCP_SERVER_IP
        tcp_port = TCP_SERVER_PORT
        try:
            if dpg.does_item_exist("tb_ip_input"):
                value = str(dpg.get_value("tb_ip_input")).strip()
                if value:
                    tcp_ip = value
            if dpg.does_item_exist("tb_port_input"):
                tcp_port = int(dpg.get_value("tb_port_input"))
        except Exception:
            pass

        dpg.configure_item("conn_label",
            color=(100, 255, 100, 255) if connected else (255, 80, 80, 255))
        dpg.set_value("conn_label", "Connected" if connected else "Disconnected")
        dpg.set_value("tb_ip_display", tcp_ip)
        dpg.set_value("tb_port_display", str(tcp_port))
        dpg.configure_item("tb_ip_input", show=editing_enabled, enabled=editing_enabled)
        dpg.configure_item("tb_port_input", show=editing_enabled, enabled=editing_enabled)
        dpg.configure_item("tb_ip_display_box", show=not editing_enabled)
        dpg.configure_item("tb_port_display_box", show=not editing_enabled)
        dpg.configure_item("btn_reconnect", show=not connected)
        dpg.configure_item("btn_disconnect", show=connected)

    ui_queue.post(_apply)


# ============================================================
# InterfaceConsoleState
# ============================================================
class InterfaceConsoleState:
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
        self.selected_tab = _initial_selected_tab()
        self._connecting = False
        self._disconnect_requested = False
        self._conn_lock  = threading.Lock()

    def send_udp_control(self, template_name: str, parser, ip: str, port: int, values: dict) -> None:
        if parser.fields_segment is None:
            raise RuntimeError("control template has no fields segment")

        payload = udp_cmd.build_udp_payload(parser.fields_segment.fields, values)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp_sock:
            udp_cmd.send_udp_payload(udp_sock, payload, ip, port, template_name.replace(".tmpl", ""))
        log_panel.append(
            f"[UDP CTRL] {template_name} -> {ip}:{port} ({len(payload)}B)",
            "INFO",
        )

    def dispatch(self, msg_type: int, send_fn, on_sent=None, on_registered=None):
        if self.tcp_sock is None:
            log_panel.append("Not connected.", "WARN")
            return
        def _send():
            try:
                rid = self.rid.next()
                pending_add(self.pending, self.lock, rid, msg_type)
                if on_registered is not None:
                    on_registered(rid)
                send_fn(rid)
                if on_sent is not None:
                    on_sent(rid)
            except OSError as e:
                log_panel.append(f"Send error: {e}", "ERROR")
        threading.Thread(target=_send, daemon=True).start()

    def toggle_auto(
        self,
        max_calls: int = MAX_CALL_NUM,
        save_mode: int = SAVE_MODE_DEFAULT,
    ) -> bool:
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
                save_mode=save_mode,
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
            log_panel.append("[FP] already running.", "WARN")
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
            log_panel.append("[TFP] already running.", "WARN")
            return
        total_rows = max((len(v["rows"]) for v in vehicles), default=0)
        if total_rows <= 0:
            log_panel.append("[TFP] no rows to play.", "WARN")
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
            log_panel.append("[AD] already running.", "WARN")
            return
        interface = vehicles[0].get("interface", "UDP") if vehicles else "UDP"
        if interface == "ROS2":
            if collision_cfg:
                log_panel.append("[AD:ROS2] collision mode is not supported in ROS2 mode yet.", "WARN")
            if len(vehicles) > 1:
                log_panel.append("[AD:ROS2] single-vehicle mode only; using the first vehicle.", "WARN")
            if not vehicles:
                au_panel.reset_ui()
                return
            v = vehicles[0]
            try:
                runner = Ros2AdRunner(
                    entity_id             = v["entity_id"],
                    vehicle_info_topic    = v.get("ros2_vehicle_info_topic", "/VehicleInfo"),
                    manual_control_topic  = v.get("ros2_manual_control_topic", "/ManualControl"),
                    path_file             = v.get("path", "path_link.csv"),
                    map_name              = v.get("map_name"),
                    max_speed_kph         = v.get("max_speed_kph"),
                    log_fn                = lambda msg, level="INFO", eid=v["entity_id"]:
                                               log_panel.append(f"[AD:{eid}:ROS2] {msg}", level),
                    status_cb             = au_panel.update_status,
                )
                runner.start()
                self.ad_runners.append(runner)
                log_panel.append(
                    f"[AD:{v['entity_id']}:ROS2] started "
                    f"(sub={v.get('ros2_vehicle_info_topic', '/VehicleInfo')}, "
                    f"pub={v.get('ros2_manual_control_topic', '/ManualControl')})"
                )
            except Exception as e:
                log_panel.append(f"[AD:{v['entity_id']}:ROS2] start failed: {e}", "ERROR")
            if not self.ad_runners:
                au_panel.reset_ui()
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
                    vehicle_info_source   = "TCP" if interface == "TCP" else "UDP",
                    request_id_ref        = self.rid,
                )
                runner.start()
                self.ad_runners.append(runner)
                if is_chaser:
                    role = f"Chaser ({speed_kph * 1.2:.0f} km/h)"
                elif is_target:
                    role = f"Target ({speed_kph:.0f} km/h)"
                else:
                    role = f"PathFollow (max={v.get('max_speed_kph', 0):.0f} km/h)"
                source = "TCP 0x1306" if interface == "TCP" else f"UDP port={v['vi_port']}"
                log_panel.append(f"[AD:{v['entity_id']}] started ({source}, {role})")
            except Exception as e:
                log_panel.append(f"[AD:{v['entity_id']}] start failed: {e}", "ERROR")
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
            log_panel.append(f"[AD:{entity_id}] max speed updated -> {max_speed_kph:.0f} km/h", "INFO")

    def start_step_ad(self, vehicles: list, save_mode: int = SAVE_MODE_SKIP,
                      collision_cfg: dict = None) -> None:
        if self.step_ad_runners:
            log_panel.append("[StepAD] already running.", "WARN")
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
                save_mode      = save_mode,
                log_fn         = lambda msg, level="INFO": log_panel.append(f"[StepAD] {msg}", level),
                status_cb      = au_panel.update_status,
                on_done        = _on_done,
                collision_cfg  = collision_cfg,
            )
            runner.start()
            self.step_ad_runners.append(runner)
            ids = ", ".join(v["entity_id"] for v in vehicles)
            save_mode_name = SAVE_MODE_MAP.get(save_mode, f"UNKNOWN({save_mode})")
            log_panel.append(
                f"[StepAD] started (vehicles: {ids}, save_mode: {save_mode_name})"
            )
        except Exception as e:
            log_panel.append(f"[StepAD] start failed: {e}", "ERROR")
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
            log_panel.append("[LC] already running.", "WARN")
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
            log_panel.append(f"[LC] start failed: {e}", "ERROR")
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
            self._disconnect_requested = False
        _set_conn_status(False, editing_enabled=False)

        def _run():
            if self.receiver:
                self.receiver.stop()
                self.receiver = None
            if self.tcp_sock:
                _close_socket(self.tcp_sock)
                self.tcp_sock = None

            sock = _make_tcp_socket()
            while True:
                with self._conn_lock:
                    if self._disconnect_requested:
                        _close_socket(sock)
                        self._connecting = False
                        return
                try:
                    log_panel.append(f"Connecting {TCP_SERVER_IP}:{TCP_SERVER_PORT}...", "INFO")
                    sock.connect((TCP_SERVER_IP, TCP_SERVER_PORT))
                    with self._conn_lock:
                        if self._disconnect_requested:
                            _close_socket(sock)
                            self._connecting = False
                            return
                    log_panel.append(f"Connected {TCP_SERVER_IP}:{TCP_SERVER_PORT}", "INFO")
                    self.tcp_sock = sock
                    _set_conn_status(True)
                    self.receiver = tcp_thread_mod.Receiver(
                        sock, self.pending, self.lock,
                        on_disconnect=self._on_disconnect,
                        on_response=self._on_tcp_response,
                        on_vehicle_info=self._on_vehicle_info_response,
                        on_create_object=self._on_create_object_response,
                    )
                    self.receiver.start()
                    cmd_panel.init(
                        tcp_sock=self.tcp_sock,
                        dispatch_fn=self.dispatch,
                        toggle_auto_fn=self.toggle_auto,
                    )
                    obj_panel.init(
                        tcp_sock=self.tcp_sock,
                        dispatch_fn=self.dispatch,
                    )
                    traffic_panel.init(
                        tcp_sock=self.tcp_sock,
                        dispatch_fn=self.dispatch,
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
                    log_panel.append(f"Connect failed: {e}", "ERROR")
                    _close_socket(sock)
                    _set_conn_status(False, editing_enabled=True)
                    with self._conn_lock:
                        self._connecting = False
                    return

            with self._conn_lock:
                self._connecting = False

        threading.Thread(target=_run, daemon=True).start()

    def disconnect(self):
        with self._conn_lock:
            self._disconnect_requested = True

        if self.receiver:
            self.receiver.stop()
            self.receiver = None
        if self.tcp_sock:
            _close_socket(self.tcp_sock)
            self.tcp_sock = None
        self._on_disconnect(manual=True)

    def _on_disconnect(self, manual: bool = False):
        if self._disconnect_requested:
            manual = True
        self.tcp_sock = None                                    # Make dispatch hit the null guard immediately.
        self.receiver = None
        # Release waiting ev.wait() calls so runners do not wait for timeout.
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
        cmd_panel.on_connection_lost()
        if manual:
            log_panel.append("Disconnected.", "INFO")
        else:
            log_panel.append("Connection lost. Click Reconnect to retry.", "ERROR")

    def _on_tcp_response(
        self,
        msg_type: int,
        request_id: int,
        result_code=None,
        detail_code=None,
    ) -> None:
        if msg_type == MSG_TYPE_LOAD_SUITE:
            cmd_panel.on_load_suite_response(request_id, result_code, detail_code)
        elif msg_type == MSG_TYPE_DELETE_OBJECT:
            obj_panel.on_delete_object_response(
                request_id,
                result_code,
                detail_code,
            )

    def _on_vehicle_info_response(self, request_id: int, parsed: dict) -> None:
        for runner in list(self.ad_runners) + list(self.step_ad_runners):
            handler = getattr(runner, "handle_vehicle_info_response", None)
            if handler and handler(request_id, parsed):
                return

    def _on_create_object_response(self, request_id: int, parsed: dict) -> None:
        obj_panel.on_create_object_response(request_id, parsed)


# ============================================================
# AutoCaller patch
# ============================================================
def _patch_auto_caller(caller: ac.AutoCaller, on_done=None):
    def patched_run():
        for i in range(caller.max_calls):
            if caller._stop_event.is_set():
                break

            rid = caller._next_rid()
            ev  = caller.pending_add(caller.pending, caller.lock, rid, MSG_TYPE_FIXED_STEP)
            tcp.send_fixed_step(
                caller.tcp_sock,
                rid,
                step_count=caller.step_count,
                save_mode=caller.save_mode,
            )
            if not caller._wait_or_stop(ev):
                caller.pending_pop(caller.pending, caller.lock, rid, MSG_TYPE_FIXED_STEP)
                log_panel.append(f"[AUTO][TIMEOUT] FixedStep i={i} rid={rid}", "WARN")
                break
            caller.pending_pop(caller.pending, caller.lock, rid, MSG_TYPE_FIXED_STEP)
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
    Replace AutoCaller.run with the CSV playback loop.
    For each row:
      1. Send ManualControlById (fire-and-forget)
      2. Send FixedStep and wait for ACK
      3. Send SaveData and wait for ACK
    """
    def patched_run():
        total = len(rows)
        log_panel.append(f"[FP] started: rows={total}, entity={entity_id}", "INFO")

        for i, row in enumerate(rows):
            if caller._stop_event.is_set():
                break

            # 1. ManualControlById (no ACK)
            rid = caller._next_rid()
            tcp.send_manual_control_by_id(
                caller.tcp_sock, rid,
                entity_id=entity_id,
                throttle=row['throttle'],
                brake=row['brake'],
                steer_angle=row['swa'],
            )

            if caller._stop_event.is_set():
                break

            # 2. FixedStep (wait for ACK)
            rid = caller._next_rid()
            ev  = caller.pending_add(caller.pending, caller.lock, rid, MSG_TYPE_FIXED_STEP)
            tcp.send_fixed_step(caller.tcp_sock, rid, step_count=caller.step_count)
            if not caller._wait_or_stop(ev):
                caller.pending_pop(caller.pending, caller.lock, rid, MSG_TYPE_FIXED_STEP)
                log_panel.append(f"[FP][TIMEOUT] FixedStep i={i} rid={rid}", "WARN")
                break
            caller.pending_pop(caller.pending, caller.lock, rid, MSG_TYPE_FIXED_STEP)

            if caller._stop_event.is_set():
                break

            # 3. SaveData (wait for ACK)
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

            # Progress
            fp_panel.update_progress(i + 1, total)

        stopped = caller._stop_event.is_set()
        fp_panel.reset_ui(stopped=stopped)
        log_panel.append(f"[FP] {'stopped' if stopped else 'playback complete'} ({total} rows)", "INFO")
        if on_done:
            on_done()

    caller.run = patched_run


# ============================================================
# TransformPlayback patch
# ============================================================
def _patch_tfp_caller(caller: ac.AutoCaller, vehicles: list, on_done=None):
    """
    Replace AutoCaller.run with the TransformControlById CSV playback loop.
    For each step:
      1. Send TransformControlById for all vehicles (fire-and-forget)
      2. Wait according to the CSV timestamp interval
    """
    def patched_run():
        total = max((len(v["rows"]) for v in vehicles), default=0)
        ids = ", ".join(v["entity_id"] for v in vehicles)
        log_panel.append(f"[TFP] started: rows={total}, vehicles={ids}", "INFO")

        def _row_time(i: int):
            for vehicle in vehicles:
                if i < len(vehicle["rows"]):
                    return vehicle["rows"][i].get("time_sec")
            return None

        for i in range(total):
            if caller._stop_event.is_set():
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

            if caller._stop_event.is_set():
                break

            tfp_panel.update_progress(i + 1, total)

            if i + 1 < total:
                t_cur = _row_time(i)
                t_next = _row_time(i + 1)
                if t_cur is not None and t_next is not None:
                    sleep_sec = max(0.0, min(t_next - t_cur, 0.2))
                else:
                    sleep_sec = max(caller.delay_sec, 0.02)
                if sleep_sec > 0 and caller._stop_event.wait(timeout=sleep_sec):
                    break

        stopped = caller._stop_event.is_set()
        tfp_panel.reset_ui(stopped=stopped)
        log_panel.append(f"[TFP] {'stopped' if stopped else 'playback complete'} ({total} rows)", "INFO")
        if on_done:
            on_done()

    caller.run = patched_run


# ============================================================
# UI build
# ============================================================
def build_ui(state: InterfaceConsoleState):

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

    # Custom tab button themes.
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
    with dpg.theme(tag="theme_tcp_editable"):
        with dpg.theme_component(dpg.mvInputText):
            dpg.add_theme_color(dpg.mvThemeCol_Text,          (245, 245, 248, 255))
            dpg.add_theme_color(dpg.mvThemeCol_FrameBg,       (42, 42, 52, 255))
            dpg.add_theme_color(dpg.mvThemeCol_FrameBgHovered,(54, 54, 66, 255))
            dpg.add_theme_color(dpg.mvThemeCol_FrameBgActive, (58, 58, 72, 255))

    def _select_tab(name: str) -> None:
        name = _normalize_tab(name)
        state.selected_tab = name
        dpg.configure_item("mon_scroll", show=(name == "udp"))
        dpg.configure_item("udp_ctrl_scroll", show=(name == "udp_ctrl"))
        dpg.configure_item("cam_sensor_scroll", show=(name == "cam_sensor"))
        dpg.configure_item("radar_sensor_scroll", show=(name == "radar_sensor"))
        dpg.configure_item("lidar_scroll", show=(name == "lidar_sensor"))
        dpg.configure_item("object_control_scroll", show=(name == "object_control"))
        dpg.configure_item("traffic_scroll", show=(name == "traffic"))
        dpg.configure_item("lc_scroll",  show=(name == "lc"))
        dpg.configure_item("au_scroll",  show=(name == "au"))
        dpg.configure_item("fp_scroll",  show=(name == "fp"))
        dpg.configure_item("tfp_scroll", show=(name == "tfp"))
        for tag, key in [("tab_btn_udp", "udp"), ("tab_btn_udp_ctrl", "udp_ctrl"),
                         ("tab_btn_cam_sensor", "cam_sensor"),
                         ("tab_btn_radar_sensor", "radar_sensor"),
                         ("tab_btn_lidar_sensor", "lidar_sensor"),
                         ("tab_btn_object_control", "object_control"),
                         ("tab_btn_traffic", "traffic"),
                         ("tab_btn_lc", "lc"),
                         ("tab_btn_au", "au"), ("tab_btn_fp", "fp"),
                         ("tab_btn_tfp", "tfp")]:
            dpg.bind_item_theme(tag, "theme_tab_active" if name == key else "theme_tab_inactive")

    with dpg.window(tag="app_info_modal", label="App Info", modal=True,
                    show=False, no_resize=True, width=420, height=180):
        dpg.add_text(INTERFACE_CONSOLE_TITLE)
        dpg.add_spacer(height=6)
        dpg.add_text("Python example client for MORAI simulator TCP/UDP control.")
        dpg.add_text("This menu bar is a scaffold for future app info and settings.")
        dpg.add_spacer(height=12)
        dpg.add_button(label="Close", callback=lambda: dpg.configure_item("app_info_modal", show=False))

    with dpg.window(tag="main_window", no_title_bar=True,
                    no_resize=True, no_move=True,
                    no_scrollbar=True, no_scroll_with_mouse=True):

        # Title bar
        tcp_input_values = {
            "ip": TCP_SERVER_IP,
            "port": str(TCP_SERVER_PORT),
        }

        def _on_tcp_ip_input(s=None, a=None):
            tcp_input_values["ip"] = str(a if a is not None else dpg.get_value("tb_ip_input")).strip()

        def _on_tcp_port_input(s=None, a=None):
            tcp_input_values["port"] = str(a if a is not None else dpg.get_value("tb_port_input")).strip()

        def _apply_conn(s=None, a=None):
            global TCP_SERVER_IP, TCP_SERVER_PORT
            new_ip = tcp_input_values["ip"]
            new_port = tcp_input_values["port"]
            try:
                new_ip, new_port = _validate_tcp_endpoint(new_ip, new_port)
            except ValueError as exc:
                log_panel.append(str(exc), "ERROR")
                dpg.configure_item("conn_label", color=(255, 170, 80, 255))
                dpg.set_value("conn_label", "Invalid IP/PORT")
                return
            TCP_SERVER_IP = new_ip
            TCP_SERVER_PORT = new_port
            _save_interface_console_state(state)
            state.connect()

        with dpg.menu_bar():
            with dpg.menu(label="App"):
                dpg.add_menu_item(
                    label="App Info",
                    callback=lambda: dpg.configure_item("app_info_modal", show=True),
                )
                dpg.add_menu_item(
                    label="Reconnect",
                    callback=_apply_conn,
                )
                dpg.add_menu_item(
                    label="Disconnect",
                    callback=lambda: state.disconnect(),
                )
            with dpg.menu(label="Settings"):
                dpg.add_menu_item(label="Preferences (Coming Soon)", enabled=False)

        with dpg.group(horizontal=True):
            if _logo_tag:
                dpg.add_image(_logo_tag, width=28, height=28)
                dpg.add_spacer(width=6)
            dpg.add_text(INTERFACE_CONSOLE_TITLE, color=(160, 160, 170))
            dpg.add_spacer(width=16)
            dpg.add_text("IP:", color=(160, 160, 170))
            dpg.add_input_text(tag="tb_ip_input",
                               default_value=TCP_SERVER_IP,
                               width=120,
                               callback=_on_tcp_ip_input)
            dpg.bind_item_theme("tb_ip_input", "theme_tcp_editable")
            with dpg.child_window(tag="tb_ip_display_box",
                                  width=120, height=24,
                                  border=False,
                                  no_scrollbar=True,
                                  no_scroll_with_mouse=True,
                                  show=False):
                dpg.add_text(TCP_SERVER_IP, tag="tb_ip_display",
                             color=(112, 112, 120))
            dpg.add_text("PORT:", color=(160, 160, 170))
            dpg.add_input_text(tag="tb_port_input",
                               default_value=str(TCP_SERVER_PORT),
                               width=100,
                               callback=_on_tcp_port_input)
            dpg.bind_item_theme("tb_port_input", "theme_tcp_editable")
            with dpg.child_window(tag="tb_port_display_box",
                                  width=100, height=24,
                                  border=False,
                                  no_scrollbar=True,
                                  no_scroll_with_mouse=True,
                                  show=False):
                dpg.add_text(str(TCP_SERVER_PORT), tag="tb_port_display",
                             color=(112, 112, 120))
            dpg.add_text("Connected", tag="conn_label", color=(140, 200, 140))
            dpg.add_spacer(width=8)
            dpg.add_button(label="Reconnect", tag="btn_reconnect",
                           width=90, callback=_apply_conn, show=False)
            dpg.add_button(label="Disconnect", tag="btn_disconnect",
                           width=90, callback=lambda: state.disconnect(), show=False)
        dpg.add_separator()

        # Top area: command panel on the left, monitor panels on the right.
        with dpg.group(horizontal=True):
            # Command panel: content can be tall, so it owns vertical scrolling.
            with dpg.child_window(tag="cmd_window",
                                  width=CMD_W, height=_top_h(),
                                  border=True,
                                  no_scrollbar=False):
                cmd_panel.build(parent="cmd_window")

            # Monitor tabs: custom buttons stay fixed, each tab child owns scrolling.
            with dpg.child_window(tag="mon_window",
                                  width=_mon_w(), height=_top_h(),
                                  border=True,
                                  no_scrollbar=True, no_scroll_with_mouse=True):
                # Custom tab button row
                with dpg.group(horizontal=True):
                    dpg.add_button(label=" UDP Monitor ", tag="tab_btn_udp",
                                   callback=lambda: _select_tab("udp"))
                    dpg.add_button(label=" UDP Control ", tag="tab_btn_udp_ctrl",
                                   callback=lambda: _select_tab("udp_ctrl"))
                    dpg.add_button(label=" Camera Sensor ", tag="tab_btn_cam_sensor",
                                   callback=lambda: _select_tab("cam_sensor"))
                    dpg.add_button(label=" Radar Sensor ", tag="tab_btn_radar_sensor",
                                   callback=lambda: _select_tab("radar_sensor"))
                    dpg.add_button(label=" LiDAR Sensor ", tag="tab_btn_lidar_sensor",
                                   callback=lambda: _select_tab("lidar_sensor"))
                    dpg.add_button(label=" Object Control ", tag="tab_btn_object_control",
                                   callback=lambda: _select_tab("object_control"))
                    dpg.add_button(label=" Traffic Scenario ", tag="tab_btn_traffic",
                                   callback=lambda: _select_tab("traffic"))
                    dpg.add_button(label=" Lane Control ", tag="tab_btn_lc",
                                   callback=lambda: _select_tab("lc"))
                    dpg.add_button(label=" Path Follow ", tag="tab_btn_au",
                                   callback=lambda: _select_tab("au"))
                    dpg.add_button(label=" File Playback ", tag="tab_btn_fp",
                                   callback=lambda: _select_tab("fp"))
                    dpg.add_button(label=" Transform Playback ", tag="tab_btn_tfp",
                                   callback=lambda: _select_tab("tfp"))
                dpg.add_separator()

                # Tab content; only one child is visible at a time.
                with dpg.child_window(tag="mon_scroll",
                                      width=-1, height=-1,
                                      border=False, show=True):
                    monitor_panel.build(parent="mon_scroll")

                with dpg.child_window(tag="udp_ctrl_scroll",
                                      width=-1, height=-1,
                                      border=False, show=False):
                    udp_ctrl_panel.build(parent="udp_ctrl_scroll")

                with dpg.child_window(tag="cam_sensor_scroll",
                                      width=-1, height=-1,
                                      border=False, show=False):
                    cam_sensor_panel.build(parent="cam_sensor_scroll")

                with dpg.child_window(tag="radar_sensor_scroll",
                                      width=-1, height=-1,
                                      border=False, show=False):
                    radar_sensor_panel.build(parent="radar_sensor_scroll")

                with dpg.child_window(tag="lidar_scroll",
                                      width=-1, height=-1,
                                      border=False, show=False):
                    lidar_panel.build(parent="lidar_scroll")

                with dpg.child_window(tag="object_control_scroll",
                                      width=-1, height=-1,
                                      border=False, show=False):
                    obj_panel.build()

                with dpg.child_window(tag="traffic_scroll",
                                      width=-1, height=-1,
                                      border=False, show=False):
                    traffic_panel.build()

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

                _select_tab(state.selected_tab)

        # Bottom log area.
        # no_scrollbar=True because log_child owns its own scrolling.
        with dpg.child_window(tag="log_window",
                              width=_vp_w() - PAD, height=-1,
                              border=True,
                              no_scrollbar=True, no_scroll_with_mouse=True):
            log_panel.build(parent="log_window")

    # The viewport resize callback only marks layout dirty.
    # Actual DPG layout changes happen in the main loop.
    _layout_state = {
        "dirty": True,
        "last_size": (-1, -1),
        "save_due_at": None,
    }

    def _mark_layout_dirty():
        _layout_state["dirty"] = True
        _layout_state["save_due_at"] = time.monotonic() + 0.5

    def _apply_layout(force: bool = False) -> bool:
        vp_w = _vp_w()
        vp_h = _vp_h()
        size = (vp_w, vp_h)
        if not force and not _layout_state["dirty"] and size == _layout_state["last_size"]:
            save_due_at = _layout_state["save_due_at"]
            if save_due_at is not None and time.monotonic() >= save_due_at:
                _layout_state["save_due_at"] = None
                _save_interface_console_state(state)
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
        save_due_at = _layout_state["save_due_at"]
        if save_due_at is not None and time.monotonic() >= save_due_at:
            _layout_state["save_due_at"] = None
            _save_interface_console_state(state)
        return True

    dpg.set_viewport_resize_callback(_mark_layout_dirty)
    return _apply_layout


# ============================================================
# Main
# ============================================================
def main():
    state = InterfaceConsoleState()
    udp_ctrl_panel.init(
        send_udp_control_fn=state.send_udp_control,
    )

    dpg.create_context()
    _apply_initial_tcp_endpoint()

    # Load textures.
    global _logo_tag
    _ASSET_DIR  = os.path.join(_BASE_DIR, "assets")
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

    # Load Korean/Unicode-capable fonts; DearPyGUI default font is ASCII-focused.
    _FONT_CANDIDATES = [
        "/mnt/c/Windows/Fonts/malgun.ttf",  # WSL path to Windows Malgun Gothic.
        "/mnt/c/Windows/Fonts/gulim.ttc",   # WSL path to Windows Gulim.
        "C:/Windows/Fonts/malgun.ttf",    # Malgun Gothic, Windows default.
        "C:/Windows/Fonts/gulim.ttc",     # Gulim.
    ]
    for _fp in _FONT_CANDIDATES:
        if os.path.exists(_fp):
            with dpg.font_registry():
                with dpg.font(_fp, 17) as _font:
                    dpg.add_font_range_hint(dpg.mvFontRangeHint_Default)
                    dpg.add_font_range_hint(dpg.mvFontRangeHint_Korean)
                    dpg.add_font_range(0x2000, 0x27FF)  # General Punctuation through Dingbats.
            dpg.bind_font(_font)
            break

    # Convert PNG to ICO for the viewport icon when needed.
    _ICO_PATH = os.path.join(_BASE_DIR, "assets", "logo.ico")
    if os.path.exists(_LOGO_PATH) and not os.path.exists(_ICO_PATH):
        try:
            from PIL import Image
            Image.open(_LOGO_PATH).save(_ICO_PATH, format="ICO", sizes=[(32,32),(48,48),(64,64)])
        except Exception:
            pass

    viewport_w, viewport_h = _initial_viewport_size()
    dpg.create_viewport(
        title=INTERFACE_CONSOLE_TITLE,
        width=viewport_w, height=viewport_h,
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
    _TARGET_FPS     = 60                    # Cap idle rendering to this FPS.
    _TARGET_FRAME_S = 1.0 / _TARGET_FPS

    while dpg.is_dearpygui_running():
        frame_start = time.perf_counter()

        # --- viewport/layout --------------------------------------------------
        apply_layout()

        # --- log flush --------------------------------------------------------
        log_panel.flush()

        # --- ui_queue drain -----------------------------------------------------
        ui_queue.drain()

        # --- DPG render ---------------------------------------------------------
        dpg.render_dearpygui_frame()

        # --- idle fps cap -----------------------------------------------------
        # Sleep only when rendering is faster than the target frame time.
        elapsed = time.perf_counter() - frame_start
        sleep_t = _TARGET_FRAME_S - elapsed
        if sleep_t > 0.001:
            time.sleep(sleep_t)

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
    cam_sensor_panel.stop()
    radar_sensor_panel.stop()
    lidar_panel.stop()
    if state.receiver:
        state.receiver.stop()
    if state.tcp_sock:
        _close_socket(state.tcp_sock)
    _save_interface_console_state(state)
    dpg.destroy_context()


if __name__ == "__main__":
    main()
