# app.py
import socket
import threading
import time

import dearpygui.dearpygui as dpg

from protocol_defs import *
import tcp_transport as tcp
import tcp_thread as tcp_thread_mod
import automation as ac
import ui_queue
import panels.log      as log_panel
import panels.monitor  as monitor_panel
import panels.commands as cmd_panel

# ── 레이아웃 상수 ─────────────────────────────────────────
W_INIT, H_INIT = 1100, 720   # 초기 뷰포트 크기
W_MIN,  H_MIN  = 900,  600   # 최소 크기
CMD_W     = 310         # 커맨드 패널 너비 (고정)
LOG_H     = 200         # 로그 패널 높이 (고정)
TITLEBAR_H = 38         # 타이틀바 + separator 높이
PAD       = 12          # 좌우/하단 여백

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
    return s

def _close_socket(sock):
    for fn in (lambda: sock.shutdown(socket.SHUT_RDWR), sock.close):
        try: fn()
        except Exception: pass

def _set_conn_status(connected: bool):
    ui_queue.post(lambda c=connected: (
        dpg.configure_item("conn_dot",
            color=(100, 255, 100, 255) if c else (255, 80, 80, 255)),
        dpg.set_value("conn_label", "Connected" if c else "Disconnected"),
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
        self._connecting = False
        self._conn_lock  = threading.Lock()

    def dispatch(self, msg_type: int, send_fn):
        rid = self.rid.next()
        pending_add(self.pending, self.lock, rid, msg_type)
        send_fn(rid)

    def toggle_auto(self) -> bool:
        if self.auto_caller is None or not self.auto_caller.is_alive():
            self.auto_caller = ac.AutoCaller(
                tcp_sock=self.tcp_sock,
                pending=self.pending,
                lock=self.lock,
                request_id_ref=self.rid,
                max_calls=MAX_CALL_NUM,
                pending_add_fn=pending_add,
                pending_pop_fn=pending_pop,
                step_count=1,
                timeout_sec=AUTO_TIMEOUT_SEC,
                delay_sec=AUTO_DELAY_BETWEEN_CMDS_SEC,
                progress_every=50,
            )
            _patch_auto_caller(self.auto_caller)
            self.auto_caller.start()
            return True
        else:
            self.auto_caller.stop()
            self.auto_caller = None
            cmd_panel.reset_auto_ui()
            return False

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
        _set_conn_status(False)
        log_panel.append("Connection lost. Click Reconnect to retry.", "ERROR")


# ============================================================
# AutoCaller patch
# ============================================================
def _patch_auto_caller(caller: ac.AutoCaller):
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

        cmd_panel.update_auto_progress(caller.max_calls, caller.max_calls)
        cmd_panel.reset_auto_ui()
        log_panel.append("AutoCaller finished.", "AUTO")

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

        with dpg.group(horizontal=True):
            dpg.add_text("MORAI Sim Control", color=(160, 160, 170))
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
            dpg.add_text(">", tag="conn_dot", color=(100, 255, 100, 255))
            dpg.add_text("Connected", tag="conn_label", color=(140, 200, 140))
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
                with dpg.tab_bar(tag="mon_tabbar"):
                    with dpg.tab(label="UDP Monitor", tag="tab_vehicle"):
                        with dpg.child_window(tag="mon_scroll",
                                              width=-1, height=-1,
                                              border=False):
                            monitor_panel.build(parent="mon_scroll")

        # ── 하단: 로그 ────────────────────────────────────
        # no_scrollbar=True: log_child 가 자체 스크롤 담당
        with dpg.child_window(tag="log_window",
                              width=_vp_w() - PAD, height=-1,
                              border=True,
                              no_scrollbar=True, no_scroll_with_mouse=True):
            log_panel.build(parent="log_window")

    # ── 리사이즈 콜백 ─────────────────────────────────────
    def _on_resize():
        top_h = _top_h()
        dpg.configure_item("cmd_window", height=top_h)
        dpg.configure_item("mon_window", width=_mon_w(), height=top_h)
        dpg.configure_item("log_window", width=_vp_w() - PAD)

    dpg.set_viewport_resize_callback(_on_resize)


# ============================================================
# Main
# ============================================================
def main():
    state = AppState()

    dpg.create_context()
    dpg.create_viewport(
        title="MORAI Sim Control",
        width=W_INIT, height=H_INIT,
        min_width=W_MIN, min_height=H_MIN,
        resizable=True,
    )
    dpg.setup_dearpygui()

    try:
        build_ui(state)
    except Exception:
        import traceback
        traceback.print_exc()
        input("UI build failed. Press Enter to exit.")
        return

    dpg.set_primary_window("main_window", True)
    dpg.show_viewport()

    state.connect()

    while dpg.is_dearpygui_running():
        ui_queue.drain()
        dpg.render_dearpygui_frame()

    if state.auto_caller and state.auto_caller.is_alive():
        state.auto_caller.stop()
    if state.receiver:
        state.receiver.stop()
    if state.tcp_sock:
        _close_socket(state.tcp_sock)
    dpg.destroy_context()


if __name__ == "__main__":
    main()