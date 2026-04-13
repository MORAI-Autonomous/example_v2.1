# panels/monitor.py
"""
Template-driven UDP monitor panel.

Layout (inside mon_scroll child_window)
───────────────────────────────────────
┌──────────────────────────────────┐  (child_window height=240)
│  UDP Monitor                     │
│  Templates                       │
│  [ listbox ]                     │
│  [▶ Open]  [▲ Refresh]          │
└──────────────────────────────────┘
┌──────────────────────────────────┐  (child_window height=-1)
│ ┌─ Vehicle Info ─┬─ IMU ─┐       │
│ │  IP/Port/...  │        │       │
│ │  Fields table │        │       │
│ └───────────────┘        │       │
└──────────────────────────────────┘
"""
import json
import os
import socket
import threading
import time
from typing import Any, Dict, List, Optional

import dearpygui.dearpygui as dpg

import utils.ui_queue as ui_queue
from receivers.template_parser import TemplateParser

# ── Paths ─────────────────────────────────────────────────────────────
_BASE_DIR  = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_TMPL_DIR  = os.path.join(_BASE_DIR, "templates")
_STATE_FILE = os.path.join(_BASE_DIR, "config", "monitor_state.json")

# 하단 탭바 태그 (build() 에서 생성, _open_monitor() 에서 참조)
_INNER_TABBAR = "mon_inner_tabbar"
_HINT_TAG     = "mon_no_monitors_hint"

# ── Per-monitor state ─────────────────────────────────────────────────
_monitors: Dict[str, dict] = {}
_win_counter: int = 0

_UPDATE_INTERVAL = 0.05   # 20 Hz UI refresh cap

# ── Static tags ───────────────────────────────────────────────────────
_T = {
    "listbox": "mon_listbox",
}

_DEFAULT_PORT = 9091


# ═══════════════════════════════════════════════════════════════════════
#  Shared helpers
# ═══════════════════════════════════════════════════════════════════════

def _get_templates() -> List[str]:
    if not os.path.isdir(_TMPL_DIR):
        return []
    return sorted(f for f in os.listdir(_TMPL_DIR) if f.lower().endswith(".tmpl"))


def _short_label(variable_name: str, n: int = 2) -> str:
    parts = variable_name.split(".")
    return ".".join(parts[-n:]) if len(parts) >= n else variable_name


def _tab_label(filename: str) -> str:
    name = filename.replace(".tmpl", "")
    return name if len(name) <= 22 else name[:21] + "…"


def _make_groups(field_list: List[Dict]) -> List[Dict]:
    groups: List[Dict] = []
    i, total = 0, len(field_list)
    while i < total:
        nm = field_list[i]["name"].lower()

        # xyzw
        if (i + 3 < total and nm == "x" and
                field_list[i+1]["name"].lower() == "y" and
                field_list[i+2]["name"].lower() == "z" and
                field_list[i+3]["name"].lower() == "w"):
            vn  = field_list[i]["variable_name"]
            pfx = vn.rsplit(".", 1)[0] if "." in vn else vn
            groups.append({"type": "xyzw", "indices": [i, i+1, i+2, i+3],
                            "label": _short_label(pfx), "tag": 0})
            i += 4; continue

        # xyz
        if (i + 2 < total and nm == "x" and
                field_list[i+1]["name"].lower() == "y" and
                field_list[i+2]["name"].lower() == "z"):
            vn  = field_list[i]["variable_name"]
            pfx = vn.rsplit(".", 1)[0] if "." in vn else vn
            groups.append({"type": "xyz", "indices": [i, i+1, i+2],
                            "label": _short_label(pfx), "tag": 0})
            i += 3; continue

        # single
        groups.append({"type": "single", "indices": [i],
                        "label": _short_label(field_list[i]["variable_name"]),
                        "tag": 0})
        i += 1
    return groups


def _fmt(val: Any, var_type: str) -> str:
    if var_type in ("FLOAT", "DOUBLE"):
        try:
            f = float(val)
            # 너무 크거나 너무 작으면 지수 표기 (:.Nf 가 수십 자리 문자열이 되는 것 방지)
            if abs(f) >= 1e6 or (f != 0.0 and abs(f) < 1e-4):
                return f"{f:.6e}" if var_type == "DOUBLE" else f"{f:.4e}"
            # DOUBLE: 8바이트 정밀도에 맞게 소수점 6자리
            # FLOAT : 4바이트 정밀도에 맞게 소수점 4자리
            return f"{f:.6f}" if var_type == "DOUBLE" else f"{f:.4f}"
        except Exception:
            return str(val)
    return str(val)


def _format_repeat_rows(rows: List[Dict]) -> str:
    if not rows:
        return "(0 items)"
    lines = [f"({len(rows)} items)"]
    for idx, row in enumerate(rows):
        fl = row.get("field_list", [])
        lines.append(f"[{idx}]")
        for g in _make_groups(fl):
            t, ix = g["type"], g["indices"]
            if t == "xyz":
                i0, i1, i2 = ix
                lines.append(
                    f"  {g['label']}: "
                    f"X={_fmt(fl[i0]['value'], fl[i0]['type'])}  "
                    f"Y={_fmt(fl[i1]['value'], fl[i1]['type'])}  "
                    f"Z={_fmt(fl[i2]['value'], fl[i2]['type'])}"
                )
            elif t == "xyzw":
                i0, i1, i2, i3 = ix
                lines.append(
                    f"  {g['label']}: "
                    f"X={_fmt(fl[i0]['value'], fl[i0]['type'])}  "
                    f"Y={_fmt(fl[i1]['value'], fl[i1]['type'])}  "
                    f"Z={_fmt(fl[i2]['value'], fl[i2]['type'])}  "
                    f"W={_fmt(fl[i3]['value'], fl[i3]['type'])}"
                )
            else:
                i0 = ix[0]
                lines.append(
                    f"  {g['label']}: "
                    f"{_fmt(fl[i0]['value'], fl[i0]['type'])}"
                )
    return "\n".join(lines)


# ═══════════════════════════════════════════════════════════════════════
#  UDP receiver thread
# ═══════════════════════════════════════════════════════════════════════

class _UDPThread(threading.Thread):
    def __init__(self, sock, parse_fn, on_data, on_error):
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
                data, _ = self.sock.recvfrom(65535)
                parsed  = self.parse_fn(data)
                if parsed is not None:
                    self.on_data(parsed)
            except OSError:
                if self.running:
                    self.on_error()
                break


# ═══════════════════════════════════════════════════════════════════════
#  build()  –  panel entry point
# ═══════════════════════════════════════════════════════════════════════

def build(parent) -> None:
    tmpls = _get_templates()

    # ── 상단: child_window 없이 parent에 직접 추가 ───────────────
    # (child_window 래퍼를 쓰면 내부 스크롤바가 생길 수 있음)
    dpg.add_text("Templates", color=(180, 180, 180, 255), parent=parent)
    dpg.add_listbox(
        tag=_T["listbox"],
        items=tmpls,
        num_items=min(len(tmpls), 6),   # 6개 표시 → 내부 스크롤로 나머지
        width=-1,
        parent=parent,
    )
    dpg.add_spacer(height=4, parent=parent)
    with dpg.group(horizontal=True, parent=parent):
        dpg.add_button(label="▶ Open",     width=84, callback=_on_open)
        dpg.add_button(label="▲ Refresh", width=84, callback=_on_refresh)
    dpg.add_separator(parent=parent)

    # ── 하단: 남은 공간 전체를 탭 영역으로 ─────────────────────
    with dpg.child_window(parent=parent,
                          height=-1, width=-1,
                          border=True,
                          no_scrollbar=True, no_scroll_with_mouse=True):

        # 탭이 없을 때 안내 문구
        dpg.add_text(
            "▲ 위에서 템플릿을 선택하고  [ ▶ Open ]  을 누르세요",
            tag=_HINT_TAG,
            color=(120, 120, 120, 255),
        )

        # 실제 탭바 (처음엔 탭 없음)
        dpg.add_tab_bar(tag=_INNER_TABBAR)

    # 이전 세션 상태 복원
    _load_state()


# ── callbacks (main panel) ────────────────────────────────────────────

def _on_open(sender=None, app_data=None, user_data=None) -> None:
    filename = dpg.get_value(_T["listbox"])
    if filename:
        _open_monitor(filename)


def _on_refresh(sender=None, app_data=None, user_data=None) -> None:
    tmpls = _get_templates()
    dpg.configure_item(_T["listbox"], items=tmpls)


# ═══════════════════════════════════════════════════════════════════════
#  Monitor tab open / close
# ═══════════════════════════════════════════════════════════════════════

def _open_monitor(filename: str,
                  ip: str = "127.0.0.1",
                  port: int = _DEFAULT_PORT) -> None:
    global _win_counter

    path = os.path.join(_TMPL_DIR, filename)
    if not os.path.isfile(path):
        return
    try:
        parser = TemplateParser(path)
    except Exception as e:
        print(f"[Monitor] template load error: {e}")
        return

    _win_counter += 1
    tab_tag = f"mon_tab_{_win_counter}"

    ip_tag          = dpg.generate_uuid()
    port_tag        = dpg.generate_uuid()
    btn_start_tag   = dpg.generate_uuid()
    btn_stop_tag    = dpg.generate_uuid()
    status_tag      = dpg.generate_uuid()
    dyn_group_tag   = dpg.generate_uuid()

    st: dict = {
        "filename":        filename,
        "parser":          parser,
        "tab_tag":         tab_tag,
        "ip_tag":          ip_tag,
        "port_tag":        port_tag,
        "btn_start_tag":   btn_start_tag,
        "btn_stop_tag":    btn_stop_tag,
        "status_tag":      status_tag,
        "dyn_group_tag":   dyn_group_tag,
        "field_groups":    [],
        "repeat_text_tag": 0,
        "running":         False,
        "sock":            None,
        "thread":          None,
        "last_update_t":   0.0,
    }
    _monitors[tab_tag] = st

    # 첫 탭 오픈 시 안내 문구 숨기기
    if len(_monitors) == 1 and dpg.does_item_exist(_HINT_TAG):
        dpg.configure_item(_HINT_TAG, show=False)

    # ── 새 탭 ────────────────────────────────────────────────────
    with dpg.tab(label=_tab_label(filename), tag=tab_tag,
                 parent=_INNER_TABBAR):
        with dpg.child_window(width=-1, height=-1, border=False):

            # IP / Port / Start / Stop / Status  |  Close (right-aligned)
            with dpg.table(header_row=False,
                           borders_innerV=False, borders_outerV=False,
                           borders_outerH=False, borders_innerH=False):
                dpg.add_table_column(width_stretch=True)
                dpg.add_table_column(width_fixed=True, init_width_or_weight=72)
                with dpg.table_row():
                    with dpg.group(horizontal=True):
                        dpg.add_text("IP:", color=(160, 160, 170))
                        dpg.add_input_text(tag=ip_tag,
                                           default_value=ip, width=110)
                        dpg.add_text("Port:", color=(160, 160, 170))
                        dpg.add_input_int(tag=port_tag,
                                          default_value=port,
                                          width=72, step=0,
                                          min_value=1, max_value=65535)
                        dpg.add_button(tag=btn_start_tag, label="▶ Start", width=62,
                                       callback=_on_start, user_data=tab_tag)
                        dpg.add_button(tag=btn_stop_tag,  label="■ Stop",  width=62,
                                       callback=_on_stop,  user_data=tab_tag)
                        dpg.add_text("○ Stopped", tag=status_tag,
                                     color=(180, 80, 80, 255))
                    dpg.add_button(label="X Close", width=-1,
                                   callback=_on_close_tab, user_data=tab_tag)

            dpg.add_separator()

            dpg.add_group(tag=dyn_group_tag)

    _rebuild_display(tab_tag)

    # 새로 열린 탭으로 포커스
    dpg.set_value(_INNER_TABBAR, tab_tag)

    # 상태 저장
    _save_state()


def _on_close_tab(sender, app_data, user_data) -> None:
    tab_tag = user_data
    st = _monitors.pop(tab_tag, None)
    if st:
        _stop_receiver(st)
    if dpg.does_item_exist(tab_tag):
        dpg.delete_item(tab_tag)
    # 탭이 모두 닫히면 안내 문구 다시 표시
    if not _monitors and dpg.does_item_exist(_HINT_TAG):
        dpg.configure_item(_HINT_TAG, show=True)
    # 상태 저장
    _save_state()


# ── Dynamic display builder ──────────────────────────────────────────

def _rebuild_display(tab_tag: str) -> None:
    st = _monitors.get(tab_tag)
    if not st:
        return

    dg = st["dyn_group_tag"]
    if not dpg.does_item_exist(dg):
        return

    dpg.delete_item(dg, children_only=True)
    st["field_groups"]    = []
    st["repeat_text_tag"] = 0

    parser = st["parser"]

    # ── FIELDS table ─────────────────────────────────────────────
    if parser.fields_segment:
        seg = parser.fields_segment

        fd_list = [{"name": f.name, "variable_name": f.variable_name}
                   for f in seg.fields]
        groups  = _make_groups(fd_list)

        # repeat 없으면 테이블이 나머지 공간을 전부 채우도록 child_window로 감쌈
        if not parser.has_repeat:
            fields_parent = dpg.add_child_window(
                parent=dg, height=-1, width=-1, border=False)
        else:
            fields_parent = dg

        with dpg.table(parent=fields_parent, header_row=True,
                       borders_innerV=True, borders_outerV=True,
                       borders_outerH=True, borders_innerH=True,
                       resizable=True):
            dpg.add_table_column(label="Field",
                                 width_fixed=True, init_width_or_weight=180)
            dpg.add_table_column(label="Value")

            for g in groups:
                val_tag = dpg.generate_uuid()
                g["tag"] = val_tag
                with dpg.table_row():
                    dpg.add_text(g["label"], color=(160, 160, 190, 255))
                    dpg.add_text("-", tag=val_tag)
                st["field_groups"].append(g)

    # ── REPEAT section ────────────────────────────────────────────
    if parser.has_repeat:
        dpg.add_spacer(height=6, parent=dg)
        rfn = parser.repeat_segment.repeat_field_name
        dpg.add_text(f"Repeat:  {rfn}", color=(180, 180, 180, 255), parent=dg)
        rtt = dpg.generate_uuid()
        st["repeat_text_tag"] = rtt
        dpg.add_input_text(
            tag=rtt,
            multiline=True,
            readonly=True,
            width=-1,
            height=-1,          # 나머지 공간 전부 채움
            default_value="(0 items)",
            parent=dg,
        )


# ═══════════════════════════════════════════════════════════════════════
#  Start / Stop
# ═══════════════════════════════════════════════════════════════════════

def _on_start(sender, app_data, user_data) -> None:
    tab_tag = user_data
    st = _monitors.get(tab_tag)
    if not st or st["running"]:
        return

    ip   = dpg.get_value(st["ip_tag"]).strip() or "0.0.0.0"
    port = dpg.get_value(st["port_tag"])

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((ip, port))
    except Exception as e:
        _set_status(st, f"✗ {e}", (255, 80, 80, 255))
        return

    st["sock"]    = sock
    st["running"] = True
    st["thread"]  = _UDPThread(
        sock     = sock,
        parse_fn = st["parser"].parse,
        on_data  = lambda p, tt=tab_tag: _on_data(tt, p),
        on_error = lambda tt=tab_tag: ui_queue.post(
                       lambda: _on_thread_error(tt)),
    )
    st["thread"].start()
    _refresh_status(tab_tag)
    # IP/Port 변경사항 영구 저장
    _save_state()


def _on_stop(sender, app_data, user_data) -> None:
    tab_tag = user_data
    st = _monitors.get(tab_tag)
    if not st or not st["running"]:
        return
    _stop_receiver(st)
    _refresh_status(tab_tag)


def _stop_receiver(st: dict) -> None:
    st["running"] = False
    if st.get("thread"):
        st["thread"].stop()
        st["thread"] = None
    if st.get("sock"):
        try:
            st["sock"].close()
        except Exception:
            pass
        st["sock"] = None


def _on_thread_error(tab_tag: str) -> None:
    st = _monitors.get(tab_tag)
    if st:
        st["running"] = False
        st["thread"]  = None
        st["sock"]    = None
        _refresh_status(tab_tag)


# ═══════════════════════════════════════════════════════════════════════
#  Data routing & display update
# ═══════════════════════════════════════════════════════════════════════

def _on_data(tab_tag: str, parsed: dict) -> None:
    st = _monitors.get(tab_tag)
    if not st:
        return
    now = time.time()
    if now - st["last_update_t"] < _UPDATE_INTERVAL:
        return
    st["last_update_t"] = now
    ui_queue.post(lambda tt=tab_tag, p=parsed: _apply_data(tt, p))


def _apply_data(tab_tag: str, parsed: dict) -> None:
    st = _monitors.get(tab_tag)
    if not st:
        return

    fl = parsed.get("field_list", [])

    for g in st["field_groups"]:
        tag = g["tag"]
        if not dpg.does_item_exist(tag):
            continue
        t, ix = g["type"], g["indices"]

        if t == "xyz" and ix[2] < len(fl):
            i0, i1, i2 = ix
            dpg.set_value(tag,
                f"X={_fmt(fl[i0]['value'], fl[i0]['type'])}  "
                f"Y={_fmt(fl[i1]['value'], fl[i1]['type'])}  "
                f"Z={_fmt(fl[i2]['value'], fl[i2]['type'])}")

        elif t == "xyzw" and ix[3] < len(fl):
            i0, i1, i2, i3 = ix
            dpg.set_value(tag,
                f"X={_fmt(fl[i0]['value'], fl[i0]['type'])}  "
                f"Y={_fmt(fl[i1]['value'], fl[i1]['type'])}  "
                f"Z={_fmt(fl[i2]['value'], fl[i2]['type'])}  "
                f"W={_fmt(fl[i3]['value'], fl[i3]['type'])}")

        elif t == "single" and ix[0] < len(fl):
            i0 = ix[0]
            dpg.set_value(tag, _fmt(fl[i0]["value"], fl[i0]["type"]))

    rtt = st.get("repeat_text_tag", 0)
    if rtt and dpg.does_item_exist(rtt):
        dpg.set_value(rtt, _format_repeat_rows(parsed.get("repeat_rows", [])))


# ═══════════════════════════════════════════════════════════════════════
#  Status helpers
# ═══════════════════════════════════════════════════════════════════════

def _set_status(st: dict, text: str, color: tuple) -> None:
    tag = st.get("status_tag", 0)
    if tag and dpg.does_item_exist(tag):
        dpg.set_value(tag, text)
        dpg.configure_item(tag, color=color)


def _refresh_status(tab_tag: str) -> None:
    st = _monitors.get(tab_tag)
    if not st:
        return
    if st["running"]:
        _set_status(st, "● Running", (80, 200, 80, 255))
    else:
        _set_status(st, "○ Stopped", (180, 80, 80, 255))


# ═══════════════════════════════════════════════════════════════════════
#  State persistence  (config/monitor_state.json)
# ═══════════════════════════════════════════════════════════════════════

def _save_state() -> None:
    """현재 열린 탭 목록(파일명 + IP + Port)을 JSON 파일에 저장."""
    entries = []
    for tab_tag, st in list(_monitors.items()):
        if not dpg.does_item_exist(tab_tag):
            continue
        try:
            ip   = dpg.get_value(st["ip_tag"])
            port = int(dpg.get_value(st["port_tag"]))
        except Exception:
            ip, port = "127.0.0.1", _DEFAULT_PORT
        entries.append({
            "filename": st["filename"],
            "ip":       ip,
            "port":     port,
        })
    try:
        os.makedirs(os.path.dirname(_STATE_FILE), exist_ok=True)
        with open(_STATE_FILE, "w", encoding="utf-8") as f:
            json.dump(entries, f, indent=2, ensure_ascii=False)
    except Exception as e:
        print(f"[Monitor] save state error: {e}")


def _load_state() -> None:
    """저장된 탭 목록을 읽어 탭을 자동으로 복원."""
    if not os.path.isfile(_STATE_FILE):
        return
    try:
        with open(_STATE_FILE, "r", encoding="utf-8") as f:
            entries = json.load(f)
    except Exception as e:
        print(f"[Monitor] load state error: {e}")
        return

    for entry in entries:
        filename = entry.get("filename", "")
        ip       = entry.get("ip", "127.0.0.1")
        port     = int(entry.get("port", _DEFAULT_PORT))
        if filename:
            _open_monitor(filename, ip=ip, port=port)
