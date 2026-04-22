from __future__ import annotations

import json
import os
import time
from typing import Callable, Optional

import dearpygui.dearpygui as dpg
import utils.ui_queue as ui_queue
import panels.log as log

_MAP_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "autonomous_driving", "config", "map"
)
_STATE_FILE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "config", "au_state.json"
)


def _get_available_maps() -> list:
    try:
        return sorted(d for d in os.listdir(_MAP_DIR)
                      if os.path.isdir(os.path.join(_MAP_DIR, d)))
    except OSError:
        return []

_MAX_VEHICLES = 6

_start_ad_fn:      Optional[Callable] = None
_stop_ad_fn:       Optional[Callable] = None
_start_step_ad_fn: Optional[Callable] = None
_stop_step_ad_fn:  Optional[Callable] = None
_running_step_mode = False
_entity_slot: dict = {}        # entity_id → slot index (1-based)
_last_status_ts: dict = {}     # slot → last post timestamp (throttle 10Hz)


def init(
    start_ad_fn:      Callable,
    stop_ad_fn:       Callable,
    start_step_ad_fn: Callable,
    stop_step_ad_fn:  Callable,
) -> None:
    global _start_ad_fn, _stop_ad_fn, _start_step_ad_fn, _stop_step_ad_fn
    _start_ad_fn      = start_ad_fn
    _stop_ad_fn       = stop_ad_fn
    _start_step_ad_fn = start_step_ad_fn
    _stop_step_ad_fn  = stop_step_ad_fn


def build(parent) -> None:
    with dpg.group(parent=parent):

        # ── CONTROL ──────────────────────────────────────────
        _section("CONTROL")

        maps = _get_available_maps()
        with dpg.group(horizontal=True):
            dpg.add_text("Map    :", color=(180, 180, 180, 255))
            dpg.add_combo(tag="au_map_combo", items=maps,
                          default_value=maps[0] if maps else "",
                          width=-1, callback=lambda: _save_state())
        dpg.add_spacer(height=6)

        with dpg.group(horizontal=True):
            dpg.add_checkbox(tag="au_fixed_step", label="Fixed Step",
                             default_value=False,
                             callback=_on_fixed_step_toggle)
            dpg.add_spacer(width=16)
            dpg.add_checkbox(tag="au_save_data", label="Save Data",
                             default_value=False, show=False)

        dpg.add_spacer(height=6)
        with dpg.group(horizontal=True):
            dpg.add_button(label="▶ Start", tag="au_btn_start", callback=_on_start)
            dpg.add_button(label="■ Stop",  tag="au_btn_stop",  callback=_on_stop)
            dpg.add_text(" ", tag="au_status", color=(160, 160, 160, 255))

        # ── VEHICLES ─────────────────────────────────────────
        _section("VEHICLES")

        with dpg.group(horizontal=True):
            dpg.add_text("차량 수 :", color=(180, 180, 180, 255))
            dpg.add_input_int(
                tag="au_vehicle_count",
                default_value=2,
                min_value=1, max_value=_MAX_VEHICLES,
                step=1, width=70,
                callback=_on_vehicle_count_change,
            )

        dpg.add_spacer(height=6)
        dpg.add_group(tag="au_vehicles_area")
        _build_vehicles(2)

        # ── COLLISION ─────────────────────────────────────────
        _section("COLLISION")

        dpg.add_checkbox(tag="au_collision_enable", label="충돌 모드",
                         default_value=False,
                         callback=_on_collision_toggle)
        dpg.add_spacer(height=4)

        with dpg.group(tag="au_collision_settings", show=False):
            with dpg.group(horizontal=True):
                dpg.add_text("Chaser Slot :", color=(180, 180, 180, 255))
                dpg.add_input_int(tag="au_collision_chaser",
                                  default_value=2,
                                  min_value=1, max_value=_MAX_VEHICLES,
                                  step=0, width=50,
                                  callback=lambda: _save_state())
                dpg.add_spacer(width=12)
                dpg.add_text("-> Target Slot :", color=(180, 180, 180, 255))
                dpg.add_input_int(tag="au_collision_target",
                                  default_value=1,
                                  min_value=1, max_value=_MAX_VEHICLES,
                                  step=0, width=50,
                                  callback=lambda: _save_state())

            dpg.add_spacer(height=4)
            with dpg.group(horizontal=True):
                _labeled("Speed   :", "Target 차량이 경로를 따라 유지할 속도.\n"
                                      "Chaser 차량은 동일 경로를 Speed × 1.2 로\n"
                                      "주행하여 자연스럽게 따라붙어 충돌합니다.\n"
                                      "예) Speed=60  →  Target 60 kph / Chaser 72 kph")
                dpg.add_input_float(tag="au_collision_speed_kph",
                                    default_value=60.0,
                                    min_value=1.0, max_value=200.0,
                                    step=0, width=110,
                                    format="%.0f",
                                    callback=lambda: _save_state())
                dpg.add_text("kph", color=(140, 140, 140, 255))
                dpg.add_spacer(width=12)
                _labeled("Trigger :", "Target 차량이 이 속도에 도달하면\n"
                                      "Chaser가 출발합니다.\n"
                                      "Target이 먼저 속도를 올린 뒤\n"
                                      "Chaser가 추돌하게 하려면 낮게 설정하세요.")
                dpg.add_input_float(tag="au_collision_trigger_kph",
                                    default_value=5.0,
                                    min_value=0.0, step=0, width=100,
                                    format="%.0f",
                                    callback=lambda: _save_state())
                dpg.add_text("kph", color=(140, 140, 140, 255))

        _load_state()


# ── 동적 차량 목록 빌드 ───────────────────────────────────────

def _build_vehicles(count: int) -> None:
    """au_vehicles_area 내 차량 설정 위젯을 (재)생성한다."""
    dpg.delete_item("au_vehicles_area", children_only=True)
    for i in range(1, count + 1):
        with dpg.group(tag=f"au_vehicle_group_{i}", parent="au_vehicles_area"):
            dpg.add_text(f"[ Vehicle {i} ]", color=(160, 200, 255, 255))

            with dpg.group(horizontal=True):
                dpg.add_text("ID    :", color=(180, 180, 180, 255))
                dpg.add_input_text(tag=f"au_entity_id_{i}",
                                   default_value=f"Car_{i}", width=100)
                dpg.add_spacer(width=10)
                dpg.add_text("Port  :", color=(180, 180, 180, 255))
                dpg.add_input_int(tag=f"au_vi_port_{i}",
                                  default_value=9090 + i,
                                  min_value=1, max_value=65535, step=0, width=80)

            with dpg.group(horizontal=True):
                for key, label in [
                    ("pos",   "Pos"),
                    ("vel",   "Vel"),
                    ("accel", "Accel"),
                    ("brake", "Brake"),
                    ("steer", "Steer"),
                ]:
                    dpg.add_text(f"{label}:", color=(140, 140, 140, 255))
                    dpg.add_text("-", tag=f"au_sv{i}_{key}",
                                 color=(200, 200, 200, 255))
                    dpg.add_spacer(width=4)

            if i < count:
                dpg.add_spacer(height=4)
                dpg.add_separator()
                dpg.add_spacer(height=4)


def _on_vehicle_count_change(sender, app_data) -> None:
    _build_vehicles(app_data)


def _on_collision_toggle(sender, app_data) -> None:
    dpg.configure_item("au_collision_settings", show=app_data)
    _save_state()


# ── Public callbacks ──────────────────────────────────────────

_STATUS_MIN_INTERVAL = 0.1   # 10Hz


def update_status(
    entity_id: str,
    x: float, y: float,
    vel_kmh: float,
    accel: float,
    brake: float,
    steer: float,
) -> None:
    slot = _entity_slot.get(entity_id)
    if slot is None:
        return
    now = time.monotonic()
    if now - _last_status_ts.get(slot, 0.0) < _STATUS_MIN_INTERVAL:
        return
    _last_status_ts[slot] = now
    def _apply(s=slot, _x=x, _y=y, v=vel_kmh, a=accel, b=brake, st=steer):
        pfx = f"au_sv{s}_"
        if not dpg.does_item_exist(pfx + "pos"):
            return
        dpg.set_value(pfx + "pos",   f"({_x:.1f}, {_y:.1f})")
        dpg.set_value(pfx + "vel",   f"{v:.1f} km/h")
        dpg.set_value(pfx + "accel", f"{a:.3f}")
        dpg.set_value(pfx + "brake", f"{b:.3f}")
        dpg.set_value(pfx + "steer", f"{st:.3f}")
    ui_queue.post(_apply)


def reset_ui() -> None:
    def _apply():
        if not dpg.does_item_exist("au_btn_start"):
            return
        dpg.configure_item("au_btn_start", enabled=True)
        dpg.set_value("au_status", "● Stopped")
        dpg.configure_item("au_status", color=(180, 80, 80, 255))
        count = dpg.get_value("au_vehicle_count") if dpg.does_item_exist("au_vehicle_count") else 0
        for i in range(1, count + 1):
            for key in ("pos", "vel", "accel", "brake", "steer"):
                tag = f"au_sv{i}_{key}"
                if dpg.does_item_exist(tag):
                    dpg.set_value(tag, "-")
    ui_queue.post(_apply)


# ── Internal ──────────────────────────────────────────────────

def _on_fixed_step_toggle(sender, app_data) -> None:
    dpg.configure_item("au_save_data", show=app_data)
    if app_data:
        dpg.set_value("au_save_data", True)


def _build_collision_cfg() -> Optional[dict]:
    """충돌 모드가 활성화된 경우 collision_cfg 딕셔너리 반환, 아니면 None."""
    if not dpg.get_value("au_collision_enable"):
        return None

    chaser_slot = dpg.get_value("au_collision_chaser")
    target_slot = dpg.get_value("au_collision_target")

    if chaser_slot == target_slot:
        log.append("[AD] 충돌 모드: Chaser와 Target이 같은 슬롯입니다.", level="WARN")
        return None

    chaser_eid = dpg.get_value(f"au_entity_id_{chaser_slot}").strip()
    target_eid = dpg.get_value(f"au_entity_id_{target_slot}").strip()

    if not chaser_eid or not target_eid:
        log.append("[AD] 충돌 모드: 차량 ID를 확인해 주세요.", level="WARN")
        return None

    return {
        "chaser_entity_id": chaser_eid,
        "target_entity_id": target_eid,
        "speed_kph":        dpg.get_value("au_collision_speed_kph"),
        "trigger_kph":      dpg.get_value("au_collision_trigger_kph"),
    }


def _on_start() -> None:
    global _running_step_mode, _entity_slot
    count    = dpg.get_value("au_vehicle_count")
    map_name = dpg.get_value("au_map_combo").strip() or None
    vehicles = []
    for i in range(1, count + 1):
        eid = dpg.get_value(f"au_entity_id_{i}").strip()
        if not eid:
            continue
        vehicles.append({
            "map_name":  map_name,
            "path":      "path_link.csv",
            "entity_id": eid,
            "vi_port":   dpg.get_value(f"au_vi_port_{i}"),
        })

    if not vehicles:
        log.append("[AD] entity_id가 없습니다. 차량 ID를 입력해 주세요.", level="WARN")
        return

    collision_cfg = _build_collision_cfg()
    if collision_cfg:
        log.append(
            f"[AD] 충돌 모드: {collision_cfg['chaser_entity_id']} -> "
            f"{collision_cfg['target_entity_id']} "
            f"(speed={collision_cfg['speed_kph']:.0f}kph, "
            f"trigger={collision_cfg['trigger_kph']:.0f}kph)"
        )

    _entity_slot = {v["entity_id"]: i for i, v in enumerate(vehicles, 1)}
    _last_status_ts.clear()
    _running_step_mode = dpg.get_value("au_fixed_step")
    dpg.configure_item("au_btn_start", enabled=False)
    dpg.set_value("au_status", "● Running")
    dpg.configure_item("au_status", color=(100, 220, 100, 255))

    if _running_step_mode:
        if _start_step_ad_fn is None:
            log.append("[AD] 초기화되지 않았습니다.", level="ERROR")
            return
        _start_step_ad_fn(vehicles, dpg.get_value("au_save_data"), collision_cfg)
    else:
        if _start_ad_fn is None:
            log.append("[AD] 초기화되지 않았습니다.", level="ERROR")
            return
        _start_ad_fn(vehicles, collision_cfg)


def _on_stop() -> None:
    if _running_step_mode:
        if _stop_step_ad_fn:
            _stop_step_ad_fn()
    else:
        if _stop_ad_fn:
            _stop_ad_fn()


def _save_state() -> None:
    try:
        os.makedirs(os.path.dirname(_STATE_FILE), exist_ok=True)
        data = {
            "au_map_combo":            dpg.get_value("au_map_combo"),
            "au_collision_enable":     dpg.get_value("au_collision_enable"),
            "au_collision_chaser":     dpg.get_value("au_collision_chaser"),
            "au_collision_target":     dpg.get_value("au_collision_target"),
            "au_collision_speed_kph":   dpg.get_value("au_collision_speed_kph"),
            "au_collision_trigger_kph": dpg.get_value("au_collision_trigger_kph"),
        }
        with open(_STATE_FILE, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
    except Exception as e:
        print(f"[AU] save state error: {e}")


def _load_state() -> None:
    if not os.path.isfile(_STATE_FILE):
        return
    try:
        with open(_STATE_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)

        saved_map = data.get("au_map_combo", "")
        if saved_map and dpg.does_item_exist("au_map_combo"):
            if saved_map in _get_available_maps():
                dpg.set_value("au_map_combo", saved_map)

        _bool("au_collision_enable",     data, False)
        _int ("au_collision_chaser",      data, 2)
        _int ("au_collision_target",      data, 1)
        _float("au_collision_speed_kph",   data, 60.0)
        _float("au_collision_trigger_kph", data, 5.0)

        # 체크박스 상태에 따라 설정 패널 표시
        if dpg.does_item_exist("au_collision_settings"):
            dpg.configure_item("au_collision_settings",
                               show=dpg.get_value("au_collision_enable"))

    except Exception as e:
        print(f"[AU] load state error: {e}")


# ── 상태 로드 헬퍼 ────────────────────────────────────────────

def _bool(tag: str, data: dict, default: bool) -> None:
    if dpg.does_item_exist(tag):
        dpg.set_value(tag, bool(data.get(tag, default)))

def _int(tag: str, data: dict, default: int) -> None:
    if dpg.does_item_exist(tag):
        dpg.set_value(tag, int(data.get(tag, default)))

def _float(tag: str, data: dict, default: float) -> None:
    if dpg.does_item_exist(tag):
        dpg.set_value(tag, float(data.get(tag, default)))


def _labeled(text: str, tooltip: str) -> None:
    """라벨 텍스트 + 호버 툴팁."""
    dpg.add_text(text, color=(180, 180, 180, 255))
    with dpg.tooltip(dpg.last_item()):
        dpg.add_text(tooltip, color=(220, 220, 180, 255))


def _section(label: str) -> None:
    dpg.add_spacer(height=6)
    dpg.add_text(label, color=(200, 200, 100, 255))
    dpg.add_separator()
    dpg.add_spacer(height=2)
