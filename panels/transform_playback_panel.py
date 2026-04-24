from __future__ import annotations

import csv
import json
import math
import os
import threading
from typing import Callable, Optional

import dearpygui.dearpygui as dpg

import panels.log as log
import utils.ui_queue as ui_queue

_STATE_FILE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "config", "tfp_state.json"
)

_MAX_VEHICLES = 6
_DEFAULT_VEHICLE_COUNT = 2

_start_fn: Optional[Callable] = None
_stop_fn: Optional[Callable] = None


def init(start_tfp_fn: Callable, stop_tfp_fn: Callable) -> None:
    global _start_fn, _stop_fn
    _start_fn = start_tfp_fn
    _stop_fn = stop_tfp_fn


def build(parent) -> None:
    with dpg.group(parent=parent):
        _section("TRANSFORM PLAYBACK")

        _section("PLAY CONTROL")
        with dpg.group(horizontal=True):
            dpg.add_text("Control   :", color=(180, 180, 180, 255))
            dpg.add_button(label="▶ Play", tag="tfp_btn_play", callback=_on_play)
            dpg.add_button(label="■ Stop", tag="tfp_btn_stop", callback=_on_stop)
            dpg.add_text(" ", tag="tfp_status", color=(160, 160, 160, 255))

        dpg.add_progress_bar(tag="tfp_progress_bar",
                             default_value=0.0, width=-1, overlay="")

        dpg.add_spacer(height=6)
        dpg.add_text("Required columns: location.x/y/z, rotation.x/y/z, steer angle, local_velocity.x/y",
                     color=(140, 140, 140, 255))

        _section("VEHICLE COUNT")
        with dpg.group(horizontal=True):
            dpg.add_text("차량 수 :", color=(180, 180, 180, 255))
            dpg.add_input_int(
                tag="tfp_vehicle_count",
                default_value=_DEFAULT_VEHICLE_COUNT,
                min_value=1, max_value=_MAX_VEHICLES,
                step=1, width=70,
                callback=_on_vehicle_count_change,
            )

        dpg.add_spacer(height=6)
        _section("VEHICLE SETTINGS")
        dpg.add_group(tag="tfp_vehicles_area")
        _build_vehicles(_DEFAULT_VEHICLE_COUNT)

        _load_state()


def update_progress(current: int, total: int) -> None:
    def _apply(c=current, t=total):
        if not dpg.does_item_exist("tfp_progress_bar"):
            return
        ratio = c / t if t > 0 else 0.0
        dpg.set_value("tfp_progress_bar", ratio)
        dpg.configure_item("tfp_progress_bar", overlay=f"{c}/{t}")
        dpg.set_value("tfp_status", f"{c} / {t}")
    ui_queue.post(_apply)


def reset_ui(stopped: bool = False) -> None:
    def _apply(s=stopped):
        if not dpg.does_item_exist("tfp_btn_play"):
            return
        dpg.configure_item("tfp_btn_play", enabled=True)
        dpg.set_value("tfp_progress_bar", 0.0)
        dpg.configure_item("tfp_progress_bar", overlay="")
        dpg.set_value("tfp_status", "Stopped" if s else "Done")
        log.append(f"[TFP] {'중단됨' if s else '재생 완료'}")
    ui_queue.post(_apply)


def _build_vehicles(count: int) -> None:
    dpg.delete_item("tfp_vehicles_area", children_only=True)
    for i in range(1, count + 1):
        with dpg.group(parent="tfp_vehicles_area"):
            dpg.add_text(f"[ Vehicle {i} ]", color=(160, 200, 255, 255))
            dpg.add_spacer(height=2)

            with dpg.group(horizontal=True):
                dpg.add_text("Browse    :", color=(180, 180, 180, 255))
                _folder_btn(callback=_on_browse_click, user_data=i)

            with dpg.group(horizontal=True):
                dpg.add_text("Path      :", color=(180, 180, 180, 255))
                dpg.add_input_text(tag=f"tfp_path_{i}", width=-1, hint="CSV file path")

            with dpg.group(horizontal=True):
                dpg.add_text("ID        :", color=(180, 180, 180, 255))
                dpg.add_input_text(tag=f"tfp_entity_id_{i}", default_value=f"Car_{i}", width=80)

            if i < count:
                dpg.add_spacer(height=6)
                dpg.add_separator()
                dpg.add_spacer(height=6)


def _on_vehicle_count_change(sender, app_data) -> None:
    _build_vehicles(app_data)
    _save_state()


def _on_browse_click(sender, app_data, user_data) -> None:
    try:
        slot = int(user_data)
    except Exception:
        log.append(f"[TFP] 잘못된 browse slot: {user_data!r}", level="ERROR")
        return
    _browse_file(slot)


def _browse_file(slot: int) -> None:
    tag = f"tfp_path_{slot}"

    def _open():
        import tkinter as tk
        from tkinter import filedialog
        root = tk.Tk()
        root.withdraw()
        root.attributes("-topmost", True)
        path = filedialog.askopenfilename(
            title="Select Transform CSV File",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        )
        root.destroy()
        if path:
            def _apply(p=path, t=tag):
                if not dpg.does_item_exist(t):
                    log.append(f"[TFP] path item not found: {t}", level="ERROR")
                    return
                dpg.set_value(t, p)
                _save_state()
            ui_queue.post(_apply)
    threading.Thread(target=_open, daemon=True).start()


def _get_float(row: dict, *names: str) -> float:
    for name in names:
        if name in row and str(row[name]).strip() != "":
            return float(row[name])
    raise KeyError(names[0])


def _load_csv(path: str) -> list:
    rows = []
    try:
        with open(path, newline="", encoding="utf-8-sig") as f:
            reader = csv.DictReader(f)
            for row in reader:
                time_sec = None
                if "Time [sec]" in row and str(row["Time [sec]"]).strip() != "":
                    time_sec = float(row["Time [sec]"])
                elif "Timestamp" in row and str(row["Timestamp"]).strip() != "":
                    time_sec = float(row["Timestamp"]) / 1000.0
                elif ("time_stamp.seconds" in row and str(row["time_stamp.seconds"]).strip() != ""
                      and "time_stamp.nanos" in row and str(row["time_stamp.nanos"]).strip() != ""):
                    time_sec = float(row["time_stamp.seconds"]) + float(row["time_stamp.nanos"]) / 1_000_000_000.0

                rows.append({
                    "time_sec": time_sec,
                    "pos_x": _get_float(row, "location.x", "pos_x", "x"),
                    "pos_y": _get_float(row, "location.y", "pos_y", "y"),
                    "pos_z": _get_float(row, "location.z", "pos_z", "z"),
                    "rot_x": _get_float(row, "rotation.x", "rot_x"),
                    "rot_y": _get_float(row, "rotation.y", "rot_y"),
                    "rot_z": _get_float(row, "rotation.z", "rot_z"),
                    "speed": math.hypot(
                        _get_float(row, "local_velocity.x", "local_velocity_x"),
                        _get_float(row, "local_velocity.y", "local_velocity_y"),
                    ),
                    "steer_angle": _get_float(
                        row,
                        "vehicle_control_attributes.steer_angle",
                        "steer_angle",
                        "SWA [deg]",
                    ),
                })
    except Exception as e:
        log.append(f"[TFP] CSV 파싱 오류: {e}", level="ERROR")
        return []
    return rows


def _on_play() -> None:
    if _start_fn is None:
        log.append("[TFP] 초기화되지 않았습니다.", level="ERROR")
        return

    count = dpg.get_value("tfp_vehicle_count")
    vehicles = []
    for i in range(1, count + 1):
        path = dpg.get_value(f"tfp_path_{i}").strip()
        entity_id = dpg.get_value(f"tfp_entity_id_{i}").strip() or f"Car_{i}"
        if not path:
            continue
        rows = _load_csv(path)
        if not rows:
            continue
        log.append(f"[TFP:{entity_id}] {len(rows)}행 로드 완료: {path}")
        vehicles.append({
            "entity_id": entity_id,
            "path": path,
            "rows": rows,
        })

    if not vehicles:
        log.append("[TFP] 재생할 차량 CSV가 없습니다.", level="WARN")
        return

    _save_state()
    total = max(len(v["rows"]) for v in vehicles)
    dpg.configure_item("tfp_btn_play", enabled=False)
    dpg.set_value("tfp_status", f"0 / {total}")
    _start_fn(vehicles)


def _on_stop() -> None:
    if _stop_fn:
        _stop_fn()


def _save_state() -> None:
    try:
        os.makedirs(os.path.dirname(_STATE_FILE), exist_ok=True)
        count = dpg.get_value("tfp_vehicle_count") if dpg.does_item_exist("tfp_vehicle_count") else _DEFAULT_VEHICLE_COUNT
        data = {
            "tfp_vehicle_count": count,
            "vehicles": [],
        }
        for i in range(1, count + 1):
            data["vehicles"].append({
                "path": dpg.get_value(f"tfp_path_{i}") if dpg.does_item_exist(f"tfp_path_{i}") else "",
                "entity_id": dpg.get_value(f"tfp_entity_id_{i}") if dpg.does_item_exist(f"tfp_entity_id_{i}") else f"Car_{i}",
            })
        with open(_STATE_FILE, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
    except Exception as e:
        print(f"[TFP] save state error: {e}")


def _load_state() -> None:
    if not os.path.isfile(_STATE_FILE):
        return
    try:
        with open(_STATE_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
        count = int(data.get("tfp_vehicle_count", _DEFAULT_VEHICLE_COUNT))
        count = max(1, min(_MAX_VEHICLES, count))
        if dpg.does_item_exist("tfp_vehicle_count"):
            dpg.set_value("tfp_vehicle_count", count)
        _build_vehicles(count)
        vehicles = data.get("vehicles", [])
        for i, vehicle in enumerate(vehicles[:count], 1):
            if dpg.does_item_exist(f"tfp_path_{i}"):
                dpg.set_value(f"tfp_path_{i}", vehicle.get("path", ""))
            if dpg.does_item_exist(f"tfp_entity_id_{i}"):
                dpg.set_value(f"tfp_entity_id_{i}", vehicle.get("entity_id", f"Car_{i}"))
    except Exception as e:
        print(f"[TFP] load state error: {e}")


def _folder_btn(callback, user_data=None) -> None:
    if dpg.does_alias_exist("folder_icon"):
        dpg.add_image_button("folder_icon", width=22, height=22, callback=callback, user_data=user_data)
    else:
        dpg.add_button(label="...", callback=callback, user_data=user_data)


def _section(label: str) -> None:
    dpg.add_spacer(height=6)
    dpg.add_text(label, color=(200, 200, 100, 255))
    dpg.add_separator()
    dpg.add_spacer(height=2)
