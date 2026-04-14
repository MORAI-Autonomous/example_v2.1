# panels/lane_control_panel.py
#
# Lane Control 전용 탭 패널
# - 제어 / 파라미터 설정
# - Debug Frame (원본+BEV+binary+조향게이지 합성 1280×480 → 640×240 표시)
# - Vehicle Info 실시간 수치
# - 튜닝 슬라이더 (Kp, Kd, EMA, Steer Rate, Offset Clip, Target Speed)
from __future__ import annotations

import time
from typing import Callable, Optional

import numpy as np
import cv2
import dearpygui.dearpygui as dpg

import utils.ui_queue as ui_queue
import panels.log as log

# ── 튜닝 기본값 (Reset 버튼용) ───────────────────────────────────
_TUNING_DEFAULTS = {
    "lc_kp":           0.50,
    "lc_kd":           0.10,
    "lc_ema":          0.30,
    "lc_steer_rate":   0.15,
    "lc_offset_clip":  1.50,
    "lc_tune_speed":   15.0,
    "lc_bev_top_crop": 80,
    "lc_min_blob":     50,
    "lc_search_ratio": 0.50,
    "lc_min_pixels":   30,
}

# ── 카메라/디버그 텍스처 해상도 ─────────────────────────────────
# 디버그 합성 1280×480 → 0.5× → 640×240 (AR 완전 보존)
_CAM_W = 640
_CAM_H = 240
_CAM_BLANK: list = [0.0] * (_CAM_W * _CAM_H * 4)

# ── 프레임 표시 제어 ─────────────────────────────────────────────
_FRAME_INTERVAL   = 1.0 / 30.0   # 최대 30fps
_last_frame_t     = 0.0
_suppress_raw_until = 0.0         # debug frame 수신 후 raw 억제 기간

# ── 모듈 상태 ────────────────────────────────────────────────────
_start_fn:  Optional[Callable] = None
_stop_fn:   Optional[Callable] = None
_runner = None   # LaneRunner 참조 (start 후 set_runner() 로 주입)


def init(start_lc_fn: Callable, stop_lc_fn: Callable) -> None:
    global _start_fn, _stop_fn
    _start_fn = start_lc_fn
    _stop_fn  = stop_lc_fn


def set_runner(runner) -> None:
    """app.py가 LaneRunner 생성/소멸 시 호출."""
    global _runner
    _runner = runner


# ── UI 빌드 ──────────────────────────────────────────────────────
def build(parent: int | str) -> None:
    with dpg.texture_registry():
        dpg.add_dynamic_texture(
            width=_CAM_W, height=_CAM_H,
            default_value=_CAM_BLANK,
            tag="lc_cam_texture",
        )

    with dpg.child_window(parent=parent, width=-1, height=-1, border=False):

        # ── CONTROL ────────────────────────────────────────────
        _section("CONTROL")
        with dpg.group(horizontal=True):
            dpg.add_button(label="▶ Start", tag="lc_btn_start",
                           width=90, callback=_on_start)
            dpg.add_button(label="■ Stop",  tag="lc_btn_stop",
                           width=90, callback=_on_stop)
            dpg.add_spacer(width=8)
            dpg.add_text("○ Stopped", tag="lc_status", color=(180, 80, 80, 255))

        # ── TARGET VEHICLE (1줄) ───────────────────────────────
        _section("TARGET VEHICLE")
        with dpg.group(horizontal=True):
            dpg.add_text("ID :", color=(180, 180, 180, 255))
            dpg.add_input_text(tag="lc_entity_id", default_value="Car_1", width=90)
            dpg.add_spacer(width=8)
            dpg.add_checkbox(tag="lc_speed_ctrl", label="Speed Ctrl",
                             default_value=True, callback=_on_speed_ctrl_toggle)
            dpg.add_spacer(width=6)
            dpg.add_text("Target :", color=(180, 180, 180, 255), tag="lc_target_label")
            dpg.add_input_float(tag="lc_target_kmh", default_value=15.0,
                                min_value=1.0, max_value=200.0,
                                format="%.1f", step=0, width=60)
            dpg.add_text("km/h", color=(160, 160, 160, 255), tag="lc_kmh_label")
            dpg.add_text("Throttle :", color=(180, 180, 180, 255),
                         tag="lc_throttle_label", show=False)
            dpg.add_input_float(tag="lc_throttle", default_value=0.3,
                                min_value=0.0, max_value=1.0,
                                format="%.2f", step=0, width=55, show=False)
            dpg.add_spacer(width=8)
            dpg.add_checkbox(tag="lc_invert_steer", label="Invert Steer",
                             default_value=True, callback=_on_invert_steer_toggle)

        # ── INTERFACE ──────────────────────────────────────────
        _section("INTERFACE")
        with dpg.group(horizontal=True):
            dpg.add_text("VI Port :", color=(180, 180, 180, 255))
            dpg.add_input_int(tag="lc_vi_port", default_value=9091,
                              min_value=1, max_value=65535, step=0, width=70)
            dpg.add_spacer(width=16)
            dpg.add_text("Cam Port :", color=(180, 180, 180, 255))
            dpg.add_input_int(tag="lc_cam_port", default_value=9090,
                              min_value=1, max_value=65535, step=0, width=70)

        # ── TUNING ─────────────────────────────────────────────
        _section("TUNING")
        dpg.add_text("* Start 이후 실시간 반영됩니다.",
                     color=(140, 140, 100, 255))
        dpg.add_spacer(height=4)

        _slider("Kp",          "lc_kp",           0.50,  0.0, 3.0,  "%.3f", _on_kp)
        _slider("Kd",          "lc_kd",           0.10,  0.0, 1.0,  "%.3f", _on_kd)
        _slider("EMA α",       "lc_ema",          0.30,  0.01,1.0,  "%.2f", _on_ema)
        _slider("Steer Rate",  "lc_steer_rate",   0.15,  0.01,0.5,  "%.3f", _on_steer_rate)
        _slider("Offset Clip", "lc_offset_clip",  1.50,  0.1, 3.0,  "%.2f", _on_offset_clip)
        _slider("Target Spd",  "lc_tune_speed",   15.0,  1.0, 100.0,"%.1f", _on_tune_speed,
                suffix=" km/h", tag_suffix="lc_tune_speed_label",
                show=dpg.get_value("lc_speed_ctrl") if dpg.does_item_exist("lc_speed_ctrl") else True)

        dpg.add_spacer(height=4)
        dpg.add_text("── 노이즈 필터 ──", color=(140, 140, 140, 255))
        dpg.add_spacer(height=2)

        _slider_int("BEV Top Crop", "lc_bev_top_crop", 80,   0, 240, _on_bev_top_crop,
                    tooltip="BEV 바이너리 상단 N행 마스킹 (터널 천장/원경 노이즈 제거)")
        _slider_int("Min Blob",     "lc_min_blob",      50,   0, 500, _on_min_blob,
                    tooltip="N픽셀 미만 blob 제거 (산점 노이즈 제거)")
        _slider("Search Ratio",     "lc_search_ratio",  0.50, 0.1, 1.0, "%.2f", _on_search_ratio,
                tooltip="히스토그램 피크 탐색 범위 (이미지 하단 비율)")
        _slider_int("Min Pixels",   "lc_min_pixels",    30,   1,  200, _on_min_pixels,
                    tooltip="슬라이딩 윈도우 최소 픽셀 수")

        dpg.add_spacer(height=6)
        dpg.add_button(label="↺ Reset Defaults", tag="lc_btn_reset",
                       width=130, callback=_on_reset_tuning)

        # ── LIVE VIEW ──────────────────────────────────────────
        _section("LIVE VIEW")
        with dpg.table(header_row=False, borders_innerV=True,
                       policy=dpg.mvTable_SizingFixedFit):
            dpg.add_table_column(width_fixed=True, init_width_or_weight=_CAM_W)
            dpg.add_table_column(width_stretch=True)
            with dpg.table_row():
                dpg.add_image("lc_cam_texture", width=_CAM_W, height=_CAM_H)
                with dpg.child_window(height=_CAM_H, border=False):
                    dpg.add_text("Vehicle Info", color=(200, 200, 100, 255))
                    dpg.add_separator()
                    dpg.add_spacer(height=4)
                    _vi_row("Speed",   "lc_vi_speed")
                    dpg.add_spacer(height=4)
                    _vi_row("Pos X",   "lc_vi_posx")
                    _vi_row("Pos Y",   "lc_vi_posy")
                    _vi_row("Pos Z",   "lc_vi_posz")
                    dpg.add_spacer(height=4)
                    _vi_row("Yaw",     "lc_vi_yaw")
                    dpg.add_spacer(height=4)
                    _vi_row("Vel X",   "lc_vi_velx")
                    _vi_row("Vel Y",   "lc_vi_vely")


# ── 헬퍼 위젯 ────────────────────────────────────────────────────

def _section(label: str) -> None:
    dpg.add_spacer(height=6)
    dpg.add_text(label, color=(200, 200, 100, 255))
    dpg.add_separator()
    dpg.add_spacer(height=2)


def _slider(label: str, tag: str, default: float,
            vmin: float, vmax: float, fmt: str,
            callback: Callable,
            suffix: str = "", tag_suffix: str = "", show: bool = True,
            tooltip: str = "") -> None:
    """라벨 + 슬라이더 + 현재값 한 줄."""
    with dpg.group(horizontal=True, show=show,
                   tag=tag + "_row" if not tag_suffix else ""):
        t = dpg.add_text(f"{label:<12}:", color=(180, 180, 180, 255))
        dpg.add_slider_float(tag=tag, default_value=default,
                             min_value=vmin, max_value=vmax,
                             format=fmt, width=160,
                             callback=callback)
        if suffix:
            dpg.add_text(suffix, color=(160, 160, 160, 255))
        if tooltip:
            with dpg.tooltip(t):
                dpg.add_text(tooltip)


def _slider_int(label: str, tag: str, default: int,
                vmin: int, vmax: int,
                callback: Callable,
                tooltip: str = "") -> None:
    """정수 슬라이더 한 줄 (label + slider)."""
    with dpg.group(horizontal=True, tag=tag + "_row"):
        t = dpg.add_text(f"{label:<12}:", color=(180, 180, 180, 255))
        s = dpg.add_slider_int(tag=tag, default_value=default,
                               min_value=vmin, max_value=vmax,
                               width=160, callback=callback)
        if tooltip:
            with dpg.tooltip(t):
                dpg.add_text(tooltip)


def _vi_row(label: str, tag: str) -> None:
    with dpg.group(horizontal=True):
        dpg.add_text(f"{label:<7}:", color=(160, 160, 160, 255))
        dpg.add_text("---", tag=tag, color=(210, 210, 215, 255))


# ── 내부 콜백 ────────────────────────────────────────────────────

def _on_speed_ctrl_toggle(sender, app_data) -> None:
    on = bool(app_data)
    dpg.configure_item("lc_target_label",   show=on)
    dpg.configure_item("lc_target_kmh",     show=on)
    dpg.configure_item("lc_kmh_label",      show=on)
    dpg.configure_item("lc_throttle_label", show=not on)
    dpg.configure_item("lc_throttle",       show=not on)
    # Target Spd 슬라이더 표시 동기화
    if dpg.does_item_exist("lc_tune_speed"):
        dpg.configure_item("lc_tune_speed", show=on)


def _on_invert_steer_toggle(sender, app_data) -> None:
    if _runner:
        _runner.update_params(invert_steer=bool(app_data))


def _on_kp(sender, app_data)          -> None:
    if _runner: _runner.update_params(kp=app_data)

def _on_kd(sender, app_data)          -> None:
    if _runner: _runner.update_params(kd=app_data)

def _on_ema(sender, app_data)         -> None:
    if _runner: _runner.update_params(ema_alpha=app_data)

def _on_steer_rate(sender, app_data)  -> None:
    if _runner: _runner.update_params(steer_rate=app_data)

def _on_offset_clip(sender, app_data) -> None:
    if _runner: _runner.update_params(offset_clip=app_data)

def _on_tune_speed(sender, app_data)  -> None:
    if _runner: _runner.update_params(target_kmh=app_data)

def _on_bev_top_crop(sender, app_data) -> None:
    if _runner: _runner.update_params(bev_top_crop=app_data)

def _on_min_blob(sender, app_data) -> None:
    if _runner: _runner.update_params(min_blob_area=app_data)

def _on_search_ratio(sender, app_data) -> None:
    if _runner: _runner.update_params(search_ratio=app_data)

def _on_min_pixels(sender, app_data) -> None:
    if _runner: _runner.update_params(min_pixels=app_data)


def _on_reset_tuning() -> None:
    """모든 튜닝 슬라이더를 기본값으로 리셋."""
    for tag, val in _TUNING_DEFAULTS.items():
        if dpg.does_item_exist(tag):
            dpg.set_value(tag, val)
    if _runner:
        _runner.update_params(
            kp           = _TUNING_DEFAULTS["lc_kp"],
            kd           = _TUNING_DEFAULTS["lc_kd"],
            ema_alpha    = _TUNING_DEFAULTS["lc_ema"],
            steer_rate   = _TUNING_DEFAULTS["lc_steer_rate"],
            offset_clip  = _TUNING_DEFAULTS["lc_offset_clip"],
            target_kmh   = _TUNING_DEFAULTS["lc_tune_speed"],
            bev_top_crop = _TUNING_DEFAULTS["lc_bev_top_crop"],
            min_blob_area= _TUNING_DEFAULTS["lc_min_blob"],
            search_ratio = _TUNING_DEFAULTS["lc_search_ratio"],
            min_pixels   = _TUNING_DEFAULTS["lc_min_pixels"],
        )


def _on_start() -> None:
    if _start_fn is None:
        log.append("[LC] start_fn이 초기화되지 않았습니다.", level="ERROR")
        return
    cam_port    = dpg.get_value("lc_cam_port")
    vi_port     = dpg.get_value("lc_vi_port")
    entity_id   = dpg.get_value("lc_entity_id").strip() or "Car_1"
    speed_ctrl  = dpg.get_value("lc_speed_ctrl")
    target_kmh  = dpg.get_value("lc_target_kmh")
    throttle    = dpg.get_value("lc_throttle")
    invert_steer= dpg.get_value("lc_invert_steer")

    dpg.configure_item("lc_btn_start", enabled=False)
    dpg.set_value("lc_status", "● Running")
    dpg.configure_item("lc_status", color=(100, 220, 100, 255))

    _start_fn(cam_port, vi_port, entity_id, speed_ctrl, target_kmh, throttle, invert_steer)


def _on_stop() -> None:
    if _stop_fn:
        _stop_fn()


# ── 공개 업데이트 함수 ────────────────────────────────────────────

def reset_ui() -> None:
    """app.py에서 LC 종료 후 호출."""
    def _apply():
        if not dpg.does_item_exist("lc_btn_start"):
            return
        dpg.configure_item("lc_btn_start", enabled=True)
        dpg.set_value("lc_status", "○ Stopped")
        dpg.configure_item("lc_status", color=(180, 80, 80, 255))
        if dpg.does_item_exist("lc_cam_texture"):
            dpg.set_value("lc_cam_texture", _CAM_BLANK)
    ui_queue.post(_apply)


def update_frame(frame: np.ndarray) -> None:
    """원본 카메라 프레임 — debug frame 수신 중에는 억제됨."""
    global _last_frame_t
    if time.monotonic() < _suppress_raw_until:
        return
    now = time.monotonic()
    if now - _last_frame_t < _FRAME_INTERVAL:
        return
    _last_frame_t = now
    _post_frame(frame)


def update_debug_frame(frame: np.ndarray) -> None:
    """디버그 합성 이미지 (1280×480) — raw frame 을 500ms 억제."""
    global _last_frame_t, _suppress_raw_until
    now = time.monotonic()
    if now - _last_frame_t < _FRAME_INTERVAL:
        return
    _last_frame_t       = now
    _suppress_raw_until = now + 0.5
    _post_frame(frame)


def update_vehicle_info(parsed: dict) -> None:
    """Vehicle Info 파싱 결과 → UI 수치 업데이트."""
    try:
        loc = parsed.get("location",       {})
        rot = parsed.get("rotation",       {})
        vel = parsed.get("local_velocity", {})
        spd_kmh = (vel.get("x", 0.0) ** 2 +
                   vel.get("y", 0.0) ** 2 +
                   vel.get("z", 0.0) ** 2) ** 0.5 * 3.6
    except Exception:
        return

    def _apply(s=spd_kmh,
               x=loc.get("x", 0.0), y=loc.get("y", 0.0), z=loc.get("z", 0.0),
               yaw=rot.get("z", 0.0),
               vx=vel.get("x", 0.0), vy=vel.get("y", 0.0)):
        if not dpg.does_item_exist("lc_vi_speed"):
            return
        dpg.set_value("lc_vi_speed", f"{s:.1f} km/h")
        dpg.set_value("lc_vi_posx",  f"{x:.2f} m")
        dpg.set_value("lc_vi_posy",  f"{y:.2f} m")
        dpg.set_value("lc_vi_posz",  f"{z:.2f} m")
        dpg.set_value("lc_vi_yaw",   f"{yaw:.1f} °")
        dpg.set_value("lc_vi_velx",  f"{vx:.2f} m/s")
        dpg.set_value("lc_vi_vely",  f"{vy:.2f} m/s")
    ui_queue.post(_apply)


# ── 내부 헬퍼 ────────────────────────────────────────────────────

def _post_frame(frame: np.ndarray) -> None:
    """배경 스레드에서 BGR 프레임을 640×240 RGBA float 로 변환 후 큐에 올린다."""
    resized = cv2.resize(frame, (_CAM_W, _CAM_H))
    rgba    = cv2.cvtColor(resized, cv2.COLOR_BGR2RGBA)
    flat    = (rgba.astype(np.float32) / 255.0).flatten()

    def _apply(data=flat):
        if dpg.does_item_exist("lc_cam_texture"):
            dpg.set_value("lc_cam_texture", data)
    ui_queue.post(_apply)
