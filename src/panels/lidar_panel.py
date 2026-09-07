from __future__ import annotations

import json
import os
import threading
import time
from dataclasses import dataclass
from typing import Optional

import cv2
import dearpygui.dearpygui as dpg
import numpy as np

import panels.log as log
from receivers.lidar_receiver import LidarReceiver
from utils import camera3d
from utils.project_paths import ROOT_DIR
from utils.template_paths import resolve_template_path
import utils.ui_queue as ui_queue

_CANVAS_W = 960
_CANVAS_H = 600
_GRID_LINES = 11                       # Grid lines per axis direction.
_TEX_BLANK = np.zeros(_CANVAS_W * _CANVAS_H * 4, dtype=np.float32)
_COLOR_GRID = (45, 50, 60)
_COLOR_AXES = ((80, 80, 230), (80, 220, 100), (255, 150, 90))   # BGR: X red, Y green, Z blue
# The axes are an origin marker, so they stay 1 m regardless of range. Scaling
# them with extent put a 50 m bar across the scene at range 100.
_AXIS_LENGTH_M = 1.0
# Clip fractionally in front of the near plane. Solving for exactly NEAR puts the
# crossing point's depth at NEAR in exact arithmetic, which float rounding can push
# just below it — and _to_screen rejects <= NEAR, dropping the segment we just saved.
_CLIP_NEAR = camera3d.NEAR * 1.0001
_ROOT_DIR = str(ROOT_DIR)
_STATE_FILE = os.path.join(_ROOT_DIR, "config", "lidar_sensor_state.json")

_TPL_LIDAR = "LiDAR PointCloud.tmpl"
_TEMPLATE_PATH = resolve_template_path(_TPL_LIDAR)
_DEFAULT_RANGE_M = 50.0

_COLOR_MODE_HEIGHT = "Height"
_COLOR_MODE_INTENSITY = "Intensity"
_COLOR_MODE_ITEMS = [_COLOR_MODE_HEIGHT, _COLOR_MODE_INTENSITY]

_STATE_VERSION = 2
_DEFAULT_IP = "0.0.0.0"
_DEFAULT_PORT = 9200
_DEFAULT_MAX_POINTS_UI = 600_000
_MAX_POINTS_MIN = 1_000
_MAX_POINTS_MAX = 5_000_000


@dataclass
class _PanelState:
    receiver: Optional[LidarReceiver] = None
    points: Optional[np.ndarray] = None
    range_m: float = _DEFAULT_RANGE_M
    color_mode: str = _COLOR_MODE_HEIGHT
    max_points: int = _DEFAULT_MAX_POINTS_UI
    fps: float = 0.0
    point_count: int = 0
    width: int = 0
    raw_size: int = 0
    frame_id: str = ""
    truncated: bool = False
    chunk_loss: float = 0.0
    last_rx_t: float = 0.0
    last_debug_log_t: float = 0.0


_state = _PanelState()
_camera = camera3d.OrbitCamera()
_worker: Optional["RenderWorker"] = None

# Allow at most one texture in flight. If the main loop stalls (a modal, a
# minimize, another panel's slow callback) the worker keeps rendering at up to
# 30Hz, and each 9.2MB closure queued on ui_queue adds up to hundreds of MB per
# second. When this is set the previous texture has not been applied yet, so
# drop this frame - the next render supersedes it anyway.
_texture_in_flight = threading.Event()


def mark_dirty() -> None:
    if _worker is not None:
        _worker.mark_dirty()


def _on_lidar_packet(packet: dict) -> None:
    """Called on the receive thread. Must only store and return immediately.

    Rendering here stalls the recvfrom loop and costs the next frame its
    chunks: at 600k that is a 45ms block during which over 4MB arrives.
    """
    now = time.monotonic()
    _state.points = packet.get("points")
    _state.fps = float(packet.get("fps", 0.0))
    _state.point_count = int(packet.get("point_count", 0))
    _state.width = int(packet.get("width", 0))
    _state.raw_size = int(packet.get("raw_size", 0))
    _state.frame_id = str(packet.get("frame_id", ""))
    _state.truncated = bool(packet.get("points_truncated", False))
    _state.chunk_loss = float(packet.get("chunk_loss", 0.0))
    _state.last_rx_t = now

    if now - _state.last_debug_log_t >= 1.0:
        _state.last_debug_log_t = now
        log.append(
            f"[Lidar] frame_id={_state.frame_id} "
            f"points={_state.point_count}/{_state.width} "
            f"loss={_state.chunk_loss * 100:.1f}% fps={_state.fps:.1f}"
            + (" (truncated)" if _state.truncated else ""),
            "INFO",
        )
    mark_dirty()


_STATS_IDLE = "FPS - | Points - | Loss - | - | - | RX -"


def _stat_float(value, spec: str, scale: float = 1.0) -> str:
    """Format one number for display, or '-' when the value cannot be trusted."""
    try:
        number = float(value) * scale
    except (TypeError, ValueError):
        return "-"
    if not np.isfinite(number) or number < 0.0:
        return "-"
    return format(number, spec)


def _stat_int(value) -> str:
    try:
        number = int(value)
    except (TypeError, ValueError):
        return "-"
    return f"{number:,}" if number >= 0 else "-"


def format_stats(state, now: float) -> str:
    """Text for the stats line.

    Shows '-' rather than 0 when no frame has been received yet, Stop
    included. 0.0 cannot distinguish "measured zero" from "never measured",
    which made a stopped panel look like it was still receiving at 10 fps.
    """
    if not state.last_rx_t:
        return _STATS_IDLE
    return (
        f"FPS {_stat_float(state.fps, '.1f')}"
        f" | Points {_stat_int(state.point_count)}/{_stat_int(state.width)}"
        + (" (cap)" if state.truncated else "")
        + f" | Loss {_stat_float(state.chunk_loss, '.1f', 100.0)}%"
        + f" | {state.frame_id or '-'}"
        + f" | {_stat_float(state.raw_size, '.1f', 1e-6)} MB"
        + f" | RX {_stat_float(max(0.0, now - state.last_rx_t), '.2f')}s ago"
    )


def _reset_stats() -> None:
    """Clear every frame-derived display value. Called from both Stop and Start."""
    _state.points = None
    _state.fps = 0.0
    _state.point_count = 0
    _state.width = 0
    _state.raw_size = 0
    _state.frame_id = ""
    _state.truncated = False
    _state.chunk_loss = 0.0
    _state.last_rx_t = 0.0


def _render_once() -> None:
    """Called on the render worker. Hands the texture and stats to the UI queue."""
    flat = render_frame(_state.points, _camera, _state.range_m, _state.color_mode)

    if _texture_in_flight.is_set():
        # The UI thread has not applied the previous texture yet. Skipping the
        # copy and the post keeps the queue from growing without bound; this
        # frame can be dropped because the next render supersedes it.
        return

    texture = flat.copy()          # the worker overwrites _OUT on the next frame
    stats_text = format_stats(_state, time.monotonic())

    def _apply() -> None:
        _texture_in_flight.clear()
        if not dpg.does_item_exist("lidar_texture"):
            return
        dpg.set_value("lidar_texture", texture)
        dpg.set_value("lidar_stats", stats_text)

    _texture_in_flight.set()
    ui_queue.post(_apply)


def _start() -> None:
    if _state.receiver is not None and _state.receiver.is_alive():
        log.append("[Lidar] already running", "WARN")
        return

    ip = dpg.get_value("lidar_ip").strip() or _DEFAULT_IP
    port = int(dpg.get_value("lidar_port"))
    _state.range_m = _normalize_range(float(dpg.get_value("lidar_range")))
    _state.color_mode = _normalize_color_mode(str(dpg.get_value("lidar_color_mode")))
    _state.max_points = max(_MAX_POINTS_MIN,
                            min(_MAX_POINTS_MAX, int(dpg.get_value("lidar_max_points"))))
    _save_state()
    _reset_stats()   # so the previous run's numbers do not linger until frame one

    try:
        _state.receiver = LidarReceiver(
            ip=ip,
            port=port,
            on_packet=_on_lidar_packet,
            tmpl_path=_TEMPLATE_PATH,
            max_points=_state.max_points,
        )
    except Exception as exc:
        log.append(f"[Lidar] start failed: {exc}", "ERROR")
        return

    _state.receiver.start()
    dpg.configure_item("lidar_btn_start", enabled=False)
    dpg.set_value("lidar_status", "Running")
    dpg.configure_item("lidar_status", color=(100, 220, 100, 255))
    log.append(f"[Lidar] start {ip}:{port} max_points={_state.max_points}", "INFO")


def _stop_capture() -> None:
    if _state.receiver is not None:
        try:
            _state.receiver.stop()
        except Exception:
            pass
        _state.receiver = None
    _reset_stats()
    if dpg.does_item_exist("lidar_btn_start"):
        dpg.configure_item("lidar_btn_start", enabled=True)
        dpg.set_value("lidar_status", "Stopped")
        dpg.configure_item("lidar_status", color=(180, 80, 80, 255))
    mark_dirty()


def build(parent) -> None:
    global _worker
    with dpg.texture_registry():
        if not dpg.does_item_exist("lidar_texture"):
            dpg.add_dynamic_texture(width=_CANVAS_W, height=_CANVAS_H,
                                    default_value=_TEX_BLANK, tag="lidar_texture")

    with dpg.group(parent=parent):
        _section("CONTROL")
        _build_panel()

    _load_state()
    _install_mouse_handlers()
    if _worker is None:
        _worker = RenderWorker(_render_once)
    _worker.start()
    mark_dirty()


def stop() -> None:
    global _worker
    _stop_capture()
    if _worker is not None:
        _worker.stop()
        # If stop() kept the thread handle because the join timed out (the
        # single-renderer guard), clearing it unconditionally here would break
        # that guard and let the next start() spawn a second renderer sharing
        # the same _CAN/_OUT buffers.
        if not _worker.is_running:
            _worker = None


# app_data is cumulative since button-down, not a per-frame delta. Subtract the
# previous value to get an increment, and reset it on release.
_drag_prev = {"left": (0.0, 0.0), "pan": (0.0, 0.0)}

# Tracks whether a gesture over the view (drag or wheel) actually changed the
# camera. The release handler lives in the global handler_registry, so it fires
# on every mouse release anywhere in the app; without this flag each of those
# would rewrite config/lidar_sensor_state.json through _save_state().
_camera_gesture_pending = False

# At the default 10.0 the first app_data after the threshold is crossed is
# already 10 pixels, so every drag would start with a visible jump.
_DRAG_THRESHOLD = 1.0


def _hovering_view() -> bool:
    return dpg.does_item_exist("lidar_image") and dpg.is_item_hovered("lidar_image")


def _on_orbit_drag(sender, app_data) -> None:
    global _camera, _camera_gesture_pending
    if not _hovering_view():
        return
    _, ax, ay = app_data
    prev = _drag_prev["left"]
    dx, dy = ax - prev[0], ay - prev[1]
    _drag_prev["left"] = (ax, ay)
    dyaw, dpitch = camera3d.drag_to_orbit(dx, dy)
    _camera = _camera.orbit(dyaw, dpitch)
    _camera_gesture_pending = True
    mark_dirty()


def _on_pan_drag(sender, app_data) -> None:
    global _camera, _camera_gesture_pending
    if not _hovering_view():
        return
    _, ax, ay = app_data
    prev = _drag_prev["pan"]
    dx, dy = ax - prev[0], ay - prev[1]
    _drag_prev["pan"] = (ax, ay)
    _camera = _camera.pan(camera3d.pan_world_delta(dx, dy, _camera, _CANVAS_H))
    _camera_gesture_pending = True
    mark_dirty()


def _on_mouse_release(sender, app_data) -> None:
    # This lives in the global handler_registry, so it also fires on mouse
    # releases in other tabs. The accumulator reset must always run, or a drag
    # ending outside the view leaves a stale value for the next one. The save,
    # however, only happens when a gesture over this view actually moved the
    # camera - otherwise any button click in any tab rewrites the state file.
    global _camera_gesture_pending
    _drag_prev["left"] = (0.0, 0.0)
    _drag_prev["pan"] = (0.0, 0.0)
    if _camera_gesture_pending:
        _camera_gesture_pending = False
        _save_state()


def _on_wheel(sender, app_data) -> None:
    global _camera, _camera_gesture_pending
    if not _hovering_view():
        return
    _camera = _camera.zoom(camera3d.wheel_to_zoom(app_data))
    _camera_gesture_pending = True
    mark_dirty()


def _install_mouse_handlers() -> None:
    if dpg.does_item_exist("lidar_handlers"):
        return
    with dpg.handler_registry(tag="lidar_handlers"):
        dpg.add_mouse_drag_handler(button=dpg.mvMouseButton_Left,
                                   threshold=_DRAG_THRESHOLD, callback=_on_orbit_drag)
        dpg.add_mouse_drag_handler(button=dpg.mvMouseButton_Middle,
                                   threshold=_DRAG_THRESHOLD, callback=_on_pan_drag)
        dpg.add_mouse_drag_handler(button=dpg.mvMouseButton_Right,
                                   threshold=_DRAG_THRESHOLD, callback=_on_pan_drag)
        dpg.add_mouse_release_handler(callback=_on_mouse_release)
        dpg.add_mouse_wheel_handler(callback=_on_wheel)


def clip_segment(start, end, camera):
    """Clip a segment at the near plane. Returns None if it is entirely behind.

    Dropping the whole segment because one endpoint sits behind the camera
    makes the grid break up at the screen edge as the viewpoint lowers.
    Solving for the crossing point and cutting there looks continuous.
    """
    origin = camera.position()
    forward = camera.basis()[2]
    a = np.asarray(start, dtype=float)
    b = np.asarray(end, dtype=float)
    da = float(np.dot(a - origin, forward))
    db = float(np.dot(b - origin, forward))
    near = _CLIP_NEAR
    if da < near and db < near:
        return None
    if da >= near and db >= near:
        return a, b
    t = (near - da) / (db - da)
    crossing = a + (b - a) * t
    return (crossing, b) if da < near else (a, crossing)


def _to_screen(point, camera):
    """Project one 3D point to integer screen coordinates, or None if outside the frustum."""
    origin = camera.position()
    rot = camera.basis()
    view = rot @ (np.asarray(point, dtype=float) - origin)
    if view[2] <= camera3d.NEAR:
        return None
    focal = camera3d.focal_length(camera.fov_deg, _CANVAS_H)
    return (int(_CANVAS_W * 0.5 + focal * view[0] / view[2]),
            int(_CANVAS_H * 0.5 - focal * view[1] / view[2]))


def _draw_world_line(canvas, start, end, camera, color, thickness=1):
    clipped = clip_segment(start, end, camera)
    if clipped is None:
        return
    p0 = _to_screen(clipped[0], camera)
    p1 = _to_screen(clipped[1], camera)
    if p0 is None or p1 is None:
        return
    cv2.line(canvas, p0, p1, color, thickness, cv2.LINE_AA)


def build_background(camera, range_m: float) -> np.ndarray:
    """A BGR canvas holding the ground grid and the XYZ axes."""
    canvas = np.zeros((_CANVAS_H, _CANVAS_W, 3), dtype=np.uint8)
    extent = float(range_m)
    half = _GRID_LINES // 2
    for i in range(-half, half + 1):
        offset = extent * i / half
        _draw_world_line(canvas, (-extent, offset, 0.0), (extent, offset, 0.0),
                         camera, _COLOR_GRID)
        _draw_world_line(canvas, (offset, -extent, 0.0), (offset, extent, 0.0),
                         camera, _COLOR_GRID)

    axis_len = _AXIS_LENGTH_M
    for axis, color in zip(np.eye(3) * axis_len, _COLOR_AXES):
        _draw_world_line(canvas, (0.0, 0.0, 0.0), tuple(axis), camera, color, thickness=2)
    return canvas


# Module-level buffers so a frame allocates nothing (P5). There is only ever
# one renderer thread, so sharing them cannot race.
_BG = np.zeros((_CANVAS_H, _CANVAS_W, 3), dtype=np.uint8)
_CAN = np.zeros((_CANVAS_H, _CANVAS_W, 3), dtype=np.uint8)
_RGBA8 = np.empty((_CANVAS_H, _CANVAS_W, 4), dtype=np.uint8)
_OUT = np.empty((_CANVAS_H, _CANVAS_W, 4), dtype=np.float32)
_bg_cache_key = None

_SPLAT_NEAR_PERCENT = 35.0      # Widen this nearest fraction of points to 2x2.
_PERCENTILE_STRIDE = 16         # P3: percentiles come from a subsample.


def _refresh_background(camera, range_m: float) -> None:
    global _bg_cache_key
    key = (camera, float(range_m))
    if key == _bg_cache_key:
        return
    np.copyto(_BG, build_background(camera, range_m))
    _bg_cache_key = key


def _colorize(values: np.ndarray) -> np.ndarray:
    """Map attribute values to TURBO BGR, clipping the bottom and top 2%."""
    sample = values[::_PERCENTILE_STRIDE]
    if sample.size == 0:
        sample = values
    low, high = np.percentile(sample, [2.0, 98.0])
    span = max(float(high) - float(low), 1e-6)
    gray = np.clip((values - low) * (255.0 / span), 0.0, 255.0).astype(np.uint8)
    return cv2.applyColorMap(gray.reshape(-1, 1), cv2.COLORMAP_TURBO).reshape(-1, 3)


def render_frame(points, camera, range_m: float, color_mode: str) -> np.ndarray:
    """Render the point cloud into a flat RGBA float32 array.

    The return value is a view of the module buffer _OUT, which every frame
    overwrites - copy it before holding on to it.
    """
    _refresh_background(camera, range_m)
    np.copyto(_CAN, _BG)

    pts = np.asarray(points, dtype=np.float32) if points is not None else None
    if pts is not None and pts.ndim == 2 and pts.shape[0] > 0:
        limit = float(range_m)
        within = (np.abs(pts[:, 0]) <= limit) & (np.abs(pts[:, 1]) <= limit)

        # P1: pts[within] is a 2-D fancy index over (N,4) and allocates a
        # fresh 9.6MB array every frame at 600k points (measured 5.90ms, 11% of
        # the budget). Project the unfiltered pts instead and combine the
        # frustum mask (keep) with the range mask (within) in 1-D only. keep is
        # a mask over the full length of pts while sx/sy/depth are already
        # filtered by it, so within[keep] is the correctly aligned selector.
        sx, sy, depth, keep = camera3d.project(pts[:, :3], camera, _CANVAS_W, _CANVAS_H)
        if depth.size > 0:
            attr_column = 3 if color_mode == _COLOR_MODE_INTENSITY else 2
            attrs = pts[:, attr_column][keep]
            wk = within[keep]
            sx, sy, depth, attrs = sx[wk], sy[wk], depth[wk], attrs[wk]

        if depth.size > 0:
            # P2: order far -> near with a uint16 bucket radix sort, not a float argsort.
            span = float(depth.max())
            if span > 0.0 and np.isfinite(span):
                buckets = (depth * (65535.0 / span)).astype(np.uint16)
                order = np.argsort(buckets, kind="stable")[::-1]
                sx, sy, depth, attrs = sx[order], sy[order], depth[order], attrs[order]

            colors = _colorize(attrs)
            _CAN[sy, sx] = colors

            # Widen only the near points to 2x2. project() bounds at W-1/H-1,
            # so the +1 offsets always stay inside the array.
            cutoff = np.percentile(depth[::_PERCENTILE_STRIDE] if depth.size > _PERCENTILE_STRIDE
                                   else depth, _SPLAT_NEAR_PERCENT)
            near = depth < cutoff
            nx, ny, nc = sx[near], sy[near], colors[near]
            _CAN[ny + 1, nx] = nc
            _CAN[ny, nx + 1] = nc
            _CAN[ny + 1, nx + 1] = nc

    # P4: convert into pre-allocated buffers to avoid a 9.2MB realloc per frame.
    cv2.cvtColor(_CAN, cv2.COLOR_BGR2RGBA, dst=_RGBA8)
    np.multiply(_RGBA8, np.float32(1.0 / 255.0), out=_OUT)
    return _OUT.reshape(-1)


def _normalize_range(range_m: float) -> float:
    if not np.isfinite(range_m) or range_m <= 0:
        return _DEFAULT_RANGE_M
    return float(range_m)


def _normalize_color_mode(color_mode: str) -> str:
    if color_mode in _COLOR_MODE_ITEMS:
        return color_mode
    return _COLOR_MODE_HEIGHT


def _coerce_float(value, fallback: float) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return fallback
    return result if np.isfinite(result) else fallback


def state_to_dict(ip, port, range_m, color_mode, max_points, camera) -> dict:
    """Turn the panel state into a serialisable dict. No DearPyGUI dependency."""
    return {
        "version": _STATE_VERSION,
        "ip": ip,
        "port": int(port),
        "range_m": float(range_m),
        "color_mode": color_mode,
        "max_points": int(max_points),
        "camera": {
            "target": list(camera.target),
            "distance": camera.distance,
            "yaw_deg": camera.yaw_deg,
            "pitch_deg": camera.pitch_deg,
            "fov_deg": camera.fov_deg,
        },
    }


def _camera_from_dict(raw) -> "camera3d.OrbitCamera":
    default = camera3d.OrbitCamera()
    if not isinstance(raw, dict):
        return default
    target = raw.get("target")
    if not isinstance(target, (list, tuple)) or len(target) != 3:
        target = default.target
    else:
        target = tuple(_coerce_float(v, 0.0) for v in target)
    return camera3d.OrbitCamera(
        target=target,
        distance=_coerce_float(raw.get("distance"), default.distance),
        yaw_deg=_coerce_float(raw.get("yaw_deg"), default.yaw_deg),
        pitch_deg=_coerce_float(raw.get("pitch_deg"), default.pitch_deg),
        fov_deg=_coerce_float(raw.get("fov_deg"), default.fov_deg),
    )


def dict_to_state(data) -> dict:
    """Turn a saved file into panel state. A v1 four-slot file migrates from slot 0."""
    if not isinstance(data, dict):
        data = {}
    source = data
    if "slots" in data:                       # v1 four-slot file
        slots = data.get("slots") or []
        source = slots[0] if isinstance(slots, list) and slots and isinstance(slots[0], dict) else {}

    port = source.get("port", _DEFAULT_PORT)
    try:
        port = int(_coerce_float(port, _DEFAULT_PORT))
    except (TypeError, ValueError, OverflowError):
        port = _DEFAULT_PORT
    if not 1 <= port <= 65535:
        port = _DEFAULT_PORT

    max_points = source.get("max_points", _DEFAULT_MAX_POINTS_UI)
    try:
        max_points = int(_coerce_float(max_points, _DEFAULT_MAX_POINTS_UI))
    except (TypeError, ValueError, OverflowError):
        max_points = _DEFAULT_MAX_POINTS_UI
    max_points = max(_MAX_POINTS_MIN, min(_MAX_POINTS_MAX, max_points))

    ip = source.get("ip", _DEFAULT_IP)
    if ip is None:
        ip = _DEFAULT_IP
    return {
        "ip": str(ip).strip() or _DEFAULT_IP,
        "port": port,
        "range_m": _normalize_range(_coerce_float(source.get("range_m"), _DEFAULT_RANGE_M)),
        "color_mode": _normalize_color_mode(str(source.get("color_mode", _COLOR_MODE_HEIGHT))),
        "max_points": max_points,
        "camera": _camera_from_dict(source.get("camera")),
    }


def _section(title: str) -> None:
    dpg.add_spacer(height=6)
    dpg.add_text(title, color=(200, 200, 100, 255))
    dpg.add_separator()
    dpg.add_spacer(height=2)


def _on_state_change() -> None:
    _save_state()


def _on_range_change(range_m: float) -> None:
    _state.range_m = _normalize_range(range_m)
    _save_state()
    mark_dirty()


def _on_color_mode_change(color_mode: str) -> None:
    _state.color_mode = _normalize_color_mode(color_mode)
    _save_state()
    mark_dirty()


def _apply_preset(name: str) -> None:
    global _camera
    distance = _state.range_m * 1.2 if name == "reset" else None
    _camera = _camera.preset("iso" if name == "reset" else name, distance=distance)
    _save_state()
    mark_dirty()


def _save_state() -> None:
    if not dpg.does_item_exist("lidar_ip"):
        return
    data = state_to_dict(
        dpg.get_value("lidar_ip").strip() or _DEFAULT_IP,
        int(dpg.get_value("lidar_port")),
        _normalize_range(float(dpg.get_value("lidar_range"))),
        _normalize_color_mode(str(dpg.get_value("lidar_color_mode"))),
        int(dpg.get_value("lidar_max_points")),
        _camera,
    )
    try:
        os.makedirs(os.path.dirname(_STATE_FILE), exist_ok=True)
        with open(_STATE_FILE, "w", encoding="utf-8") as fp:
            json.dump(data, fp, indent=2, ensure_ascii=False)
    except Exception as exc:
        log.append(f"[Lidar] save state error: {exc}", "ERROR")


def _load_state() -> None:
    global _camera
    raw = {}
    if os.path.isfile(_STATE_FILE):
        try:
            with open(_STATE_FILE, "r", encoding="utf-8") as fp:
                raw = json.load(fp)
        except Exception as exc:
            log.append(f"[Lidar] load state error: {exc}", "ERROR")
            raw = {}

    restored = dict_to_state(raw)
    _state.range_m = restored["range_m"]
    _state.color_mode = restored["color_mode"]
    _state.max_points = restored["max_points"]
    _camera = restored["camera"]
    for tag, value in (("lidar_ip", restored["ip"]),
                       ("lidar_port", restored["port"]),
                       ("lidar_range", restored["range_m"]),
                       ("lidar_color_mode", restored["color_mode"]),
                       ("lidar_max_points", restored["max_points"])):
        if dpg.does_item_exist(tag):
            dpg.set_value(tag, value)


def _build_panel() -> None:
    with dpg.group(horizontal=True):
        dpg.add_text("IP:", color=(180, 180, 180, 255))
        dpg.add_input_text(tag="lidar_ip", default_value=_DEFAULT_IP, width=120,
                           callback=lambda: _on_state_change())
        dpg.add_text("Port:", color=(180, 180, 180, 255))
        dpg.add_input_int(tag="lidar_port", default_value=_DEFAULT_PORT, width=90,
                          min_value=1, max_value=65535, step=0,
                          callback=lambda: _on_state_change())
        dpg.add_text("Max pts:", color=(180, 180, 180, 255))
        dpg.add_input_int(tag="lidar_max_points", default_value=_DEFAULT_MAX_POINTS_UI,
                          width=110, min_value=_MAX_POINTS_MIN, max_value=_MAX_POINTS_MAX,
                          step=0, callback=lambda: _on_state_change())
        dpg.add_button(label="Start", tag="lidar_btn_start", width=80,
                       callback=lambda: _start())
        dpg.add_button(label="Stop", tag="lidar_btn_stop", width=80,
                       callback=lambda: _stop_capture())
        dpg.add_text("Stopped", tag="lidar_status", color=(180, 80, 80, 255))

    with dpg.group(horizontal=True):
        dpg.add_text("Range(m):", color=(180, 180, 180, 255))
        dpg.add_input_float(tag="lidar_range", default_value=_DEFAULT_RANGE_M, width=90,
                            min_value=1.0, max_value=1000.0, min_clamped=True,
                            max_clamped=True, step=0,
                            callback=lambda s, a: _on_range_change(a))
        dpg.add_text("Color:", color=(180, 180, 180, 255))
        dpg.add_combo(items=_COLOR_MODE_ITEMS, default_value=_COLOR_MODE_HEIGHT, width=110,
                      tag="lidar_color_mode",
                      callback=lambda s, a: _on_color_mode_change(a))
        dpg.add_spacer(width=16)
        dpg.add_text("View:", color=(180, 180, 180, 255))
        for label, name in (("Top", "top"), ("Front", "front"), ("Side", "side"),
                            ("Iso", "iso"), ("Reset", "reset")):
            dpg.add_button(label=label, width=64, user_data=name,
                           callback=lambda s, a, u: _apply_preset(u))

    dpg.add_text(_STATS_IDLE, tag="lidar_stats",
                 color=(200, 200, 210, 255))
    dpg.add_spacer(height=4)

    with dpg.child_window(tag="lidar_view", width=-1, height=-1, border=False,
                          no_scrollbar=True, no_scroll_with_mouse=True):
        dpg.add_spacer(height=0, tag="lidar_view_pad")
        dpg.add_image("lidar_texture", width=_CANVAS_W, height=_CANVAS_H,
                      tag="lidar_image")
    dpg.add_text("drag: orbit    middle-drag: pan    wheel: zoom",
                 color=(140, 140, 150, 255))

    with dpg.item_handler_registry(tag="lidar_view_handlers"):
        dpg.add_item_resize_handler(callback=lambda: _fit_view())
    dpg.bind_item_handler_registry("lidar_view", "lidar_view_handlers")


_VIEW_MARGIN_PX = 8
_VIEW_ASPECT = _CANVAS_W / _CANVAS_H


def _fit_view() -> None:
    """Fit the image to the view while preserving the texture aspect ratio.

    The texture itself stays 960x600. Recreating it to follow the view size
    would rebuild a 9.2MB texture on every window drag.
    """
    if not dpg.does_item_exist("lidar_view") or not dpg.does_item_exist("lidar_image"):
        return
    avail_w, avail_h = dpg.get_item_rect_size("lidar_view")
    avail_w = int(avail_w) - _VIEW_MARGIN_PX
    avail_h = int(avail_h) - _VIEW_MARGIN_PX
    if avail_w < 1 or avail_h < 1:
        return

    width = min(avail_w, int(avail_h * _VIEW_ASPECT))
    height = int(width / _VIEW_ASPECT)
    if width < 1 or height < 1:
        return
    dpg.configure_item("lidar_image", width=width, height=height,
                       indent=max(0, (avail_w - width) // 2))
    if dpg.does_item_exist("lidar_view_pad"):
        dpg.configure_item("lidar_view_pad", height=max(0, (avail_h - height) // 2))


_MIN_FRAME_INTERVAL = 1.0 / 30.0


class RenderWorker:
    """A dedicated thread that renders when a dirty flag is set.

    If the receive thread and the UI thread each rendered, a canvas lock would
    be needed and dragging would stall on the UI thread. Funnelling both
    through one renderer removes both problems. Event coalesces the dirty
    signals on its own, so no separate throttle is required.
    """

    _ERROR_LOG_INTERVAL = 3.0   # Keep a persistent render error off the log ring at 30 lines/s.

    def __init__(self, render_fn):
        self._render_fn = render_fn
        self._wake = threading.Event()
        self._running = False
        self._thread = None
        self._last_error_log_t = 0.0

    @property
    def is_running(self) -> bool:
        # Look only at real thread liveness: judging by a flag would call a
        # thread whose join timed out "dead" and let start() spawn a second
        # renderer.
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return                      # never run two renderers
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True,
                                        name="lidar-render")
        self._thread.start()

    def mark_dirty(self) -> None:
        if self._running:
            self._wake.set()

    def stop(self, timeout: float = 2.0) -> None:
        self._running = False
        self._wake.set()
        thread = self._thread
        if thread is None:
            return
        thread.join(timeout=timeout)
        if thread.is_alive():
            log.append(f"[Lidar] render worker did not stop within {timeout:.1f}s", "WARN")
            return                      # keep the handle so start() can refuse
        self._thread = None

    def _loop(self) -> None:
        while self._running:
            self._wake.wait()
            self._wake.clear()
            if not self._running:
                break
            started = time.monotonic()
            try:
                self._render_fn()
            except Exception as exc:
                # Letting the thread die here freezes the view permanently, so
                # log and carry on. A persistent error would recur as fast as
                # dirty signals arrive (up to 30Hz), so throttle it the way
                # LidarReceiver._debug does: emit the first one immediately,
                # then at most one every few seconds, so the original cause is
                # not pushed out of the log ring.
                now = time.monotonic()
                if now - self._last_error_log_t >= self._ERROR_LOG_INTERVAL:
                    self._last_error_log_t = now
                    log.append(f"[Lidar] render error: {exc}", "ERROR")
            remaining = _MIN_FRAME_INTERVAL - (time.monotonic() - started)
            if remaining > 0:
                time.sleep(remaining)
