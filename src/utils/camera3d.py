from __future__ import annotations

"""Orbit camera and vectorized perspective projection.

This module does not depend on DearPyGUI. The panel uses it; it does not know
the panel. Coordinates follow the ROS convention: +x forward, +y left, +z up.
"""

import math
from dataclasses import dataclass, replace
from typing import Dict, Tuple

import numpy as np

NEAR = 0.1              # Near plane. Points closer than this are dropped.
PITCH_LIMIT = 89.0      # At 90 deg forward becomes parallel to world_up and basis degenerates.
DISTANCE_MIN = 0.5
DISTANCE_MAX = 2000.0
FOV_MIN = 10.0
FOV_MAX = 120.0

_WORLD_UP = np.array([0.0, 0.0, 1.0])

# (yaw_deg, pitch_deg). Derived by projection calculation, not guessed.
# top is 180 deg so the axes match the 2D view this replaces: at 0 deg the
# +x forward axis lands at the bottom of the screen, flipping it vertically.
PRESETS: Dict[str, Tuple[float, float]] = {
    "top": (180.0, 89.0),
    "front": (0.0, 0.0),
    "side": (270.0, 0.0),
    "iso": (315.0, 25.0),
}


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, float(value)))


@dataclass(frozen=True)
class OrbitCamera:
    """A camera orbiting a target.

    Frozen for thread safety: every change returns a new instance, so the UI
    thread can publish a pose and the render worker can read it with no lock.
    """

    target: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    distance: float = 60.0
    yaw_deg: float = 315.0
    pitch_deg: float = 25.0
    fov_deg: float = 60.0

    def __post_init__(self) -> None:
        object.__setattr__(self, "target", tuple(float(v) for v in self.target))
        object.__setattr__(self, "distance", _clamp(self.distance, DISTANCE_MIN, DISTANCE_MAX))
        object.__setattr__(self, "yaw_deg", float(self.yaw_deg) % 360.0)
        object.__setattr__(self, "pitch_deg", _clamp(self.pitch_deg, -PITCH_LIMIT, PITCH_LIMIT))
        object.__setattr__(self, "fov_deg", _clamp(self.fov_deg, FOV_MIN, FOV_MAX))

    def _offset(self) -> np.ndarray:
        yaw = math.radians(self.yaw_deg)
        pitch = math.radians(self.pitch_deg)
        return self.distance * np.array([
            math.cos(pitch) * math.cos(yaw),
            math.cos(pitch) * math.sin(yaw),
            math.sin(pitch),
        ])

    def position(self) -> np.ndarray:
        return np.asarray(self.target, dtype=float) + self._offset()

    def basis(self) -> np.ndarray:
        """(3,3) rotation whose rows are right, up, forward.

        This is the gluLookAt / GLM lookAt construction, so by that row order
        the triple is left-handed: right x up == -forward. Projection only
        needs forward to point at the target and the three axes to be
        orthonormal, both of which hold.
        """
        offset = self._offset()
        forward = -offset / np.linalg.norm(offset)
        right = np.cross(forward, _WORLD_UP)
        right = right / np.linalg.norm(right)
        up = np.cross(right, forward)
        return np.stack([right, up, forward])

    def orbit(self, dyaw_deg: float, dpitch_deg: float) -> "OrbitCamera":
        return replace(self,
                       yaw_deg=self.yaw_deg + float(dyaw_deg),
                       pitch_deg=self.pitch_deg + float(dpitch_deg))

    def zoom(self, factor: float) -> "OrbitCamera":
        return replace(self, distance=self.distance * float(factor))

    def pan(self, world_delta: np.ndarray) -> "OrbitCamera":
        delta = np.asarray(world_delta, dtype=float)
        if not np.all(np.isfinite(delta)):
            return self
        moved = np.asarray(self.target, dtype=float) + delta
        return replace(self, target=(float(moved[0]), float(moved[1]), float(moved[2])))

    def preset(self, name: str, distance: float = None) -> "OrbitCamera":
        yaw, pitch = PRESETS[name]
        return replace(self,
                       target=(0.0, 0.0, 0.0),
                       yaw_deg=yaw,
                       pitch_deg=pitch,
                       distance=self.distance if distance is None else float(distance))


def focal_length(fov_deg: float, height: int) -> float:
    """Focal length from the vertical field of view."""
    return 0.5 * float(height) / math.tan(math.radians(float(fov_deg)) * 0.5)


def project(xyz, camera: OrbitCamera, width: int, height: int):
    """Project world points to screen pixels.

    Returns (px, py, depth, keep). px/py/depth are 1-D arrays already filtered
    by keep; keep is a bool mask the same length as the input, so callers can
    line up a parallel attribute array with attrs[keep].

    The bounds stop one pixel short of width / height so a caller's 2x2 splat
    can write +1 offsets without a range check.
    """
    points = np.asarray(xyz, dtype=np.float32)
    if points.ndim != 2 or points.shape[0] == 0:
        empty_int = np.empty(0, np.int32)
        empty_float = np.empty(0, np.float32)
        return empty_int, empty_int, empty_float, np.zeros(points.shape[0], bool)

    origin = camera.position().astype(np.float32)
    rot = camera.basis().astype(np.float32)

    # With NaN/Inf present an inf*0 term can appear as early as the matmul,
    # and the later int32 cast produces a RuntimeWarning plus garbage. The
    # isfinite mask below discards those values, so only the warning is
    # suppressed. Dividing just the valid subset is also safe but adds another
    # fancy-indexing pass and runs about 60% slower.
    with np.errstate(all="ignore"):
        view = (points[:, :3] - origin) @ rot.T   # x=right, y=up, z=depth
        depth = view[:, 2]
        focal = np.float32(focal_length(camera.fov_deg, height))
        px = (width * 0.5 + focal * view[:, 0] / depth).astype(np.int32)
        py = (height * 0.5 - focal * view[:, 1] / depth).astype(np.int32)

    keep = (
        np.isfinite(depth)
        & (depth > NEAR)
        & (px >= 0) & (px < width - 1)
        & (py >= 0) & (py < height - 1)
    )
    return px[keep], py[keep], depth[keep], keep


ORBIT_SENSITIVITY = 0.4   # Degrees per pixel: half a screen width is about half a turn.
ZOOM_STEP = 0.9           # Distance multiplier per wheel notch.


def drag_to_orbit(dx_px: float, dy_px: float,
                  sensitivity: float = ORBIT_SENSITIVITY):
    """Turn a drag increment into (yaw, pitch) deltas.

    yaw goes negative for a rightward drag so the scene appears to follow the
    cursor.
    """
    return (-float(dx_px) * sensitivity, float(dy_px) * sensitivity)


def wheel_to_zoom(delta) -> float:
    """Turn a wheel notch into a distance multiplier. Scrolling up moves closer."""
    return ZOOM_STEP ** float(delta)


def pan_world_delta(dx_px: float, dy_px: float,
                    camera: OrbitCamera, height: int) -> np.ndarray:
    """Turn a screen-pixel drag into a world-space displacement.

    Treating pixels as world distance makes panning run away when zoomed in.
    Scaling by the world height the viewport spans keeps the grabbed point
    under the cursor at any zoom level.
    """
    rot = camera.basis()
    world_per_px = (2.0 * camera.distance
                    * math.tan(math.radians(camera.fov_deg) * 0.5) / float(height))
    return (-float(dx_px) * world_per_px) * rot[0] + (float(dy_px) * world_per_px) * rot[1]
