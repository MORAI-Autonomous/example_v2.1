from __future__ import annotations

from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

import numpy as np

from panels import lidar_panel
from utils import camera3d


class StateSerializationTests(unittest.TestCase):
    def test_round_trip_preserves_every_field(self):
        cam = camera3d.OrbitCamera(target=(1.0, 2.0, 3.0), distance=77.0,
                                   yaw_deg=123.0, pitch_deg=-40.0, fov_deg=45.0)
        data = lidar_panel.state_to_dict("10.0.0.5", 9300, 80.0, "Intensity", 400_000, cam)
        restored = lidar_panel.dict_to_state(data)
        self.assertEqual(restored["ip"], "10.0.0.5")
        self.assertEqual(restored["port"], 9300)
        self.assertAlmostEqual(restored["range_m"], 80.0)
        self.assertEqual(restored["color_mode"], "Intensity")
        self.assertEqual(restored["max_points"], 400_000)
        self.assertEqual(restored["camera"], cam)

    def test_written_dict_declares_version_2(self):
        data = lidar_panel.state_to_dict("0.0.0.0", 9200, 50.0, "Height",
                                         600_000, camera3d.OrbitCamera())
        self.assertEqual(data["version"], 2)

    def test_v1_four_slot_file_migrates_from_first_slot(self):
        v1 = {
            "lidar_count": 4,
            "slots": [
                {"name": "LiDAR 1", "ip": "192.168.0.9", "port": 9205,
                 "range_m": 120.0, "color_mode": "Intensity"},
                {"name": "LiDAR 2", "ip": "0.0.0.0", "port": 9201,
                 "range_m": 50.0, "color_mode": "Height"},
            ],
        }
        restored = lidar_panel.dict_to_state(v1)
        self.assertEqual(restored["ip"], "192.168.0.9")
        self.assertEqual(restored["port"], 9205)
        self.assertAlmostEqual(restored["range_m"], 120.0)
        self.assertEqual(restored["color_mode"], "Intensity")

    def test_v1_file_with_empty_slots_falls_back_to_defaults(self):
        restored = lidar_panel.dict_to_state({"lidar_count": 4, "slots": []})
        self.assertEqual(restored["ip"], "0.0.0.0")
        self.assertEqual(restored["port"], 9200)

    def test_empty_dict_yields_defaults(self):
        restored = lidar_panel.dict_to_state({})
        self.assertEqual(restored["ip"], "0.0.0.0")
        self.assertEqual(restored["port"], 9200)
        self.assertAlmostEqual(restored["range_m"], 50.0)
        self.assertEqual(restored["color_mode"], "Height")
        self.assertEqual(restored["max_points"], 600_000)
        self.assertEqual(restored["camera"], camera3d.OrbitCamera())

    def test_missing_camera_block_uses_default_camera(self):
        restored = lidar_panel.dict_to_state({"version": 2, "ip": "0.0.0.0"})
        self.assertEqual(restored["camera"], camera3d.OrbitCamera())

    def test_corrupt_camera_fields_fall_back_per_field(self):
        restored = lidar_panel.dict_to_state({
            "version": 2,
            "camera": {"yaw_deg": "not a number", "distance": 33.0,
                       "target": "nope", "pitch_deg": None},
        })
        cam = restored["camera"]
        self.assertAlmostEqual(cam.distance, 33.0)
        self.assertEqual(cam.target, (0.0, 0.0, 0.0))
        self.assertAlmostEqual(cam.yaw_deg, camera3d.OrbitCamera().yaw_deg)

    def test_invalid_range_falls_back_to_default(self):
        self.assertAlmostEqual(lidar_panel.dict_to_state({"range_m": 0.0})["range_m"], 50.0)
        self.assertAlmostEqual(lidar_panel.dict_to_state({"range_m": -5.0})["range_m"], 50.0)
        self.assertAlmostEqual(lidar_panel.dict_to_state({"range_m": "x"})["range_m"], 50.0)

    def test_unknown_color_mode_falls_back_to_height(self):
        self.assertEqual(lidar_panel.dict_to_state({"color_mode": "Rainbow"})["color_mode"], "Height")

    def test_max_points_is_clamped(self):
        self.assertEqual(lidar_panel.dict_to_state({"max_points": 1})["max_points"], 1_000)
        self.assertEqual(lidar_panel.dict_to_state({"max_points": 10 ** 9})["max_points"], 5_000_000)

    def test_invalid_port_falls_back_to_default(self):
        self.assertEqual(lidar_panel.dict_to_state({"port": 0})["port"], 9200)
        self.assertEqual(lidar_panel.dict_to_state({"port": 999999})["port"], 9200)

    def test_non_finite_port_falls_back_to_default(self):
        self.assertEqual(lidar_panel.dict_to_state({"port": float("inf")})["port"], 9200)
        self.assertEqual(lidar_panel.dict_to_state({"port": float("nan")})["port"], 9200)

    def test_non_finite_max_points_falls_back_to_default(self):
        self.assertEqual(
            lidar_panel.dict_to_state({"max_points": float("inf")})["max_points"], 600_000)

    def test_null_ip_falls_back_to_default(self):
        self.assertEqual(lidar_panel.dict_to_state({"ip": None})["ip"], "0.0.0.0")


class StatsLineTests(unittest.TestCase):
    """The stats line must distinguish "nothing received yet" from "received zero"."""

    def _state(self, **kw):
        st = lidar_panel._PanelState()
        st.last_rx_t = 100.0
        st.fps, st.point_count, st.width = 10.0, 94772, 94772
        st.raw_size, st.frame_id, st.chunk_loss = 2_084_985, "LiDAR_1", 0.0
        for k, v in kw.items():
            setattr(st, k, v)
        return st

    def test_idle_state_shows_dashes_not_zeros(self):
        st = lidar_panel._PanelState()
        self.assertEqual(lidar_panel.format_stats(st, 100.0), lidar_panel._STATS_IDLE)
        self.assertNotIn("0.0", lidar_panel.format_stats(st, 100.0))

    def test_live_state_shows_numbers(self):
        line = lidar_panel.format_stats(self._state(), 100.25)
        self.assertIn("FPS 10.0", line)
        self.assertIn("Points 94,772/94,772", line)
        self.assertIn("Loss 0.0%", line)
        self.assertIn("LiDAR_1", line)
        self.assertIn("2.1 MB", line)
        self.assertIn("RX 0.25s ago", line)

    def test_truncated_frames_are_marked(self):
        self.assertIn("(cap)", lidar_panel.format_stats(self._state(truncated=True), 100.0))

    def test_non_finite_and_negative_values_degrade_to_dash(self):
        for field, bad in (("fps", float("nan")), ("fps", float("inf")),
                           ("fps", -1.0), ("raw_size", -5), ("chunk_loss", float("nan"))):
            line = lidar_panel.format_stats(self._state(**{field: bad}), 100.0)
            self.assertIn("-", line, f"{field}={bad!r} was not degraded to '-'")

    def test_garbage_types_do_not_raise(self):
        for field, bad in (("fps", "x"), ("point_count", None), ("width", object()),
                           ("raw_size", "x"), ("chunk_loss", None)):
            line = lidar_panel.format_stats(self._state(**{field: bad}), 100.0)
            self.assertIsInstance(line, str)

    def test_reset_stats_clears_every_frame_derived_field(self):
        # Regression: fps and raw_size survived Stop, so a stopped panel still
        # read as 10 fps.
        for name, value in (("fps", 10.0), ("point_count", 5), ("width", 5),
                            ("raw_size", 999), ("frame_id", "LiDAR_1"),
                            ("truncated", True), ("chunk_loss", 0.5),
                            ("last_rx_t", 100.0)):
            setattr(lidar_panel._state, name, value)
        lidar_panel._state.points = np.zeros((3, 4), np.float32)
        lidar_panel._reset_stats()
        self.assertIsNone(lidar_panel._state.points)
        for name in ("fps", "point_count", "width", "raw_size", "chunk_loss", "last_rx_t"):
            self.assertFalse(getattr(lidar_panel._state, name), name)
        self.assertEqual(lidar_panel._state.frame_id, "")
        self.assertFalse(lidar_panel._state.truncated)
        self.assertEqual(lidar_panel.format_stats(lidar_panel._state, 1.0),
                         lidar_panel._STATS_IDLE)


class PanelFlatteningTests(unittest.TestCase):
    def test_slot_machinery_is_gone(self):
        for name in ("_SLOT_COUNT", "_GRID_COLUMNS", "_slots", "_SlotState", "_tag"):
            self.assertFalse(hasattr(lidar_panel, name),
                             f"{name} must be gone after the single-slot flattening")

    def test_module_singletons_exist(self):
        self.assertIsInstance(lidar_panel._camera, camera3d.OrbitCamera)
        self.assertTrue(hasattr(lidar_panel, "_state"))
        self.assertTrue(hasattr(lidar_panel, "mark_dirty"))

    def test_two_dimensional_render_helpers_are_gone(self):
        for name in ("_CANVAS_SIZE", "_RING_STEP_M"):
            self.assertFalse(hasattr(lidar_panel, name),
                             f"{name} was 2D top-down only and must be gone")

    def test_texture_blank_is_a_numpy_array(self):
        # A Python list here would build 2.3M float objects at import time.
        import numpy as np
        self.assertIsInstance(lidar_panel._TEX_BLANK, np.ndarray)
        self.assertEqual(lidar_panel._TEX_BLANK.dtype, np.float32)


if __name__ == "__main__":
    unittest.main()
