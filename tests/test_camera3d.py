from __future__ import annotations

from pathlib import Path
import math
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from utils import camera3d


class OrbitCameraStateTests(unittest.TestCase):
    def test_pitch_is_clamped_to_avoid_gimbal_lock(self):
        self.assertEqual(camera3d.OrbitCamera(pitch_deg=120.0).pitch_deg, 89.0)
        self.assertEqual(camera3d.OrbitCamera(pitch_deg=-120.0).pitch_deg, -89.0)

    def test_distance_is_clamped(self):
        self.assertEqual(camera3d.OrbitCamera(distance=0.0).distance, 0.5)
        self.assertEqual(camera3d.OrbitCamera(distance=99999.0).distance, 2000.0)

    def test_fov_is_clamped(self):
        self.assertEqual(camera3d.OrbitCamera(fov_deg=1.0).fov_deg, 10.0)
        self.assertEqual(camera3d.OrbitCamera(fov_deg=200.0).fov_deg, 120.0)

    def test_yaw_wraps_into_0_360(self):
        self.assertAlmostEqual(camera3d.OrbitCamera(yaw_deg=-45.0).yaw_deg, 315.0)
        self.assertAlmostEqual(camera3d.OrbitCamera(yaw_deg=725.0).yaw_deg, 5.0)

    def test_orbit_returns_new_camera_and_leaves_original_unchanged(self):
        base = camera3d.OrbitCamera(yaw_deg=10.0, pitch_deg=10.0)
        moved = base.orbit(5.0, 5.0)
        self.assertAlmostEqual(base.yaw_deg, 10.0)
        self.assertAlmostEqual(base.pitch_deg, 10.0)
        self.assertAlmostEqual(moved.yaw_deg, 15.0)
        self.assertAlmostEqual(moved.pitch_deg, 15.0)
        self.assertIsNot(base, moved)

    def test_camera_is_frozen(self):
        cam = camera3d.OrbitCamera()
        with self.assertRaises(Exception):
            cam.distance = 10.0

    def test_zoom_scales_distance(self):
        cam = camera3d.OrbitCamera(distance=100.0)
        self.assertAlmostEqual(cam.zoom(0.9).distance, 90.0)

    def test_pan_moves_target(self):
        cam = camera3d.OrbitCamera(target=(1.0, 2.0, 3.0))
        moved = cam.pan(np.array([1.0, 1.0, 1.0]))
        self.assertEqual(moved.target, (2.0, 3.0, 4.0))
        self.assertEqual(cam.target, (1.0, 2.0, 3.0))

    def test_position_is_target_plus_spherical_offset(self):
        cam = camera3d.OrbitCamera(target=(0.0, 0.0, 0.0), distance=10.0,
                                   yaw_deg=0.0, pitch_deg=0.0)
        np.testing.assert_allclose(cam.position(), [10.0, 0.0, 0.0], atol=1e-9)

    def test_basis_is_orthonormal(self):
        cam = camera3d.OrbitCamera(yaw_deg=37.0, pitch_deg=23.0)
        rot = cam.basis()
        np.testing.assert_allclose(rot @ rot.T, np.eye(3), atol=1e-9)

    def test_basis_forward_points_at_target(self):
        cam = camera3d.OrbitCamera(target=(0.0, 0.0, 0.0), distance=10.0,
                                   yaw_deg=0.0, pitch_deg=0.0)
        forward = cam.basis()[2]
        np.testing.assert_allclose(forward, [-1.0, 0.0, 0.0], atol=1e-9)

    def test_presets_have_verified_angles(self):
        self.assertEqual(camera3d.PRESETS["top"], (180.0, 89.0))
        self.assertEqual(camera3d.PRESETS["front"], (0.0, 0.0))
        self.assertEqual(camera3d.PRESETS["side"], (270.0, 0.0))
        self.assertEqual(camera3d.PRESETS["iso"], (315.0, 25.0))

    def test_preset_keeps_distance_unless_given(self):
        cam = camera3d.OrbitCamera(distance=42.0).preset("top")
        self.assertAlmostEqual(cam.distance, 42.0)
        self.assertAlmostEqual(cam.yaw_deg, 180.0)
        self.assertAlmostEqual(cam.pitch_deg, 89.0)
        self.assertAlmostEqual(cam.preset("iso", distance=77.0).distance, 77.0)

    def test_preset_resets_target_to_origin(self):
        cam = camera3d.OrbitCamera(target=(5.0, 5.0, 5.0)).preset("iso")
        self.assertEqual(cam.target, (0.0, 0.0, 0.0))

    def test_unknown_preset_raises(self):
        with self.assertRaises(KeyError):
            camera3d.OrbitCamera().preset("nope")


class ProjectionTests(unittest.TestCase):
    W = 960
    H = 600

    def _cam(self, **kwargs):
        base = dict(target=(0.0, 0.0, 0.0), distance=10.0,
                    yaw_deg=0.0, pitch_deg=0.0, fov_deg=60.0)
        base.update(kwargs)
        return camera3d.OrbitCamera(**base)

    def test_focal_length_matches_vertical_fov(self):
        expected = 0.5 * self.H / math.tan(math.radians(30.0))
        self.assertAlmostEqual(camera3d.focal_length(60.0, self.H), expected)

    def test_target_point_lands_at_screen_center(self):
        px, py, _, keep = camera3d.project(
            np.array([[0.0, 0.0, 0.0]]), self._cam(), self.W, self.H)
        self.assertTrue(keep[0])
        self.assertEqual((px[0], py[0]), (self.W // 2, self.H // 2))

    def test_point_behind_camera_is_culled(self):
        # The camera sits at +x=10 looking at the origin, so x=50 is behind it.
        _, _, _, keep = camera3d.project(
            np.array([[50.0, 0.0, 0.0]]), self._cam(), self.W, self.H)
        self.assertFalse(keep[0])

    def test_point_outside_frustum_is_culled(self):
        _, _, _, keep = camera3d.project(
            np.array([[0.0, 5000.0, 0.0]]), self._cam(), self.W, self.H)
        self.assertFalse(keep[0])

    def test_left_axis_goes_to_screen_right_in_front_view(self):
        # The front preset faces the vehicle, so its left side (+y) appears on
        # the right of the screen.
        px, _, _, keep = camera3d.project(
            np.array([[0.0, 2.0, 0.0]]), self._cam(), self.W, self.H)
        self.assertTrue(keep[0])
        self.assertGreater(px[0], self.W // 2)

    def test_up_axis_goes_to_screen_top(self):
        _, py, _, keep = camera3d.project(
            np.array([[0.0, 0.0, 2.0]]), self._cam(), self.W, self.H)
        self.assertTrue(keep[0])
        self.assertLess(py[0], self.H // 2)

    def test_depth_increases_with_distance_from_camera(self):
        pts = np.array([[0.0, 0.0, 0.0], [-5.0, 0.0, 0.0], [-9.0, 0.0, 0.0]])
        _, _, depth, keep = camera3d.project(pts, self._cam(), self.W, self.H)
        self.assertTrue(keep.all())
        self.assertTrue(np.all(np.diff(depth) > 0.0))

    def test_empty_input_returns_empty_arrays(self):
        px, py, depth, keep = camera3d.project(
            np.zeros((0, 3), np.float32), self._cam(), self.W, self.H)
        self.assertEqual(px.size, 0)
        self.assertEqual(py.size, 0)
        self.assertEqual(depth.size, 0)
        self.assertEqual(keep.size, 0)

    def test_nan_and_inf_points_are_culled(self):
        # An infinite depth passes depth > NEAR, and x/inf == 0 puts it exactly
        # at the screen centre. Without the isfinite mask, invalid points pile
        # up there as a visible blob.
        pts = np.array([
            [0.0, 0.0, 0.0],
            [np.nan, 0.0, 0.0],
            [np.inf, 0.0, 0.0],
            [0.0, np.nan, 0.0],
            [-np.inf, 0.0, 0.0],
        ])
        px, py, _, keep = camera3d.project(pts, self._cam(), self.W, self.H)
        self.assertEqual(int(keep.sum()), 1)
        self.assertTrue(keep[0])
        self.assertEqual(int(((px == self.W // 2) & (py == self.H // 2)).sum()), 1)

    def test_projection_emits_no_runtime_warnings_for_invalid_points(self):
        import warnings
        pts = np.array([[np.nan, np.nan, np.nan], [np.inf, 0.0, 0.0]])
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            camera3d.project(pts, self._cam(), self.W, self.H)
        self.assertEqual([w for w in caught if issubclass(w.category, RuntimeWarning)], [])

    def test_bounds_leave_one_pixel_margin_for_splatting(self):
        # keep bounds at width-1 / height-1, so a +1 splat always stays in range.
        pts = np.array([[0.0, 0.0, 0.0]])
        px, py, _, _ = camera3d.project(pts, self._cam(), self.W, self.H)
        self.assertLess(px.max() + 1, self.W)
        self.assertLess(py.max() + 1, self.H)

    def test_top_preset_puts_forward_axis_at_screen_top(self):
        # The evidence for deleting the 2D view: the top preset reproduces its
        # axis orientation.
        cam = camera3d.OrbitCamera(distance=60.0).preset("top")
        px, py, _, keep = camera3d.project(
            np.array([[20.0, 0.0, 0.0], [0.0, 20.0, 0.0]]), cam, self.W, self.H)
        self.assertTrue(keep.all())
        self.assertLess(py[0], self.H // 2)     # +x forward -> top of screen
        self.assertLess(px[1], self.W // 2)     # +y left -> left of screen

    def test_side_preset_puts_forward_axis_at_screen_right(self):
        cam = camera3d.OrbitCamera(distance=60.0).preset("side")
        px, _, _, keep = camera3d.project(
            np.array([[20.0, 0.0, 0.0]]), cam, self.W, self.H)
        self.assertTrue(keep[0])
        self.assertGreater(px[0], self.W // 2)


class InputMappingTests(unittest.TestCase):
    def test_drag_right_orbits_left(self):
        dyaw, _ = camera3d.drag_to_orbit(10.0, 0.0)
        self.assertLess(dyaw, 0.0)

    def test_drag_down_raises_pitch(self):
        _, dpitch = camera3d.drag_to_orbit(0.0, 10.0)
        self.assertGreater(dpitch, 0.0)

    def test_drag_sensitivity_is_degrees_per_pixel(self):
        dyaw, dpitch = camera3d.drag_to_orbit(-100.0, 100.0, sensitivity=0.4)
        self.assertAlmostEqual(dyaw, 40.0)
        self.assertAlmostEqual(dpitch, 40.0)

    def test_zero_drag_produces_no_rotation(self):
        self.assertEqual(camera3d.drag_to_orbit(0.0, 0.0), (0.0, 0.0))

    def test_wheel_up_zooms_in(self):
        self.assertLess(camera3d.wheel_to_zoom(1), 1.0)

    def test_wheel_down_zooms_out(self):
        self.assertGreater(camera3d.wheel_to_zoom(-1), 1.0)

    def test_wheel_round_trip_restores_distance(self):
        cam = camera3d.OrbitCamera(distance=100.0)
        back = cam.zoom(camera3d.wheel_to_zoom(1)).zoom(camera3d.wheel_to_zoom(-1))
        self.assertAlmostEqual(back.distance, 100.0, places=9)

    def test_pan_delta_scales_with_distance(self):
        near = camera3d.OrbitCamera(distance=10.0)
        far = camera3d.OrbitCamera(distance=100.0)
        near_delta = camera3d.pan_world_delta(10.0, 0.0, near, 600)
        far_delta = camera3d.pan_world_delta(10.0, 0.0, far, 600)
        self.assertAlmostEqual(
            float(np.linalg.norm(far_delta) / np.linalg.norm(near_delta)), 10.0, places=6)

    def test_pan_delta_is_perpendicular_to_view_direction(self):
        cam = camera3d.OrbitCamera(distance=50.0, yaw_deg=33.0, pitch_deg=17.0)
        delta = camera3d.pan_world_delta(7.0, -3.0, cam, 600)
        forward = cam.basis()[2]
        self.assertAlmostEqual(float(np.dot(delta, forward)), 0.0, places=9)

    def test_pan_delta_is_zero_for_zero_drag(self):
        delta = camera3d.pan_world_delta(0.0, 0.0, camera3d.OrbitCamera(), 600)
        np.testing.assert_allclose(delta, np.zeros(3), atol=1e-12)

    def test_pan_then_unpan_restores_target(self):
        cam = camera3d.OrbitCamera(target=(3.0, 4.0, 5.0), distance=25.0)
        delta = camera3d.pan_world_delta(12.0, -8.0, cam, 600)
        back = cam.pan(delta).pan(-delta)
        for got, want in zip(back.target, (3.0, 4.0, 5.0)):
            self.assertAlmostEqual(got, want, places=9)


if __name__ == "__main__":
    unittest.main()
