from __future__ import annotations

from pathlib import Path
import os
import sys
import threading
import time
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from panels import lidar_panel
from utils import camera3d
from utils import ui_queue


def _camera(**kwargs):
    base = dict(target=(0.0, 0.0, 0.0), distance=60.0,
                yaw_deg=315.0, pitch_deg=25.0, fov_deg=60.0)
    base.update(kwargs)
    return camera3d.OrbitCamera(**base)


class ClipSegmentTests(unittest.TestCase):
    def test_segment_fully_in_front_is_unchanged(self):
        cam = _camera(distance=10.0, yaw_deg=0.0, pitch_deg=0.0)
        a, b = np.array([0.0, -1.0, 0.0]), np.array([0.0, 1.0, 0.0])
        clipped = lidar_panel.clip_segment(a, b, cam)
        self.assertIsNotNone(clipped)
        np.testing.assert_allclose(clipped[0], a)
        np.testing.assert_allclose(clipped[1], b)

    def test_segment_fully_behind_is_dropped(self):
        cam = _camera(distance=10.0, yaw_deg=0.0, pitch_deg=0.0)
        a, b = np.array([50.0, 0.0, 0.0]), np.array([80.0, 0.0, 0.0])
        self.assertIsNone(lidar_panel.clip_segment(a, b, cam))

    def test_straddling_segment_is_clipped_not_dropped(self):
        # A segment with one endpoint behind the camera is clipped at the near
        # plane, not dropped: dropping it breaks the grid up as the viewpoint
        # lowers.
        cam = _camera(distance=10.0, yaw_deg=0.0, pitch_deg=0.0)
        a, b = np.array([80.0, 0.0, 0.0]), np.array([-20.0, 0.0, 0.0])
        clipped = lidar_panel.clip_segment(a, b, cam)
        self.assertIsNotNone(clipped)
        forward = cam.basis()[2]
        origin = cam.position()
        self.assertGreaterEqual(float(np.dot(clipped[0] - origin, forward)), camera3d.NEAR - 1e-6)
        np.testing.assert_allclose(clipped[1], b)


class ClipProjectionIntegrationTests(unittest.TestCase):
    def test_clipped_endpoint_survives_projection(self):
        # Regression: clipping to exactly NEAR put the crossing point fractionally
        # below NEAR after rounding, and _to_screen then dropped it.
        cam = _camera(distance=10.0, yaw_deg=0.0, pitch_deg=0.0)
        clipped = lidar_panel.clip_segment(
            np.array([80.0, 0.0, 0.0]), np.array([-20.0, 0.0, 0.0]), cam)
        self.assertIsNotNone(clipped)
        self.assertIsNotNone(lidar_panel._to_screen(clipped[0], cam))
        self.assertIsNotNone(lidar_panel._to_screen(clipped[1], cam))

    def test_clipped_segments_survive_projection_across_poses(self):
        # Regression sweep: clipping to exactly NEAR dropped ~45% of clipped
        # endpoints end to end once float rounding pushed the crossing point's
        # depth fractionally below NEAR (reviewer's 191/421 measurement).
        # Seeded, so this is deterministic, not flaky.
        rng = np.random.default_rng(20260904)
        poses = [
            dict(distance=10.0, yaw_deg=0.0, pitch_deg=0.0),
            dict(distance=25.0, yaw_deg=45.0, pitch_deg=10.0),
            dict(distance=60.0, yaw_deg=315.0, pitch_deg=25.0),
            dict(distance=15.0, yaw_deg=180.0, pitch_deg=60.0),
            dict(distance=5.0, yaw_deg=90.0, pitch_deg=-30.0),
        ]
        checked = 0
        for pose in poses:
            cam = _camera(**pose)
            origin = cam.position()
            forward = cam.basis()[2]
            for _ in range(80):
                # Random ground-plane segment endpoints, like the grid lines
                # this guards against breaking up.
                pts = rng.uniform(-100.0, 100.0, size=(2, 3))
                pts[:, 2] = 0.0
                a, b = pts[0], pts[1]
                da = float(np.dot(a - origin, forward))
                db = float(np.dot(b - origin, forward))
                straddles = (da < camera3d.NEAR) != (db < camera3d.NEAR)
                if not straddles:
                    continue
                checked += 1
                clipped = lidar_panel.clip_segment(a, b, cam)
                self.assertIsNotNone(clipped, msg=(pose, a.tolist(), b.tolist()))
                self.assertIsNotNone(
                    lidar_panel._to_screen(clipped[0], cam), msg=(pose, a.tolist(), b.tolist()))
                self.assertIsNotNone(
                    lidar_panel._to_screen(clipped[1], cam), msg=(pose, a.tolist(), b.tolist()))
        # Sanity: the sweep must actually exercise straddling segments, else the
        # asserts above never run and the test is vacuously true.
        self.assertGreater(checked, 20)


class BackgroundTests(unittest.TestCase):
    def test_background_shape_and_dtype(self):
        bg = lidar_panel.build_background(_camera(), 50.0)
        self.assertEqual(bg.shape, (lidar_panel._CANVAS_H, lidar_panel._CANVAS_W, 3))
        self.assertEqual(bg.dtype, np.uint8)

    def test_background_draws_something(self):
        bg = lidar_panel.build_background(_camera(), 50.0)
        self.assertGreater(int((bg > 0).sum()), 0)

    def test_background_changes_with_camera(self):
        first = lidar_panel.build_background(_camera(yaw_deg=0.0), 50.0)
        second = lidar_panel.build_background(_camera(yaw_deg=90.0), 50.0)
        self.assertFalse(np.array_equal(first, second))

    def test_background_changes_with_range(self):
        first = lidar_panel.build_background(_camera(), 20.0)
        second = lidar_panel.build_background(_camera(), 200.0)
        self.assertFalse(np.array_equal(first, second))

    def _axis_pixels(self, bg):
        """Pixels painted in an axis colour, which is distinct from the grid colour."""
        mask = np.zeros(bg.shape[:2], bool)
        for color in lidar_panel._COLOR_AXES:
            mask |= np.all(bg == np.array(color, np.uint8), axis=2)
        return set(zip(*np.nonzero(mask)))

    def test_axes_are_one_metre_regardless_of_range(self):
        # The axes are an origin marker. Scaling them with extent puts a 50 m
        # bar across the scene at range 100. For one camera the axis pixels
        # must be identical whatever the range.
        self.assertEqual(lidar_panel._AXIS_LENGTH_M, 1.0)
        cam = _camera(distance=40.0)
        near = self._axis_pixels(lidar_panel.build_background(cam, 10.0))
        far = self._axis_pixels(lidar_panel.build_background(cam, 100.0))
        self.assertGreater(len(near), 0)
        self.assertEqual(near, far)

    def test_axes_occupy_a_small_corner_of_the_canvas(self):
        # Counting grid pixels is unreliable because LINE_AA blends colours.
        # Judge the axis size from its on-screen bounding box instead.
        cam = _camera(distance=40.0)
        pixels = self._axis_pixels(lidar_panel.build_background(cam, 100.0))
        ys, xs = zip(*pixels)
        span_x = max(xs) - min(xs)
        span_y = max(ys) - min(ys)
        self.assertLess(span_x, lidar_panel._CANVAS_W * 0.05)
        self.assertLess(span_y, lidar_panel._CANVAS_H * 0.10)

    def test_canvas_is_not_square(self):
        # The square constraint existed for the 2D polar rings; 3D has no such reason.
        self.assertNotEqual(lidar_panel._CANVAS_W, lidar_panel._CANVAS_H)


class RenderFrameTests(unittest.TestCase):
    def _render(self, points, camera=None, range_m=50.0, color_mode="Height"):
        cam = camera if camera is not None else _camera()
        return lidar_panel.render_frame(points, cam, range_m, color_mode).copy()

    def _as_image(self, flat):
        return flat.reshape(lidar_panel._CANVAS_H, lidar_panel._CANVAS_W, 4)

    def test_output_shape_dtype_and_range(self):
        out = self._render(np.zeros((0, 4), np.float32))
        self.assertEqual(out.shape, (lidar_panel._CANVAS_W * lidar_panel._CANVAS_H * 4,))
        self.assertEqual(out.dtype, np.float32)
        self.assertGreaterEqual(float(out.min()), 0.0)
        self.assertLessEqual(float(out.max()), 1.0)

    def test_empty_points_render_background_only(self):
        cam = _camera()
        out = self._render(np.zeros((0, 4), np.float32), camera=cam)
        bg = lidar_panel.build_background(cam, 50.0)
        rendered = (self._as_image(out)[:, :, :3] * 255.0).round().astype(np.uint8)
        expected = bg[:, :, ::-1]                      # BGR -> RGB
        np.testing.assert_array_equal(rendered, expected)

    def test_points_beyond_range_are_dropped(self):
        cam = _camera()
        far = np.array([[500.0, 500.0, 0.0, 1.0]], np.float32)
        with_far = self._render(far, camera=cam, range_m=50.0)
        empty = self._render(np.zeros((0, 4), np.float32), camera=cam, range_m=50.0)
        np.testing.assert_array_equal(with_far, empty)

    def test_a_visible_point_changes_its_pixel(self):
        cam = _camera(distance=10.0, yaw_deg=0.0, pitch_deg=0.0)
        pts = np.array([[0.0, 0.0, 0.0, 1.0]], np.float32)
        out = self._as_image(self._render(pts, camera=cam))
        empty = self._as_image(self._render(np.zeros((0, 4), np.float32), camera=cam))
        cx, cy = lidar_panel._CANVAS_W // 2, lidar_panel._CANVAS_H // 2
        self.assertFalse(np.array_equal(out[cy, cx], empty[cy, cx]))

    def test_nearer_point_wins_when_two_land_on_same_pixel(self):
        # Checks the painter's ordering (far -> near): where two points land on
        # the same pixel, the nearer colour must survive.
        cam = _camera(distance=20.0, yaw_deg=0.0, pitch_deg=0.0)
        near_only = np.array([[0.0, 0.0, 0.0, 0.0]], np.float32)
        far_only = np.array([[-10.0, 0.0, 0.0, 1.0]], np.float32)
        both = np.concatenate([far_only, near_only])
        cx, cy = lidar_panel._CANVAS_W // 2, lidar_panel._CANVAS_H // 2
        px_both = self._as_image(self._render(both, camera=cam))[cy, cx].copy()
        px_near = self._as_image(self._render(near_only, camera=cam))[cy, cx].copy()
        np.testing.assert_allclose(px_both, px_near, atol=1e-6)

    def test_invalid_points_do_not_blob_at_screen_center(self):
        # A +inf depth passes depth > NEAR and lands exactly at the screen
        # centre. Without the isfinite mask (P7) this catches the resulting
        # blob of invalid points.
        cam = _camera()
        bad = np.full((5000, 4), np.inf, np.float32)
        bad[:, 3] = 1.0
        out = self._render(bad, camera=cam)
        empty = self._render(np.zeros((0, 4), np.float32), camera=cam)
        np.testing.assert_array_equal(out, empty)

    def test_nan_points_are_ignored(self):
        cam = _camera()
        pts = np.full((100, 4), np.nan, np.float32)
        out = self._render(pts, camera=cam)
        empty = self._render(np.zeros((0, 4), np.float32), camera=cam)
        np.testing.assert_array_equal(out, empty)

    def test_intensity_mode_differs_from_height_mode(self):
        cam = _camera(distance=30.0)
        rng = np.random.default_rng(0)
        pts = np.empty((4000, 4), np.float32)
        pts[:, 0] = rng.uniform(-20, 20, 4000)
        pts[:, 1] = rng.uniform(-20, 20, 4000)
        pts[:, 2] = rng.uniform(-3, 3, 4000)
        pts[:, 3] = rng.uniform(0, 255, 4000)
        by_height = self._render(pts, camera=cam, color_mode="Height")
        by_intensity = self._render(pts, camera=cam, color_mode="Intensity")
        self.assertFalse(np.array_equal(by_height, by_intensity))

    def test_constant_attribute_does_not_crash(self):
        # When the low and high percentiles match, normalisation divides by zero.
        cam = _camera(distance=30.0)
        pts = np.zeros((500, 4), np.float32)
        pts[:, 0] = np.linspace(-10, 10, 500)
        out = self._render(pts, camera=cam)
        self.assertTrue(np.all(np.isfinite(out)))

    def test_render_allocates_no_new_canvas_between_frames(self):
        # P5: buffers live at module level and are not reallocated per frame.
        cam = _camera()
        pts = np.zeros((10, 4), np.float32)
        first = lidar_panel.render_frame(pts, cam, 50.0, "Height")
        second = lidar_panel.render_frame(pts, cam, 50.0, "Height")
        self.assertIs(first.base, second.base)

    def test_render_output_is_byte_identical_to_pre_p1_fix_golden_hash(self):
        # Finding 2: render_frame's range filter used to do a 2D fancy index
        # (pts = pts[within]) that allocated a fresh 9.6MB array every frame.
        # The fix folds the range mask and the frustum mask from
        # camera3d.project into a single 1D selector instead. These hashes
        # were captured from the *pre-fix* implementation on this exact
        # deterministic scenario -- if the refactor ever changes pixel output,
        # this must fail rather than silently drift.
        #
        # Re-baselined once, deliberately, when the XYZ axes were pinned to 1 m
        # instead of scaling with range. That change was verified to be the ONLY
        # cause of the new hashes: restoring axis_len = extent * 0.5 reproduces
        # the previous ones exactly. Any other drift is a regression.
        import hashlib
        cam = _camera(distance=60.0, yaw_deg=315.0, pitch_deg=25.0)
        rng = np.random.default_rng(0)
        n = 50_000
        pts = np.empty((n, 4), np.float32)
        theta = rng.uniform(0.0, 2.0 * np.pi, n)
        radius = rng.uniform(1.0, 80.0, n)     # deliberately extends past range_m=50
        pts[:, 0] = radius * np.cos(theta)
        pts[:, 1] = radius * np.sin(theta)
        pts[:, 2] = rng.uniform(-2.0, 4.0, n)
        pts[:, 3] = rng.uniform(0.0, 255.0, n)

        expected = {
            "Height": "065dd8e1776e67a6ca15349fe69459d7219ffc417fd2f2b588ae9e32754c383e",
            "Intensity": "d13d7eb5a7c52232b0e827edb7913f58f07919e84641e931819d79247c3f134a",
        }
        for color_mode, expected_hash in expected.items():
            out = self._render(pts, camera=cam, range_m=50.0, color_mode=color_mode)
            got_hash = hashlib.sha256(out.tobytes()).hexdigest()
            self.assertEqual(got_hash, expected_hash,
                             f"render_frame output changed for color_mode={color_mode}")

    def test_background_is_cached_until_camera_or_range_changes(self):
        cam = _camera()
        pts = np.zeros((0, 4), np.float32)
        lidar_panel.render_frame(pts, cam, 50.0, "Height")
        cached = lidar_panel._BG.copy()
        lidar_panel.render_frame(pts, cam, 50.0, "Height")
        np.testing.assert_array_equal(lidar_panel._BG, cached)
        lidar_panel.render_frame(pts, cam.orbit(45.0, 0.0), 50.0, "Height")
        self.assertFalse(np.array_equal(lidar_panel._BG, cached))


@unittest.skipUnless(os.environ.get("LIDAR_PERF"), "perf test runs only with LIDAR_PERF=1")
class RenderPerformanceTests(unittest.TestCase):
    """Detects the loss of P1 (1-D attribute pull) and P2 (uint16 bucket sort).

    Statistic: the minimum of 15 renders of 600k points. min-of-15 rather than
    mean-of-5, because a mean is dragged up by any single OS scheduling hiccup.
    Measured on Windows 11 / Python 3.12 / numpy 2.5 at 600k points:
    - with P1+P2:    52.2ms (range 52.2-55.3ms)
    - without them:  64.9ms (range 64.9-67.5ms)
    The 60ms budget is the midpoint of those minima (58.5, rounded). It is
    stable across repeated runs and does NOT detect the loss of a single
    smaller technique.
    """

    BUDGET_MS = 60.0

    def test_600k_points_render_within_budget(self):
        rng = np.random.default_rng(0)
        n = 600_000
        pts = np.empty((n, 4), np.float32)
        theta = rng.uniform(0.0, 2.0 * np.pi, n)
        radius = rng.uniform(1.0, 50.0, n)
        pts[:, 0] = radius * np.cos(theta)
        pts[:, 1] = radius * np.sin(theta)
        pts[:, 2] = rng.uniform(-2.0, 4.0, n)
        pts[:, 3] = rng.uniform(0.0, 255.0, n)
        cam = _camera()

        lidar_panel.render_frame(pts, cam, 50.0, "Height")   # warm-up
        times_ms = []
        for _ in range(15):
            start = time.perf_counter()
            lidar_panel.render_frame(pts, cam, 50.0, "Height")
            elapsed_ms = (time.perf_counter() - start) * 1000.0
            times_ms.append(elapsed_ms)
        min_ms = min(times_ms)
        # This guard assumes a quiet machine. With the simulator or the console
        # app running, the same code measured 52ms -> 62ms, about 20% slower,
        # which can exceed the budget. On failure, remove the load and re-measure
        # before investigating.
        self.assertLess(min_ms, self.BUDGET_MS,
                        f"600k render min {min_ms:.1f}ms > budget {self.BUDGET_MS}ms. "
                        f"If it still exceeds on a quiet machine, check constraints P1-P7.")


class RenderWorkerTests(unittest.TestCase):
    def setUp(self):
        self.workers = []

    def tearDown(self):
        for worker in self.workers:
            worker.stop()

    def _worker(self, fn):
        worker = lidar_panel.RenderWorker(fn)
        self.workers.append(worker)
        return worker

    def test_worker_renders_when_marked_dirty(self):
        done = threading.Event()
        worker = self._worker(done.set)
        worker.start()
        worker.mark_dirty()
        self.assertTrue(done.wait(2.0))

    def test_worker_stays_idle_until_marked(self):
        calls = []
        worker = self._worker(lambda: calls.append(1))
        worker.start()
        time.sleep(0.2)
        self.assertEqual(calls, [])

    def test_worker_survives_a_render_exception(self):
        # A dead worker freezes the view permanently and looks to the user like
        # nothing more than a stall.
        calls = []

        def flaky():
            calls.append(1)
            if len(calls) == 1:
                raise RuntimeError("boom")

        worker = self._worker(flaky)
        worker.start()
        worker.mark_dirty()
        deadline = time.time() + 2.0
        while len(calls) < 2 and time.time() < deadline:
            worker.mark_dirty()
            time.sleep(0.02)
        self.assertGreaterEqual(len(calls), 2)
        self.assertTrue(worker.is_running)

    def test_stop_terminates_the_thread(self):
        worker = self._worker(lambda: None)
        worker.start()
        worker.stop()
        self.assertFalse(worker.is_running)

    def test_mark_dirty_after_stop_is_a_noop(self):
        worker = self._worker(lambda: None)
        worker.start()
        worker.stop()
        worker.mark_dirty()          # must not raise
        self.assertFalse(worker.is_running)

    def test_repeated_marks_coalesce_into_fewer_renders(self):
        # Event coalesces the dirty signals: a drag can fire hundreds of events
        # per second and still needs only one render per frame.
        calls = []

        def slow():
            calls.append(1)
            time.sleep(0.05)

        worker = self._worker(slow)
        worker.start()
        for _ in range(200):
            worker.mark_dirty()
        time.sleep(0.3)
        self.assertLess(len(calls), 200)
        self.assertGreater(len(calls), 0)

    def test_stop_wakes_a_waiting_worker(self):
        # stop() must wake a worker idling in wait() and actually end the thread.
        # Removing _wake.set() from stop() leaves it blocked in wait() forever
        # while the join times out silently, so this asserts on the thread
        # object's liveness directly rather than on a flag.
        worker = self._worker(lambda: None)
        worker.start()
        thread = worker._thread
        self.assertIsNotNone(thread)
        worker.stop(timeout=1.0)
        self.assertFalse(thread.is_alive())

    def test_no_second_renderer_after_timed_out_stop(self):
        # A timed-out join must not relinquish the thread handle: doing so lets
        # the next start() spawn a second renderer, and two threads then draw
        # into the same canvas. The whole lock-free design rests on there being
        # exactly one renderer.
        started = threading.Event()
        release = threading.Event()

        def wedge():
            started.set()
            release.wait()

        worker = self._worker(wedge)
        worker.start()
        first_thread = worker._thread
        worker.mark_dirty()
        self.assertTrue(started.wait(2.0), "render never started")

        worker.stop(timeout=0.1)          # render_fn is wedged; join times out
        self.assertTrue(first_thread.is_alive(), "wedged thread should still be alive")

        worker.start()                    # a live renderer still exists; must refuse
        self.assertIs(worker._thread, first_thread)

        release.set()                     # let the wedged render_fn return
        worker.stop(timeout=2.0)          # now the thread can actually exit
        self.assertFalse(first_thread.is_alive())


class MouseHandlerTests(unittest.TestCase):
    """DPG event dispatch cannot run headless, but the callbacks are ordinary
    functions and can be called directly with synthetic app_data. The one
    obstacle, _hovering_view(), needs a rendered frame, so it is monkeypatched."""

    @classmethod
    def setUpClass(cls):
        import dearpygui.dearpygui as dpg
        dpg.create_context()          # dpg.does_item_exist segfaults without a context

    def setUp(self):
        self._orig_camera = lidar_panel._camera
        self._orig_hovering = lidar_panel._hovering_view
        self._orig_save_state = lidar_panel._save_state
        lidar_panel._camera = _camera()
        lidar_panel._drag_prev["left"] = (0.0, 0.0)
        lidar_panel._drag_prev["pan"] = (0.0, 0.0)
        lidar_panel._camera_gesture_pending = False
        lidar_panel._hovering_view = lambda: True

    def tearDown(self):
        lidar_panel._camera = self._orig_camera
        lidar_panel._hovering_view = self._orig_hovering
        lidar_panel._save_state = self._orig_save_state
        lidar_panel._drag_prev["left"] = (0.0, 0.0)
        lidar_panel._drag_prev["pan"] = (0.0, 0.0)
        lidar_panel._camera_gesture_pending = False

    def test_orbit_drag_uses_increment_not_cumulative_value(self):
        # Treating the cumulative 25px as a delta makes the camera run away:
        # 10 -> 25 must be an increment of 15.
        base = lidar_panel._camera
        lidar_panel._on_orbit_drag(None, [0, 10.0, 0.0])
        after_first = lidar_panel._camera
        dyaw1, _ = camera3d.drag_to_orbit(10.0, 0.0)
        self.assertAlmostEqual(after_first.yaw_deg, (base.yaw_deg + dyaw1) % 360.0, places=6)

        lidar_panel._on_orbit_drag(None, [0, 25.0, 0.0])
        after_second = lidar_panel._camera
        dyaw2, _ = camera3d.drag_to_orbit(15.0, 0.0)          # increment, not 25px
        expected_yaw = (after_first.yaw_deg + dyaw2) % 360.0
        self.assertAlmostEqual(after_second.yaw_deg, expected_yaw, places=6)

        # Sanity: confirm it did NOT apply the cumulative 25px as an increment.
        wrong_dyaw, _ = camera3d.drag_to_orbit(25.0, 0.0)
        wrong_yaw = (after_first.yaw_deg + wrong_dyaw) % 360.0
        self.assertNotAlmostEqual(after_second.yaw_deg, wrong_yaw, places=6)

    def test_release_resets_the_accumulator(self):
        lidar_panel._on_orbit_drag(None, [0, 40.0, 0.0])
        lidar_panel._on_mouse_release(None, None)
        self.assertEqual(lidar_panel._drag_prev["left"], (0.0, 0.0))

        after_release = lidar_panel._camera
        lidar_panel._on_orbit_drag(None, [0, 3.0, 0.0])
        dyaw, _ = camera3d.drag_to_orbit(3.0, 0.0)            # fresh increment, not 3-40=-37
        expected_yaw = (after_release.yaw_deg + dyaw) % 360.0
        self.assertAlmostEqual(lidar_panel._camera.yaw_deg, expected_yaw, places=6)

    def test_release_without_a_camera_gesture_does_not_save(self):
        # Regression for the global-handler-registry bug: _on_mouse_release fires
        # on every mouse release anywhere in the app (other tabs included), not
        # just releases over the LiDAR view. Without a real drag/wheel gesture
        # first, it must not touch config/lidar_sensor_state.json.
        calls = []
        lidar_panel._save_state = lambda: calls.append(1)
        lidar_panel._on_mouse_release(None, None)
        self.assertEqual(calls, [])

    def test_drag_then_release_saves_exactly_once(self):
        calls = []
        lidar_panel._save_state = lambda: calls.append(1)
        lidar_panel._on_orbit_drag(None, [0, 10.0, 0.0])
        self.assertEqual(calls, [])          # drag itself must not save
        lidar_panel._on_mouse_release(None, None)
        self.assertEqual(len(calls), 1)
        # A second release with no new gesture must not save again.
        lidar_panel._on_mouse_release(None, None)
        self.assertEqual(len(calls), 1)

    def test_wheel_notches_do_not_save_per_notch_only_release_does(self):
        # A single trackpad flick used to fire _save_state once per wheel
        # notch (dozens of file writes). Now it must accumulate into one
        # save on release.
        calls = []
        lidar_panel._save_state = lambda: calls.append(1)
        for _ in range(5):
            lidar_panel._on_wheel(None, 1)
        self.assertEqual(calls, [])
        lidar_panel._on_mouse_release(None, None)
        self.assertEqual(len(calls), 1)

    def test_hover_guard_blocks_drag_and_wheel(self):
        lidar_panel._hovering_view = lambda: False
        before = lidar_panel._camera
        lidar_panel._on_orbit_drag(None, [0, 50.0, 0.0])
        lidar_panel._on_pan_drag(None, [2, 50.0, 0.0])
        lidar_panel._on_wheel(None, 5)
        self.assertEqual(lidar_panel._camera, before)

    def test_wheel_direction(self):
        before_distance = lidar_panel._camera.distance
        lidar_panel._on_wheel(None, 1)
        self.assertLess(lidar_panel._camera.distance, before_distance)

        lidar_panel._camera = lidar_panel._camera.orbit(0.0, 0.0)  # no-op, keep state simple
        mid_distance = lidar_panel._camera.distance
        lidar_panel._on_wheel(None, -1)
        self.assertGreater(lidar_panel._camera.distance, mid_distance)

    def test_pan_uses_world_space_conversion(self):
        # The same pixel drag must move the target further when the camera is
        # further away, because pan_world_delta scales by the world height the
        # viewport spans.
        lidar_panel._camera = _camera(distance=10.0, target=(0.0, 0.0, 0.0))
        lidar_panel._drag_prev["pan"] = (0.0, 0.0)
        lidar_panel._on_pan_drag(None, [2, 20.0, 0.0])
        near_shift = np.linalg.norm(
            np.asarray(lidar_panel._camera.target) - np.array([0.0, 0.0, 0.0]))

        lidar_panel._camera = _camera(distance=200.0, target=(0.0, 0.0, 0.0))
        lidar_panel._drag_prev["pan"] = (0.0, 0.0)
        lidar_panel._on_pan_drag(None, [2, 20.0, 0.0])
        far_shift = np.linalg.norm(
            np.asarray(lidar_panel._camera.target) - np.array([0.0, 0.0, 0.0]))

        self.assertGreater(far_shift, near_shift)

    def test_install_mouse_handlers_is_idempotent(self):
        import dearpygui.dearpygui as dpg
        if dpg.does_item_exist("lidar_handlers"):
            dpg.delete_item("lidar_handlers")
        try:
            lidar_panel._install_mouse_handlers()
            self.assertTrue(dpg.does_item_exist("lidar_handlers"))
            first_children = dpg.get_item_children("lidar_handlers", slot=1)
            lidar_panel._install_mouse_handlers()          # must not raise or duplicate
            second_children = dpg.get_item_children("lidar_handlers", slot=1)
            self.assertEqual(first_children, second_children)
        finally:
            if dpg.does_item_exist("lidar_handlers"):
                dpg.delete_item("lidar_handlers")


class TextureBacklogTests(unittest.TestCase):
    """Finding 3: at most one texture may be in flight in ui_queue at a time.

    Regression for the unbounded backlog: _render_once used to post a fresh
    9.2MB closure every call regardless of whether the UI thread had drained
    the previous one. If the main loop stalls (a modal, a minimize, another
    panel's slow callback) the render worker keeps rendering at up to 30Hz,
    so the queue must cap at one in-flight texture, not grow without bound.
    """

    @classmethod
    def setUpClass(cls):
        import dearpygui.dearpygui as dpg
        try:
            dpg.create_context()          # idempotent-ish; ignore if already created
        except Exception:
            pass

    def _drain_queue_object(self):
        while not ui_queue._q.empty():
            try:
                ui_queue._q.get_nowait()
            except Exception:
                break

    def setUp(self):
        self._drain_queue_object()
        lidar_panel._texture_in_flight.clear()
        self._orig_points = lidar_panel._state.points
        lidar_panel._state.points = np.zeros((0, 4), np.float32)

    def tearDown(self):
        lidar_panel._state.points = self._orig_points
        lidar_panel._texture_in_flight.clear()
        self._drain_queue_object()

    def test_repeated_renders_while_ui_thread_is_stalled_post_at_most_one_texture(self):
        # Simulate the UI thread never running _apply (the stall scenario):
        # the in-flight flag never clears, so repeated renders must not grow
        # the backlog past the one texture already queued.
        for _ in range(20):
            lidar_panel._render_once()
        self.assertEqual(ui_queue._q.qsize(), 1)

    def test_draining_clears_the_flag_and_allows_the_next_post(self):
        lidar_panel._render_once()
        self.assertEqual(ui_queue._q.qsize(), 1)
        ui_queue.drain()                      # runs _apply, which must clear the flag
        self.assertFalse(lidar_panel._texture_in_flight.is_set())
        lidar_panel._render_once()
        self.assertEqual(ui_queue._q.qsize(), 1)


class PanelStopGuardTests(unittest.TestCase):
    """Finding 4: stop() must not discard the single-renderer guard.

    RenderWorker.stop() deliberately keeps its thread handle when join times
    out, so a later start() refuses to create a second renderer against the
    shared _CAN/_OUT buffers. The panel's stop() must only drop its module
    reference once the worker actually stopped.
    """

    class _FakeWorker:
        def __init__(self, stays_alive):
            self._stays_alive = stays_alive
            self.stop_called = False

        def stop(self):
            self.stop_called = True

        def mark_dirty(self):
            pass

        @property
        def is_running(self):
            return self._stays_alive

    def setUp(self):
        self._orig_worker = lidar_panel._worker
        self._orig_receiver = lidar_panel._state.receiver
        lidar_panel._state.receiver = None

    def tearDown(self):
        lidar_panel._worker = self._orig_worker
        lidar_panel._state.receiver = self._orig_receiver

    def test_stop_keeps_worker_reference_when_it_is_still_running(self):
        # Models a timed-out join: RenderWorker.stop() returned but is_running
        # is still True. The module reference must survive so a later start()
        # sees the live worker and refuses to spin up a second one.
        fake = self._FakeWorker(stays_alive=True)
        lidar_panel._worker = fake
        lidar_panel.stop()
        self.assertTrue(fake.stop_called)
        self.assertIs(lidar_panel._worker, fake)

    def test_stop_clears_worker_reference_once_it_has_actually_stopped(self):
        fake = self._FakeWorker(stays_alive=False)
        lidar_panel._worker = fake
        lidar_panel.stop()
        self.assertTrue(fake.stop_called)
        self.assertIsNone(lidar_panel._worker)


if __name__ == "__main__":
    unittest.main()
