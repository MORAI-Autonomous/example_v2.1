from __future__ import annotations

from pathlib import Path
import struct
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from receivers.lidar_receiver import LidarReceiver
from receivers import lidar_receiver


def _make_receiver(max_points: int = 300_000) -> LidarReceiver:
    return LidarReceiver(tmpl_path=None, max_points=max_points)


def _pack_payload(
    receiver: LidarReceiver,
    frame_id: str,
    height: int,
    width: int,
    is_bigendian: bool,
    point_step: int,
    row_step: int,
    points: list,
    is_dense=True,
    include_is_dense: bool = True,
    point_count_override=None,
) -> bytes:
    frame_id_bytes = frame_id.encode("utf-8")
    header = struct.pack(
        receiver._header_fmt,
        frame_id_bytes,
        height,
        width,
        is_bigendian,
        point_step,
        row_step,
    )
    n_points = point_count_override if point_count_override is not None else len(points)
    data = b"".join(struct.pack("<4f", *p) for p in points[:n_points])
    payload = header + data
    if include_is_dense:
        payload += struct.pack("<?", is_dense)
    return payload


class LidarReceiverHeaderTests(unittest.TestCase):
    def test_header_field_offsets(self) -> None:
        receiver = _make_receiver()
        points = [(1.0, 2.0, 3.0, 0.5), (4.0, 5.0, 6.0, 0.9)]
        payload = _pack_payload(
            receiver,
            frame_id="frame_abc",
            height=1,
            width=2,
            is_bigendian=False,
            point_step=16,
            row_step=32,
            points=points,
            is_dense=True,
        )

        packet = receiver._parse_payload(payload)
        self.assertIsNotNone(packet)
        self.assertEqual(packet["frame_id"], "frame_abc")
        self.assertEqual(packet["height"], 1)
        self.assertEqual(packet["width"], 2)
        self.assertEqual(packet["point_step"], 16)
        self.assertEqual(packet["row_step"], 32)
        self.assertIs(packet["is_dense"], True)
        self.assertEqual(packet["raw_size"], len(payload))
        self.assertFalse(packet["points_truncated"])

    def test_point_array_values(self) -> None:
        receiver = _make_receiver()
        points = [(1.0, 2.0, 3.0, 0.5), (4.0, 5.0, 6.0, 0.9)]
        payload = _pack_payload(
            receiver,
            frame_id="frame_xyz",
            height=1,
            width=2,
            is_bigendian=False,
            point_step=16,
            row_step=32,
            points=points,
        )

        packet = receiver._parse_payload(payload)
        self.assertIsNotNone(packet)
        self.assertEqual(packet["point_count"], 2)
        for actual, expected in zip(packet["points"].tolist(), points):
            for a, e in zip(actual, expected):
                self.assertAlmostEqual(a, e, places=5)

    def test_max_points_cap(self) -> None:
        receiver = _make_receiver(max_points=3)
        points = [
            (1.0, 0.0, 0.0, 0.1),
            (2.0, 0.0, 0.0, 0.2),
            (3.0, 0.0, 0.0, 0.3),
            (4.0, 0.0, 0.0, 0.4),
            (5.0, 0.0, 0.0, 0.5),
        ]
        payload = _pack_payload(
            receiver,
            frame_id="frame_cap",
            height=1,
            width=5,
            is_bigendian=False,
            point_step=16,
            row_step=80,
            points=points,
            is_dense=True,
        )

        packet = receiver._parse_payload(payload)
        self.assertIsNotNone(packet)
        self.assertEqual(packet["width"], 5)
        self.assertEqual(packet["point_count"], 3)
        self.assertEqual(packet["points"].shape[0], 3)
        self.assertTrue(packet["points_truncated"])
        # is_dense sits after the FULL reported width (5), not the capped count (3).
        self.assertIs(packet["is_dense"], True)

    def test_short_payload_rejected(self) -> None:
        receiver = _make_receiver()
        # Declares width=10 but only ships 3 points worth of point bytes, no is_dense.
        payload = _pack_payload(
            receiver,
            frame_id="frame_short",
            height=1,
            width=10,
            is_bigendian=False,
            point_step=16,
            row_step=160,
            points=[(1.0, 1.0, 1.0, 1.0)] * 3,
            include_is_dense=False,
            point_count_override=3,
        )

        packet = receiver._parse_payload(payload)
        self.assertIsNone(packet)

    def test_empty_scan_width_zero(self) -> None:
        receiver = _make_receiver()
        payload = _pack_payload(
            receiver,
            frame_id="frame_empty",
            height=1,
            width=0,
            is_bigendian=False,
            point_step=16,
            row_step=0,
            points=[],
            is_dense=True,
        )

        packet = receiver._parse_payload(payload)
        self.assertIsNotNone(packet)
        self.assertEqual(packet["width"], 0)
        self.assertEqual(packet["point_count"], 0)
        self.assertEqual(packet["points"].shape, (0, 4))

    def test_absurd_width_is_rejected_before_offset_math(self):
        receiver = _make_receiver()
        payload = _pack_payload(
            receiver, "base_link", 1, 2_000_000_000, False, 16, 32,
            [(1.0, 2.0, 3.0, 4.0)], point_count_override=1,
        )
        self.assertIsNone(receiver._parse_payload(payload))

    def test_width_clamp_rejects_when_short_data_guard_would_not(self):
        receiver = _make_receiver(max_points=1)
        payload = _pack_payload(receiver, "base_link", 1, 2_000_000_000, False, 16, 32,
                                [(1.0, 2.0, 3.0, 4.0)])
        self.assertIsNone(receiver._parse_payload(payload))

    def test_parse_header_returns_fields(self):
        receiver = _make_receiver()
        payload = _pack_payload(
            receiver, "base_link", 1, 1, False, 16, 16, [(1.0, 2.0, 3.0, 4.0)])
        parsed = receiver._parse_header(payload)
        self.assertIsNotNone(parsed)
        frame_id, height, width, is_bigendian, point_step, row_step = parsed
        self.assertEqual(frame_id, "base_link")
        self.assertEqual(height, 1)
        self.assertEqual(width, 1)
        self.assertFalse(is_bigendian)
        self.assertEqual(point_step, 16)
        self.assertEqual(row_step, 16)

    def test_parse_header_returns_none_when_too_short(self):
        receiver = _make_receiver()
        self.assertIsNone(receiver._parse_header(b"\x00" * 4))


class LidarReceiverChunkReassemblyTests(unittest.TestCase):
    def test_chunked_reassembly_out_of_order(self) -> None:
        receiver = _make_receiver()
        points = [(float(i), float(i) * 2, float(i) * 3, 0.5) for i in range(8)]
        payload = _pack_payload(
            receiver,
            frame_id="frame_chunked",
            height=1,
            width=len(points),
            is_bigendian=False,
            point_step=16,
            row_step=16 * len(points),
            points=points,
            is_dense=True,
        )
        direct_packet = receiver._parse_payload(payload)
        self.assertIsNotNone(direct_packet)

        chunk_size = 20
        chunks = [payload[i : i + chunk_size] for i in range(0, len(payload), chunk_size)]
        total_chunks = len(chunks)
        framed = [
            struct.pack("<IHH", 1, idx, total_chunks) + chunk
            for idx, chunk in enumerate(chunks)
        ]
        # Shuffle deterministically (reverse) to prove order independence.
        shuffled = list(reversed(framed))

        received = []
        receiver.on_packet = lambda pkt: received.append(pkt)
        for datagram in shuffled:
            receiver._handle_datagram(datagram)

        self.assertEqual(len(received), 1)
        self.assertEqual(received[0]["frame_id"], direct_packet["frame_id"])
        self.assertEqual(received[0]["point_count"], direct_packet["point_count"])
        self.assertEqual(received[0]["raw_size"], direct_packet["raw_size"])


_CHUNK_HDR = struct.Struct("<IHH")


def _chunk_datagrams(payload: bytes, packet_id: int, chunk_size: int) -> list:
    """Split a payload into equally sized chunk datagrams."""
    parts = [payload[i:i + chunk_size] for i in range(0, len(payload), chunk_size)]
    total = len(parts)
    return [_CHUNK_HDR.pack(packet_id, idx, total) + part
            for idx, part in enumerate(parts)]


class LidarReceiverPartialAssemblyTests(unittest.TestCase):
    CHUNK = 64

    def setUp(self):
        self.received = []
        self.receiver = _make_receiver()
        self.receiver.on_packet = self.received.append

    def _payload(self, n_points: int) -> bytes:
        pts = [(float(i), float(i) + 0.5, float(i) + 0.25, float(i % 7)) for i in range(n_points)]
        return _pack_payload(self.receiver, "base_link", 1, n_points, False, 16, 16, pts)

    def _feed(self, datagrams, drop=()):
        for idx, dgram in enumerate(datagrams):
            if idx in drop:
                continue
            self.receiver._handle_datagram(dgram)

    def test_lossless_reassembly_matches_direct_parse(self):
        payload = self._payload(40)
        expected = self.receiver._parse_payload(payload)
        self._feed(_chunk_datagrams(payload, 1, self.CHUNK))
        self.assertEqual(len(self.received), 1)
        got = self.received[0]
        self.assertEqual(got["point_count"], expected["point_count"])
        self.assertEqual(got["chunk_loss"], 0.0)
        np.testing.assert_array_equal(got["points"], expected["points"])

    def test_missing_middle_chunk_recovers_remaining_points(self):
        payload = self._payload(40)
        dgrams = _chunk_datagrams(payload, 1, self.CHUNK)
        self.assertGreater(len(dgrams), 3)
        self._feed(dgrams, drop={2})
        self.receiver._handle_datagram(_chunk_datagrams(self._payload(1), 2, self.CHUNK)[0])
        self.assertGreaterEqual(len(self.received), 1)
        partial = self.received[0]
        self.assertGreater(partial["point_count"], 0)
        self.assertLess(partial["point_count"], 40)
        self.assertGreater(partial["chunk_loss"], 0.0)

    def test_missing_header_chunk_yields_no_packet_and_no_crash(self):
        # NOTE: this does NOT discriminate gap-aware recovery from the old
        # all-or-nothing path -- both deliver nothing when chunk 0 (the header)
        # is missing, since chunk 0 is the only place frame_id/width/etc. live.
        # What this genuinely covers is crash-safety: `_chunk_runs`'s
        # `if 0 not in asm.chunks: return []` guard prevents a KeyError from
        # `len(asm.chunks[0])` when chunk 0 never arrived.
        payload = self._payload(40)
        dgrams = _chunk_datagrams(payload, 1, self.CHUNK)
        self._feed(dgrams, drop={0})
        self.receiver._handle_datagram(_chunk_datagrams(self._payload(1), 2, self.CHUNK)[0])
        self.assertEqual([p for p in self.received if p["packet_seq"] == 1], [])

    def test_missing_last_chunk_recovers_prefix(self):
        payload = self._payload(40)
        dgrams = _chunk_datagrams(payload, 1, self.CHUNK)
        self._feed(dgrams, drop={len(dgrams) - 1})
        self.receiver._handle_datagram(_chunk_datagrams(self._payload(1), 2, self.CHUNK)[0])
        self.assertGreater(self.received[0]["point_count"], 0)

    def test_multiple_gaps_recover_every_run(self):
        payload = self._payload(80)
        dgrams = _chunk_datagrams(payload, 1, self.CHUNK)
        self.assertGreater(len(dgrams), 6)
        self._feed(dgrams, drop={2, 5})
        self.receiver._handle_datagram(_chunk_datagrams(self._payload(1), 2, self.CHUNK)[0])
        recovered = self.received[0]["point_count"]
        # Only two chunks were lost, so most points must survive.
        self.assertGreater(recovered, 80 * 0.6)

    def test_non_uniform_chunk_sizes_fall_back_to_prefix(self):
        n_points = 40
        pts = [(float(i), float(i) + 0.5, float(i) + 0.25, float(i % 7)) for i in range(n_points)]
        payload = _pack_payload(self.receiver, "base_link", 1, n_points, False, 16, 16, pts)
        dgrams = _chunk_datagrams(payload, 1, self.CHUNK)
        # Chunk index 2 is pure point data (chunks 0-1 already cover the full
        # 81-byte header), so truncating it isolates the non-uniform-size
        # behaviour from header corruption. Shrink it from 64 to 16 bytes and
        # drop a later chunk (index 4) to combine truncation with an outright gap.
        head, idx, total = _CHUNK_HDR.unpack(dgrams[2][:8])
        dgrams[2] = _CHUNK_HDR.pack(head, idx, total) + dgrams[2][8:8 + 16]
        self._feed(dgrams, drop={4})
        self.receiver._handle_datagram(_chunk_datagrams(self._payload(1), 2, self.CHUNK)[0])

        recovered = self.received[0]["points"]
        self.assertGreater(recovered.shape[0], 0)
        # Prefix reconstruction must yield the ORIGINAL leading points, byte-exact.
        # Without the uniform-size fallback stopping precisely at the short chunk,
        # the run keeps extending past it and silently splices in bytes from a
        # later chunk at a miscomputed offset -- which can land exactly on a
        # point boundary and return a *different real point* (e.g. point 7's
        # values in point 4's slot) instead of crashing or looking obviously wrong.
        expected = np.array(pts[:recovered.shape[0]], dtype=np.float32)
        np.testing.assert_array_equal(recovered, expected)

    def test_new_packet_id_flushes_incomplete_previous_frame(self):
        first = self._payload(40)
        self._feed(_chunk_datagrams(first, 1, self.CHUNK), drop={3})
        self.assertEqual(self.received, [])
        self._feed(_chunk_datagrams(self._payload(8), 2, self.CHUNK))
        self.assertEqual(len(self.received), 2)

    def test_lossless_reassembly_preserves_is_dense(self):
        # is_dense is the byte after the full reported width. Switching to
        # partial reassembly must not lose it on the normal path.
        payload = self._payload(40)
        self._feed(_chunk_datagrams(payload, 1, self.CHUNK))
        self.assertIs(self.received[0]["is_dense"], True)

    def test_chunk_loss_reports_fraction_of_missing_chunks(self):
        payload = self._payload(80)
        dgrams = _chunk_datagrams(payload, 1, self.CHUNK)
        total = len(dgrams)
        self._feed(dgrams, drop={4})
        self.receiver._handle_datagram(_chunk_datagrams(self._payload(1), 2, self.CHUNK)[0])
        self.assertAlmostEqual(self.received[0]["chunk_loss"], 1.0 / total, places=6)

    def test_reassembled_points_are_a_fresh_writable_array(self):
        # Finding 5: np.concatenate(blocks).copy() was a redundant ~9.6MB copy
        # per frame -- concatenate already returns a fresh, owned array. If a
        # future edit turns this back into a view over the chunk buffers
        # (e.g. a single-block fast path that skips concatenate), this must
        # catch it: a view over np.frombuffer output is read-only.
        payload = self._payload(40)
        self._feed(_chunk_datagrams(payload, 1, self.CHUNK))
        pts = self.received[0]["points"]
        self.assertTrue(pts.flags.writeable)
        self.assertTrue(pts.flags.owndata)
        pts[0, 0] = 12345.0             # must not raise ValueError: read-only
        self.assertEqual(pts[0, 0], 12345.0)

    def test_zero_block_fallback_yields_a_fresh_writable_empty_array(self):
        # Finding 5: the zero-block fallback (no point data in any run, only
        # the header) must still hand back a fresh, owned, writable (0, 4)
        # array -- not a stale reference or a read-only view.
        header = struct.pack(self.receiver._header_fmt, b"base_link", 1, 5, False, 16, 16)
        packet = self.receiver._parse_runs([(0, header)], received=1, total=1)
        self.assertIsNotNone(packet)
        pts = packet["points"]
        self.assertEqual(pts.shape, (0, 4))
        self.assertEqual(pts.dtype, np.float32)
        self.assertTrue(pts.flags.writeable)
        self.assertTrue(pts.flags.owndata)


class LidarReceiverLimitsTests(unittest.TestCase):
    def test_default_max_points_covers_600k_requirement(self):
        self.assertGreaterEqual(lidar_receiver._DEFAULT_MAX_POINTS, 600_000)
        self.assertEqual(_make_receiver.__defaults__[0], 300_000)  # helper default unchanged

    def test_receive_buffer_is_large_enough_for_one_600k_frame(self):
        # A 600k frame is 9.6MB; the buffer must absorb what arrives while a
        # render blocks the loop.
        self.assertGreaterEqual(lidar_receiver._RCVBUF_BYTES, 32 * 1024 * 1024)

    def test_assembly_timeout_is_shorter_than_a_10fps_frame_budget(self):
        self.assertLessEqual(lidar_receiver._ASSEMBLY_TIMEOUT, 1.0)

    def test_max_points_is_configurable_per_receiver(self):
        self.assertEqual(_make_receiver(max_points=1234).max_points, 1234)


def _pack_strided(receiver, points, point_step, width=None):
    """Build a payload of point_step-byte records; bytes past 16 stand in for ring/time."""
    n = len(points)
    w = n if width is None else width
    header = struct.pack(
        receiver._header_fmt, b"LiDAR_1", 1, w, False, point_step, w * point_step
    )
    body = b"".join(
        struct.pack("<4f", *pt) + bytes(range(1, point_step - 15))[: point_step - 16]
        for pt in points
    )
    return header + body + struct.pack("<?", True)


class PointStepTests(unittest.TestCase):
    """MORAI sends 22 bytes: x, y, z, intensity, ring(u16), time.

    Parsing at a fixed 16 misaligns every field from the second point on and
    turns the coordinates into garbage. A live capture confirmed point_step=22
    and row_step = width * 22.
    """

    PTS = [(1.0, 2.0, 3.0, 0.5), (4.0, 5.0, 6.0, 0.25), (7.0, 8.0, 9.0, 0.75)]

    def test_point_step_22_decodes_exact_coordinates(self):
        receiver = _make_receiver()
        packet = receiver._parse_payload(_pack_strided(receiver, self.PTS, 22))
        self.assertIsNotNone(packet)
        self.assertEqual(packet["point_step"], 22)
        np.testing.assert_allclose(packet["points"], np.array(self.PTS, np.float32))

    def test_point_step_22_survives_chunked_reassembly(self):
        receiver = _make_receiver()
        received = []
        receiver.on_packet = received.append
        payload = _pack_strided(receiver, self.PTS, 22)
        for dgram in _chunk_datagrams(payload, 1, 40):
            receiver._handle_datagram(dgram)
        self.assertEqual(len(received), 1)
        np.testing.assert_allclose(received[0]["points"], np.array(self.PTS, np.float32))

    def test_point_step_16_still_decodes(self):
        receiver = _make_receiver()
        packet = receiver._parse_payload(_pack_strided(receiver, self.PTS, 16))
        np.testing.assert_allclose(packet["points"], np.array(self.PTS, np.float32))

    def test_implausible_point_step_falls_back_to_16(self):
        receiver = _make_receiver()
        self.assertEqual(receiver._point_stride(0), 16)
        self.assertEqual(receiver._point_stride(4), 16)
        self.assertEqual(receiver._point_stride(10 ** 9), 16)
        self.assertEqual(receiver._point_stride(22), 22)

    def test_short_data_guard_uses_the_real_stride(self):
        receiver = _make_receiver()
        payload = _pack_strided(receiver, self.PTS, 22, width=len(self.PTS) + 5)
        self.assertIsNone(receiver._parse_payload(payload))


if __name__ == "__main__":
    unittest.main()
