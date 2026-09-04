from __future__ import annotations

import json
import os
import select
import socket
import struct
import threading
import time
from typing import Callable, Dict, Optional, Tuple

import numpy as np

from utils.template_paths import resolve_template_path

_HEADER_FMT = "<IHH"
_HEADER_SIZE = struct.calcsize(_HEADER_FMT)
_RECV_BUF = 65535
_RCVBUF_BYTES = 32 * 1024 * 1024   # A 600k frame is 9.6MB; headroom for what arrives mid-render.
_ASSEMBLY_TIMEOUT = 0.5            # Backstop only; a packet_id change drives the normal flush.

_POINT_FIELDS = 4  # x, y, z, intensity
_XYZI_SIZE = 16  # Size of the four leading fields we read (4 x float32).
# The real record size comes from the header's point_step. MORAI sends a
# Velodyne-style layout - x, y, z, intensity, ring(u16), time - which is 22
# bytes. Reading at a fixed 16 misaligns every field from the second point on
# and turns the coordinates into garbage.
_MAX_POINT_STEP = 1024

_DEFAULT_MAX_POINTS = 600_000

MAX_REPORTED_POINTS = 50_000_000  # width upper clamp; above this, offset math can overflow.


class _AssemblyState:
    def __init__(self):
        self.packet_id: Optional[int] = None
        self.total_chunks: int = 0
        self.chunks: Dict[int, bytes] = {}
        self.started_at: float = 0.0

    def reset(self) -> None:
        self.packet_id = None
        self.total_chunks = 0
        self.chunks.clear()
        self.started_at = 0.0


class LidarReceiver(threading.Thread):
    def __init__(
        self,
        ip: str = "0.0.0.0",
        port: int = 9200,
        on_packet: Optional[Callable[[dict], None]] = None,
        tmpl_path: Optional[str] = None,
        max_points: int = _DEFAULT_MAX_POINTS,
    ):
        super().__init__(daemon=True)
        self.ip = ip
        self.port = port
        self.on_packet = on_packet
        self.max_points = max_points
        self.running = False

        self._asm = _AssemblyState()
        self._lock = threading.Lock()
        self._fps_ts = time.time()
        self._frame_count = 0
        self.fps = 0.0
        self.last_packet: Optional[dict] = None
        self._debug_last: Dict[str, float] = {}
        self._packet_seq = 0

        self._tmpl_path = tmpl_path or resolve_template_path("LiDAR PointCloud.tmpl")
        if self._tmpl_path is None:
            raise FileNotFoundError("LiDAR PointCloud.tmpl")
        self._frame_id_len, self._header_fmt, self._header_size = self._load_header_layout()
        self._debug(
            f"template={os.path.basename(self._tmpl_path)} frame_id_len={self._frame_id_len} "
            f"header_size={self._header_size}",
            key="init",
            interval_sec=0.0,
        )

    def stop(self) -> None:
        self.running = False

    def get_latest_packet(self) -> Optional[dict]:
        with self._lock:
            return self.last_packet

    def run(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, _RCVBUF_BYTES)
        except OSError as e:
            # If the platform refuses, carry on with the default buffer: more
            # loss, but still working.
            print(f"[LidarReceiver] SO_RCVBUF {_RCVBUF_BYTES} failed, using default: {e}")
        sock.bind((self.ip, self.port))
        sock.setblocking(False)

        self.running = True
        print(f"[LidarReceiver] Listening on {self.ip}:{self.port}")

        try:
            while self.running:
                readable, _, _ = select.select([sock], [], [], 0.5)
                if not readable:
                    continue

                while True:
                    try:
                        data, _addr = sock.recvfrom(_RECV_BUF)
                    except BlockingIOError:
                        break
                    except OSError as e:
                        if self.running:
                            print(f"[LidarReceiver] recv error: {e}")
                        break
                    self._handle_datagram(data)
        finally:
            sock.close()
            print(f"[LidarReceiver] Stopped ({self.ip}:{self.port})")

    def _load_header_layout(self) -> Tuple[int, str, int]:
        with open(self._tmpl_path, "r", encoding="utf-8") as fp:
            raw = json.load(fp)

        segs = raw.get("messageTemplate", {}).get("segmentList", [])
        fields_seg = next(
            (s for s in segs if str(s.get("type", "")).upper() == "FIELDS"),
            None,
        )
        if fields_seg is None:
            raise ValueError("LiDAR PointCloud.tmpl has no FIELDS segment")

        frame_id_field = next(
            (
                f
                for f in fields_seg.get("fieldList", [])
                if f.get("variableName") == "frame_id" or f.get("name") == "frame_id"
            ),
            None,
        )
        if frame_id_field is None:
            raise ValueError("LiDAR PointCloud.tmpl has no frame_id field")

        frame_id_len = int(frame_id_field.get("length", 0))
        if frame_id_len <= 0:
            raise ValueError("LiDAR PointCloud.tmpl frame_id field has invalid length")

        # frame_id(F) + height(u32) + width(u32) + is_bigendian(bool) + point_step(u32) + row_step(u32)
        header_fmt = f"<{frame_id_len}sII?II"
        header_size = struct.calcsize(header_fmt)
        return frame_id_len, header_fmt, header_size

    def _handle_datagram(self, data: bytes) -> None:
        if self._is_chunked(data):
            self._handle_chunked(data)
        else:
            self._deliver(data)

    @staticmethod
    def _is_chunked(data: bytes) -> bool:
        if len(data) < _HEADER_SIZE:
            return False
        try:
            packet_id, chunk_idx, total = struct.unpack(_HEADER_FMT, data[:_HEADER_SIZE])
        except struct.error:
            return False
        return packet_id != 0 and 0 < total <= 10000 and chunk_idx < total

    def _handle_chunked(self, data: bytes) -> None:
        packet_id, chunk_idx, total = struct.unpack(_HEADER_FMT, data[:_HEADER_SIZE])
        payload = data[_HEADER_SIZE:]
        asm = self._asm

        if asm.packet_id != packet_id:
            # A new frame starting is the signal that the previous one ended.
            # Deliver whatever was collected instead of waiting for a timeout.
            self._flush_partial()
            asm.reset()
            asm.packet_id = packet_id
            asm.total_chunks = total
            asm.started_at = time.time()
        elif time.time() - asm.started_at > _ASSEMBLY_TIMEOUT:
            self._flush_partial()
            asm.reset()
            return

        asm.chunks[chunk_idx] = payload
        if len(asm.chunks) < asm.total_chunks:
            return

        runs = self._chunk_runs()
        received, total_chunks = len(asm.chunks), asm.total_chunks
        asm.reset()
        packet = self._parse_runs(runs, received, total_chunks)
        if packet is not None:
            self._publish(packet)

    def _flush_partial(self) -> None:
        """Deliver an incomplete frame from the chunks that did arrive."""
        asm = self._asm
        if not asm.chunks or asm.total_chunks <= 0:
            return
        runs = self._chunk_runs()
        packet = self._parse_runs(runs, len(asm.chunks), asm.total_chunks)
        if packet is not None:
            self._publish(packet)

    def _chunk_runs(self):
        """Group received chunks into (start byte offset, data) runs.

        The protocol header carries no offset field, so chunk i's position is
        only derivable when every chunk shares a size. When sizes disagree,
        use only the prefix in which no chunk is missing and every chunk
        matches size0 (or is genuinely the last one). Concatenating past that
        point shifts the remaining bytes by a multiple of the record size, so
        they land on point boundaries and silently return the wrong points -
        for example point 7's values in point 4's slot - with no error.
        """
        asm = self._asm
        if 0 not in asm.chunks:
            return []                       # No header chunk, nothing can be parsed.
        size0 = len(asm.chunks[0])
        if size0 <= 0:
            return []
        indices = sorted(asm.chunks)
        last_idx = asm.total_chunks - 1
        uniform = all(len(asm.chunks[i]) == size0 for i in indices if i != last_idx)
        if not uniform:
            end = 0
            while end in asm.chunks and (
                end == last_idx or len(asm.chunks[end]) == size0
            ):
                end += 1
            indices = list(range(end))

        runs = []
        start = indices[0]
        buf = [asm.chunks[start]]
        for prev, cur in zip(indices, indices[1:]):
            if cur == prev + 1:
                buf.append(asm.chunks[cur])
            else:
                runs.append((start * size0, b"".join(buf)))
                start, buf = cur, [asm.chunks[cur]]
        runs.append((start * size0, b"".join(buf)))
        return runs

    def _parse_runs(self, runs, received: int, total: int) -> Optional[dict]:
        """Recover only the whole points contained in the given runs."""
        if not runs or runs[0][0] != 0:
            return None
        parsed = self._parse_header(runs[0][1])
        if parsed is None:
            return None
        frame_id, height, width, is_bigendian, point_step, row_step = parsed
        if width < 0 or width > MAX_REPORTED_POINTS:
            self._debug(f"implausible width: {width}", key="bad_width", interval_sec=1.0)
            return None

        n_effective = min(width, self.max_points)
        stride = self._point_stride(point_step)
        data_start = self._header_size
        blocks = []
        for offset, blob in runs:
            lo = max(offset, data_start)
            hi = offset + len(blob)
            first = -(-(lo - data_start) // stride)               # ceil
            last = min((hi - data_start) // stride, n_effective)  # floor
            count = last - first
            if count <= 0:
                continue
            byte_offset = data_start + first * stride - offset
            blocks.append(self._extract_xyzi(blob, byte_offset, count, stride))
        # np.concatenate already allocates a fresh, owned, writable array, so a
        # trailing .copy() here was a redundant ~9.6MB copy per frame on the
        # receive thread. The zero-block fallback also yields a fresh array.
        points = (np.concatenate(blocks) if blocks
                  else np.zeros((0, _POINT_FIELDS), dtype=np.float32))

        # is_dense sits after the FULL reported width. Read it if some run
        # covers that byte; on a partial frame it may be absent and stay None.
        is_dense = None
        dense_at = data_start + width * stride
        for offset, blob in runs:
            if offset <= dense_at < offset + len(blob):
                (raw,) = struct.unpack_from("<?", blob, dense_at - offset)
                is_dense = bool(raw)
                break

        loss = 0.0 if total <= 0 else 1.0 - (received / float(total))
        return {
            "points": points,
            "frame_id": frame_id,
            "height": height,
            "width": width,
            "point_count": int(points.shape[0]),
            "recovered_points": int(points.shape[0]),
            "chunk_loss": max(0.0, loss),
            "points_truncated": width > self.max_points,
            "is_bigendian": is_bigendian,
            "point_step": point_step,
            "row_step": row_step,
            "is_dense": is_dense,
            "raw_size": sum(len(blob) for _, blob in runs),
        }

    def _deliver(self, payload: bytes) -> None:
        packet = self._parse_payload(payload)
        if packet is None:
            return
        packet.setdefault("chunk_loss", 0.0)
        packet.setdefault("recovered_points", packet["point_count"])
        self._publish(packet)

    def _publish(self, packet: dict) -> None:
        self._packet_seq += 1
        packet["packet_seq"] = self._packet_seq
        packet["timestamp"] = time.time()

        with self._lock:
            self.last_packet = packet

        self._frame_count += 1
        now = time.time()
        elapsed = now - self._fps_ts
        if elapsed >= 1.0:
            self.fps = self._frame_count / elapsed
            self._frame_count = 0
            self._fps_ts = now
        packet["fps"] = self.fps
        self._debug(
            f"packet#{self._packet_seq} frame_id={packet['frame_id']!r} "
            f"width={packet['width']} points={packet['point_count']} "
            f"loss={packet['chunk_loss']:.3f}",
            key="parsed",
            interval_sec=1.0,
        )

        if self.on_packet is not None:
            try:
                self.on_packet(packet)
            except Exception as e:
                print(f"[LidarReceiver] on_packet error: {e}")

    def _point_stride(self, point_step: int) -> int:
        """Record stride. Falls back to the minimum if the header value is unusable."""
        if _XYZI_SIZE <= point_step <= _MAX_POINT_STEP:
            return point_step
        self._debug(
            f"implausible point_step={point_step}, falling back to {_XYZI_SIZE}",
            key="bad_step",
            interval_sec=2.0,
        )
        return _XYZI_SIZE

    @staticmethod
    def _extract_xyzi(buf: bytes, byte_offset: int, count: int, stride: int) -> np.ndarray:
        """Pull the four leading float32 fields out of count records of stride bytes."""
        if count <= 0:
            return np.zeros((0, _POINT_FIELDS), dtype=np.float32)
        if stride == _XYZI_SIZE:
            return np.frombuffer(
                buf, dtype="<f4", count=count * _POINT_FIELDS, offset=byte_offset
            ).reshape(count, _POINT_FIELDS)
        raw = np.frombuffer(
            buf, dtype=np.uint8, count=count * stride, offset=byte_offset
        ).reshape(count, stride)
        return np.ascontiguousarray(raw[:, :_XYZI_SIZE]).view("<f4").reshape(
            count, _POINT_FIELDS
        )

    def _parse_header(self, head: bytes):
        """Extract header fields from the front of a payload. Returns None on failure."""
        if len(head) < self._header_size:
            self._debug(
                f"payload too short for header: {len(head)} < {self._header_size}",
                key="short_hdr",
                interval_sec=1.0,
            )
            return None
        (frame_id_raw, height, width, is_bigendian,
         point_step, row_step) = struct.unpack_from(self._header_fmt, head, 0)
        frame_id = frame_id_raw.split(b"\x00", 1)[0].decode("utf-8", errors="replace")
        return frame_id, int(height), int(width), bool(is_bigendian), int(point_step), int(row_step)

    def _parse_payload(self, payload: bytes) -> Optional[dict]:
        parsed = self._parse_header(payload)
        if parsed is None:
            return None
        frame_id, height, width, is_bigendian, point_step, row_step = parsed

        n_reported = width
        if n_reported < 0 or n_reported > MAX_REPORTED_POINTS:
            self._debug(f"implausible width: {n_reported}", key="bad_width", interval_sec=1.0)
            return None

        truncated = n_reported > self.max_points
        n_effective = min(n_reported, self.max_points)

        stride = self._point_stride(point_step)
        data_start = self._header_size
        data_end_effective = data_start + n_effective * stride
        if len(payload) < data_end_effective:
            self._debug(
                f"payload shorter than declared width: width={n_reported} "
                f"need={data_end_effective} have={len(payload)}",
                key="short_data",
                interval_sec=1.0,
            )
            return None

        points = self._extract_xyzi(payload, data_start, n_effective, stride)
        if stride == _XYZI_SIZE and points.size:
            points = points.copy()   # frombuffer returns a view into payload

        if truncated:
            self._debug(
                f"width={n_reported} exceeds max_points={self.max_points}, truncated for display",
                key="cap",
                interval_sec=2.0,
            )

        # is_dense follows the FULL reported point array on the wire, not the
        # (possibly capped) effective slice we actually decoded.
        is_dense_offset = data_start + n_reported * stride
        is_dense = None
        if len(payload) >= is_dense_offset + 1:
            (is_dense_raw,) = struct.unpack_from("<?", payload, is_dense_offset)
            is_dense = bool(is_dense_raw)

        return {
            "points": points,
            "frame_id": frame_id,
            "height": height,
            "width": n_reported,
            "point_count": int(points.shape[0]),
            "points_truncated": truncated,
            "is_bigendian": is_bigendian,
            "point_step": point_step,
            "row_step": row_step,
            "is_dense": is_dense,
            "raw_size": len(payload),
        }

    def _debug(self, msg: str, key: str, interval_sec: float) -> None:
        now = time.monotonic()
        last = self._debug_last.get(key, 0.0)
        if interval_sec > 0.0 and now - last < interval_sec:
            return
        self._debug_last[key] = now
        print(f"[LidarReceiver][DBG] {msg}")
