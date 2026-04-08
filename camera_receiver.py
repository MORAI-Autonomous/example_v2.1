# camera_receiver.py
#
# UDP 카메라 이미지 수신기
# - MORAI 시뮬레이터의 Dynamic Camera 패킷 포맷 지원
#   ① Chunked  : [PacketID:4B][ChunkIdx:2B][TotalChunks:2B] + payload
#                조립 완료 후 → [size:4B][JPEG bytes]
#   ② Headerless: [size:4B][JPEG bytes]
# - on_frame(numpy_bgr) 콜백으로 프레임 전달 → 제어 파이프라인 연결용
# - show=True 시 OpenCV 창으로 실시간 확인 가능

import socket
import select
import struct
import threading
import time
from typing import Callable, Optional

import numpy as np
import cv2

# ─── 패킷 상수 ──────────────────────────────────────────────────
_HEADER_FMT  = "<IHH"   # PacketID(4), ChunkIdx(2), TotalChunks(2)
_HEADER_SIZE = struct.calcsize(_HEADER_FMT)  # 8 bytes
_RECV_BUF    = 65535
_ASSEMBLY_TIMEOUT = 5.0  # 청크 조립 대기 최대 시간 (초)


# ─── 청크 조립 상태 ──────────────────────────────────────────────
class _AssemblyState:
    def __init__(self):
        self.packet_id: Optional[int] = None
        self.total_chunks: int = 0
        self.chunks: dict = {}
        self.started_at: float = 0.0

    def reset(self):
        self.packet_id   = None
        self.total_chunks = 0
        self.chunks.clear()
        self.started_at  = 0.0


# ─── CameraReceiver ─────────────────────────────────────────────
class CameraReceiver(threading.Thread):
    """
    UDP 카메라 이미지 수신 스레드

    Parameters
    ----------
    ip       : 바인딩할 IP (기본 "0.0.0.0" → 모든 인터페이스)
    port     : 수신 포트
    on_frame : 프레임 콜백 fn(frame: np.ndarray) — BGR uint8
               None 이면 콜백 없이 show 전용으로만 동작
    show     : True 시 OpenCV 창에 실시간 렌더링
    window_name : OpenCV 창 이름 (None 이면 자동 생성)
    """

    def __init__(
        self,
        ip: str = "127.0.0.1",
        port: int = 9090,
        on_frame: Optional[Callable[[np.ndarray], None]] = None,
        show: bool = True,
        window_name: Optional[str] = None,
    ):
        super().__init__(daemon=True)
        self.ip          = ip
        self.port        = port
        self.on_frame    = on_frame
        self.show        = show
        self.window_name = window_name or f"Camera [{ip}:{port}]"
        self.running     = False

        # 통계
        self._frame_count = 0
        self._fps_ts      = time.time()
        self.fps          = 0.0          # 외부에서 읽을 수 있는 최신 FPS
        self.last_frame: Optional[np.ndarray] = None  # 최신 프레임 (외부 참조용)
        self._lock = threading.Lock()

        self._asm = _AssemblyState()

    # ── 공개 API ─────────────────────────────────────────────────
    def stop(self):
        self.running = False

    def get_latest_frame(self) -> Optional[np.ndarray]:
        """최신 프레임을 스레드 안전하게 반환 (없으면 None)"""
        with self._lock:
            return self.last_frame.copy() if self.last_frame is not None else None

    # ── 내부 ─────────────────────────────────────────────────────
    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        except OSError:
            pass
        sock.bind((self.ip, self.port))
        sock.setblocking(False)

        self.running = True
        print(f"[CameraReceiver] Listening on {self.ip}:{self.port}")

        try:
            while self.running:
                readable, _, _ = select.select([sock], [], [], 0.5)
                if not readable:
                    if self.show:
                        cv2.waitKey(1)
                    continue

                try:
                    while True:
                        try:
                            data, addr = sock.recvfrom(_RECV_BUF)
                        except BlockingIOError:
                            break
                        self._handle(data)
                except Exception as e:
                    if self.running:
                        print(f"[CameraReceiver] recv error: {e}")

                if self.show:
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord("q"), 27):   # q / ESC → 종료
                        self.running = False
                        break
        finally:
            sock.close()
            if self.show:
                cv2.destroyWindow(self.window_name)
            print(f"[CameraReceiver] Stopped ({self.ip}:{self.port})")

    # ── 패킷 처리 ────────────────────────────────────────────────
    def _handle(self, data: bytes):
        if self._is_chunked(data):
            self._handle_chunked(data)
        else:
            self._handle_headerless(data)

    @staticmethod
    def _is_chunked(data: bytes) -> bool:
        if len(data) < _HEADER_SIZE:
            return False
        try:
            pid, cidx, total = struct.unpack(_HEADER_FMT, data[:_HEADER_SIZE])
        except struct.error:
            return False
        return pid != 0 and 0 < total <= 10000 and cidx < total

    def _handle_headerless(self, data: bytes):
        """[uint32 size][JPEG bytes]"""
        if len(data) < 4:
            return
        (img_size,) = struct.unpack("<I", data[:4])
        if img_size == 0 or len(data) < 4 + img_size:
            return
        self._deliver(data[4: 4 + img_size])

    def _handle_chunked(self, data: bytes):
        """청크 조립 후 [uint32 size][JPEG bytes] 로 전달"""
        pid, cidx, total = struct.unpack(_HEADER_FMT, data[:_HEADER_SIZE])
        payload = data[_HEADER_SIZE:]
        asm = self._asm

        # 새 패킷 시작
        if asm.packet_id != pid:
            asm.reset()
            asm.packet_id    = pid
            asm.total_chunks = total
            asm.started_at   = time.time()

        # 타임아웃 체크
        if time.time() - asm.started_at > _ASSEMBLY_TIMEOUT:
            asm.reset()
            return

        asm.chunks[cidx] = payload

        # 조립 완료?
        if len(asm.chunks) < asm.total_chunks:
            return

        try:
            full = b"".join(asm.chunks[i] for i in range(asm.total_chunks))
        except KeyError:
            asm.reset()
            return

        asm.reset()

        if len(full) < 4:
            return
        (img_size,) = struct.unpack("<I", full[:4])
        if img_size == 0 or len(full) < 4 + img_size:
            return
        self._deliver(full[4: 4 + img_size])

    def _deliver(self, img_bytes: bytes):
        """디코딩 → 콜백 + 표시 + 통계"""
        np_buf = np.frombuffer(img_bytes, dtype=np.uint8)
        frame  = cv2.imdecode(np_buf, cv2.IMREAD_COLOR)
        if frame is None:
            return

        # 최신 프레임 저장
        with self._lock:
            self.last_frame = frame

        # FPS 계산
        self._frame_count += 1
        now = time.time()
        elapsed = now - self._fps_ts
        if elapsed >= 1.0:
            self.fps      = self._frame_count / elapsed
            self._frame_count = 0
            self._fps_ts  = now

        # OpenCV 창 렌더링
        if self.show:
            title = f"{self.window_name}  FPS: {self.fps:.1f}"
            cv2.imshow(self.window_name, frame)
            cv2.setWindowTitle(self.window_name, title)

        # 콜백
        if self.on_frame is not None:
            try:
                self.on_frame(frame)
            except Exception as e:
                print(f"[CameraReceiver] on_frame error: {e}")


# ─── 단독 실행 (수신 확인용) ─────────────────────────────────────
def main():
    import argparse
    parser = argparse.ArgumentParser(description="Camera UDP Receiver — 수신 확인용")
    parser.add_argument("--ip",   default="127.0.0.1",  help="바인딩 IP (기본: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=9090, help="수신 포트 (기본: 9090)")
    args = parser.parse_args()

    receiver = CameraReceiver(ip=args.ip, port=args.port, show=True)
    receiver.start()

    print("수신 중... 창에서 q 또는 ESC 키로 종료")
    try:
        while receiver.is_alive():
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        receiver.stop()
        receiver.join(timeout=2.0)
        print("종료")


if __name__ == "__main__":
    main()
