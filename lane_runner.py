# lane_runner.py
#
# LaneController + CameraReceiver 래퍼 클래스
# app.py GUI와 연동하여 차선 자율주행을 실행합니다.
#
# 사용:
#   runner = LaneRunner(tcp_sock=sock, entity_id="Car_1", ...)
#   runner.start()   # 논블로킹
#   runner.stop()    # 정지 명령 전송 후 종료

from __future__ import annotations

import itertools
import socket

import transport.tcp_transport as tcp
from lane_control.lane_controller import LaneController
from receivers.camera_receiver import CameraReceiver

_rid_iter = itertools.count(500_000)

def _next_rid() -> int:
    return next(_rid_iter)


class LaneRunner:
    """
    LaneController + CameraReceiver 를 함께 관리하는 래퍼.

    Parameters
    ----------
    tcp_sock    : 연결된 TCP 소켓
    entity_id   : 제어 대상 엔티티 ID
    cam_ip      : 카메라 UDP 바인딩 IP (기본: "0.0.0.0")
    cam_port    : 카메라 UDP 수신 포트 (기본: 9090)
    vi_ip       : Vehicle Info UDP 바인딩 IP (기본: "0.0.0.0")
    vi_port     : Vehicle Info UDP 수신 포트 (기본: 9091)
    speed_ctrl  : True → 속도 PI 제어, False → 고정 스로틀
    target_kmh  : 목표 속도 km/h (speed_ctrl=True 시 사용)
    throttle    : 고정 스로틀 (speed_ctrl=False 시 사용)
    log_fn      : 로그 콜백 fn(msg, level="INFO") — None 이면 print
    """

    def __init__(
        self,
        tcp_sock:   socket.socket,
        entity_id:  str   = "Car_1",
        cam_ip:     str   = "0.0.0.0",
        cam_port:   int   = 9090,
        vi_ip:      str   = "0.0.0.0",
        vi_port:    int   = 9091,
        speed_ctrl:   bool  = True,
        target_kmh:   float = 15.0,
        throttle:     float = 0.3,
        invert_steer: bool  = True,
        log_fn=None,
        frame_cb=None,   # fn(frame: np.ndarray BGR)          — 원본 카메라 프레임 콜백
        vi_cb=None,      # fn(parsed: dict)                  — Vehicle Info 파싱 콜백
        debug_cb=None,   # fn(composite: np.ndarray BGR 1280×480) — 디버그 합성 이미지 콜백
    ):
        self._tcp_sock  = tcp_sock
        self._entity_id = entity_id
        self._log       = log_fn or (lambda msg, level="INFO": print(f"[LC] {msg}"))

        self._controller = LaneController(
            tcp_sock     = tcp_sock,
            entity_id    = entity_id,
            throttle     = throttle,
            show         = False,
            tuning       = False,
            speed_ctrl   = speed_ctrl,
            vi_ip        = vi_ip,
            vi_port      = vi_port,
            target_kmh   = target_kmh,
            invert_steer = invert_steer,
            log_fn       = self._log,
            vi_data_cb   = vi_cb,
            debug_cb     = debug_cb,
        )

        # 카메라 on_frame: 제어 루프 + 표시 콜백 동시 호출
        _ctrl_on_frame = self._controller.on_frame
        def _on_frame(frame, _ctrl=_ctrl_on_frame, _fcb=frame_cb):
            _ctrl(frame)
            if _fcb:
                try:
                    _fcb(frame)
                except Exception:
                    pass

        self._receiver = CameraReceiver(
            ip       = cam_ip,
            port     = cam_port,
            on_frame = _on_frame,
            show     = False,
        )
        self._log(
            f"초기화 완료 — Camera={cam_ip}:{cam_port}, VI={vi_ip}:{vi_port}, "
            f"entity={entity_id}, "
            f"{'target=' + str(target_kmh) + 'km/h' if speed_ctrl else 'throttle=' + str(throttle)}"
        )

    def start(self) -> None:
        """카메라 수신기 + 제어 루프를 데몬 스레드로 시작 (논블로킹)."""
        self._receiver.start()
        self._controller.start()
        self._log("Lane Control 시작")

    def update_params(self, **kwargs) -> None:
        """GUI 슬라이더 값 변경 → LaneController 실시간 반영."""
        self._controller.update_params(**kwargs)

    def stop(self) -> None:
        """제어 루프 정지 → 정지 명령 전송 → 수신기 정지."""
        self._controller.stop()
        self._receiver.stop()
        try:
            tcp.send_manual_control_by_id(
                self._tcp_sock, _next_rid(), self._entity_id,
                throttle=0.0, brake=1.0, steer_angle=0.0,
            )
            self._log("정지 명령 전송 (brake=1.0)")
        except OSError:
            pass
        self._log("Lane Control 종료")
