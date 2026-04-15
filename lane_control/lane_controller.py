# lane_controller.py
# Camera → LanePreprocessor → LaneDetector → EMA → PD → ManualControlById TCP
# Vehicle Info UDP → Speed PI → 스로틀 자동 제어
from __future__ import annotations

import math
import socket
import time
import threading
import argparse
import itertools

import cv2
import numpy as np

import transport.tcp_transport as tcp
import transport.protocol_defs as proto
from receivers.camera_receiver import CameraReceiver
from lane_control.lane_preprocessor import LanePreprocessor
from lane_control.lane_detector import LaneDetector
from lane_control.controllers   import EMAFilter, PDController, SpeedPIController
from lane_control.vehicle_info  import VehicleInfoThread
from lane_control.tune_panel    import TunePanel


_rid_iter = itertools.count(1)

def _next_rid() -> int:
    return next(_rid_iter)



class LaneController:
    """카메라 프레임 → 차선 검출 → PD 제어 → TCP ManualControlById 전송"""

    def __init__(
        self,
        tcp_sock:      socket.socket,
        entity_id:     str   = "Car_1",
        throttle:      float = 0.3,       # 고정 스로틀 (speed_ctrl=False 시 사용)
        kp:            float = 0.5,
        kd:            float = 0.1,
        ema_alpha:     float = 0.3,
        steer_max:     float = 1.0,
        show:          bool  = True,
        ctrl_hz:       float = 20.0,
        invert_steer:  bool  = True,
        min_det_go:    int   = 5,
        # 속도 피드백
        speed_ctrl:    bool  = True,
        vi_ip:         str   = "127.0.0.1",
        vi_port:       int   = 9091,
        target_kmh:    float = 30.0,
        kp_spd:        float = 0.05,
        ki_spd:        float = 0.01,
        throttle_max:  float = 0.8,
        # 실시간 튜닝
        tuning:        bool  = False,
        # 녹화
        record_path:   str | None = None,
        # 로그 콜백
        log_fn = None,
        # Vehicle Info 파싱 콜백 fn(parsed: dict)
        vi_data_cb = None,
        # 디버그 합성 이미지 콜백 fn(frame: np.ndarray BGR 1280×480)
        debug_cb = None,
    ):
        self._sock         = tcp_sock
        self._log          = log_fn or (lambda msg, level="INFO": print(f"[LC] {msg}"))
        self._entity_id    = entity_id
        self._throttle     = throttle
        self._show         = show
        self._period       = 1.0 / ctrl_hz
        self._invert_steer = invert_steer
        self._min_det_go   = min_det_go
        self._speed_ctrl   = speed_ctrl

        self._preprocessor = LanePreprocessor()
        self._detector     = LaneDetector()
        self._ema          = EMAFilter(alpha=ema_alpha)
        self._pd           = PDController(kp=kp, kd=kd, steer_max=steer_max)

        # 속도 피드백
        self._vi_thread: VehicleInfoThread | None = None
        self._speed_pi:  SpeedPIController  | None = None
        if speed_ctrl:
            self._vi_thread = VehicleInfoThread(
                ip=vi_ip, port=vi_port,
                log_fn=self._log, data_cb=vi_data_cb,
            )
            self._speed_pi  = SpeedPIController(
                target_kmh   = target_kmh,
                kp           = kp_spd,
                ki           = ki_spd,
                throttle_max = throttle_max,
            )

        self._running     = False
        self._lock        = threading.Lock()
        self._latest_frame: np.ndarray | None = None

        self._last_steer:   float = 0.0
        self._no_det_cnt:   int   = 0
        self._no_valid_cnt: int   = 0   # BAD_W + NO_DET 합산
        self._det_streak:   int   = 0
        self._ready:        bool  = False
        self._OFFSET_CLIP:  float = 1.5
        self._STEER_RATE:   float = 0.15
        self._tuning       = tuning
        self._tune_panel:  TunePanel | None = None
        self._record_path  = record_path
        self._writer:      cv2.VideoWriter | None = None
        self._debug_cb     = debug_cb

    # ── 카메라 콜백 ──────────────────────────────────────────────
    def on_frame(self, frame: np.ndarray):
        """CameraReceiver.on_frame 콜백 — 최신 프레임 저장"""
        with self._lock:
            self._latest_frame = frame

    # ── 실시간 파라미터 업데이트 ────────────────────────────────
    def update_params(self, **kwargs) -> None:
        """GUI 슬라이더 콜백에서 실시간으로 파라미터를 변경한다."""
        if 'kp'           in kwargs: self._pd.kp          = float(kwargs['kp'])
        if 'kd'           in kwargs: self._pd.kd          = float(kwargs['kd'])
        if 'ema_alpha'    in kwargs: self._ema.alpha       = float(kwargs['ema_alpha'])
        if 'steer_rate'   in kwargs: self._STEER_RATE      = float(kwargs['steer_rate'])
        if 'offset_clip'  in kwargs: self._OFFSET_CLIP     = float(kwargs['offset_clip'])
        if 'invert_steer' in kwargs: self._invert_steer    = bool(kwargs['invert_steer'])
        if 'target_kmh'   in kwargs and self._speed_pi:
            self._speed_pi.set_target(float(kwargs['target_kmh']))
        # ── 전처리기 (BEVParams) ────────────────────────────────
        if 'bev_top_crop'  in kwargs:
            self._preprocessor.params.bev_top_crop  = int(kwargs['bev_top_crop'])
        if 'min_blob_area' in kwargs:
            self._preprocessor.params.min_blob_area = int(kwargs['min_blob_area'])
        # ── 검출기 (LaneDetector) ───────────────────────────────
        if 'search_ratio' in kwargs:
            self._detector.search_ratio = float(kwargs['search_ratio'])
        if 'min_pixels'   in kwargs:
            self._detector.min_pixels   = int(kwargs['min_pixels'])

    # ── 시작 / 종료 ──────────────────────────────────────────────
    def start(self) -> threading.Thread:
        if self._vi_thread is not None:
            self._vi_thread.start()
        if self._tuning:
            self._tune_panel = TunePanel(self)
            self._log("[Tuner] 실시간 튜닝 창 활성화 — S: 값 출력  R: 초기화")
        self._running = True
        t = threading.Thread(target=self._loop, daemon=True, name="ctrl-loop")
        t.start()
        return t

    def stop(self):
        self._running = False
        if self._vi_thread is not None:
            self._vi_thread.stop()
        if self._writer is not None:
            self._writer.release()
            self._writer = None
            self._log(f"[Record] 저장 완료: {self._record_path}")

    # ── 제어 루프 ────────────────────────────────────────────────
    def _loop(self):
        self._log(f"제어 루프 시작 ({1/self._period:.0f} Hz) "
                  f"entity={self._entity_id} throttle={self._throttle}")
        while self._running:
            t0 = time.time()

            with self._lock:
                frame = self._latest_frame.copy() if self._latest_frame is not None else None

            if frame is not None:
                self._step(frame)

            elapsed = time.time() - t0
            sleep_t = self._period - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

        self._log("제어 루프 종료")

    def _step(self, frame: np.ndarray):
        # 1. BEV 전처리
        pre    = self._preprocessor.preprocess(frame)

        # 2. Sliding Window 차선 검출
        result = self._detector.detect(pre["binary"])

        bad_offset    = math.isnan(result.offset_m)
        lane_seen     = result.left_detected or result.right_detected
        detected      = lane_seen and not bad_offset
        prev_no_valid = self._no_valid_cnt

        if lane_seen:
            self._det_streak += 1
            self._no_det_cnt  = 0
            if not self._ready:
                if self._det_streak >= self._min_det_go:
                    self._ready = True
                    self._log(f"차선 확보 완료 ({self._det_streak}회 연속) → 주행 시작")
                else:
                    self._log(f"차선 대기 중... ({self._det_streak}/{self._min_det_go})")

            if detected:
                raw_off    = float(np.clip(result.offset_m, -self._OFFSET_CLIP, self._OFFSET_CLIP))
                smooth_off = self._ema.update(raw_off)
                steer_raw  = self._pd.compute(smooth_off)

                if prev_no_valid > 0:
                    # BAD_W/NO_DET 후 복귀: 비대칭 rate limit
                    # 감소방향 0.3×, 증가방향 no_valid 비례로 확대 → 빠른 복원
                    max_inc = self._STEER_RATE * (1.0 + min(prev_no_valid * 0.04, 2.0))
                    max_dec = self._STEER_RATE * 0.3
                    steer   = float(np.clip(steer_raw,
                                            self._last_steer - max_dec,
                                            self._last_steer + max_inc))
                    status  = f"REC({prev_no_valid})"
                else:
                    steer  = float(np.clip(steer_raw,
                                           self._last_steer - self._STEER_RATE,
                                           self._last_steer + self._STEER_RATE))
                    status = "DET"

                self._last_steer   = steer
                self._no_valid_cnt = 0
            else:
                # BAD WIDTH: steer 홀드 (차선 보이므로 decay 불필요)
                smooth_off          = self._ema._val if self._ema._val is not None else 0.0
                steer               = self._last_steer
                self._last_steer    = steer
                self._no_valid_cnt += 1
                status = f"BAD_W({self._no_valid_cnt})"
        else:
            self._det_streak   = 0
            self._no_det_cnt  += 1
            self._no_valid_cnt += 1
            smooth_off = self._ema._val if self._ema._val is not None else 0.0
            # 미검출 시 완만한 감쇠 (0.985^20≈0.74, 1초 후에도 steer 70% 유지)
            decay = max(0.985 - self._no_det_cnt * 0.001, 0.975)
            steer = self._last_steer * decay
            self._last_steer = steer
            status = f"NO_DET×{self._no_det_cnt}"

        # 5. 스로틀 / 브레이크 결정
        current_mps = 0.0
        if self._speed_ctrl and self._vi_thread is not None:
            current_mps = self._vi_thread.get_speed_mps()

        if self._ready:
            if self._speed_ctrl and self._speed_pi is not None:
                throttle_cmd, brake_cmd = self._speed_pi.compute(current_mps)
            else:
                throttle_cmd = self._throttle
                brake_cmd    = 0.0
            steer_out = (-steer) if self._invert_steer else steer
        else:
            throttle_cmd = 0.0
            brake_cmd    = 0.5
            steer_out    = 0.0
            status       = f"WAIT({self._det_streak}/{self._min_det_go})"

        # 6. TCP 전송
        try:
            tcp.send_manual_control_by_id(
                self._sock,
                _next_rid(),
                self._entity_id,
                throttle    = throttle_cmd,
                brake       = brake_cmd,
                steer_angle = steer_out,
            )
        except OSError as e:
            self._log(f"TCP 오류: {e}", "ERROR")
            self._running = False
            return

        # 7. 터미널 출력 (per-frame — GUI 로그 패널에는 보내지 않음)
        side    = "L" if smooth_off > 0 else "R"
        r_str   = f"{result.curve_radius_m:.0f}m" if result.curve_radius_m < 5000 else "STRAIGHT"
        spd_str = f"{current_mps*3.6:.1f}km/h" if self._speed_ctrl else f"thr={throttle_cmd:.2f}"
        print(
            f"[{status:^12s}] "
            f"spd={spd_str}  "
            f"offset={smooth_off:+.3f}m({side})  "
            f"radius={r_str:>10s}  "
            f"steer={steer_out:+.4f}  "
            f"thr={throttle_cmd:.2f}  "
            f"lane={'L' if result.left_detected else '-'}"
                 f"{'R' if result.right_detected else '-'}"
        )

        # 8. OpenCV 시각화 / 디버그 콜백
        _need_composite = self._show or self._debug_cb or self._record_path
        if _need_composite and result.viz is not None:
            _tgt_mps = self._speed_pi.target_mps if self._speed_pi else None
            composite = self._build_debug_frame(
                pre, result.viz, steer_out, smooth_off, status, current_mps, _tgt_mps)
            if self._show:
                cv2.imshow("Lane Controller", composite)
                cv2.waitKey(1)
            if self._debug_cb:
                try:
                    self._debug_cb(composite)
                except Exception:
                    pass
            if self._record_path is not None:
                if self._writer is None:
                    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                    self._writer = cv2.VideoWriter(
                        self._record_path, fourcc, 20.0, (1280, 480))
                    self._log(f"[Record] 녹화 시작: {self._record_path}  (1280×480 @20fps)")
                self._writer.write(composite)

        # 9. 튜너 파라미터 읽기 (GUI 없음 — 메인 스레드와 분리)
        if self._tune_panel is not None:
            self._tune_panel.read_params()

    def _build_debug_frame(
        self,
        pre:        dict,
        viz:        np.ndarray,
        steer:      float,
        offset:     float,
        status:     str,
        speed_mps:  float = 0.0,
        target_mps: float | None = None,
    ) -> np.ndarray:
        """1280×480 디버그 합성 이미지: 원본(640)+BEV(320×240)+이진화(320×240)+게이지(640×240)"""
        W, H = 1280, 480
        PW   = W // 2
        RW   = W - PW
        CW   = RW // 2
        TH   = H // 2
        BH   = H - TH

        orig = pre["original"].copy()
        pts  = self._preprocessor.params.src_pts().astype(np.int32)
        cv2.polylines(orig, [pts], True, (0, 255, 0), 2)
        left_panel = cv2.resize(orig, (PW, H))

        top_left = cv2.resize(viz, (CW, TH))

        binary_color = cv2.cvtColor(pre["binary"], cv2.COLOR_GRAY2BGR)
        top_right = cv2.resize(binary_color, (CW, TH))
        cv2.putText(top_right, "Binary", (5, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 255, 100), 1)

        bot_panel = np.zeros((BH, RW, 3), dtype=np.uint8)
        _draw_steer_bar_panel(bot_panel, steer, offset, status, speed_mps, target_mps)

        top_row   = np.hstack([top_left, top_right])
        right_col = np.vstack([top_row, bot_panel])
        return np.hstack([left_panel, right_col])

    def _show_debug(
        self,
        pre:        dict,
        viz:        np.ndarray,
        steer:      float,
        offset:     float,
        status:     str,
        speed_mps:  float = 0.0,
        target_mps: float | None = None,
    ):
        """CLI --show 모드 전용 래퍼. GUI 모드에서는 debug_cb 를 사용한다."""
        combined = self._build_debug_frame(
            pre, viz, steer, offset, status, speed_mps, target_mps)
        cv2.imshow("Lane Controller", combined)
        cv2.waitKey(1)


# ─── 조향각 게이지 패널 ─────────────────────────────────────────
def _draw_steer_bar_panel(
    panel:      np.ndarray,
    steer:      float,
    offset:     float,
    status:     str,
    speed_mps:  float = 0.0,
    target_mps: float | None = None,
):
    """하단 패널 전용: 텍스트 + 조향 게이지 바"""
    h, w  = panel.shape[:2]
    cx    = w // 2
    bar_w = int(w * 0.42)

    # ── 텍스트 ───────────────────────────────────────────────────
    side     = "◀ LEFT" if offset > 0 else "RIGHT ▶"
    spd_kmh  = speed_mps * 3.6
    tgt_str  = f" / {target_mps*3.6:.0f}" if target_mps is not None else ""
    lines = [
        (f"Status : {status}",                           (200, 200, 100)),
        (f"Speed  : {spd_kmh:.1f}{tgt_str} km/h",       (100, 200, 255)),
        (f"Offset : {abs(offset):.3f} m  {side}",        (100, 255, 200)),
        (f"Steer  : {steer:+.4f}",                       (255, 200, 100)),
    ]
    for i, (txt, color) in enumerate(lines):
        cv2.putText(panel, txt, (12, 24 + i * 27),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1)

    # ── 조향 게이지 바 ────────────────────────────────────────────
    bar_y = h - 40
    # 배경
    cv2.rectangle(panel,
                  (cx - bar_w, bar_y - 10),
                  (cx + bar_w, bar_y + 10),
                  (60, 60, 60), -1)
    # 값 채움
    fill_px = int(steer * bar_w)
    if steer >= 0:
        x0, x1, col = cx, cx + fill_px, (0, 180, 255)
    else:
        x0, x1, col = cx + fill_px, cx, (255, 100, 50)
    if abs(fill_px) > 0:
        cv2.rectangle(panel, (x0, bar_y - 8), (x1, bar_y + 8), col, -1)
    # 중앙선 + 끝 마커
    cv2.line(panel, (cx, bar_y - 14), (cx, bar_y + 14), (220, 220, 220), 2)
    cv2.line(panel, (cx - bar_w, bar_y - 12), (cx - bar_w, bar_y + 12), (150, 150, 150), 1)
    cv2.line(panel, (cx + bar_w, bar_y - 12), (cx + bar_w, bar_y + 12), (150, 150, 150), 1)
    # 레이블
    cv2.putText(panel, "L", (cx - bar_w - 16, bar_y + 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)
    cv2.putText(panel, "R", (cx + bar_w + 4,  bar_y + 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)
    cv2.putText(panel, f"{steer:+.3f}", (cx - 22, bar_y + 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)


# ─── 단독 실행 ───────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="차선 추종 자율 주행 컨트롤러 (Camera → PD → ManualControlById TCP)"
    )
    parser.add_argument("--cam-port",  type=int,   default=proto.UDP_PORT,
                        help=f"카메라 UDP 수신 포트 (기본: {proto.UDP_PORT})")
    parser.add_argument("--cam-ip",    default="127.0.0.1",
                        help="카메라 바인딩 IP (기본: 127.0.0.1)")
    parser.add_argument("--tcp-ip",    default=proto.TCP_SERVER_IP,
                        help=f"시뮬레이터 TCP IP (기본: {proto.TCP_SERVER_IP})")
    parser.add_argument("--tcp-port",  type=int,   default=proto.TCP_SERVER_PORT,
                        help=f"시뮬레이터 TCP 포트 (기본: {proto.TCP_SERVER_PORT})")
    parser.add_argument("--entity-id", default="Car_1",
                        help="제어 엔티티 ID (기본: Car_1)")
    parser.add_argument("--throttle",  type=float, default=0.3,
                        help="고정 스로틀 (기본: 0.3)")
    parser.add_argument("--kp",        type=float, default=0.5,
                        help="PD Kp 게인 (기본: 0.5)")
    parser.add_argument("--kd",        type=float, default=0.1,
                        help="PD Kd 게인 (기본: 0.1)")
    parser.add_argument("--ema",       type=float, default=0.3,
                        help="EMA alpha (기본: 0.3, 범위 0~1)")
    parser.add_argument("--steer-max", type=float, default=1.0,
                        help="최대 조향각 절대값 (기본: 1.0)")
    parser.add_argument("--hz",          type=float, default=20.0,
                        help="제어 루프 주파수 Hz (기본: 20)")
    parser.add_argument("--no-show",     action="store_true",
                        help="OpenCV 창 비활성화")
    parser.add_argument("--no-invert-steer", dest="invert_steer",
                        action="store_false",
                        help="조향 부호 반전 비활성화 (기본: 반전 ON)")
    parser.set_defaults(invert_steer=True)
    parser.add_argument("--min-det-go",  type=int, default=3,
                        help="주행 시작 전 필요한 연속 차선 검출 횟수 (기본: 3)")
    # 속도 피드백
    parser.add_argument("--no-speed-ctrl", dest="speed_ctrl", action="store_false",
                        help="속도 PI 비활성화 → 고정 스로틀 사용")
    parser.set_defaults(speed_ctrl=True)
    parser.add_argument("--vi-port",      type=int,   default=9091,
                        help="Vehicle Info UDP 포트 (기본: 9091)")
    parser.add_argument("--vi-ip",        default="127.0.0.1",
                        help="Vehicle Info 바인딩 IP (기본: 127.0.0.1)")
    parser.add_argument("--target-speed", type=float, default=15.0,
                        help="목표 속도 km/h (기본: 15)")
    parser.add_argument("--kp-spd",       type=float, default=0.05,
                        help="속도 PI Kp (기본: 0.05)")
    parser.add_argument("--ki-spd",       type=float, default=0.01,
                        help="속도 PI Ki (기본: 0.01)")
    parser.add_argument("--throttle-max", type=float, default=0.8,
                        help="최대 스로틀 (기본: 0.8)")
    parser.add_argument("--tune", action="store_true",
                        help="실시간 파라미터 튜닝 창 활성화")
    parser.add_argument("--record", type=str, default=None, metavar="OUTPUT.mp4",
                        help="주행 영상 녹화 저장 경로 (예: --record drive.mp4)")
    args = parser.parse_args()

    # ── TCP 연결 ───────────────────────────────────────────────
    print(f"[Main] TCP 연결 중 → {args.tcp_ip}:{args.tcp_port} ...")
    tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        tcp_sock.connect((args.tcp_ip, args.tcp_port))
    except OSError as e:
        print(f"[Main] TCP 연결 실패: {e}")
        return
    print("[Main] TCP 연결 성공")

    # ── 컨트롤러 & 수신기 초기화 ──────────────────────────────
    print(f"[Main] 조향 부호 반전: {'ON (기본)' if args.invert_steer else 'OFF (--no-invert-steer)'}")

    mode_str = f"속도PI target={args.target_speed}km/h" if args.speed_ctrl else f"고정스로틀 thr={args.throttle}"
    print(f"[Main] 스로틀 모드: {mode_str}")

    controller = LaneController(
        tcp_sock     = tcp_sock,
        entity_id    = args.entity_id,
        throttle     = args.throttle,
        kp           = args.kp,
        kd           = args.kd,
        ema_alpha    = args.ema,
        steer_max    = args.steer_max,
        show         = not args.no_show,
        ctrl_hz      = args.hz,
        invert_steer = args.invert_steer,
        min_det_go   = args.min_det_go,
        speed_ctrl   = args.speed_ctrl,
        tuning       = args.tune,
        vi_ip        = args.vi_ip,
        vi_port      = args.vi_port,
        target_kmh   = args.target_speed,
        kp_spd       = args.kp_spd,
        ki_spd       = args.ki_spd,
        throttle_max = args.throttle_max,
        record_path  = args.record,
    )

    receiver = CameraReceiver(
        ip       = args.cam_ip,
        port     = args.cam_port,
        on_frame = controller.on_frame,
        show     = False,
    )

    receiver.start()
    print(f"[Main] 카메라 수신 시작 ({args.cam_ip}:{args.cam_port})")
    time.sleep(0.5)   # 첫 프레임 수신 대기

    ctrl_thread = controller.start()

    # ── 메인 루프 ─────────────────────────────────────────────
    print("[Main] 실행 중... (Ctrl+C 로 종료)")
    try:
        while ctrl_thread.is_alive():
            # 튜너 패널 표시는 반드시 메인 스레드에서 (OpenCV 규칙)
            if controller._tune_panel is not None:
                controller._tune_panel.draw()
                time.sleep(0.05)   # ~20Hz
            else:
                time.sleep(0.3)
    except KeyboardInterrupt:
        print("\n[Main] 사용자 종료 요청")
    finally:
        controller.stop()

        # 차량 정지 명령 전송
        try:
            tcp.send_manual_control_by_id(
                tcp_sock, _next_rid(), args.entity_id,
                throttle=0.0, brake=1.0, steer_angle=0.0,
            )
            print("[Main] 정지 명령 전송 (brake=1.0)")
        except OSError:
            pass

        receiver.stop()
        ctrl_thread.join(timeout=1.0)
        tcp_sock.close()
        cv2.destroyAllWindows()
        print("[Main] 종료 완료")


if __name__ == "__main__":
    main()
