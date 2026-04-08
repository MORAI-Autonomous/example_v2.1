# lane_controller.py
#
# Step 4+5: 차선 추종 자율 주행 컨트롤러
#   Camera UDP → LanePreprocessor → LaneDetector → EMA → PD Controller → ManualControlById (TCP)
#   Vehicle Info UDP (9091) → 속도 피드백 → Speed PI → 스로틀 자동 제어
#
# 실행:
#   python lane_controller.py
#   python lane_controller.py --target-speed 30 --kp-spd 0.05
#   python lane_controller.py --no-speed-ctrl --throttle 0.3   # 고정 스로틀 모드
#
# 조작:
#   Ctrl+C : 정지 명령 전송 후 종료

import math
import socket
import time
import threading
import argparse
import itertools

import cv2
import numpy as np

import tcp_transport as tcp
import protocol_defs as proto
from camera_receiver import CameraReceiver
from lane_preprocessor import LanePreprocessor
from lane_detector import LaneDetector
from vehicle_info_with_wheel_receiver import parse_vehicle_info_payload


# ─── Request ID 카운터 (단방향 송신이므로 간단한 카운터로 충분) ──
_rid_iter = itertools.count(1)

def _next_rid() -> int:
    return next(_rid_iter)


# ─── EMA 필터 ────────────────────────────────────────────────────
class EMAFilter:
    """지수 이동 평균 (Exponential Moving Average)"""

    def __init__(self, alpha: float = 0.3):
        self.alpha = alpha
        self._val: float | None = None

    def update(self, x: float) -> float:
        if self._val is None:
            self._val = x
        else:
            self._val = self.alpha * x + (1.0 - self.alpha) * self._val
        return self._val

    def reset(self):
        self._val = None


# ─── PD 컨트롤러 ────────────────────────────────────────────────
class PDController:
    """
    steer = Kp × e + Kd × (e - e_prev) / dt

    e > 0  : 차량이 차선 중앙보다 좌측 → 우측으로 조향 (steer > 0)
    e < 0  : 차량이 차선 중앙보다 우측 → 좌측으로 조향 (steer < 0)
    """

    def __init__(self, kp: float, kd: float, steer_max: float = 1.0):
        self.kp        = kp
        self.kd        = kd
        self.steer_max = steer_max
        self._prev_e   = 0.0
        self._prev_t:  float | None = None

    def compute(self, error: float) -> float:
        now = time.time()
        dt  = max(now - self._prev_t, 1e-3) if self._prev_t is not None else 0.05
        steer = self.kp * error + self.kd * (error - self._prev_e) / dt
        self._prev_e = error
        self._prev_t = now
        return float(np.clip(steer, -self.steer_max, self.steer_max))

    def reset(self):
        self._prev_e  = 0.0
        self._prev_t  = None


# ─── 실시간 파라미터 튜너 ────────────────────────────────────────
_TUNE_WIN = "Controller Tuner"

class TunePanel:
    """
    OpenCV trackbar 기반 실시간 파라미터 조정 패널.
    lane_controller.py --tune 옵션으로 활성화.

    슬라이더 목록:
      Kp, Kd, EMA, SteerMax, SteerRate, OffsetClip, TargetSpeed
    S 키 : 현재 값 터미널 출력
    R 키 : 초기값 복원
    """

    def __init__(self, controller: "LaneController"):
        self._ctrl = controller

        # 초기값 저장 (R키로 복원)
        self._defaults = {
            "kp":         controller._pd.kp,
            "kd":         controller._pd.kd,
            "ema":        controller._ema.alpha,
            "steer_max":  controller._pd.steer_max,
            "steer_rate": controller._STEER_RATE,
            "offset_clip":controller._OFFSET_CLIP,
            "speed":      controller._speed_pi.target_mps * 3.6
                          if controller._speed_pi else 30.0,
        }

        cv2.namedWindow(_TUNE_WIN)
        # 더미 이미지 (trackbar 표시를 위해 창 생성 필요)
        cv2.imshow(_TUNE_WIN, np.zeros((10, 420, 3), dtype=np.uint8))

        def n(_): pass

        # 값 범위: 슬라이더 정수 → 실제 값 = 슬라이더 / 배율
        cv2.createTrackbar("Kp x100",    _TUNE_WIN, int(self._defaults["kp"]          * 100), 300, n)
        cv2.createTrackbar("Kd x100",    _TUNE_WIN, int(self._defaults["kd"]          * 100), 100, n)
        cv2.createTrackbar("EMA x100",   _TUNE_WIN, int(self._defaults["ema"]         *  99),  99, n)
        cv2.createTrackbar("SMax x100",  _TUNE_WIN, int(self._defaults["steer_max"]   * 100), 100, n)
        cv2.createTrackbar("SRate x100", _TUNE_WIN, int(self._defaults["steer_rate"]  * 100),  80, n)
        cv2.createTrackbar("OClip x10",  _TUNE_WIN, int(self._defaults["offset_clip"] *  10),  30, n)
        cv2.createTrackbar("Speed kmh",  _TUNE_WIN, int(self._defaults["speed"]),               80, n)

        self._has_speed = controller._speed_pi is not None

    def _g(self, name: str) -> int:
        return cv2.getTrackbarPos(name, _TUNE_WIN)

    def _set(self, name: str, val: int):
        cv2.setTrackbarPos(name, _TUNE_WIN, val)

    def read_params(self):
        """trackbar 값 읽기 → controller 파라미터 반영 (스레드 안전, GUI 없음)"""
        ctrl = self._ctrl
        ctrl._pd.kp        = self._g("Kp x100")   / 100.0
        ctrl._pd.kd        = self._g("Kd x100")   / 100.0
        ctrl._ema.alpha    = max(0.01, self._g("EMA x100")  /  99.0)
        ctrl._pd.steer_max = max(0.10, self._g("SMax x100") / 100.0)
        ctrl._STEER_RATE   = max(0.01, self._g("SRate x100")/ 100.0)
        ctrl._OFFSET_CLIP  = max(0.30, self._g("OClip x10") /  10.0)
        if self._has_speed and ctrl._speed_pi is not None:
            ctrl._speed_pi.set_target(max(5.0, float(self._g("Speed kmh"))))

    def draw(self):
        """패널 이미지 갱신 + 키 처리 — 반드시 메인 스레드에서 호출"""
        ctrl = self._ctrl
        spd  = ctrl._speed_pi.target_mps * 3.6 if ctrl._speed_pi else 0.0

        panel = np.zeros((155, 420, 3), dtype=np.uint8)
        rows = [
            ("Kp",     f"{ctrl._pd.kp:.3f}"),
            ("Kd",     f"{ctrl._pd.kd:.3f}"),
            ("EMA",    f"{ctrl._ema.alpha:.2f}"),
            ("SMax",   f"{ctrl._pd.steer_max:.2f}"),
            ("SRate",  f"{ctrl._STEER_RATE:.3f}"),
            ("OClip",  f"{ctrl._OFFSET_CLIP:.1f}m"),
            ("Speed",  f"{spd:.0f}km/h"),
        ]
        # 2열 레이아웃으로 압축
        for i, (label, val) in enumerate(rows):
            col = (i % 2) * 210
            row = (i // 2) * 22 + 18
            cv2.putText(panel, f"{label}:{val}",
                        (col + 8, row),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.52, (130, 255, 130), 1)
        cv2.putText(panel, "S=print  R=reset", (8, 148),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (150, 150, 150), 1)
        cv2.imshow(_TUNE_WIN, panel)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("s"):
            self._print_params()
        elif key == ord("r"):
            self._reset()

    def _print_params(self):
        ctrl = self._ctrl
        spd  = ctrl._speed_pi.target_mps * 3.6 if ctrl._speed_pi else 0.0
        print("\n─── 현재 파라미터 ───────────────────────────────────")
        print(f"  Kp          = {ctrl._pd.kp:.3f}")
        print(f"  Kd          = {ctrl._pd.kd:.3f}")
        print(f"  EMA alpha   = {ctrl._ema.alpha:.2f}")
        print(f"  Steer Max   = {ctrl._pd.steer_max:.2f}")
        print(f"  Steer Rate  = {ctrl._STEER_RATE:.3f}")
        print(f"  Offset Clip = {ctrl._OFFSET_CLIP:.1f} m")
        print(f"  Target Spd  = {spd:.0f} km/h")
        print("─────────────────────────────────────────────────────\n")

    def _reset(self):
        d = self._defaults
        self._set("Kp x100",    int(d["kp"]          * 100))
        self._set("Kd x100",    int(d["kd"]          * 100))
        self._set("EMA x100",   int(d["ema"]         *  99))
        self._set("SMax x100",  int(d["steer_max"]   * 100))
        self._set("SRate x100", int(d["steer_rate"]  * 100))
        self._set("OClip x10",  int(d["offset_clip"] *  10))
        self._set("Speed kmh",  int(d["speed"]))
        print("[Tuner] 파라미터 초기값으로 복원")


# ─── Vehicle Info UDP 수신 스레드 (속도 피드백) ──────────────────
class _VehicleInfoThread(threading.Thread):
    """
    Vehicle Info with Wheel UDP 수신 → 최신 속도(m/s) 저장
    포트: 9091 (vehicle_info_with_wheel_receiver)
    """

    def __init__(self, ip: str = "127.0.0.1", port: int = 9091):
        super().__init__(daemon=True, name="vi-recv")
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.settimeout(1.0)
        self._sock.bind((ip, port))
        self._running     = False
        self._lock        = threading.Lock()
        self._speed_mps:  float = 0.0
        self._speed_valid: bool = False   # 한 번이라도 수신됐는지

    def get_speed_mps(self) -> float:
        with self._lock:
            return self._speed_mps

    def is_valid(self) -> bool:
        with self._lock:
            return self._speed_valid

    def stop(self):
        self._running = False
        try:
            self._sock.close()
        except OSError:
            pass

    def run(self):
        self._running = True
        print(f"[VehicleInfo] 수신 시작 {self._sock.getsockname()}")
        while self._running:
            try:
                data, _ = self._sock.recvfrom(4096)
                parsed  = parse_vehicle_info_payload(data)
                if parsed:
                    v   = parsed["local_velocity"]
                    spd = (v["x"]**2 + v["y"]**2 + v["z"]**2) ** 0.5
                    with self._lock:
                        self._speed_mps   = spd
                        self._speed_valid = True
            except socket.timeout:
                continue
            except OSError:
                break
        print("[VehicleInfo] 수신 종료")


# ─── 속도 PI 컨트롤러 ────────────────────────────────────────────
class SpeedPIController:
    """
    target_kmh 로 일정 속도 유지.
    throttle = clip(Kp*e + Ki*∫e dt, 0, throttle_max)
    current > target + brake_tol_kmh → 브레이크
    """

    def __init__(
        self,
        target_kmh:    float = 30.0,
        kp:            float = 0.05,
        ki:            float = 0.01,
        throttle_max:  float = 0.8,
        brake_tol_kmh: float = 3.0,
    ):
        self.target_mps    = target_kmh / 3.6
        self.kp            = kp
        self.ki            = ki
        self.throttle_max  = throttle_max
        self.brake_tol_mps = brake_tol_kmh / 3.6
        self._integral     = 0.0
        self._prev_t: float | None = None

    def compute(self, current_mps: float) -> tuple[float, float]:
        """(throttle, brake) 반환"""
        now = time.time()
        dt  = max(now - self._prev_t, 1e-3) if self._prev_t is not None else 0.05
        self._prev_t = now

        error          = self.target_mps - current_mps
        self._integral = float(np.clip(self._integral + error * dt, -20.0, 20.0))

        raw_throttle = self.kp * error + self.ki * self._integral
        throttle     = float(np.clip(raw_throttle, 0.0, self.throttle_max))
        brake        = 0.0

        # 목표 초과 시 브레이크
        if current_mps > self.target_mps + self.brake_tol_mps:
            over  = current_mps - (self.target_mps + self.brake_tol_mps)
            brake = float(np.clip(self.kp * over * 3.0, 0.0, 0.5))
            throttle = 0.0
            self._integral = max(self._integral, 0.0)  # windup 방지

        return throttle, brake

    def set_target(self, kmh: float):
        self.target_mps = kmh / 3.6

    def reset(self):
        self._integral = 0.0
        self._prev_t   = None


# ─── 메인 컨트롤러 ───────────────────────────────────────────────
class LaneController:
    """
    카메라 프레임을 받아 차선 검출 → PD 제어 → TCP ManualControlById 전송

    Parameters
    ----------
    tcp_sock    : 이미 연결된 TCP 소켓
    entity_id   : 제어 대상 엔티티 이름 (기본 "Car_1")
    throttle    : 고정 스로틀 (기본 0.3)
    kp, kd      : PD 게인
    ema_alpha   : EMA 스무딩 계수 (0~1, 작을수록 더 매끄러움)
    steer_max   : 조향각 클리핑 범위
    show        : True 시 OpenCV 창에 시각화 표시
    ctrl_hz     : 제어 루프 주파수 (Hz)
    """

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
    ):
        self._sock         = tcp_sock
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
        self._vi_thread: _VehicleInfoThread | None = None
        self._speed_pi:  SpeedPIController  | None = None
        if speed_ctrl:
            self._vi_thread = _VehicleInfoThread(ip=vi_ip, port=vi_port)
            self._speed_pi  = SpeedPIController(
                target_kmh   = target_kmh,
                kp           = kp_spd,
                ki           = ki_spd,
                throttle_max = throttle_max,
            )

        self._running     = False
        self._lock        = threading.Lock()
        self._latest_frame: np.ndarray | None = None

        # 안정화 파라미터
        self._last_steer:   float = 0.0
        self._no_det_cnt:   int   = 0
        self._no_valid_cnt: int   = 0   # BAD_W + NO_DET 합산 (재검출 복원율 계산용)
        self._det_streak:   int   = 0
        self._ready:        bool  = False
        self._OFFSET_CLIP:  float = 1.5
        self._STEER_RATE:   float = 0.15

        # 실시간 튜닝 패널
        self._tuning   = tuning
        self._tune_panel: TunePanel | None = None

        # 녹화
        self._record_path = record_path
        self._writer: cv2.VideoWriter | None = None

    # ── 카메라 콜백 ──────────────────────────────────────────────
    def on_frame(self, frame: np.ndarray):
        """CameraReceiver.on_frame 콜백 — 최신 프레임 저장"""
        with self._lock:
            self._latest_frame = frame

    # ── 시작 / 종료 ──────────────────────────────────────────────
    def start(self) -> threading.Thread:
        if self._vi_thread is not None:
            self._vi_thread.start()
        if self._tuning:
            self._tune_panel = TunePanel(self)
            print(f"[Tuner] 실시간 튜닝 창 활성화 — S: 값 출력  R: 초기화")
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
            print(f"[Record] 저장 완료: {self._record_path}")

    # ── 제어 루프 ────────────────────────────────────────────────
    def _loop(self):
        print(f"[Controller] 제어 루프 시작 ({1/self._period:.0f} Hz) "
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

        print("[Controller] 루프 종료")

    def _step(self, frame: np.ndarray):
        # 1. BEV 전처리
        pre    = self._preprocessor.preprocess(frame)

        # 2. Sliding Window 차선 검출
        result = self._detector.detect(pre["binary"])

        bad_offset  = math.isnan(result.offset_m)           # BAD WIDTH nan 체크
        lane_seen   = result.left_detected or result.right_detected
        detected    = lane_seen and not bad_offset           # 오프셋까지 유효한 경우

        # prev_no_valid: BAD_W + NO_DET 합산 카운터 (직전 스텝까지 쌓인 값)
        # BAD_W도 steer를 0으로 decay시키므로, 복원율 계산에 함께 사용
        prev_no_valid = self._no_valid_cnt

        if lane_seen:
            # streak / ready 는 차선이 보이기만 해도 카운트 (BAD WIDTH 무관)
            self._det_streak += 1
            self._no_det_cnt  = 0
            if not self._ready:
                if self._det_streak >= self._min_det_go:
                    self._ready = True
                    print(f"[Controller] 차선 확보 완료 ({self._det_streak}회 연속) → 주행 시작")
                else:
                    print(f"[Controller] 차선 대기 중... ({self._det_streak}/{self._min_det_go})")

            if detected:
                # 정상 검출: 오프셋 EMA + PD 업데이트
                raw_off    = float(np.clip(result.offset_m, -self._OFFSET_CLIP, self._OFFSET_CLIP))
                smooth_off = self._ema.update(raw_off)
                steer_raw  = self._pd.compute(smooth_off)

                if prev_no_valid > 0:
                    # ── BAD_W / NO_DET 이후 복귀: 비대칭 rate limit ──────
                    # 문제 패턴: steer_raw ≈ 0 (오검출) → rate limit 하한이 매 프레임
                    #   last_steer - STEER_RATE 씩 감소 → 등속 선형 낙하 (BAD_W 후 0으로 떨어지는 현상)
                    # 해결: 감소 방향은 STEER_RATE * 0.3 으로 제한
                    #       증가 방향은 no_valid 카운트 비례로 넓혀 빠른 복원 허용
                    max_inc = self._STEER_RATE * (1.0 + min(prev_no_valid * 0.04, 2.0))
                    max_dec = self._STEER_RATE * 0.3
                    steer   = float(np.clip(steer_raw,
                                            self._last_steer - max_dec,
                                            self._last_steer + max_inc))
                    status  = f"REC({prev_no_valid})"
                else:
                    # ── 정상 주행: 대칭 rate limit ───────────────────────
                    steer  = float(np.clip(steer_raw,
                                           self._last_steer - self._STEER_RATE,
                                           self._last_steer + self._STEER_RATE))
                    status = "DET"

                self._last_steer   = steer
                self._no_valid_cnt = 0   # 정상 검출 → 카운터 초기화
            else:
                # BAD WIDTH: 오프셋/steer 업데이트 스킵, 직전 steer 홀드
                # (차선이 한쪽이라도 보이는 상황 → 굳이 decay 하지 않음)
                smooth_off          = self._ema._val if self._ema._val is not None else 0.0
                steer               = self._last_steer          # hold (0.99 decay 제거)
                self._last_steer    = steer
                self._no_valid_cnt += 1   # BAD_W도 누적
                status = f"BAD_W({self._no_valid_cnt})"
        else:
            self._det_streak   = 0
            self._no_det_cnt  += 1
            self._no_valid_cnt += 1
            smooth_off = self._ema._val if self._ema._val is not None else 0.0

            # 미검출 시: 감쇠 속도 완화 (1초 후에도 steer 70% 유지)
            # 0.985^20 ≈ 0.74, 0.975^20 ≈ 0.60
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
            print(f"[Controller] TCP 오류: {e}")
            self._running = False
            return

        # 7. 터미널 출력
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

        # 8. OpenCV 시각화
        if self._show and result.viz is not None:
            self._show_debug(pre, result.viz, steer_out, smooth_off, status,
                             current_mps,
                             self._speed_pi.target_mps if self._speed_pi else None)

        # 9. 튜너 파라미터 읽기 (GUI 없음 — 메인 스레드와 분리)
        if self._tune_panel is not None:
            self._tune_panel.read_params()

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
        """
        레이아웃 (1280 × 480):
        ┌─────────────────────┬──────────────┬──────────────┐
        │  원본 + ROI (좌)    │  BEV 검출    │  이진화      │
        │     640 × 480       │  320 × 240   │  320 × 240   │
        │                     ├──────────────┴──────────────┤
        │                     │  스테이터스 / 조향 게이지   │
        │                     │        640 × 240            │
        └─────────────────────┴─────────────────────────────┘
        """
        W, H = 1280, 480
        PW   = W // 2   # 패널 너비 640
        RW   = W - PW   # 우측 영역 640 (2열)
        CW   = RW // 2  # 우측 각 열 320
        TH   = H // 2   # 상단 높이 240
        BH   = H - TH   # 하단 높이 240

        # ── 좌 패널: 원본 + ROI 사다리꼴 ─────────────────────────
        orig = pre["original"].copy()
        pts  = self._preprocessor.params.src_pts().astype(np.int32)
        cv2.polylines(orig, [pts], True, (0, 255, 0), 2)
        left_panel = cv2.resize(orig, (PW, H))

        # ── 우측 상단 좌: BEV 검출 viz ────────────────────────────
        top_left = cv2.resize(viz, (CW, TH))

        # ── 우측 상단 우: 이진화 ──────────────────────────────────
        binary_color = cv2.cvtColor(pre["binary"], cv2.COLOR_GRAY2BGR)
        top_right = cv2.resize(binary_color, (CW, TH))
        cv2.putText(top_right, "Binary", (5, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 255, 100), 1)

        # ── 우측 하단: 상태 + 조향 게이지 ────────────────────────
        bot_panel = np.zeros((BH, RW, 3), dtype=np.uint8)
        _draw_steer_bar_panel(bot_panel, steer, offset, status, speed_mps, target_mps)

        # 조립
        top_row    = np.hstack([top_left, top_right])   # 640 × 240
        right_col  = np.vstack([top_row, bot_panel])    # 640 × 480
        combined   = np.hstack([left_panel, right_col]) # 1280 × 480

        cv2.imshow("Lane Controller", combined)
        cv2.waitKey(1)

        # ── 녹화 ─────────────────────────────────────────────────────
        if self._record_path is not None:
            if self._writer is None:
                fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                self._writer = cv2.VideoWriter(
                    self._record_path, fourcc, 20.0, (W, H)
                )
                print(f"[Record] 녹화 시작: {self._record_path}  ({W}×{H} @20fps)")
            self._writer.write(combined)


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
