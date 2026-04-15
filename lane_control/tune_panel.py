from __future__ import annotations
# lane_control/tune_panel.py
#
# TunePanel — OpenCV trackbar 기반 실시간 파라미터 조정 패널
#   lane_controller.py --tuning 옵션으로 활성화
#
#   슬라이더: Kp, Kd, EMA, SteerMax, SteerRate, OffsetClip, TargetSpeed
#   S 키 : 현재 값 터미널 출력
#   R 키 : 초기값 복원

import cv2
import numpy as np

_TUNE_WIN = "Controller Tuner"


class TunePanel:
    """
    OpenCV trackbar 기반 실시간 파라미터 조정 패널.
    lane_controller.py --tuning 옵션으로 활성화.
    """

    def __init__(self, controller: "LaneController"):  # type: ignore[name-defined]
        self._ctrl = controller

        # 초기값 저장 (R키로 복원)
        self._defaults = {
            "kp":          controller._pd.kp,
            "kd":          controller._pd.kd,
            "ema":         controller._ema.alpha,
            "steer_max":   controller._pd.steer_max,
            "steer_rate":  controller._STEER_RATE,
            "offset_clip": controller._OFFSET_CLIP,
            "speed":       controller._speed_pi.target_mps * 3.6
                           if controller._speed_pi else 30.0,
        }

        cv2.namedWindow(_TUNE_WIN)
        cv2.imshow(_TUNE_WIN, np.zeros((10, 420, 3), dtype=np.uint8))

        def n(_): pass

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
        """trackbar 값 읽기 → controller 파라미터 반영"""
        ctrl = self._ctrl
        ctrl._pd.kp        = self._g("Kp x100")    / 100.0
        ctrl._pd.kd        = self._g("Kd x100")    / 100.0
        ctrl._ema.alpha    = max(0.01, self._g("EMA x100")   /  99.0)
        ctrl._pd.steer_max = max(0.10, self._g("SMax x100")  / 100.0)
        ctrl._STEER_RATE   = max(0.01, self._g("SRate x100") / 100.0)
        ctrl._OFFSET_CLIP  = max(0.30, self._g("OClip x10")  /  10.0)
        if self._has_speed and ctrl._speed_pi is not None:
            ctrl._speed_pi.set_target(max(5.0, float(self._g("Speed kmh"))))

    def draw(self):
        """패널 이미지 갱신 + 키 처리 — 반드시 메인 스레드에서 호출"""
        ctrl = self._ctrl
        spd  = ctrl._speed_pi.target_mps * 3.6 if ctrl._speed_pi else 0.0

        panel = np.zeros((155, 420, 3), dtype=np.uint8)
        rows = [
            ("Kp",    f"{ctrl._pd.kp:.3f}"),
            ("Kd",    f"{ctrl._pd.kd:.3f}"),
            ("EMA",   f"{ctrl._ema.alpha:.2f}"),
            ("SMax",  f"{ctrl._pd.steer_max:.2f}"),
            ("SRate", f"{ctrl._STEER_RATE:.3f}"),
            ("OClip", f"{ctrl._OFFSET_CLIP:.1f}m"),
            ("Speed", f"{spd:.0f}km/h"),
        ]
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
        print("\n--- 현재 파라미터 ---")
        print(f"  Kp          = {ctrl._pd.kp:.3f}")
        print(f"  Kd          = {ctrl._pd.kd:.3f}")
        print(f"  EMA alpha   = {ctrl._ema.alpha:.2f}")
        print(f"  Steer Max   = {ctrl._pd.steer_max:.2f}")
        print(f"  Steer Rate  = {ctrl._STEER_RATE:.3f}")
        print(f"  Offset Clip = {ctrl._OFFSET_CLIP:.1f} m")
        print(f"  Target Spd  = {spd:.0f} km/h")
        print("--------------------\n")

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
