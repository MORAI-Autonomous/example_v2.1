from __future__ import annotations
# lane_control/controllers.py
#
# 차선 제어에 사용되는 경량 컨트롤러 클래스 모음
#   EMAFilter        — 지수 이동 평균 필터
#   PDController     — 조향 PD 제어기
#   SpeedPIController — 속도 PI 제어기

import time
import numpy as np


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
