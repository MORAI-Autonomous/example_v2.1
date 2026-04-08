# lane_detector.py
#
# Sliding Window 차선 검출 모듈
#   1. BEV 이진화 이미지에서 히스토그램으로 차선 기준점 탐색
#   2. Sliding Window 로 차선 픽셀 수집
#   3. 2차 다항식 피팅 (f(y) = Ay² + By + C)
#   4. 차선 중앙 오프셋, 곡률 반경 계산
#   5. 시각화 + 단독 실행 확인 모드

import cv2
import numpy as np
from dataclasses import dataclass
from typing import Optional, Tuple
from lane_preprocessor import LanePreprocessor, BEVParams


# ─── 실세계 스케일 (BEV 픽셀 → 미터 변환) ──────────────────────
# BEVParams 튜닝값과 반드시 일치시킬 것:
#   dst_margin=129 → 유효폭 = 640 - 2*129 = 382 px
#   가로 실세계 약 8m (BEV 전폭 = src 0~640 → 배리어 포함 도로 전체)
#
# ★ BEVParams.dst_margin 변경 시 BEV_DST_MARGIN 도 같이 수정 ★
BEV_DST_MARGIN = 120        # BEVParams.dst_margin 과 일치
ROAD_WIDTH_M   = 8.0        # BEV 가시 가로 실거리 (m)
BEV_IMG_W      = 640        # BEV 이미지 폭 (px)

YM_PER_PIX = 15.0 / 480    # 전방 약 15m → 480px
XM_PER_PIX = ROAD_WIDTH_M / (BEV_IMG_W - 2 * BEV_DST_MARGIN)  # = 0.02000 m/px


# ─── 검출 결과 ────────────────────────────────────────────────────
@dataclass
class LaneResult:
    # 다항식 계수 [A, B, C]  (x = Ay² + By + C)
    left_fit:  Optional[np.ndarray] = None
    right_fit: Optional[np.ndarray] = None

    # 차선 중앙 오프셋 (미터, + → 차량이 중앙 기준 좌측)
    offset_m: float = 0.0

    # 곡률 반경 (미터, 클수록 직선)
    curve_radius_m: float = 9999.0

    # 검출 신뢰도
    left_detected:  bool = False
    right_detected: bool = False

    # 시각화 이미지 (BEV 위에 차선/윈도우 표시)
    viz: Optional[np.ndarray] = None


# ─── LaneDetector ────────────────────────────────────────────────
class LaneDetector:
    """
    Parameters
    ----------
    n_windows  : 슬라이딩 윈도우 개수 (기본 9)
    margin     : 윈도우 좌우 폭 (픽셀, 기본 60)
    min_pixels : 윈도우 내 최소 픽셀 수 (재중심 기준, 기본 30)
    img_w, img_h : BEV 이미지 크기
    """

    def __init__(
        self,
        n_windows:      int   = 9,
        margin:         int   = 80,    # 점선 차선 대응: 넓은 탐색 범위
        min_pixels:     int   = 15,    # 점선 차선 대응: 낮은 최소 픽셀
        img_w:          int   = 640,
        img_h:          int   = 480,
        search_ratio:   float = 0.65,  # 히스토그램 탐색에 사용할 하단 비율
        dst_margin:     int   = BEV_DST_MARGIN,  # BEVParams.dst_margin 과 일치
        hist_bot_crop:  int   = 80,    # 히스토그램 하단 제외 행 수
        # (카메라 근접 배리어/장애물이 히스토그램을 오염하는 것 방지)
    ):
        self.n_windows    = n_windows
        self.margin       = margin
        self.min_pixels   = min_pixels
        self.img_w        = img_w
        self.img_h        = img_h
        self.search_ratio = search_ratio  # 하단 65%만 히스토그램 탐색

        # 히스토그램 마스킹 범위 — 배리어/벽이 포함된 BEV 양 끝단을 제외
        # BEV 유효 도로 영역: [dst_margin, img_w - dst_margin]
        self._hmask_l     = dst_margin
        self._hmask_r     = img_w - dst_margin
        self._hist_bot_crop = hist_bot_crop  # 하단 N행 제외

        # 이전 프레임 결과 (탐색 범위 좁히기용)
        self._prev_left:  Optional[np.ndarray] = None
        self._prev_right: Optional[np.ndarray] = None

        # 연속 미검출 카운터 — 즉시 캐시를 삭제하지 않고 N프레임 유지
        # (합류/그림자 구간에서 _sliding_window 재실행으로 인한 오검출 방지)
        self._left_miss_cnt:  int = 0
        self._right_miss_cnt: int = 0

    # ── 공개 API ─────────────────────────────────────────────────
    def detect(self, binary: np.ndarray) -> LaneResult:
        """
        binary : BEV 이진화 이미지 (np.uint8, 단채널)
        반환   : LaneResult
        """
        if self._prev_left is None or self._prev_right is None:
            # 처음 or 놓친 후 → 전체 히스토그램 탐색
            return self._sliding_window(binary)
        else:
            # 이전 결과 기반 빠른 탐색
            return self._search_around_poly(binary)

    def reset(self):
        self._prev_left      = None
        self._prev_right     = None
        self._left_miss_cnt  = 0
        self._right_miss_cnt = 0

    # ── 차선 폭 기반 베이스 포인트 선택 ─────────────────────────
    @staticmethod
    def _find_best_base(histogram: np.ndarray, w: int):
        """
        히스토그램에서 차선 폭(2.5~4.5 m)에 맞는 좌우 베이스를 선택한다.

        1. 각 절반에서 후보 피크를 모두 추출
        2. 폭이 합리적인 (좌, 우) 쌍 중 이미지 중앙에 가장 가까운 쌍 선택
        3. 조건 만족 쌍이 없으면 단순 argmax 사용 (폴백)
        """
        # XM_PER_PIX = 0.02094 기준:
        #   2.0m → ~95px,  6.0m → ~286px
        # dst_margin 변경 후 차선 간격 픽셀이 달라지므로 넉넉하게 설정
        LANE_W_MIN = int(2.0 / XM_PER_PIX)   # ≈  95 px
        LANE_W_MAX = int(6.0 / XM_PER_PIX)   # ≈ 286 px
        MIN_PEAK_H = max(histogram.max() * 0.15, 5)  # 최소 피크 높이 (노이즈 제거)

        def _peaks(arr, offset=0):
            """단순 local-max 피크 추출 (scipy 없이)"""
            cands = []
            for i in range(1, len(arr) - 1):
                if arr[i] >= MIN_PEAK_H and arr[i] >= arr[i-1] and arr[i] >= arr[i+1]:
                    cands.append(i + offset)
            return cands if cands else [int(np.argmax(arr)) + offset]

        mid = w // 2
        left_cands  = _peaks(histogram[:mid],      offset=0)
        right_cands = _peaks(histogram[mid:],      offset=mid)

        # 차선 폭 조건을 만족하는 (좌, 우) 쌍 중 중앙에 가장 가까운 것
        best_pair   = None
        best_dist   = float("inf")
        for lc in left_cands:
            for rc in right_cands:
                lane_w = rc - lc
                if LANE_W_MIN <= lane_w <= LANE_W_MAX:
                    # 차선 중심이 이미지 중앙(mid)에 얼마나 가까운가
                    center_dist = abs((lc + rc) / 2.0 - mid)
                    if center_dist < best_dist:
                        best_dist = center_dist
                        best_pair = (lc, rc)

        if best_pair:
            return best_pair

        # 폴백: 단순 argmax
        return int(np.argmax(histogram[:mid])), int(np.argmax(histogram[mid:])) + mid

    # ── Sliding Window ───────────────────────────────────────────
    def _sliding_window(self, binary: np.ndarray) -> LaneResult:
        h, w  = binary.shape
        win_h = h // self.n_windows

        # 히스토그램 — 상단 노이즈 + 하단 카메라 근접 배리어 동시 제외
        # top_row : 상단 (1 - search_ratio) 비율 제외
        # bot_row : 하단 hist_bot_crop 행 제외 (배리어/장애물이 집중되는 구간)
        top_row = int(h * (1.0 - self.search_ratio))
        bot_row = max(h - self._hist_bot_crop, top_row + 1)
        histogram = np.sum(binary[top_row:bot_row, :], axis=0).astype(np.float32)

        # BEV 유효 도로 영역 바깥 마스킹 — 배리어/벽/가드레일 픽셀 제거
        histogram[:self._hmask_l] = 0
        histogram[self._hmask_r:] = 0

        # ── 신호 강도 체크 ─────────────────────────────────────────
        # 한쪽 히스토그램이 너무 약하면 해당 슬라이딩 윈도우를 건너뜀
        # → 억지로 잘못된 피크를 찾는 대신 single_lane_fallback 으로 위임
        global_max  = float(histogram.max())
        _side_min   = max(global_max * 0.05, 2.0)   # 전체 최대의 5% 또는 2픽셀
        _mid        = self.img_w // 2
        run_left    = float(histogram[:_mid].max()) >= _side_min
        run_right   = float(histogram[_mid:].max()) >= _side_min

        left_base, right_base = self._find_best_base(histogram, w)

        nonzero   = binary.nonzero()
        nz_y      = np.array(nonzero[0])
        nz_x      = np.array(nonzero[1])
        left_cur  = left_base
        right_cur = right_base

        left_inds_list  = []
        right_inds_list = []
        viz = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)

        for i in range(self.n_windows):
            y_low  = h - (i + 1) * win_h
            y_high = h - i * win_h

            xl_lo, xl_hi = left_cur  - self.margin, left_cur  + self.margin
            xr_lo, xr_hi = right_cur - self.margin, right_cur + self.margin

            cv2.rectangle(viz, (xl_lo, y_low), (xl_hi, y_high), (0, 255, 0), 2)
            cv2.rectangle(viz, (xr_lo, y_low), (xr_hi, y_high), (0, 255, 0), 2)

            if run_left:
                good_l = np.where(
                    (nz_y >= y_low) & (nz_y < y_high) &
                    (nz_x >= xl_lo) & (nz_x < xl_hi)
                )[0]
                left_inds_list.append(good_l)
                if len(good_l) >= self.min_pixels:
                    left_cur = int(np.mean(nz_x[good_l]))

            if run_right:
                good_r = np.where(
                    (nz_y >= y_low) & (nz_y < y_high) &
                    (nz_x >= xr_lo) & (nz_x < xr_hi)
                )[0]
                right_inds_list.append(good_r)
                if len(good_r) >= self.min_pixels:
                    right_cur = int(np.mean(nz_x[good_r]))

        left_inds  = (np.concatenate(left_inds_list)  if left_inds_list
                      else np.array([], dtype=np.intp))
        right_inds = (np.concatenate(right_inds_list) if right_inds_list
                      else np.array([], dtype=np.intp))
        return self._fit_and_result(binary, nz_x, nz_y, left_inds, right_inds, viz)

    # ── 이전 다항식 주변 탐색 ────────────────────────────────────
    def _search_around_poly(self, binary: np.ndarray) -> LaneResult:
        nz   = binary.nonzero()
        nz_y = np.array(nz[0])
        nz_x = np.array(nz[1])
        m    = self.margin

        lf = self._prev_left
        rf = self._prev_right

        left_x_pred  = lf[0] * nz_y**2 + lf[1] * nz_y + lf[2]
        right_x_pred = rf[0] * nz_y**2 + rf[1] * nz_y + rf[2]

        left_inds  = np.where(np.abs(nz_x - left_x_pred)  < m)[0]
        right_inds = np.where(np.abs(nz_x - right_x_pred) < m)[0]

        viz = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
        return self._fit_and_result(binary, nz_x, nz_y, left_inds, right_inds, viz)

    # ── 피팅 및 결과 계산 ────────────────────────────────────────
    def _fit_and_result(
        self,
        binary: np.ndarray,
        nz_x:   np.ndarray,
        nz_y:   np.ndarray,
        l_inds: np.ndarray,
        r_inds: np.ndarray,
        viz:    np.ndarray,
    ) -> LaneResult:
        h, w = binary.shape
        result = LaneResult()

        lx, ly = nz_x[l_inds], nz_y[l_inds]
        rx, ry = nz_x[r_inds], nz_y[r_inds]

        # 좌 다항식 — 픽셀 적으면 1차(직선), 충분하면 2차
        # 2차 피팅 최소 픽셀 80 (40에서 상향): 점선 차선처럼 희박한 점에서
        # 2차 피팅이 wildly 구부러지는 현상 방지
        _CACHE_MAX_MISS = 8   # 연속 미검출 허용 프레임 (0.4s @ 20Hz)
        if len(lx) >= 5:
            degree = 2 if len(lx) >= 80 else 1
            fit = np.polyfit(ly, lx, degree)
            fit = fit if degree == 2 else np.array([0.0, fit[0], fit[1]])
            if self._is_fit_sane(fit, h):
                result.left_fit      = fit
                result.left_detected = True
                self._prev_left      = fit
                self._left_miss_cnt  = 0          # 검출 성공 → 카운터 리셋
                viz[ly, lx] = (255, 50, 50)
            else:
                # 비정상 폴리핏(배리어 글씨·대각선 노이즈) → 미검출 처리
                cv2.putText(viz, "L-FIT ERR", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 100, 255), 2)
                result.left_fit  = self._prev_left
                self._left_miss_cnt += 1
                if self._left_miss_cnt > _CACHE_MAX_MISS:
                    self._prev_left     = None
                    self._left_miss_cnt = 0
        else:
            result.left_fit  = self._prev_left
            self._left_miss_cnt += 1
            # _CACHE_MAX_MISS 프레임 동안은 캐시 유지 →
            # _search_around_poly 가 안정적으로 캐시 주변을 재탐색할 수 있게 함.
            # 즉시 삭제 시 _sliding_window 가 재실행되어 그림자/합류선을
            # 잘못된 기준점으로 잡는 문제 방지
            if self._left_miss_cnt > _CACHE_MAX_MISS:
                self._prev_left     = None
                self._left_miss_cnt = 0

        # 우 다항식 — 픽셀 적으면 1차(직선), 충분하면 2차
        if len(rx) >= 5:
            degree = 2 if len(rx) >= 80 else 1
            fit = np.polyfit(ry, rx, degree)
            fit = fit if degree == 2 else np.array([0.0, fit[0], fit[1]])
            if self._is_fit_sane(fit, h):
                result.right_fit      = fit
                result.right_detected = True
                self._prev_right      = fit
                self._right_miss_cnt  = 0         # 검출 성공 → 카운터 리셋
                viz[ry, rx] = (50, 50, 255)
            else:
                cv2.putText(viz, "R-FIT ERR", (10, 55),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 100, 255), 2)
                result.right_fit  = self._prev_right
                self._right_miss_cnt += 1
                if self._right_miss_cnt > _CACHE_MAX_MISS:
                    self._prev_right     = None
                    self._right_miss_cnt = 0
        else:
            result.right_fit  = self._prev_right
            self._right_miss_cnt += 1
            if self._right_miss_cnt > _CACHE_MAX_MISS:
                self._prev_right     = None
                self._right_miss_cnt = 0

        # 다항식 곡선 그리기 + 오프셋/곡률 계산
        plot_y  = np.linspace(0, h - 1, h)

        if result.left_fit is not None and result.right_fit is not None:
            lf, rf  = result.left_fit, result.right_fit

            # 실제 검출된 픽셀 y 범위만 그리기 (엉뚱한 외삽 방지)
            ly_range = ly if result.left_detected  else np.array([h // 2, h - 1])
            ry_range = ry if result.right_detected else np.array([h // 2, h - 1])
            y_min = int(max(np.min(ly_range), np.min(ry_range)))
            y_max = h - 1
            draw_y = np.linspace(y_min, y_max, y_max - y_min + 1)

            left_x  = lf[0] * draw_y**2 + lf[1] * draw_y + lf[2]
            right_x = rf[0] * draw_y**2 + rf[1] * draw_y + rf[2]

            # 차선 채우기 (검출 구간만)
            pts_left  = np.array([np.transpose(np.vstack([left_x,  draw_y]))], dtype=np.int32)
            pts_right = np.array([np.flipud(np.transpose(np.vstack([right_x, draw_y])))], dtype=np.int32)
            pts_lane  = np.hstack((pts_left, pts_right))
            lane_img  = np.zeros_like(viz)
            cv2.fillPoly(lane_img, pts_lane, (0, 60, 0))
            viz = cv2.addWeighted(viz, 1.0, lane_img, 0.4, 0)

            # 곡선 라인 (검출 구간만)
            for pts, color in [(pts_left, (255, 180, 0)), (pts_right, (0, 180, 255))]:
                cv2.polylines(viz, pts, False, color, 3)

            # 오프셋: 하단 40% 픽셀의 중앙값으로 계산 (다항식 외삽 오류 방지)
            bot_thresh = int(h * 0.6)
            l_bot = lx[ly >= bot_thresh] if result.left_detected  and len(ly) > 0 else np.array([])
            r_bot = rx[ry >= bot_thresh] if result.right_detected and len(ry) > 0 else np.array([])
            y_eval = y_max
            # 다항식 외삽값은 이미지 경계 [0, w] 로 클램핑 (극단적 외삽 방지)
            raw_l = float(np.median(l_bot)) if len(l_bot) > 0 else lf[0]*y_eval**2+lf[1]*y_eval+lf[2]
            raw_r = float(np.median(r_bot)) if len(r_bot) > 0 else rf[0]*y_eval**2+rf[1]*y_eval+rf[2]
            left_x_bot  = float(np.clip(raw_l, 0, w))
            right_x_bot = float(np.clip(raw_r, 0, w))

            # ── BEV 경계 위치 유효성 검사 ─────────────────────────────
            # dst_margin 영역은 유효 도로 바깥 (배리어·벽·합류선 등)
            #   좌측 차선이 dst_margin 안쪽(x < 129)에 있으면 합류/경계 오검출
            #   우측 차선이 dst_margin 바깥(x > 511)에 있으면 배리어 오검출
            # → BAD_W 폭 체크는 통과해도 이 조건에 걸릴 수 있음
            #   (예: 합류선 left_x=92, cache right_x=440 → 폭 3.5m → BAD_W 통과)
            _margin = BEV_DST_MARGIN
            if left_x_bot < _margin or right_x_bot > (w - _margin):
                label = ("L-BOUNDARY" if left_x_bot < _margin
                         else "R-BOUNDARY")
                cv2.putText(viz, label, (10, viz.shape[0] - 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
                result.offset_m = float("nan")
                # 잘못된 쪽 캐시 강제 초기화
                if left_x_bot < _margin:
                    self._prev_left     = None
                    self._left_miss_cnt = 0
                if right_x_bot > (w - _margin):
                    self._prev_right     = None
                    self._right_miss_cnt = 0
                result.viz = viz
                return result

            lane_center  = (left_x_bot + right_x_bot) / 2.0
            result.offset_m = (lane_center - w / 2.0) * XM_PER_PIX

            # 차선 폭 이상값 처리
            # 범위를 넉넉하게: 1.5~6.5m (XM_PER_PIX 오차 및 곡선부 여유 포함)
            lane_w_m = (right_x_bot - left_x_bot) * XM_PER_PIX
            if not (1.5 <= lane_w_m <= 6.5):
                cv2.putText(viz, f"BAD WIDTH {lane_w_m:.1f}m", (10, viz.shape[0] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)

                if lane_w_m < 1.5:
                    # ── 너무 좁음: 우측이 좌측에 붙어있음 ──────────────
                    # (right_x_bot ≈ left_x_bot, 또는 right < left)
                    # 우측 검출이 잘못됨 → 우측 캐시 강제 초기화 후 left 단독 폴백
                    self._prev_right      = None
                    self._right_miss_cnt  = 0
                    result.right_fit      = None
                    result.right_detected = False
                    result = LaneDetector._single_lane_fallback(result, h, w, plot_y, viz)
                    result.viz = viz
                    return result

                # 한쪽만 실제 검출된 경우 → 캐시 fit 제거 후 single_lane_fallback
                elif result.left_detected != result.right_detected:
                    if not result.left_detected:
                        result.left_fit  = None
                    if not result.right_detected:
                        result.right_fit = None
                    result = LaneDetector._single_lane_fallback(result, h, w, plot_y, viz)
                    result.viz = viz
                    return result
                else:
                    # 양쪽 다 잘못됐거나 양쪽 다 캐시 → nan으로 표시
                    result.offset_m = float("nan")

            # 곡률 반경 (미터) — 검출된 픽셀로만 계산
            y_eval_m = y_eval * YM_PER_PIX
            radii = []
            if result.left_detected and len(ly) >= 3:
                lf_m = np.polyfit(ly * YM_PER_PIX, lx * XM_PER_PIX, 2)
                denom = abs(2 * lf_m[0])
                if denom > 1e-6:
                    radii.append((1 + (2 * lf_m[0] * y_eval_m + lf_m[1])**2)**1.5 / denom)
            if result.right_detected and len(ry) >= 3:
                rf_m = np.polyfit(ry * YM_PER_PIX, rx * XM_PER_PIX, 2)
                denom = abs(2 * rf_m[0])
                if denom > 1e-6:
                    radii.append((1 + (2 * rf_m[0] * y_eval_m + rf_m[1])**2)**1.5 / denom)
            result.curve_radius_m = float(np.mean(radii)) if radii else 9999.0

            # HUD 텍스트
            _put_hud(viz,
                     offset=result.offset_m,
                     radius=result.curve_radius_m,
                     left_ok=result.left_detected,
                     right_ok=result.right_detected)

        elif result.left_fit is not None or result.right_fit is not None:
            if result.left_detected or result.right_detected:
                # 실제 픽셀이 있는 쪽 기반으로 반대편 추정
                result = self._single_lane_fallback(result, h, w, plot_y, viz)
            else:
                # ── 캐시 전용 (실제 검출 픽셀 없음) ─────────────────────
                # 좌우 모두 실제 픽셀이 없으면 offset_m = nan 처리
                # → 컨트롤러가 NO_DET 로 진입해 steer 를 decay
                # (터널/그림자 구간에서 잘못된 캐시 fit 을 따라가는 것 방지)
                # ※ _prev_left/right 캐시는 _CACHE_MAX_MISS 프레임 동안은 유지됨
                #   → 다음 프레임에서 _search_around_poly 가 재시도
                result.offset_m = float("nan")
                cv2.putText(viz, "CACHE ONLY", (10, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (80, 80, 255), 2)

        result.viz = viz
        return result

    # ── 폴리핏 물리적 타당성 검사 ────────────────────────────────
    @staticmethod
    def _is_fit_sane(fit: np.ndarray, img_h: int) -> bool:
        """
        BEV 다항식 x = Ay² + By + C 가 실제 차선으로 타당한지 검사.

        조건:
          1. 수평 이동량(sweep): 화면 상단~하단 간 x 변화 ≤ MAX_SWEEP
             → 대각선으로 가로지르는 배리어 글씨·그림자 픽셀 차단
          2. 선형 기울기 |B| ≤ MAX_B
          3. 2차 곡률  |A| ≤ MAX_A

        기준값 근거:
          straight  : B≈0,   sweep≈0
          gentle    : B≈0.2, sweep≈96px
          sharp     : B≈0.4, sweep≈192px
          INVALID   : B≈0.9, sweep≈432px  (배리어 글씨 대각선)
        """
        if fit is None or len(fit) < 3:
            return False
        A, B, C = float(fit[0]), float(fit[1]), float(fit[2])
        sweep   = abs(A * img_h ** 2 + B * img_h)   # x_bot - x_top
        MAX_SWEEP = img_h * 0.65   # ≈ 312px  (480×0.65)
        MAX_B     = 0.80           # 선형 기울기 상한
        MAX_A     = 0.004          # 2차 곡률 상한
        if sweep > MAX_SWEEP:
            return False
        if abs(B) > MAX_B:
            return False
        if abs(A) > MAX_A:
            return False
        return True

    # ── 한쪽 차선만 검출된 경우 ──────────────────────────────────
    @staticmethod
    def _single_lane_fallback(result: LaneResult, h, w, plot_y, viz) -> LaneResult:
        """검출된 한쪽 차선으로 반대편 위치를 추정 (직선 주행 유지용)"""
        LANE_WIDTH_PX = int(3.5 / XM_PER_PIX)  # 차선폭 약 3.5m

        if result.left_fit is not None:
            lf = result.left_fit
            y_eval = h - 1
            left_x_bot = lf[0] * y_eval**2 + lf[1] * y_eval + lf[2]

            # ── 위치 유효성 검사 ─────────────────────────────────────
            # 좌측 차선 하한: BEV 유효 영역 안(x >= dst_margin)이어야 함
            #   (합류선이 경계 바깥에서 좌측으로 오검출되는 케이스 방지)
            # 좌측 차선 상한: 이미지 우측 65% 이하
            #   (그림자·합류선이 우측에 있을 때 좌측으로 오인 방지)
            if not (BEV_DST_MARGIN <= left_x_bot <= w * 0.65):
                result.offset_m = float("nan")
                cv2.putText(viz, "L-POS ERR", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 80, 255), 2)
                return result

            lx = lf[0] * plot_y**2 + lf[1] * plot_y + lf[2]
            rx = lx + LANE_WIDTH_PX
            pts_l = np.array([np.transpose(np.vstack([lx, plot_y]))], dtype=np.int32)
            cv2.polylines(viz, pts_l, False, (255, 180, 0), 3)
            right_x_bot = left_x_bot + LANE_WIDTH_PX
        else:
            rf = result.right_fit
            y_eval = h - 1
            right_x_bot = rf[0] * y_eval**2 + rf[1] * y_eval + rf[2]

            # ── 위치 유효성 검사 ─────────────────────────────────────
            # 우측 차선 하한: 이미지 좌측 35% 이상
            # 우측 차선 상한: BEV 유효 영역 안(x <= w - dst_margin)이어야 함
            if not (w * 0.35 <= right_x_bot <= w - BEV_DST_MARGIN):
                result.offset_m = float("nan")
                cv2.putText(viz, "R-POS ERR", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 80, 255), 2)
                return result

            rx = rf[0] * plot_y**2 + rf[1] * plot_y + rf[2]
            lx = rx - LANE_WIDTH_PX
            pts_r = np.array([np.transpose(np.vstack([rx, plot_y]))], dtype=np.int32)
            cv2.polylines(viz, pts_r, False, (0, 180, 255), 3)
            left_x_bot = right_x_bot - LANE_WIDTH_PX

        lane_center     = (left_x_bot + right_x_bot) / 2.0
        result.offset_m = (lane_center - w / 2.0) * XM_PER_PIX

        _put_hud(viz, result.offset_m, result.curve_radius_m,
                 result.left_detected, result.right_detected)
        return result


# ─── HUD 헬퍼 ───────────────────────────────────────────────────
def _put_hud(img, offset, radius, left_ok, right_ok):
    side  = "LEFT " if offset > 0 else "RIGHT"
    r_str = f"{radius:.0f}m" if radius < 5000 else "STRAIGHT"
    lines = [
        f"Offset : {abs(offset):.2f}m {side}",
        f"Radius : {r_str}",
        f"Lane   : {'L' if left_ok else '-'} {'R' if right_ok else '-'}",
    ]
    for i, txt in enumerate(lines):
        cv2.putText(img, txt, (10, 25 + i * 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 2)


# ─── 단독 실행 (검출 확인용) ─────────────────────────────────────
def main():
    import argparse, time
    from camera_receiver import CameraReceiver

    parser = argparse.ArgumentParser(description="Sliding Window 차선 검출 확인")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--image", type=str, help="정지 이미지 파일")
    group.add_argument("--port",  type=int, help="카메라 UDP 포트")
    group.add_argument("--video", type=str, help="영상 파일 오프라인 재분석 (.mp4 등)")
    parser.add_argument("--ip", default="127.0.0.1")
    args = parser.parse_args()

    preprocessor = LanePreprocessor()   # 튜닝된 기본값 사용
    detector     = LaneDetector()

    def process(frame):
        result_pre = preprocessor.preprocess(frame)
        result_det = detector.detect(result_pre["binary"])

        # 4분할 뷰: 전처리 디버그(좌) + 검출 BEV(우 상단) + 히스토그램(우 하단)
        bev_h, bev_w = result_det.viz.shape[:2]
        hist = np.sum(result_pre["binary"][bev_h // 2:, :], axis=0)
        hist_img = np.zeros((bev_h // 2, bev_w, 3), dtype=np.uint8)
        for x, val in enumerate(hist):
            val_scaled = int(val * (bev_h // 2) / max(hist.max(), 1))
            cv2.line(hist_img, (x, bev_h // 2), (x, bev_h // 2 - val_scaled),
                     (0, 255, 100), 1)
        cv2.putText(hist_img, "Histogram", (5, 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        right_col = np.vstack([result_det.viz, hist_img])
        # 높이 맞추기
        lh = result_pre["debug"].shape[0]
        rh = right_col.shape[0]
        if lh != rh:
            right_col = cv2.resize(right_col, (bev_w, lh))

        combined = np.hstack([result_pre["debug"], right_col])
        cv2.imshow("Lane Detection", combined)

    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            print(f"이미지를 읽을 수 없습니다: {args.image}")
            return
        frame = cv2.resize(frame, (640, 480))
        while True:
            process(frame)
            key = cv2.waitKey(30) & 0xFF
            if key in (27, ord("q")):
                break

    elif args.video:
        cap = cv2.VideoCapture(args.video)
        if not cap.isOpened():
            print(f"영상을 열 수 없습니다: {args.video}")
            return

        total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        fps          = cap.get(cv2.CAP_PROP_FPS) or 30.0
        delay_ms     = max(1, int(1000.0 / fps))

        paused    = False
        frame_idx = 0
        cur_frame = None
        save_cnt  = 0

        print(f"[Video] {args.video}  총 {total_frames} 프레임  {fps:.1f}fps")
        print("  Space : 일시정지/재생   A/← : 이전 프레임   D/→ : 다음 프레임")
        print("  S : 현재 프레임 저장   Q/ESC : 종료")

        def _read_frame(idx: int):
            cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
            ok, f = cap.read()
            return f if ok else None

        while True:
            if not paused:
                ok, raw = cap.read()
                if not ok:
                    # 끝까지 재생 → 마지막 프레임에서 일시정지
                    paused = True
                    print("[Video] 재생 끝 — 일시정지 (A 로 되감기)")
                else:
                    frame_idx = int(cap.get(cv2.CAP_PROP_POS_FRAMES)) - 1
                    cur_frame = cv2.resize(raw, (640, 480))

            if cur_frame is not None:
                result_pre = preprocessor.preprocess(cur_frame)
                result_det = detector.detect(result_pre["binary"])

                bev_h, bev_w = result_det.viz.shape[:2]
                hist = np.sum(result_pre["binary"][bev_h // 2:, :], axis=0)
                hist_img = np.zeros((bev_h // 2, bev_w, 3), dtype=np.uint8)
                for x, val in enumerate(hist):
                    val_scaled = int(val * (bev_h // 2) / max(hist.max(), 1))
                    cv2.line(hist_img, (x, bev_h // 2), (x, bev_h // 2 - val_scaled),
                             (0, 255, 100), 1)
                cv2.putText(hist_img, "Histogram", (5, 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

                right_col = np.vstack([result_det.viz, hist_img])
                lh = result_pre["debug"].shape[0]
                rh = right_col.shape[0]
                if lh != rh:
                    right_col = cv2.resize(right_col, (bev_w, lh))

                combined = np.hstack([result_pre["debug"], right_col])

                # ── 오버레이: 프레임 번호 + 상태 ─────────────────────
                pct = frame_idx / max(total_frames - 1, 1) * 100
                info1 = f"Frame {frame_idx}/{total_frames-1}  ({pct:.1f}%)"
                info2 = "|| PAUSE" if paused else "▶ PLAY"
                cv2.putText(combined, info1, (10, combined.shape[0] - 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 80), 1)
                cv2.putText(combined, info2, (10, combined.shape[0] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (80, 80, 220) if paused else (80, 220, 80), 1)

                # ── 재생 바 ───────────────────────────────────────────
                bar_x0, bar_x1 = 10, combined.shape[1] - 10
                bar_y          = combined.shape[0] - 5
                cv2.line(combined, (bar_x0, bar_y), (bar_x1, bar_y), (80, 80, 80), 3)
                pos_x = int(bar_x0 + (bar_x1 - bar_x0) * frame_idx / max(total_frames - 1, 1))
                cv2.circle(combined, (pos_x, bar_y), 5, (0, 200, 255), -1)

                cv2.imshow("Lane Detection", combined)

            wait = 0 if paused else delay_ms
            key  = cv2.waitKey(wait) & 0xFF

            if key in (27, ord("q")):
                break
            elif key in (ord(" "),):
                paused = not paused
                print(f"[Video] {'일시정지' if paused else '재생'} (프레임 {frame_idx})")
            elif key in (ord("a"), 81):   # A 또는 ← 화살표
                frame_idx = max(0, frame_idx - 1)
                cur_frame_raw = _read_frame(frame_idx)
                if cur_frame_raw is not None:
                    cur_frame = cv2.resize(cur_frame_raw, (640, 480))
                paused = True
            elif key in (ord("d"), 83):   # D 또는 → 화살표
                frame_idx = min(total_frames - 1, frame_idx + 1)
                cur_frame_raw = _read_frame(frame_idx)
                if cur_frame_raw is not None:
                    cur_frame = cv2.resize(cur_frame_raw, (640, 480))
                paused = True
            elif key == ord("s"):
                save_cnt += 1
                fname = f"frame_{frame_idx:06d}_{save_cnt:03d}.png"
                cv2.imwrite(fname, combined)
                print(f"[Video] 저장: {fname}")

        cap.release()

    else:
        receiver = CameraReceiver(ip=args.ip, port=args.port, show=False)
        receiver.start()
        print("차선 검출 중... Q/ESC 종료")
        while True:
            frame = receiver.get_latest_frame()
            if frame is not None:
                process(frame)
            key = cv2.waitKey(30) & 0xFF
            if key in (27, ord("q")):
                break
        receiver.stop()

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
