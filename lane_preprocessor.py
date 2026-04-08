# lane_preprocessor.py
#
# 차선 검출 전처리 모듈
#   1. ROI 마스킹
#   2. Bird's Eye View (원근 변환)
#   3. 흰색 차선 이진화
#   4. 대화형 파라미터 튜닝 모드 (--tune)
#
# 실행:
#   python lane_preprocessor.py --tune --image <이미지 파일>
#   python lane_preprocessor.py --tune --port 9090   (실시간 카메라)

import cv2
import numpy as np
import argparse
import time
from dataclasses import dataclass, field
from typing import Tuple


# ─── BEV 파라미터 ────────────────────────────────────────────────
# 카메라 스펙 기반 초기 추정값
#   - 해상도 640x480, 높이 1.5m, FOV H=90, 전방 1.6m
#
# src: 원본 이미지에서 차선이 포함된 사다리꼴 꼭짓점
#      [(좌하), (우하), (우상), (좌상)]
# dst: BEV 이미지에서 대응되는 직사각형 꼭짓점

@dataclass
class BEVParams:
    img_w: int = 640
    img_h: int = 480

    # 사다리꼴 src (픽셀 좌표) — 튜닝 완료값
    src_bot_left_x:  int = 0     # 하단 좌
    src_bot_right_x: int = 640   # 하단 우
    src_bot_y:       int = 480   # 하단 y
    src_top_left_x:  int = 280   # 상단 좌 (수렴점)
    src_top_right_x: int = 362   # 상단 우 (수렴점)
    src_top_y:       int = 242   # 상단 y (수평선 근처)

    # 목적지 직사각형 여백
    dst_margin: int = 120

    # 흰색 차선 HSV 임계값
    # S_max를 낮게 유지: 흰색/밝은회색만 통과, 황색 배리어(S≈150+) 차단
    white_h_min: int = 0
    white_h_max: int = 180
    white_s_min: int = 0
    white_s_max: int = 72    # 배리어/벽 오검출 방지 (황색 S≈150, 흰색 S≈0~50)
    white_v_min: int = 53    # 적응형 V_min의 절대 하한선 (낮게 유지)
    white_v_max: int = 255

    # 노란색 차선 HSV 임계값 (터널 중앙선 등)
    # 흰색 S_max=70 으로 차단되는 노란 중앙선을 별도 채널로 검출
    # S 범위를 좁게: 노란색 차선(S≈80~180)만 통과, 고채도 배리어(S≈180+) 차단
    yellow_h_min: int = 15   # 노란색 Hue 범위 (15~35)
    yellow_h_max: int = 35
    yellow_s_min: int = 80   # 흰색과 구분
    yellow_s_max: int = 160  # 고채도 배리어/화살표 억제
    yellow_v_min: int = 63   # white_v_min 과 동일한 절대 하한선
    yellow_enable: bool = False  # 노란 차선 검출 활성화
    # ↑ 주황 배리어(H≈15, S≈150)가 노란 범위(H:15-35)와 겹쳐 오검출됨
    #   터널 노란 중앙선 구간 필요 시 True 로 변경

    def src_pts(self) -> np.ndarray:
        return np.float32([
            [self.src_bot_left_x,  self.src_bot_y],
            [self.src_bot_right_x, self.src_bot_y],
            [self.src_top_right_x, self.src_top_y],
            [self.src_top_left_x,  self.src_top_y],
        ])

    def dst_pts(self) -> np.ndarray:
        m = self.dst_margin
        return np.float32([
            [m,                  self.img_h],
            [self.img_w - m,     self.img_h],
            [self.img_w - m,     0],
            [m,                  0],
        ])

    def M(self) -> np.ndarray:
        """원근 변환 행렬"""
        return cv2.getPerspectiveTransform(self.src_pts(), self.dst_pts())

    def M_inv(self) -> np.ndarray:
        """역변환 행렬 (BEV → 원본)"""
        return cv2.getPerspectiveTransform(self.dst_pts(), self.src_pts())


# ─── 전처리 파이프라인 ───────────────────────────────────────────
class LanePreprocessor:
    def __init__(self, params: BEVParams = None):
        self.params = params or BEVParams()

    def update_params(self, params: BEVParams):
        self.params = params

    def preprocess(self, frame: np.ndarray) -> dict:
        """
        전처리 전체 파이프라인 실행

        Returns
        -------
        dict with keys:
            original   : 원본 프레임
            roi        : ROI 마스킹 적용
            bev        : Bird's Eye View 변환
            binary     : 흰색 차선 이진화 (BEV 기반)
            debug      : 시각화용 합성 이미지
        """
        p = self.params

        # 1. ROI 마스킹 (사다리꼴 영역만 남김)
        roi = self._apply_roi(frame, p)

        # 2. Bird's Eye View 변환
        M = p.M()
        bev = cv2.warpPerspective(roi, M, (p.img_w, p.img_h))

        # 3. 흰색 차선 이진화
        binary = self._white_threshold(bev, p)

        # 4. 디버그 시각화
        debug = self._make_debug(frame, bev, binary, p)

        return {
            "original": frame,
            "roi":      roi,
            "bev":      bev,
            "binary":   binary,
            "debug":    debug,
        }

    # ── 내부 헬퍼 ────────────────────────────────────────────────
    @staticmethod
    def _apply_roi(frame: np.ndarray, p: BEVParams) -> np.ndarray:
        mask = np.zeros_like(frame)
        pts  = p.src_pts().astype(np.int32)
        cv2.fillPoly(mask, [pts], (255, 255, 255))
        return cv2.bitwise_and(frame, mask)

    @staticmethod
    def _white_threshold(bev: np.ndarray, p: BEVParams) -> np.ndarray:
        hsv = cv2.cvtColor(bev, cv2.COLOR_BGR2HSV)

        # ── 어두운 환경(터널) 감지 — CLAHE 적용 전 원본 밝기 기준 ────
        #   CLAHE 후에는 히스토그램이 늘어나므로 적용 전 값으로 판단
        v_raw         = hsv[:, :, 2]
        v_raw_nonblack = v_raw[v_raw > 10]
        mean_v_raw    = (float(np.mean(v_raw_nonblack))
                         if len(v_raw_nonblack) > 100 else 128.0)
        # 비검정 픽셀 평균 V < 60 → 터널·야간 등 저조도 환경
        is_dark = mean_v_raw < 70

        # ── CLAHE: V 채널 적응형 히스토그램 평활화 ───────────────────
        #   터널: clipLimit 4.0 (대비 강화) / 일반: 2.0
        clip_limit = 4.0 if is_dark else 2.0
        clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=(8, 8))
        hsv[:, :, 2] = clahe.apply(hsv[:, :, 2])

        # ── 적응형 V_min ─────────────────────────────────────────────
        #   p90을 저채도(흰색/회색 계열) 픽셀만으로 계산
        #   → 주황 배리어(S≈150+)가 BEV를 점령해도 p90이 오염되지 않음
        #
        #   터널보정: p90 × 0.80 → 상위 ~30% 인식 (희미한 차선 포함)
        #   일반환경: p90 × 0.90 → 상위 ~15% 인식 (노이즈 억제 유지)
        #
        #   예시 (터널 입구, 배리어 점령 장면):
        #     전체 p90≈200 → adapt_vmin=180 → 그늘 차선(V≈120) 탈락 ✗
        #     저채도 p90≈140 → adapt_vmin=126 → 그늘 차선(V≈120) 통과 ✓
        v_channel  = hsv[:, :, 2]
        s_channel  = hsv[:, :, 1]
        v_nonblack = v_channel[v_channel > 10].flatten()
        # 저채도(S ≤ white_s_max) 비검정 픽셀 = 흰색/회색 도로면 후보
        _wcand_mask  = (v_channel > 10) & (s_channel <= p.white_s_max)
        v_white_cand = v_channel[_wcand_mask].flatten()
        if len(v_white_cand) > 200:
            p90 = float(np.percentile(v_white_cand, 90))
        elif len(v_nonblack) > 200:          # 저채도 픽셀 부족 시 전체로 fallback
            p90 = float(np.percentile(v_nonblack, 90))
        else:
            p90 = 128.0
        factor     = 0.80 if is_dark else 0.90
        adapt_vmin = int(np.clip(p90 * factor, p.white_v_min, 250))

        # ── 흰색 차선 마스크 ─────────────────────────────────────────
        lower = np.array([p.white_h_min, p.white_s_min, adapt_vmin])
        upper = np.array([p.white_h_max, p.white_s_max, p.white_v_max])
        mask  = cv2.inRange(hsv, lower, upper)

        # ── 노란색 차선 마스크 (터널 중앙선 등) ──────────────────────
        if p.yellow_enable:
            adapt_y_vmin = int(np.clip(adapt_vmin, p.yellow_v_min, 250))
            lower_y = np.array([p.yellow_h_min, p.yellow_s_min, adapt_y_vmin])
            upper_y = np.array([p.yellow_h_max, p.yellow_s_max, p.white_v_max])
            mask_y  = cv2.inRange(hsv, lower_y, upper_y)
            mask    = cv2.bitwise_or(mask, mask_y)

        # ── 배리어 색상 영역 제거 (터널/저조도 전용) ────────────────
        #   적/주황 배리어(H=0~20, S≥100)를 검출 → 팽창 → 인접 흰색 줄무늬 제거
        #
        #   ★ is_dark(터널) 일 때만 적용하는 이유:
        #     실외 환경: 배리어/배너는 BEV 가장자리에 위치
        #       → dst_margin 엣지 마스킹(x<129, x>511)으로 이미 충분히 처리됨
        #       → 25px 팽창 적용 시 배너 인근 흰색 차선 픽셀까지 제거되는 부작용
        #     터널 환경: 배리어가 차량에 근접 → BEV 중앙 대각선으로 침투
        #       → 엣지 마스킹만으로 부족 → 색상 기반 제거가 필요
        #
        #   OpenCV HSV 빨강 wrap-around: orange-red H=0~20, pure red H=160~180
        if is_dark:
            _bar1 = cv2.inRange(hsv,
                                np.array([0,   100, 80]),
                                np.array([20,  255, 255]))
            _bar2 = cv2.inRange(hsv,
                                np.array([160, 100, 80]),
                                np.array([180, 255, 255]))
            _bar  = cv2.bitwise_or(_bar1, _bar2)
            if cv2.countNonZero(_bar) > 50:      # 배리어가 실제로 존재하는 경우만
                k_bar = cv2.getStructuringElement(cv2.MORPH_RECT, (25, 25))
                mask  = cv2.bitwise_and(mask,
                                        cv2.bitwise_not(cv2.dilate(_bar, k_bar)))

        # ── BEV 유효 도로 영역 외 완전 마스킹 ────────────────────────
        #   dst_margin 바깥(좌우 가장자리)은 배리어·벽 영역:
        #   히스토그램 마스킹(_hmask_l/_hmask_r)과 달리 binary 자체를 제거
        #   → 슬라이딩 윈도우가 배리어 픽셀을 차선으로 잡는 것 원천 차단
        m = p.dst_margin
        if m > 0:
            mask[:, :m]            = 0   # 좌측 배리어/벽 영역 제거
            mask[:, p.img_w - m:]  = 0   # 우측 배리어/벽 영역 제거

        # ── 노이즈 제거 ───────────────────────────────────────────────
        #   터널: OPEN 3×3  (5×5는 희미한 얇은 차선 픽셀을 지워버림)
        #   일반: OPEN 5×5  (나뭇잎 햇살 소점 노이즈 제거)
        #   CLOSE 3×3: 차선 내 작은 구멍 메우기
        k_open_sz = 3 if is_dark else 5
        k_open  = cv2.getStructuringElement(cv2.MORPH_RECT, (k_open_sz, k_open_sz))
        k_close = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        mask    = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k_open)
        mask    = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k_close)
        return mask

    @staticmethod
    def _make_debug(original: np.ndarray, bev: np.ndarray,
                    binary: np.ndarray, p: BEVParams) -> np.ndarray:
        """4분할 디버그 뷰: 원본+ROI선 | BEV | 이진화 | 이진화(컬러)"""
        h, w = original.shape[:2]

        # 원본 + ROI 사다리꼴 그리기
        vis_orig = original.copy()
        pts = p.src_pts().astype(np.int32)
        cv2.polylines(vis_orig, [pts], True, (0, 255, 0), 2)

        # 이진화 → 3채널
        binary_color = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)

        # 이진화 오버레이 on BEV
        bev_overlay = bev.copy()
        bev_overlay[binary > 0] = (0, 200, 255)

        # 2x2 그리드
        top = np.hstack([vis_orig, bev])
        bot = np.hstack([binary_color, bev_overlay])
        return np.vstack([top, bot])


# ─── 대화형 튜닝 도구 ────────────────────────────────────────────
_TUNE_WIN   = "BEV Tuner"
_BINARY_WIN = "Binary (White Lane)"


def _create_trackbars(p: BEVParams):
    cv2.namedWindow(_TUNE_WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(_TUNE_WIN, 1280, 600)

    def tb(name, val, max_v): cv2.createTrackbar(name, _TUNE_WIN, val, max_v, lambda x: None)

    tb("BotLeft X",  p.src_bot_left_x,  p.img_w)
    tb("BotRight X", p.src_bot_right_x, p.img_w)
    tb("Bot Y",      p.src_bot_y,       p.img_h)
    tb("TopLeft X",  p.src_top_left_x,  p.img_w)
    tb("TopRight X", p.src_top_right_x, p.img_w)
    tb("Top Y",      p.src_top_y,       p.img_h)
    tb("DstMargin",  p.dst_margin,      200)

    cv2.namedWindow("White Thresh", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("White Thresh", 640, 200)
    def tw(name, val, max_v): cv2.createTrackbar(name, "White Thresh", val, max_v, lambda x: None)
    tw("H min", p.white_h_min, 180)
    tw("H max", p.white_h_max, 180)
    tw("S min", p.white_s_min, 255)
    tw("S max", p.white_s_max, 255)
    tw("V min", p.white_v_min, 255)
    tw("V max", p.white_v_max, 255)


def _read_trackbars(p: BEVParams) -> BEVParams:
    def g(win, name): return cv2.getTrackbarPos(name, win)
    p.src_bot_left_x  = g(_TUNE_WIN, "BotLeft X")
    p.src_bot_right_x = g(_TUNE_WIN, "BotRight X")
    p.src_bot_y       = g(_TUNE_WIN, "Bot Y")
    p.src_top_left_x  = g(_TUNE_WIN, "TopLeft X")
    p.src_top_right_x = g(_TUNE_WIN, "TopRight X")
    p.src_top_y       = g(_TUNE_WIN, "Top Y")
    p.dst_margin      = g(_TUNE_WIN, "DstMargin")
    p.white_h_min = g("White Thresh", "H min")
    p.white_h_max = g("White Thresh", "H max")
    p.white_s_min = g("White Thresh", "S min")
    p.white_s_max = g("White Thresh", "S max")
    p.white_v_min = g("White Thresh", "V min")
    p.white_v_max = g("White Thresh", "V max")
    return p


def _print_params(p: BEVParams):
    print("\n─── 현재 파라미터 (코드에 복사하세요) ───")
    print(f"BEVParams(")
    print(f"    src_bot_left_x  = {p.src_bot_left_x},")
    print(f"    src_bot_right_x = {p.src_bot_right_x},")
    print(f"    src_bot_y       = {p.src_bot_y},")
    print(f"    src_top_left_x  = {p.src_top_left_x},")
    print(f"    src_top_right_x = {p.src_top_right_x},")
    print(f"    src_top_y       = {p.src_top_y},")
    print(f"    dst_margin      = {p.dst_margin},")
    print(f"    white_s_max     = {p.white_s_max},")
    print(f"    white_v_min     = {p.white_v_min},")
    print(f")")
    print("─────────────────────────────────────────")


def run_tuner(source):
    """
    source: np.ndarray (단일 이미지) 또는 CameraReceiver 인스턴스
    """
    from camera_receiver import CameraReceiver

    p  = BEVParams()
    pp = LanePreprocessor(p)
    _create_trackbars(p)

    print("조작 키:")
    print("  S  : 현재 파라미터 터미널에 출력")
    print("  R  : 파라미터 초기화")
    print("  ESC / Q : 종료")

    is_live = isinstance(source, CameraReceiver)

    while True:
        # 프레임 가져오기
        if is_live:
            frame = source.get_latest_frame()
            if frame is None:
                cv2.waitKey(30)
                continue
        else:
            frame = source.copy()

        # 트랙바 값 읽기
        _read_trackbars(p)
        pp.update_params(p)

        # 전처리
        try:
            result = pp.preprocess(frame)
            cv2.imshow(_TUNE_WIN, result["debug"])
        except cv2.error:
            pass

        key = cv2.waitKey(30) & 0xFF
        if key in (27, ord("q")):
            break
        elif key == ord("s"):
            _print_params(p)
        elif key == ord("r"):
            p = BEVParams()
            _create_trackbars(p)

    cv2.destroyAllWindows()
    return p


# ─── 단독 실행 ───────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="BEV 파라미터 튜닝 도구")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--image", type=str, help="정지 이미지 파일 경로")
    group.add_argument("--port",  type=int, help="실시간 카메라 UDP 포트")
    parser.add_argument("--ip",   default="127.0.0.1", help="카메라 IP (기본: 127.0.0.1)")
    args = parser.parse_args()

    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            print(f"이미지를 읽을 수 없습니다: {args.image}")
            return
        # 640x480으로 리사이즈
        frame = cv2.resize(frame, (640, 480))
        run_tuner(frame)
    else:
        from camera_receiver import CameraReceiver
        receiver = CameraReceiver(ip=args.ip, port=args.port, show=False)
        receiver.start()
        print(f"카메라 수신 대기 중 ({args.ip}:{args.port})...")
        time.sleep(1.0)
        run_tuner(receiver)
        receiver.stop()


if __name__ == "__main__":
    main()
