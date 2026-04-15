# lane_preprocessor.py
# 차선 검출 전처리: ROI → BEV → 흰색/노란색 이진화 → 노이즈 제거
# 실행: python -m lane_control.lane_preprocessor --image <file> | --port <port>

import cv2
import numpy as np
import argparse
import time
from dataclasses import dataclass, field
from typing import Tuple


@dataclass
class BEVParams:
    """BEV 원근변환 + 이진화 파라미터 (해상도 640×480, 높이 1.5m, FOV H=90 기준)"""
    img_w: int = 640
    img_h: int = 480

    # 사다리꼴 src 꼭짓점 [(좌하), (우하), (우상), (좌상)]
    src_bot_left_x:  int = 0
    src_bot_right_x: int = 640
    src_bot_y:       int = 480
    src_top_left_x:  int = 280
    src_top_right_x: int = 362
    src_top_y:       int = 242
    dst_margin:      int = 120   # BEV 좌우 여백 (배리어 영역 차단)

    # 흰색 차선 HSV (S_max 낮춤 → 황색 배리어 S≈150 차단)
    white_h_min: int = 0;   white_h_max: int = 180
    white_s_min: int = 0;   white_s_max: int = 72
    white_v_min: int = 53;  white_v_max: int = 255

    # 노란색 차선 HSV (터널 중앙선, yellow_enable=True 시 활성)
    # 주황 배리어(H≈15, S≈150)와 겹치므로 기본 비활성
    yellow_h_min: int = 15; yellow_h_max: int = 35
    yellow_s_min: int = 80; yellow_s_max: int = 160
    yellow_v_min: int = 63; yellow_enable: bool = False

    # 후처리 노이즈 제거
    bev_top_crop:  int = 80   # BEV 상단 N행 마스킹 (표지판/화살표 오검출 차단, 0=비활성)
    min_blob_area: int = 50   # CC 면적 필터: N픽셀 미만 blob 제거 (0=비활성)

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
        """ROI → BEV → 이진화 → 노이즈제거 → 디버그뷰
        반환: original, roi, bev, binary, debug"""
        p      = self.params
        roi    = self._apply_roi(frame, p)
        bev    = cv2.warpPerspective(roi, p.M(), (p.img_w, p.img_h))
        binary = self._white_threshold(bev, p)
        if p.bev_top_crop > 0:
            binary[:p.bev_top_crop, :] = 0
        if p.min_blob_area > 0:
            binary = self._remove_small_blobs(binary, p.min_blob_area)
        debug = self._make_debug(frame, bev, binary, p)
        return {"original": frame, "roi": roi, "bev": bev,
                "binary": binary, "debug": debug}

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

        # 터널 감지: CLAHE 적용 전 비검정 픽셀 평균 V < 70
        v_raw         = hsv[:, :, 2]
        v_raw_nonblack = v_raw[v_raw > 10]
        mean_v_raw    = float(np.mean(v_raw_nonblack)) if len(v_raw_nonblack) > 100 else 128.0
        is_dark = mean_v_raw < 70

        # CLAHE: 터널 clipLimit 4.0 / 일반 2.0
        clip_limit = 4.0 if is_dark else 2.0
        clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=(8, 8))
        hsv[:, :, 2] = clahe.apply(hsv[:, :, 2])

        # 적응형 V_min: 저채도(흰색계열) 픽셀 p90 기준
        # 터널: p90×0.80 (희미한 차선 포함) / 일반: p90×0.90 (노이즈 억제)
        v_channel  = hsv[:, :, 2]
        s_channel  = hsv[:, :, 1]
        v_nonblack = v_channel[v_channel > 10].flatten()
        _wcand_mask  = (v_channel > 10) & (s_channel <= p.white_s_max)
        v_white_cand = v_channel[_wcand_mask].flatten()
        if len(v_white_cand) > 200:
            p90 = float(np.percentile(v_white_cand, 90))
        elif len(v_nonblack) > 200:
            p90 = float(np.percentile(v_nonblack, 90))
        else:
            p90 = 128.0
        factor     = 0.80 if is_dark else 0.90
        adapt_vmin = int(np.clip(p90 * factor, p.white_v_min, 250))

        # 흰색 차선 마스크
        lower = np.array([p.white_h_min, p.white_s_min, adapt_vmin])
        upper = np.array([p.white_h_max, p.white_s_max, p.white_v_max])
        mask  = cv2.inRange(hsv, lower, upper)

        # 노란색 차선 마스크 (터널 중앙선, yellow_enable=True 시 활성)
        if p.yellow_enable:
            adapt_y_vmin = int(np.clip(adapt_vmin, p.yellow_v_min, 250))
            lower_y = np.array([p.yellow_h_min, p.yellow_s_min, adapt_y_vmin])
            upper_y = np.array([p.yellow_h_max, p.yellow_s_max, p.white_v_max])
            mask    = cv2.bitwise_or(mask, cv2.inRange(hsv, lower_y, upper_y))

        # 배리어 색상 제거 (터널 전용): 적/주황 H=0~20, H=160~180, S≥100 → dilate → 마스크
        if is_dark:
            _bar1 = cv2.inRange(hsv, np.array([0,   100, 80]), np.array([20,  255, 255]))
            _bar2 = cv2.inRange(hsv, np.array([160, 100, 80]), np.array([180, 255, 255]))
            _bar  = cv2.bitwise_or(_bar1, _bar2)
            if cv2.countNonZero(_bar) > 50:
                k_bar = cv2.getStructuringElement(cv2.MORPH_RECT, (25, 25))
                mask  = cv2.bitwise_and(mask, cv2.bitwise_not(cv2.dilate(_bar, k_bar)))

        # dst_margin 바깥 마스킹 (배리어/벽 영역 차단)
        m = p.dst_margin
        if m > 0:
            mask[:, :m] = 0
            mask[:, p.img_w - m:] = 0

        # Morphology: 터널 OPEN 3×3 / 일반 OPEN 5×5, CLOSE 3×3
        k_open_sz = 3 if is_dark else 5
        k_open  = cv2.getStructuringElement(cv2.MORPH_RECT, (k_open_sz, k_open_sz))
        k_close = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        mask    = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k_open)
        mask    = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k_close)
        return mask

    @staticmethod
    def _remove_small_blobs(mask: np.ndarray, min_area: int) -> np.ndarray:
        """Connected-component 면적 필터 — min_area 픽셀 미만 blob 제거."""
        n_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
            mask, connectivity=8)
        cleaned = np.zeros_like(mask)
        for i in range(1, n_labels):          # 0 = 배경 스킵
            if stats[i, cv2.CC_STAT_AREA] >= min_area:
                cleaned[labels == i] = 255
        return cleaned

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
    from receivers.camera_receiver import CameraReceiver

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
        from receivers.camera_receiver import CameraReceiver
        receiver = CameraReceiver(ip=args.ip, port=args.port, show=False)
        receiver.start()
        print(f"카메라 수신 대기 중 ({args.ip}:{args.port})...")
        time.sleep(1.0)
        run_tuner(receiver)
        receiver.stop()


if __name__ == "__main__":
    main()
