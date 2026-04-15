from __future__ import annotations
# lane_control/lane_detector_cli.py
#
# Sliding Window 차선 검출 단독 실행 스크립트 (디버그/확인용)
#
# 실행:
#   python -m lane_control.lane_detector_cli --image frame.png
#   python -m lane_control.lane_detector_cli --port 9090
#   python -m lane_control.lane_detector_cli --video output.mp4

import argparse
import time

import cv2
import numpy as np

from lane_control.lane_preprocessor import LanePreprocessor
from lane_control.lane_detector import LaneDetector


def main():
    parser = argparse.ArgumentParser(description="Sliding Window 차선 검출 확인")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--image", type=str, help="정지 이미지 파일")
    group.add_argument("--port",  type=int, help="카메라 UDP 포트")
    group.add_argument("--video", type=str, help="영상 파일 오프라인 재분석 (.mp4 등)")
    parser.add_argument("--ip", default="127.0.0.1")
    args = parser.parse_args()

    preprocessor = LanePreprocessor()
    detector     = LaneDetector()

    def process(frame):
        result_pre = preprocessor.preprocess(frame)
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
        if lh != right_col.shape[0]:
            right_col = cv2.resize(right_col, (bev_w, lh))
        cv2.imshow("Lane Detection", np.hstack([result_pre["debug"], right_col]))

    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            print(f"이미지를 읽을 수 없습니다: {args.image}")
            return
        frame = cv2.resize(frame, (640, 480))
        while True:
            process(frame)
            if cv2.waitKey(30) & 0xFF in (27, ord("q")):
                break

    elif args.video:
        cap = cv2.VideoCapture(args.video)
        if not cap.isOpened():
            print(f"영상을 열 수 없습니다: {args.video}")
            return

        total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        fps   = cap.get(cv2.CAP_PROP_FPS) or 30.0
        delay = max(1, int(1000.0 / fps))
        paused, frame_idx, cur_frame, save_cnt = False, 0, None, 0

        print(f"[Video] {args.video}  {total}f  {fps:.1f}fps")
        print("  Space:일시정지  A/D:프레임이동  S:저장  Q:종료")

        def _seek(idx):
            cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
            ok, f = cap.read()
            return cv2.resize(f, (640, 480)) if ok else None

        while True:
            if not paused:
                ok, raw = cap.read()
                if not ok:
                    paused = True
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
                    vs = int(val * (bev_h // 2) / max(hist.max(), 1))
                    cv2.line(hist_img, (x, bev_h // 2), (x, bev_h // 2 - vs), (0, 255, 100), 1)
                cv2.putText(hist_img, "Histogram", (5, 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
                right_col = np.vstack([result_det.viz, hist_img])
                lh = result_pre["debug"].shape[0]
                if lh != right_col.shape[0]:
                    right_col = cv2.resize(right_col, (bev_w, lh))
                combined = np.hstack([result_pre["debug"], right_col])
                pct  = frame_idx / max(total - 1, 1) * 100
                cv2.putText(combined, f"Frame {frame_idx}/{total-1} ({pct:.1f}%)",
                            (10, combined.shape[0]-30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220,220,80), 1)
                cv2.putText(combined, "|| PAUSE" if paused else "PLAY",
                            (10, combined.shape[0]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (80,80,220) if paused else (80,220,80), 1)
                bx0, bx1, by = 10, combined.shape[1]-10, combined.shape[0]-5
                cv2.line(combined, (bx0, by), (bx1, by), (80,80,80), 3)
                px = int(bx0 + (bx1-bx0) * frame_idx / max(total-1, 1))
                cv2.circle(combined, (px, by), 5, (0,200,255), -1)
                cv2.imshow("Lane Detection", combined)

            key = cv2.waitKey(0 if paused else delay) & 0xFF
            if key in (27, ord("q")):
                break
            elif key == ord(" "):
                paused = not paused
            elif key in (ord("a"), 81):
                cur_frame = _seek(max(0, frame_idx - 1))
                frame_idx = max(0, frame_idx - 1)
                paused = True
            elif key in (ord("d"), 83):
                cur_frame = _seek(min(total-1, frame_idx + 1))
                frame_idx = min(total-1, frame_idx + 1)
                paused = True
            elif key == ord("s") and cur_frame is not None:
                save_cnt += 1
                fname = f"frame_{frame_idx:06d}_{save_cnt:03d}.png"
                cv2.imwrite(fname, combined)
                print(f"[Video] 저장: {fname}")
        cap.release()

    else:
        from receivers.camera_receiver import CameraReceiver
        receiver = CameraReceiver(ip=args.ip, port=args.port, show=False)
        receiver.start()
        print("차선 검출 중... Q/ESC 종료")
        while True:
            frame = receiver.get_latest_frame()
            if frame is not None:
                process(frame)
            if cv2.waitKey(30) & 0xFF in (27, ord("q")):
                break
        receiver.stop()

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
