# MORAI Sim Control

MORAI 시뮬레이터 TCP/UDP 제어 클라이언트. Python 3.8 · dearpygui · opencv-python · numpy

## Run
```
pip install -r requirements.txt
python app.py       # GUI
python app_cli.py   # CLI
```

## Structure
```
app.py / app_cli.py       진입점
transport/                TCP/UDP 통신
receivers/                UDP 수신기
panels/                   DearPyGUI 패널 (commands, monitor, lane_control_panel, log)
lane_control/             차선 인식 + PD/PI 제어
autonomous_driving/       MGeo 경로 기반 자율주행
lane_runner.py            LaneController GUI 래퍼
ad_runner.py              AutonomousDriving GUI 래퍼 (다중 차량: ad_runners list)
config/                   런타임 상태 자동 저장 (없어도 정상 시작)
```

## Critical Rules

| 규칙 | 내용 |
|------|------|
| Python 3.8 타입힌트 | 파일 최상단 `from __future__ import annotations` 필수 |
| Edit 전 Read | old_string 불일치 시 즉시 실패 — 항상 먼저 Read |
| DearPyGUI 스레드 | 백그라운드에서 UI 변경 시 `ui_queue.post(fn)` 필수 |
| DearPyGUI 탭바 | `add_tab_bar` 너비 버그 — 버튼+show/hide 방식 사용 (app.py 참고) |
| DearPyGUI 아이콘 | `↺` 등 유니코드 깨짐 — ASCII 텍스트만 사용 |
| 패널 의존성 | 패널은 app.py import 금지 — `init(callback)` 주입 패턴 |
| 큰 파일 읽기 | Grep으로 위치 확인 후 offset+limit으로 부분 Read |

## Detailed Docs
- `docs/architecture.md` — 패턴 상세 (ui_queue, Runner, 콜백 주입, config)
- `docs/dearpygui.md`    — DearPyGUI 규칙 상세 (탭바, 텍스처, 테마)
- `docs/workflow.md`     — 개발 워크플로, 반복 실수 목록
