# MORAI Sim Control

TCP/UDP를 통해 MORAI 시뮬레이터와 연동하는 Python 클라이언트입니다.

- **`app.py`** — DearPyGUI 기반 GUI 컨트롤 패널
- **`app_cli.py`** — 키보드 인터랙티브 CLI (터미널 전용)
- **`lane_control/`** — 카메라 영상 기반 차선 인식 및 자율주행 컨트롤러
- **`lane_runner.py`** — LaneController + CameraReceiver 통합 실행기 (GUI 연동용)
- **`autonomous_driving/`** — MGeo 경로 기반 자율주행

---

## Requirements

- Windows 10/11 또는 Linux (WSL2 포함)
- Python **3.8+**

| 용도 | 패키지 |
|------|--------|
| GUI (`app.py`) | `dearpygui` |
| 차선 제어 (`lane_control/`) | `opencv-python`, `numpy` |
| 아이콘 변환 (선택) | `Pillow` |

```bash
pip install -r requirements.txt
```

---

## Project Structure

```
├── app.py                          # GUI 진입점
├── app_cli.py                      # CLI 진입점
├── ad_runner.py                    # 자율주행 실행기
│
├── templates/                      # MORAI .tmpl 파일 모음
│   ├── Vehicle Info.tmpl
│   ├── Vehicle Info with wheel.tmpl
│   ├── IMU Template.tmpl
│   ├── GNSS Template.tmpl
│   ├── Collision Event Data.tmpl
│   ├── Detected Object.tmpl
│   ├── Camera Template.tmpl
│   └── ...
│
├── config/                         # 런타임 상태 저장 (자동 생성)
│   ├── monitor_state.json          # 마지막으로 열었던 UDP Monitor 탭 목록
│   └── fp_state.json               # 마지막 File Playback CSV 경로 / Entity ID
│
├── transport/                      # TCP/UDP 통신 레이어
│   ├── protocol_defs.py            # 상수, 포맷 문자열, 크기 정의
│   ├── tcp_transport.py            # 패킷 빌드 / 송수신 / 파싱
│   ├── tcp_thread.py               # TCP 수신 스레드
│   └── commands.py                 # UDP 송신 (ManualCommand)
│
├── receivers/                      # UDP 수신기
│   ├── template_parser.py          # .tmpl 기반 범용 바이너리 파서
│   ├── camera_receiver.py          # 카메라 영상 UDP 수신
│   ├── vehicle_info_receiver.py    # VehicleInfo UDP 수신 (포트 9097)
│   ├── vehicle_info_with_wheel_receiver.py
│   └── collision_event_receiver.py
│
├── automation/
│   └── automation.py               # FixedStep ↔ SaveData 자동 반복 스레드
│
├── lane_runner.py                   # LaneController + CameraReceiver 통합 실행기 (GUI 연동)
│
├── panels/                         # GUI 패널 (app.py 전용)
│   ├── commands.py                 # 커맨드 패널 (Sim / Scenario / Object / FixedStep / File Playback)
│   ├── monitor.py                  # UDP Monitor 패널 (.tmpl 기반 동적 표시)
│   ├── lane_control_panel.py       # Lane Control 탭 (디버그 뷰 + 튜닝 슬라이더 + Vehicle Info)
│   └── log.py                      # 로그 패널
│
├── lane_control/                   # 카메라 기반 차선 인식 + 자율주행
│   ├── lane_preprocessor.py        # BEV 변환, 이진화, 노이즈 필터 (bev_top_crop, min_blob_area)
│   ├── lane_detector.py            # Sliding Window 차선 검출 (search_ratio, min_pixels)
│   └── lane_controller.py          # PD 조향 + 속도 PI 제어, 실시간 파라미터 튜닝
│
├── autonomous_driving/             # MGeo 경로 기반 자율주행
│   ├── autonomous_driving.py
│   ├── vehicle_state.py
│   ├── control/                    # Pure Pursuit, PID
│   ├── localization/               # Path Manager
│   ├── planning/                   # ACC
│   └── mgeo/                       # MGeo 맵 파싱 / 경로 탐색
│
└── utils/
    ├── ui_queue.py                 # 백그라운드 → DPG 안전 업데이트 큐
    ├── key_input.py                # 플랫폼별 raw 키 입력
    └── input_helper.py             # CLI 프롬프트 헬퍼
```

---

## Configuration

`transport/protocol_defs.py` 상단 값을 환경에 맞게 수정합니다.

```python
TCP_SERVER_IP   = "127.0.0.1"
TCP_SERVER_PORT = 20000

AUTO_TIMEOUT_SEC            = 2.0
AUTO_DELAY_BETWEEN_CMDS_SEC = 0.0
MAX_CALL_NUM                = 2000
```

> **WSL2**: mirrored networking이 활성화된 환경(Windows 11 22H2+)에서는 `127.0.0.1` 그대로 사용 가능합니다.

---

## How To Run

### GUI

```bash
python app.py
```

### CLI

```bash
python app_cli.py
```

### 차선 자율주행 컨트롤러

```bash
# 기본 실행 (target 15km/h, 카메라 포트 9090)
python lane_control/lane_controller.py

# 속도 / 게인 조정
python lane_control/lane_controller.py --target-speed 30 --kp-spd 0.05

# 고정 스로틀 모드
python lane_control/lane_controller.py --no-speed-ctrl --throttle 0.3

# 주행 영상 녹화
python lane_control/lane_controller.py --record output.mp4
```

주요 옵션:

| 옵션 | 기본값 | 설명 |
|------|--------|------|
| `--camera-port` | `9090` | 카메라 UDP 수신 포트 |
| `--vi-port` | `9091` | VehicleInfo UDP 수신 포트 |
| `--target-speed` | `15.0` | 목표 속도 (km/h) |
| `--kp` | `0.5` | 조향 PD Kp 게인 |
| `--kd` | `0.08` | 조향 PD Kd 게인 |
| `--kp-spd` | `0.05` | 속도 PI Kp 게인 |
| `--no-speed-ctrl` | — | 고정 스로틀 모드 |
| `--throttle` | `0.3` | 고정 스로틀 값 |
| `--record` | — | 디버그 영상 저장 경로 (.mp4) |

GUI에서 실행할 경우 `app.py` 우측 탭바의 **Lane Control 탭**을 사용합니다. 포트, 게인, 노이즈 필터 파라미터를 실시간으로 조정할 수 있습니다.

#### 노이즈 필터 파라미터

차선 인식 오검출(터널·그림자·합류 구간) 발생 시 다음 파라미터를 조정합니다:

| 파라미터 | 적용 대상 | 효과 |
|----------|----------|------|
| `bev_top_crop` | `BEVParams` | BEV 바이너리 상단 행 마스킹 — 터널 천장·원거리 표지판 제거 |
| `min_blob_area` | `BEVParams` | 소면적 연결 성분 제거 — 그림자·반사광 산점 노이즈 제거 |
| `search_ratio` | `LaneDetector` | 히스토그램 탐색 범위 축소 — 원거리 노이즈 피크 억제 |
| `min_pixels` | `LaneDetector` | 슬라이딩 윈도우 최소 픽셀 임계값 상향 — 희미한 노이즈 무시 |

---

## GUI 주요 기능

### Lane Control 탭

카메라 영상을 수신하여 차선을 인식하고 자율주행을 수행하는 전용 탭입니다.

**레이아웃:**

| 섹션 | 내용 |
|------|------|
| CONTROL | ▶ Start / ■ Stop |
| TARGET VEHICLE | Entity ID, 속도 제어 On/Off, 목표 속도, Invert Steer |
| INTERFACE | Vehicle Info 수신 포트, 카메라 수신 포트 |
| TUNING | 실시간 PD 게인 / 노이즈 필터 슬라이더 + Reset 버튼 |
| LIVE VIEW | 디버그 합성 영상 (640×240) + Vehicle Info 수치 |

**TUNING 슬라이더:**

| 슬라이더 | 범위 | 기본값 | 설명 |
|----------|------|--------|------|
| Kp | 0.0 – 3.0 | 0.50 | 조향 PD 비례 게인 |
| Kd | 0.0 – 1.0 | 0.10 | 조향 PD 미분 게인 |
| EMA α | 0.01 – 1.0 | 0.30 | 조향값 EMA 스무딩 계수 |
| Steer Rate | 0.01 – 0.5 | 0.15 | 최대 조향 변화율 (rad/step) |
| Offset Clip | 0.1 – 3.0 | 1.50 | 차선 오프셋 클리핑 범위 |
| Target Spd | 1.0 – 100.0 | 15.0 | 목표 속도 (km/h) |
| BEV Top Crop | 0 – 240 | 80 | BEV 바이너리 상단 N행 마스킹 — 터널 천장/원경 노이즈 제거 |
| Min Blob | 0 – 500 | 50 | N픽셀 미만 연결 성분(blob) 제거 — 산점 노이즈 제거 |
| Search Ratio | 0.1 – 1.0 | 0.50 | 히스토그램 피크 탐색에 사용할 이미지 하단 비율 |
| Min Pixels | 1 – 200 | 30 | 슬라이딩 윈도우 유효 인식 최소 픽셀 수 |

`↺ Reset Defaults` 버튼으로 전체 슬라이더를 초기값으로 일괄 복원합니다.

**LIVE VIEW:**

- 좌측: 원본 / BEV / 바이너리 / 조향 게이지를 합성한 1280×480 디버그 영상을 640×240으로 표시 (30fps)
- 우측: 속도, 위치(X/Y/Z), Yaw, 속도 벡터(Vx/Vy) 실시간 수치

---

### UDP Monitor

`templates/` 폴더의 `.tmpl` 파일을 읽어 UDP 데이터를 동적으로 표시합니다.

- 템플릿 목록에서 항목 선택 후 `▶ Open` → 새 탭으로 열림
- 탭마다 IP / Port / Start / Stop 개별 설정
- xyz / xyzw 연속 필드는 자동으로 한 줄에 묶어 표시
- Repeat 섹션(가변 항목 배열)은 멀티라인 텍스트로 표시
- FLOAT: 소수점 4자리 / DOUBLE: 소수점 6자리 표시, 극단값은 지수 표기
- 열었던 탭 목록과 IP/Port 설정은 재시작 후에도 유지 (`config/monitor_state.json`)

### File Playback (Fixed Step Mode)

CSV 파일의 제어 값을 읽어 시뮬레이터에 FixedStep 단위로 재생합니다.

**CSV 형식:**

| 컬럼 | 설명 |
|------|------|
| `Time [sec]` | 시간 (참고용) |
| `Acc [0~1]` | Throttle 값 |
| `Brk [0~1]` | Brake 값 |
| `SWA [deg]` | Steer Wheel Angle |

**동작 순서 (행마다 반복):**

```
ManualControlById 전송 (fire-and-forget)
    → FixedStep 전송 + ACK 대기
    → SaveData 전송 + ACK 대기
    → 다음 행
```

마지막으로 사용한 CSV 경로 / Entity ID는 재시작 후에도 복원됩니다 (`config/fp_state.json`).

### AutoCaller

`FixedStep → SaveData` 를 지정 횟수만큼 자동 반복합니다.

```python
# transport/protocol_defs.py
MAX_CALL_NUM                = 2000
AUTO_TIMEOUT_SEC            = 2.0
AUTO_DELAY_BETWEEN_CMDS_SEC = 0.0
```

---

## TCP Commands

### Simulation Time

| msg_type | 커맨드 | 설명 |
|----------|--------|------|
| `0x1101` | GetSimulationTimeStatus | Time Mode, step_index, 시뮬레이션 시각 조회 |
| `0x1102` | SetSimulationTimeModeCommand | `VARIABLE(1)` / `FIXED_DELTA(2)` / `FIXED_STEP(3)` 설정. Hz 입력 → fixed_delta(ms) 자동 변환 |

### Fixed Step

| msg_type | 커맨드 | 설명 |
|----------|--------|------|
| `0x1201` | FixedStep | `step_count`만큼 시뮬레이션 tick 진행 |
| `0x1202` | SaveData | 데이터 저장 (`Documents/MORAI SIM/SimulationRes`) |

### Object Control

| msg_type | 커맨드 | 설명 |
|----------|--------|------|
| `0x1301` | CreateObject | entity_type, 위치/회전, driving_mode, 차량 모델 지정 후 생성 |
| `0x1302` | ManualControlById | entity_id 지정, throttle / brake / steer_angle 전송 |
| `0x1303` | TransformControlById | entity_id 지정, 위치/회전/steer_angle 직접 설정 |
| `0x1304` | SetTrajectory | entity_id, follow_mode, trajectory_name, waypoint 배열 전송 |

### Suite / Scenario

| msg_type | 커맨드 | 설명 |
|----------|--------|------|
| `0x1401` | ActiveSuiteStatus | Suite 이름, 활성 시나리오, 전체 시나리오 목록 조회 |
| `0x1402` | LoadSuite | `.msuite` 파일 경로 지정 후 Suite 로드 |
| `0x1504` | ScenarioStatus | 현재 시나리오 상태 (`PLAY` / `PAUSE` / `STOP`) 조회 |
| `0x1505` | ScenarioControl | `PLAY(1)` / `PAUSE(2)` / `STOP(3)` / `PREV(4)` / `NEXT(5)` 제어 |

---

## UDP

### 수신 — Template Parser

`receivers/template_parser.py` 가 `.tmpl` JSON을 읽어 바이너리 패킷을 파싱합니다.

지원 타입: `FLOAT`, `DOUBLE`, `INT32`, `INT64`, `UINT32`, `ENUM`, `STRING`

| 섹션 | 설명 |
|------|------|
| `FIELDS` | 고정 헤더 필드 (1회 파싱) |
| `REPEAT` | 가변 반복 배열 (count 필드 기반 자동 감지) |

수신 포트는 UDP Monitor 탭의 Port 입력란에서 탭마다 개별 설정합니다.

### 송신

| 포트 | 설명 | Payload |
|------|------|---------|
| `9090` | ManualCommand — throttle, brake, steer | `<ddd` (24 bytes) |

---

## Protocol

### TCP Header (`<BBIIIH`, 16 bytes)

| Field | Type | Size |
|-------|------|------|
| magic_number (`0x4D`) | uint8 | 1 |
| msg_class (`0x01`=REQ / `0x02`=RESP) | uint8 | 1 |
| msg_type | uint32 | 4 |
| payload_size | uint32 | 4 |
| request_id | uint32 | 4 |
| flag | uint16 | 2 |

수신 측은 `0x4D` MAGIC 바이트 기반으로 스트림 동기화(resync)를 수행합니다.

### ResultCode

| result_code | 의미 |
|-------------|------|
| 0 | OK |
| 101 | Invalid State |
| 102 | Invalid Param |
| 200 | Failed |
| 201 | Timeout |
| 202 | Not Supported |

### 가변 길이 문자열

TCP payload 내 문자열은 `uint32 length + UTF-8 bytes` 형식입니다.
