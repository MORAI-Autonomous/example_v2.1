# MORAI Sim Control

TCP/UDP를 통해 MORAI 시뮬레이터와 연동하는 Python 클라이언트입니다.

- **`app.py`** — DearPyGUI 기반 GUI 컨트롤 패널
- **`app_cli.py`** — 키보드 인터랙티브 CLI (터미널 전용)
- **`lane_control/`** — 카메라 영상 기반 차선 인식 및 자율주행 컨트롤러

---

## Requirements

- Windows / Linux
- Python 3.10+

| 용도 | 패키지 |
|------|--------|
| GUI (`app.py`) | `dearpygui` |
| 차선 제어 (`lane_control/`) | `opencv-python`, `numpy` |

```bash
pip install -r requirements.txt
```

---

## Project Structure

```
├── app.py                          # GUI 진입점
├── app_cli.py                      # CLI 진입점
│
├── lane_control/                   # 차선 인식 + 자율주행
│   ├── lane_preprocessor.py        # BEV 변환 및 이진화
│   ├── lane_detector.py            # Sliding Window 차선 검출
│   └── lane_controller.py          # PD 조향 + 속도 PI 제어
│
├── transport/                      # TCP/UDP 통신 레이어
│   ├── protocol_defs.py            # 상수, 포맷 문자열, 크기 정의
│   ├── tcp_transport.py            # 패킷 빌드 / 송수신 / 파싱
│   ├── tcp_thread.py               # TCP 수신 스레드
│   └── commands.py                 # UDP 송신 (ManualCommand)
│
├── receivers/                      # UDP 수신기
│   ├── camera_receiver.py          # 카메라 영상 UDP 수신
│   ├── vehicle_info_receiver.py    # VehicleInfo UDP 수신 (포트 9097)
│   ├── vehicle_info_with_wheel_receiver.py  # VehicleInfo + Wheel (포트 9091)
│   └── collision_event_receiver.py # CollisionEvent UDP 수신 (포트 9094)
│
├── automation/
│   └── automation.py               # FixedStep ↔ SaveData 자동 반복 스레드
│
├── panels/                         # GUI 패널 (app.py 전용)
│   ├── commands.py                 # 커맨드 패널
│   ├── monitor.py                  # UDP Monitor 패널
│   └── log.py                      # 로그 패널
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

UDP_IP          = "127.0.0.1"   # ManualCommand 송신 대상
UDP_PORT        = 9090
```

UDP 수신 포트는 각 수신 모듈 상단에서 별도로 설정합니다.

| 모듈 | 상수 | 기본값 |
|------|------|--------|
| `receivers/vehicle_info_receiver.py` | `VEHICLE_INFO_PORT` | `9097` |
| `receivers/vehicle_info_with_wheel_receiver.py` | `VEHICLE_INFO_PORT` | `9091` |
| `receivers/collision_event_receiver.py` | `COLLISION_EVENT_PORT` | `9094` |

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

### BEV 파라미터 튜닝

```bash
python lane_control/lane_preprocessor.py --tune --image <이미지>
python lane_control/lane_preprocessor.py --tune --port 9090
```

### 차선 검출 오프라인 분석

```bash
python lane_control/lane_detector.py --video <녹화파일.mp4>
```

영상 재생 키:

| Key | 동작 |
|-----|------|
| `Space` | 일시정지 / 재생 |
| `D` / `→` | 다음 프레임 |
| `A` / `←` | 이전 프레임 |
| `S` | 현재 프레임 저장 |
| `Q` / `ESC` | 종료 |

---

## TCP Commands

### Simulation Time

| msg_type | 커맨드 | 설명 |
|----------|--------|------|
| `0x1101` | GetSimulationTimeStatus | 현재 Time Mode, step_index, 시뮬레이션 시각 조회 |
| `0x1102` | SetSimulationTimeModeCommand | `VARIABLE(1)` / `FIXED_DELTA(2)` / `FIXED_STEP(3)` 설정. Hz 입력 → fixed_delta(ms) 자동 변환 |

### Fixed Step

| msg_type | 커맨드 | 설명 |
|----------|--------|------|
| `0x1201` | FixedStep | `step_count`만큼 시뮬레이션 tick 진행 |
| `0x1202` | SaveData | 데이터 저장. 경로: `C:\Users\<User>\Documents\MORAI SIM\SimulationRes` |

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
| `0x1401` | ActiveSuiteStatus | 로드된 Suite 이름, 활성 시나리오, 전체 시나리오 목록 조회 |
| `0x1402` | LoadSuite | `.msuite` 파일 경로 지정 후 Suite 로드 |
| `0x1504` | ScenarioStatus | 현재 시나리오 상태 (`PLAY` / `PAUSE` / `STOP`) 조회 |
| `0x1505` | ScenarioControl | `PLAY(1)` / `PAUSE(2)` / `STOP(3)` / `PREV(4)` / `NEXT(5)` 제어 |

---

## UDP

### 수신

| 모듈 | 포트 | 데이터 |
|------|------|--------|
| `vehicle_info_receiver.py` | `9097` | Location, Rotation, Velocity, Acceleration, Angular Vel, Control |
| `vehicle_info_with_wheel_receiver.py` | `9091` | 위 항목 + Wheel 위치 |
| `collision_event_receiver.py` | `9094` | 충돌 entity 정보, 위치/회전/크기/속도 |

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

---

## AutoCaller

`automation/automation.py`의 `AutoCaller`는 `FixedStep → SaveData`를 `MAX_CALL_NUM`회 반복합니다.
각 요청은 `pending` dict의 `threading.Event`로 동기화되며, `AUTO_TIMEOUT_SEC` 초과 시 중단됩니다.

`transport/protocol_defs.py`에서 아래 값을 조정합니다.

```python
MAX_CALL_NUM                = 2000
AUTO_TIMEOUT_SEC            = 2.0
AUTO_DELAY_BETWEEN_CMDS_SEC = 0.0
```
