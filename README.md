# MORAI Sim Control

TCP/UDP를 통해 MORAI 시뮬레이터를 제어하는 Python 클라이언트입니다.

- `app.py`: DearPyGUI 기반 GUI
- `app_cli.py`: CLI 진입점
- `lane_control/`: 카메라 기반 차선 인식 및 제어
- `autonomous_driving/`: MGeo 경로 기반 자율주행

---

## Requirements

- Windows 10/11 또는 Linux
- Python `3.8+`

| 용도 | 패키지 |
|------|--------|
| GUI | `dearpygui` |
| Lane Control | `opencv-python`, `numpy` |
| 아이콘 변환(선택) | `Pillow` |

```bash
pip install -r requirements.txt
```

---

## Run

### GUI

```bash
python app.py
```

### CLI

```bash
python app_cli.py
```

### Lane Control 단독 실행

```bash
python lane_control/lane_controller.py
python lane_control/lane_controller.py --target-speed 30 --kp-spd 0.05
python lane_control/lane_controller.py --no-speed-ctrl --throttle 0.3
```

---

## Project Structure

```text
app.py
app_cli.py
ad_runner.py
step_ad_runner.py
lane_runner.py

templates/
config/
transport/
receivers/
automation/
panels/
lane_control/
autonomous_driving/
utils/
docs/
```

주요 디렉터리:

- `transport/`: TCP 패킷 빌드/파싱, 수신 스레드
- `receivers/`: UDP 수신기와 템플릿 기반 파서
- `panels/`: DearPyGUI 패널
- `lane_control/`: 차선 인식 + PD/PI 제어
- `autonomous_driving/`: 경로 계획 + Pure Pursuit + ACC
- `config/`: 런타임 상태 저장 파일

---

## GUI Tabs

| 탭 | 설명 |
|----|------|
| UDP Monitor | `.tmpl` 기반 UDP 데이터 모니터 |
| Lane Control | 카메라 기반 차선 제어 |
| Path Follow | 경로 기반 자율주행 |
| File Playback | CSV 기반 Manual Control 재생 |
| Transform Playback | CSV 기반 Transform Control 재생 |

---

## Main Features

### Path Follow

MGeo 또는 CSV 경로를 기반으로 Pure Pursuit와 속도 제어를 수행합니다.

- Fixed 모드: 차량별 `AdRunner`
- Fixed Step 모드: `StepAdRunner`가 전체 차량 관리
- 다중 차량 지원
- 차량별 path, entity id, vehicle info port 설정 가능

### Lane Control

카메라 프레임을 받아 차선을 검출하고 조향/속도 제어를 수행합니다.

- 실시간 파라미터 튜닝
- Vehicle Info 수치 표시
- 디버그 프레임 표시

### File Playback

CSV에서 throttle, brake, steer 값을 읽어 Fixed Step 기반으로 재생합니다.

주요 컬럼:

- `Time [sec]`
- `Acc [0~1]`
- `Brk [0~1]`
- `SWA [deg]`

동작 순서:

1. `ManualControlById` 전송
2. `FixedStep` 전송 및 ACK 대기
3. `SaveData` 전송 및 ACK 대기

### Transform Playback

CSV에서 transform과 speed를 읽어 `TransformControlById`를 순차 전송합니다.

- multi-vehicle 지원, 기본 2대
- 상태 저장: `config/tfp_state.json`
- `FixedStep` 없이 timestamp 간격대로 재생

필수 컬럼:

- `location.x/y/z`
- `rotation.x/y/z`
- `steer angle`
- `local_velocity.x/y`

속도 계산:

```text
speed = sqrt(local_velocity.x^2 + local_velocity.y^2)
```

현재 `Vehicle Info` CSV의 velocity 단위는 `m/s` 기준으로 사용합니다.

---

## Configuration

`transport/protocol_defs.py`에서 기본 네트워크와 자동 호출 관련 값을 설정합니다.

```python
TCP_SERVER_IP = "127.0.0.1"
TCP_SERVER_PORT = 20000

AUTO_TIMEOUT_SEC = 2.0
AUTO_DELAY_BETWEEN_CMDS_SEC = 0.0
MAX_CALL_NUM = 1000
```

---

## TCP Commands

### Simulation Time

| msg_type | command |
|----------|---------|
| `0x1101` | GetSimulationTimeStatus |
| `0x1102` | SetSimulationTimeModeCommand |

### Fixed Step

| msg_type | command |
|----------|---------|
| `0x1201` | FixedStep |
| `0x1202` | SaveData |

### Object Control

| msg_type | command |
|----------|---------|
| `0x1301` | CreateObject |
| `0x1302` | ManualControlById |
| `0x1303` | TransformControlById |
| `0x1304` | SetTrajectory |

현재 `TransformControlById`는 다음 값을 포함합니다.

- position `x, y, z`
- rotation `x, y, z`
- `steer_angle`
- `speed`

### Suite / Scenario

| msg_type | command |
|----------|---------|
| `0x1401` | ActiveSuiteStatus |
| `0x1402` | LoadSuite |
| `0x1504` | ScenarioStatus |
| `0x1505` | ScenarioControl |

---

## UDP

### Template Parser

`receivers/template_parser.py`가 `.tmpl` 파일을 읽어 UDP payload를 파싱합니다.

지원 타입:

- `FLOAT`
- `DOUBLE`
- `INT32`
- `INT64`
- `UINT32`
- `ENUM`
- `STRING`

---

## Notes

- DearPyGUI UI 변경은 메인 스레드에서만 수행해야 하므로, 백그라운드 스레드에서는 `utils.ui_queue.post()`를 사용합니다.
- viewport resize callback에서 직접 레이아웃을 바꾸지 않고, 메인 루프에서 dirty flag 기반으로 반영합니다.
- `config/` 아래 상태 파일은 실행 중 자동 생성될 수 있습니다.
