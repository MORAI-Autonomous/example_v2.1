# Fixed-Step Mode Control Example

- **TCP**로 모라이 시뮬레이터의 Fixed Step Mode 를 제어 할 수 있습니다. 
- **UDP**로 모라이 시뮬레이터에 설정한 사용자 Interface와 연결합니다. 
- **UDP**로 Vehicle Info 데이터를 수신할 수 있습니다.

키 입력으로 커맨드를 보내고, TCP 응답은 별도 수신 스레드에서 파싱/출력합니다.

---

## Features

### TCP (Control)
- `0x1200` **Fixed Step Command**: step_count 전송 (응답: ResultCode) : step count 만큼 tick을 이동합니다. 
- `0x1201` **Get Fixed Step Mode Status**: payload 없음 (응답: ResultCode + Status) : 현재 Time Mode Status 를 요청 합니다. 
- `0x1101` **SaveData Command**: payload 없음 (응답: ResultCode) : 데이타 저장 요청 
- 저장 경로는 윈도우 기준
C:\Users\<User>\Documents\MORAI SIM\SimulationRes

### UDP (Manual Control)
- 헤더 없음
- payload: `throttle, brake, steer` (`double x 3` = 24 bytes)
- 결과 응답을 기다리지 않음 (fire-and-forget)

### UDP (Vehicle Info)
- 헤더 없음
- 포맷: 바이너리 데이터 (Little Endian, 108 bytes)
- 구조: `int64(sec), int32(nanos), char[24](id), float32[18](sensors/control)`
- 포함 데이터: 위치(XYZ), 회전(RPY), 선속도, 가속도, 각속도, 제어값(Thr, Brk, Str)
- Size: 108 bytes

| Field | Type | Size |
|---|---:|---:|
| seconds | int64 | 8 |
| nanos | int32 | 4 |
| id | char[24] (null-terminated utf-8) | 24 |
| location | float32 x3 | 12 |
| rotation | float32 x3 | 12 |
| local_velocity | float32 x3 | 12 |
| local_acceleration | float32 x3 | 12 |
| angular_velocity | float32 x3 | 12 |
| control(throttle, brake, steer_angle) | float32 x3 | 12 |

### 기타
- TCP 수신은 **MAGIC(0x4D)** 기반으로 헤더 동기화(resync)
- `request_id` 기반 RTT(ms) 출력

---

## Protocol Summary

### TCP Header
- Struct: `<BBIIIH` (Little Endian)
- Size: **16 bytes**

| Field | Type | Size |
|---|---:|---:|
| magic_number (MAGIC) | uint8 | 1 |
| msg_class | uint8 | 1 |
| msg_type | uint32 | 4 |
| payload_size | uint32 | 4 |
| request_id | uint32 | 4 |
| flag | uint16 | 2 |

### ResultCode
- Struct: `<II` (8 bytes)
- `result_code`, `detail_code`

### GetStatus Response Payload
- ResultCode(8B) + Status(24B) = **32 bytes**
- Status Struct: `<fQqI`

| Field | Type |
|---|---|
| fixed_delta | float32 |
| step_index | uint64 |
| seconds | int64 |
| nanos | uint32 |

### Manual UDP Payload
- Struct: `<ddd`
- Size: **24 bytes**
- `throttle`, `brake`, `steer`

---

## Requirements

- Windows
- Python 3.8+
- Standard Library only (no external dependencies)

---

## Configuration

Edit the following values at the top of the script if needed:

```python
TCP_SERVER_IP = "127.0.0.1"
TCP_SERVER_PORT = 9093

UDP_IP = "127.0.0.1"
UDP_PORT = 9090

VEHICLE_INFO_PORT = 9092
```

## How To Run
```python example.py```

## Key Bindings

| Key | Action                                                      |
| --- | ----------------------------------------------------------- |
| `1` | Send ManualCommand (UDP, 24B payload)                       |
| `2` | Send GetStatusCommand (TCP, msg_type=0x1201)                |
| `3` | Send FixedStepCommand (TCP, msg_type=0x1200, step_count=1) |
| `4` | Send SaveDataCommand (TCP, msg_type=0x1101)                 |
| `Q` | Quit                                                        |


## Example Suite
- 예제 Suite가 추가 되었습니다.