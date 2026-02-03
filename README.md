# Fixed-Step TCP Controller + Manual UDP Sender (Windows)

Unreal 기반 시뮬레이터(또는 유사 서버)에 대해:

- **TCP**로 Fixed Step / Get Status / Save Data 커맨드를 전송하고
- **UDP**로 Manual Command(Throttle/Brake/Steer)를 헤더 없이 전송하는
간단한 테스트 클라이언트입니다.

키 입력으로 커맨드를 보내고, TCP 응답은 별도 수신 스레드에서 파싱/출력합니다.

---

## Features

### TCP (Control)
- `0x1200` **FixedStepCommand**: step_count 전송 (응답: ResultCode)
- `0x1201` **GetStatusCommand**: payload 없음 (응답: ResultCode + Status)
- `0x1101` **SaveDataCommand**: payload 없음 (응답: ResultCode)

### UDP (Manual)
- 헤더 없음
- payload: `throttle, brake, steer` (`double x 3` = 24 bytes)
- 결과 응답을 기다리지 않음 (fire-and-forget)

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
```

## How To Run
```python example.py```

## Key Bindings

| Key | Action                                                      |
| --- | ----------------------------------------------------------- |
| `1` | Send ManualCommand (UDP, 24B payload)                       |
| `2` | Send GetStatusCommand (TCP, msg_type=0x1201)                |
| `3` | Send FixedStepCommand (TCP, msg_type=0x1200, step_count=10) |
| `4` | Send SaveDataCommand (TCP, msg_type=0x1101)                 |
| `Q` | Quit                                                        |
