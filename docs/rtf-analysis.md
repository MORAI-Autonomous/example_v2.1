# Fixed_Step RTF 분석 결과 및 클라이언트 개선 방향

> 서버(UnrealMoraiSimulator) 측 분석 완료 기준: 2026-04-22
> 관련 브랜치: `feature/hwjung/msvm-2679_fixed_step_rtf`

---

## 측정 결과 (빈맵 / 차량 2대 / 센서 없음)

| Hz | Fixed RTF | Fixed_Step RTF |
|----|-----------|----------------|
| 30 | **1.004** | **0.33** |
| 60 | **0.77**  | **0.30** |

**측정 기준:**
- Fixed_Step RTF = `TotalSimTimeNs / 실경과시간` (누적, 대기시간 포함)
- avg/min/max = 배치 완료 → 다음 배치 완료 기준 (대기시간 포함, Fixed 모드와 동일 기준)

---

## 원인 분석

### Fixed 모드
- 30Hz: RTF ≈ 1.0 → 엔진이 실시간을 충분히 따라감
- 60Hz: RTF ≈ 0.77 → 차량 Dynamics(Chaos 물리) 연산이 16.7ms 내에 완료되지 않음 (서버 수정 불가 영역)

### Fixed_Step 모드
- Hz와 무관하게 RTF ≈ 0.3 수준으로 수렴
- 엔진 처리 자체는 문제 없음 → **병목은 클라이언트-서버 사이클 구간**

```
30Hz 기준 사이클 분해:
  전체 사이클  = 33ms / 0.33 = ~100ms
  엔진 처리    = 33ms / 1.0  =  ~33ms  (Fixed RTF 기준)
  클라이언트 대기 = 100 - 33  =  ~67ms  ← 여기가 병목
```

### 서버 측 검토 완료 항목 (모두 병목 아님으로 결론)

| 항목 | 측정 결과 |
|------|-----------|
| `SetGamePaused()` × 2 per step | ≈ 0ms |
| `WaitPhysScenes()` 물리 동기화 | ≈ 0ms |
| `UpdateFixedStepModeStatusToDataModel()` | scalar 6개, 무시 가능 |
| `AsyncTask` 래퍼 (제거 완료) | 오버헤드 제거됨 |

**결론: 서버 엔진 측에서 개선 가능한 병목 없음**

---

## 클라이언트 측 개선 방향

### 병목 구간
```
[서버 스텝 완료]
      ↓ 응답 전송
[네트워크 전송]         ← RTT 절반
      ↓
[클라이언트 응답 수신]
      ↓ 처리 (데이터 파싱, 콜백, 다음 커맨드 준비)
[클라이언트 커맨드 송신] ← 여기까지가 ~67ms
      ↓
[네트워크 전송]         ← RTT 절반
      ↓
[서버 커맨드 수신 → 다음 스텝 시작]
```

### 개선 옵션

#### A. 파이프라이닝 (권장)
- 서버 응답을 기다리지 않고 다음 `SetFixedStep` 커맨드를 미리 송신
- 네트워크 왕복 대기 시간을 오버랩
- 주의: 서버 `PendingStepQueue`(MPSC)는 다중 커맨드를 수용할 수 있음

#### B. step_count 배치 증가
- `SetFixedStepCommand.step_count` > 1로 설정해 한 번의 왕복에 여러 스텝 처리
- 예: step_count=3 → 네트워크 오버헤드가 3스텝에 분산
- 단점: 스텝 완료 피드백 빈도 감소

#### C. 클라이언트 처리 최적화
- 응답 수신 → 다음 커맨드 송신 사이의 처리 시간 프로파일링
- 콜백 체인, 데이터 파싱 등 지연 요인 확인

---

## 서버 API 참고

### SetFixedStepCommand
```protobuf
// step_count: 한 번에 처리할 스텝 수 (기본 1)
// 서버는 step_count 만큼 연속 틱 후 응답
message SetFixedStepCommand {
    int32 step_count = 1;
}
```

### 서버 처리 흐름
```
클라이언트 송신
  → PendingStepQueue (MPSC, thread-safe)
  → Tick() 소비 → SetStep() → World Unpause
  → OnWorldPostActorTick (step_count 소진 시)
  → World Pause → OnCompleted 응답
```

### 관련 서버 파일
- `Source/MoraiSimulator/Private/System/SimulationTimeModeSubsystem.cpp`
- `Source/MoraiSimulator/Private/NetworkInterface/Commands/SetFixedStepCommand.cpp`
