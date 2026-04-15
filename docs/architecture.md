# Architecture Patterns

## ui_queue — 스레드 안전 UI 업데이트

DearPyGUI는 메인 스레드에서만 API를 호출할 수 있다.
백그라운드 스레드(Runner, 수신 스레드 등)에서 UI를 변경할 때는 반드시 `ui_queue.post()` 를 사용한다.

```python
# ❌ 백그라운드 스레드에서 직접 호출 — 크래시 또는 undefined behavior
dpg.set_value("tag", value)

# ✅ 올바른 방법
import utils.ui_queue as ui_queue
ui_queue.post(lambda: dpg.set_value("tag", value))
```

`ui_queue.drain()` 은 app.py 메인 루프(`while dpg.is_dearpygui_running()`)에서 매 프레임 호출된다.

---

## 콜백 주입 패턴 — 패널 초기화

패널 모듈은 `app.py` 를 직접 import하지 않는다.
순환 참조 방지 및 테스트 용이성을 위해 `init()` 으로 콜백을 주입받는다.

```python
# panels/some_panel.py
_start_fn = None
_stop_fn  = None

def init(start_fn, stop_fn):
    global _start_fn, _stop_fn
    _start_fn = start_fn
    _stop_fn  = stop_fn

# app.py
some_panel.init(start_fn=state.start_something, stop_fn=state.stop_something)
```

---

## Runner 패턴

`LaneRunner`, `AdRunner` 는 각각 독립 스레드로 동작한다.
`app.py` 의 `AppState` 가 인스턴스를 소유하고 생명주기를 관리한다.

| Runner | 소유 필드 | 비고 |
|--------|-----------|------|
| `LaneRunner` | `self.lc_runner` | 단일 인스턴스 |
| `AdRunner` | `self.ad_runners: list` | 다중 차량, 차량별 1개 |

```python
# 시작
runner = AdRunner(tcp_sock=..., entity_id=..., vi_port=..., ...)
runner.start()
self.ad_runners.append(runner)

# 종료
for r in self.ad_runners:
    r.stop()
self.ad_runners.clear()
```

---

## config/ 상태 저장

`config/fp_state.json`, `config/monitor_state.json` 은 앱이 자동 생성한다.

- 파일 없음 → 조용히 기본값으로 시작
- 저장 실패 → `print` 후 앱 계속 동작 (예외 전파 없음)
- 폴더 없음 → `os.makedirs(..., exist_ok=True)` 로 자동 생성

---

## lane_control/ 모듈 구조

```
lane_preprocessor.py   BEV 변환, 이진화, 노이즈 필터 (BEVParams 참조)
lane_detector.py       Sliding Window 검출 (search_ratio, min_pixels 참조)
controllers.py         EMAFilter, PDController, SpeedPIController
vehicle_info.py        VehicleInfoThread (UDP 수신, 파싱)
tune_panel.py          TunePanel (OpenCV 키보드 튜닝 창, --tuning 플래그)
lane_controller.py     LaneController (메인 제어 루프, update_params)
```

`LaneController.update_params(**kwargs)` 로 실행 중 파라미터 실시간 변경 가능:
`kp`, `kd`, `ema_alpha`, `steer_rate`, `offset_clip`, `invert_steer`, `target_kmh`,
`bev_top_crop`, `min_blob_area`, `search_ratio`, `min_pixels`
