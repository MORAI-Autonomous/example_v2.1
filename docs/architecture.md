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

# app.py (connect() 내부)
some_panel.init(start_fn=state.start_something, stop_fn=state.stop_something)
```

---

## Runner 패턴

Runner는 독립 스레드로 동작하며, `app.py`의 `AppState`가 인스턴스를 소유하고 생명주기를 관리한다.

| Runner | 소유 필드 | 모드 | 비고 |
|--------|-----------|------|------|
| `LaneRunner` | `self.lc_runner` | Fixed | 단일 인스턴스 |
| `AdRunner` | `self.ad_runners: list` | Fixed | 차량당 1개, 다중 인스턴스 |
| `StepAdRunner` | `self.step_ad_runners: list` | Fixed Step | 단일 인스턴스로 전체 차량 관리 |

```python
# Fixed 모드 — 차량별 AdRunner
for v in vehicles:
    runner = AdRunner(tcp_sock=..., entity_id=v["entity_id"], vi_port=v["vi_port"], ...)
    runner.start()
    self.ad_runners.append(runner)

# Fixed Step 모드 — StepAdRunner 하나가 전체 차량 순환 제어
runner = StepAdRunner(tcp_sock=..., vehicles=vehicles, ...)
runner.start()
self.step_ad_runners.append(runner)
```

---

## status_cb 패턴 — 실시간 상태 UI 표시

매 tick마다 로그를 append하면 UI 텍스트 누적 + DPG 재빌드 오버헤드가 크다.
Runner에서 UI 상태를 갱신할 때는 `status_cb`를 주입받아 `dpg.set_value`로 직접 업데이트한다.

```python
# Runner 생성 시 콜백 주입
runner = AdRunner(
    ...,
    status_cb=au_panel.update_status,   # (entity_id, x, y, vel_kmh, accel, brake, steer) → None
)

# Runner 내부 — 매 tick
if self._status_cb:
    self._status_cb(entity_id, x, y, vel * 3.6, accel, brake, steer_n)

# panels/autonomous_panel.py — ui_queue 경유, 5개 set_value 한 번에
def update_status(entity_id, x, y, vel_kmh, accel, brake, steer):
    slot = _entity_slot.get(entity_id)
    if slot is None:
        return
    def _apply(...):
        pfx = f"au_sv{slot}_"
        if not dpg.does_item_exist(pfx + "pos"):
            return
        dpg.set_value(pfx + "pos",   f"({x:.1f}, {y:.1f})")
        dpg.set_value(pfx + "vel",   f"{vel_kmh:.1f} km/h")
        ...
    ui_queue.post(_apply)
```

**핵심:** `log.append` (텍스트 증가 + DPG 재빌드) 대신 `set_value` (값 교체) 를 사용해
tick당 UI 오버헤드를 대폭 줄인다.

---

## 동적 차량 목록 — `_build_vehicles`

`autonomous_panel.py`에서 차량 수를 런타임에 추가/삭제할 때 DPG 아이템을 동적으로 재생성한다.

```python
# 컨테이너 그룹을 미리 만들어 두고
dpg.add_group(tag="au_vehicles_area")
_build_vehicles(2)   # 기본 2대

# 수 변경 시 자식만 삭제하고 재생성
def _build_vehicles(count: int) -> None:
    dpg.delete_item("au_vehicles_area", children_only=True)
    for i in range(1, count + 1):
        with dpg.group(tag=f"au_vehicle_group_{i}", parent="au_vehicles_area"):
            dpg.add_input_text(tag=f"au_path_{i}", ...)
            dpg.add_input_text(tag=f"au_entity_id_{i}", ...)
            dpg.add_input_int(tag=f"au_vi_port_{i}", ...)
            dpg.add_text("-", tag=f"au_sv{i}_pos", ...)
            ...
```

**주의:** `delete_item(children_only=True)` 후 재생성 시 반드시 `with dpg.group(parent=...)` 컨텍스트 안에서
아이템을 추가해야 DPG 부모 컨텍스트 스택이 올바르게 유지된다.

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

---

## autonomous_driving/ 성능 최적화

### PathManager — 윈도우 탐색 캐시

이전 구현은 매 tick 전체 경로를 O(n) 순회했다.
`_last_wp` 캐시를 도입해 ±5 뒤 / +100 앞 범위만 탐색한다.

```python
# localization/path_manager.py
BACK, FRONT = 5, 100
for offset in range(-BACK, FRONT + 1):
    i = (self._last_wp + offset) % n   # closed path
    ...
self._last_wp = current_waypoint
```

경로 길이에 무관하게 매 tick O(106) 으로 고정된다.

### PurePursuit — lookahead 인덱스 캐시

`_last_lfd_idx` 에서 전방 탐색을 시작하고, 실패 시 인덱스 0부터 재탐색(fallback)한다.

```python
# control/pure_pursuit.py
for attempt in range(2):
    start = self._last_lfd_idx if attempt == 0 else 0
    for i in range(start, n):
        if dis >= lfd:
            self._last_lfd_idx = i
            return steering_angle
```

경로 setter 에서 `_last_lfd_idx = 0` 으로 리셋한다.
---

## Transform Playback Notes

`Transform Playback` panel??CSV 湲곕컲 `TransformControlById` ?ъ깮 ?꾩슜 panel?대떎.

- panel init pattern: `panels.transform_playback_panel.init(start_tfp_fn, stop_tfp_fn)`
- state file: `config/tfp_state.json`
- default vehicle count: `2`
- per-vehicle settings: `path`, `entity_id`

### CSV parse

`panels/transform_playback_panel.py::_load_csv()`? ?꾩쓬 媛믪쓣 row dict濡??뚯떛?쒕떎.

- `time_sec`
- `pos_x`, `pos_y`, `pos_z`
- `rot_x`, `rot_y`, `rot_z`
- `steer_angle`
- `speed`

`speed`? `local_velocity.x`, `local_velocity.y`瑜??ъ슜??`sqrt(x^2 + y^2)` 濡?怨꾩궛?쒕떎. `Vehicle Info` CSV??velocity??`m/s`濡???ν븯誘濡?`speed`??`m/s`濡??꾨쭏?섎룄濡??묎렐?쒕떎.

### Runtime flow

`AppState.start_tfp()`?먯꽌 `AutoCaller`瑜??ъ슜?섎굹, `Transform Playback`? `FixedStep` / `SaveData` 瑜??ъ슜?섏? ?딅뒗??`TransformControlById`瑜??쒖감 ?꾩넚?쒕떎.

```python
for i in range(total_rows):
    for vehicle in vehicles:
        row = vehicle["rows"][i]
        tcp.send_transform_control_by_id(..., speed=row["speed"])

    sleep(next_time_sec - current_time_sec)
```

CSV ?쒓컙 而щ읆?대? ?덉쑝硫?timestamp 媛꾧꺽?쇰줈 ?ъ깮?섍퀬, ?놁쑝硫?fallback delay瑜??ъ슜?쒕떎.

### Transport coupling

`TransformControlById` payload??`position`, `rotation`, `steer_angle`, `speed`瑜??ы븿?쒕떎. protocol?대? 諛붾뀌硫?`transport/protocol_defs.py`, `transport/tcp_transport.py`, `templates/TransformControl.tmpl`, panel CSV parser瑜?媛숈씠 ?묎렐?섏뼱?쇳븳??
