# DearPyGUI 개발 규칙

## 탭바 너비 버그 — 버튼+show/hide 사용

`dpg.add_tab_bar` 는 첫 번째 탭을 컨테이너 전체 너비로 늘리는 버그가 있다.
이 프로젝트는 탭 전환을 **버튼 + show/hide** 방식으로 구현한다.

```python
# app.py 의 mon_window 참고
def _select_tab(name: str) -> None:
    dpg.configure_item("panel_a", show=(name == "a"))
    dpg.configure_item("panel_b", show=(name == "b"))
    dpg.bind_item_theme("btn_a", "theme_tab_active"   if name == "a" else "theme_tab_inactive")
    dpg.bind_item_theme("btn_b", "theme_tab_inactive" if name == "a" else "theme_tab_active")

with dpg.group(horizontal=True):
    dpg.add_button(label=" Tab A ", tag="btn_a", callback=lambda: _select_tab("a"))
    dpg.add_button(label=" Tab B ", tag="btn_b", callback=lambda: _select_tab("b"))
dpg.add_separator()
with dpg.child_window(tag="panel_a", show=True):  ...
with dpg.child_window(tag="panel_b", show=False): ...
```

테마는 `build_ui()` 에서 전역으로 생성한다 (`theme_tab_active`, `theme_tab_inactive`).

---

## 유니코드 / 아이콘 금지

DearPyGUI 기본 폰트는 특수 유니코드를 지원하지 않는다.

```python
# ❌ 깨짐
dpg.add_button(label="↺ Reset")
dpg.add_button(label=">> Start")

# ✅ 안전
dpg.add_button(label="Reset Defaults")
dpg.add_button(label="Start")
```

아이콘이 필요하면 이미지 텍스처를 직접 등록해서 `add_image_button` 을 사용한다.
(`_folder_btn()` 함수 참고 — 텍스처 없으면 텍스트 버튼으로 폴백)

---

## 동적 텍스처 (카메라/디버그 프레임)

```python
# 등록 (한 번만)
blank = [0.0] * (W * H * 4)
with dpg.texture_registry():
    dpg.add_dynamic_texture(W, H, blank, tag="my_texture")

# 업데이트 (매 프레임, ui_queue 경유)
rgba_f32 = frame_bgr_to_rgba_float(frame)   # shape: (H*W*4,) float32 0~1
ui_queue.post(lambda d=rgba_f32: dpg.set_value("my_texture", d))
```

프레임은 반드시 `float32` RGBA 1D 배열 (0.0~1.0 범위)로 변환해야 한다.

---

## 아이템 존재 확인

콜백이나 업데이트 함수에서 아이템에 접근하기 전 반드시 확인한다.
(탭 전환으로 숨겨진 패널의 아이템은 여전히 존재하지만, 초기화 순서 문제가 생길 수 있다)

```python
if dpg.does_item_exist("my_tag"):
    dpg.set_value("my_tag", new_value)
```

---

## 슬라이더 헬퍼 패턴

`panels/lane_control_panel.py` 의 `_slider()`, `_slider_int()` 참고.
- `tooltip` 파라미터: 라벨 텍스트에 마우스오버 툴팁 추가
- `show` 파라미터: 조건부 표시 (속도 제어 활성 시에만 보이는 슬라이더 등)
- 테이블 2열 배치로 세로 공간 절약 (`mvTable_SizingStretchSame`)
