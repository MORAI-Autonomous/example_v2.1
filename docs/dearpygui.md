# DearPyGUI 개발 규칙

## 탭바 너비 버그 — 버튼+show/hide 사용

`dpg.add_tab_bar` 는 첫 번째 탭을 컨테이너 전체 너비로 늘리는 버그가 있다.
이 프로젝트는 탭 전환을 **버튼 + show/hide** 방식으로 구현한다.

```python
# app.py 의 mon_window 참고
def _select_tab(name: str) -> None:
    dpg.configure_item("panel_a", show=(name == "a"))
    dpg.configure_item("panel_b", show=(name == "b"))
    for tag, key in [("btn_a", "a"), ("btn_b", "b")]:
        dpg.bind_item_theme(tag, "theme_tab_active" if name == key else "theme_tab_inactive")

with dpg.group(horizontal=True):
    dpg.add_button(label=" Tab A ", tag="btn_a", callback=lambda: _select_tab("a"))
    dpg.add_button(label=" Tab B ", tag="btn_b", callback=lambda: _select_tab("b"))
dpg.add_separator()
with dpg.child_window(tag="panel_a", show=True):  ...
with dpg.child_window(tag="panel_b", show=False): ...
```

테마는 `build_ui()` 에서 전역으로 생성한다 (`theme_tab_active`, `theme_tab_inactive`).
새 탭 추가 시 `_select_tab()` 의 show/hide 목록과 테마 바인딩 목록에 모두 추가해야 한다.

---

## 유니코드 / 아이콘 금지

DearPyGUI 기본 폰트는 특수 유니코드를 지원하지 않는다.
한글 폰트(`malgun.ttf`)를 로드해도 일반 유니코드 심볼(`↺`, `→` 등)은 여전히 깨질 수 있다.

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

## 스크롤 API — ChildWindow/Window 전용

`set_y_scroll` / `get_y_scroll_max` 는 `mvChildWindow` / `mvWindowAppItem` 에만 동작한다.
`mvInputText` 에서 호출하면 DPG 내부 에러가 터미널에 출력되고 무시된다.

```python
# ❌ mvInputText 에서는 동작 안 함
dpg.set_y_scroll("my_input_text", dpg.get_y_scroll_max("my_input_text"))

# ✅ ChildWindow 에서만 사용
dpg.set_y_scroll("my_child_window", dpg.get_y_scroll_max("my_child_window"))
```

---

## collapsing_header — 섹션 내 그룹 구분

같은 섹션 안에서 기능적으로 구분된 그룹을 접을 수 있게 만들 때 사용한다.
`_section()` 헬퍼의 separator와 시각적으로 충돌하지 않는다.

```python
# panels/commands.py 의 OBJECT CONTROL 참고
with dpg.collapsing_header(label="Manual Control", default_open=True):
    dpg.add_spacer(height=2)
    # ... 위젯들 ...
    dpg.add_button(label="Send", callback=...)

with dpg.collapsing_header(label="Transform Control", default_open=True):
    dpg.add_spacer(height=2)
    # ... 위젯들 ...
    dpg.add_button(label="Send", callback=...)
```

`default_open=True` 로 기본 펼침 상태를 유지한다.

---

## 동적 아이템 생성/삭제

런타임에 위젯을 추가/삭제할 때는 컨테이너 그룹의 자식만 삭제하고 재생성한다.

```python
# 컨테이너 그룹 미리 선언
dpg.add_group(tag="my_container")

# 재생성 함수
def _rebuild(count: int) -> None:
    dpg.delete_item("my_container", children_only=True)
    for i in range(count):
        # 반드시 with 블록으로 부모 컨텍스트 지정
        with dpg.group(parent="my_container"):
            dpg.add_text(f"Item {i}", ...)
            dpg.add_input_text(tag=f"my_input_{i}", ...)
```

**주의사항:**
- `delete_item(children_only=True)` 후에는 DPG 컨텍스트 스택이 비어 있다.
  반드시 `parent=` 를 명시하거나 `with dpg.group(parent=...)` 블록 안에서 아이템을 추가해야 한다.
- 삭제된 아이템의 태그(`my_input_0` 등)는 자동으로 해제되므로 재생성 시 동일 태그를 재사용할 수 있다.
- 실행 중 재생성하면 Runner가 참조하던 태그(`au_sv1_pos` 등)가 사라진다.
  업데이트 함수에서 반드시 `does_item_exist()` 로 체크해 무효 접근을 방어한다.

---

## 슬라이더 헬퍼 패턴

`panels/lane_control_panel.py` 의 `_slider()`, `_slider_int()` 참고.
- `tooltip` 파라미터: 라벨 텍스트에 마우스오버 툴팁 추가
- `show` 파라미터: 조건부 표시 (속도 제어 활성 시에만 보이는 슬라이더 등)
- 테이블 2열 배치로 세로 공간 절약 (`mvTable_SizingStretchSame`)
---

## Viewport Resize Rule

viewport resize callback?먯꽌 `dpg.configure_item()`?쇰줈 layout??吏곸젒 諛붽씀吏 ?딄퀬, dirty flag留?set?섏뿬 硫붿씤 猷⑦봽?먯꽌留?layout??諛섏쁺?쒕떎.

```python
_layout_dirty = True

def _mark_layout_dirty():
    global _layout_dirty
    _layout_dirty = True

dpg.set_viewport_resize_callback(_mark_layout_dirty)

while dpg.is_dearpygui_running():
    if _layout_dirty:
        _apply_layout()
    dpg.render_dearpygui_frame()
```

?대쾭? 李?move / resize ?꾨줈?꾩뒪?먯꽌 callback?댁슜?섍꼬 layout / hit-test / scroll ?곗뿭?대? 瑗ъ씠??寃쎌슦瑜?以꾩씠湲??꾪븳 洹쒖튃?대떎. `app.py`?먯꽌 viewport callback? dirty flag留??섏젙?섍퀬, layout ?곸슜? 硫붿씤 猷⑦봽?먯꽌留??섑뻾?쒕떎.
