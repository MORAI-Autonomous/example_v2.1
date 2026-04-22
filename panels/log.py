from __future__ import annotations

# panels/log.py
#
# append() → _pending deque (임의 스레드, lock-free in CPython)
# flush()  → 메인 루프에서 프레임당 1회 호출 → set_value 최대 1회
#
# 이전 구조(ui_queue.post → drain() 200회 set_value)를 없애고
# 프레임당 set_value 1회로 줄여 render() 부담을 최소화.
#
# _MAX_LINES     : 검색 대상 보관 줄 수
# _DISPLAY_LINES : 표시할 최근 줄 수
#   LOG_H≈280px / 21px per line ≈ 13줄 가시 → 30줄이면 충분한 스크롤 버퍼

import collections
import time
import dearpygui.dearpygui as dpg

_MAX_LINES     = 500
_DISPLAY_LINES = 30

_TAG_TEXT   = "log_text"
_TAG_SEARCH = "log_search"
_TAG_FOUND  = "log_found"

_lines:    list[str] = []
_search_kw = ""

_BOTTOM_THRESHOLD = 30   # px

# 스레드-안전 수신 버퍼
# deque.append / popleft 는 CPython GIL 하에서 원자적
_pending: collections.deque = collections.deque()


def build(parent) -> None:
    with dpg.group(parent=parent):
        with dpg.group(horizontal=True):
            dpg.add_button(label="Go to End", callback=_go_to_end)
            dpg.add_button(label="Clear",     callback=_clear)
            dpg.add_spacer(width=8)
            dpg.add_text("Search:")
            dpg.add_input_text(
                tag=_TAG_SEARCH,
                width=160,
                hint="keyword  (Enter)",
                on_enter=True,
                callback=lambda s, v: _on_search(v),
            )
            dpg.add_button(label="Find",
                           callback=lambda: _on_search(dpg.get_value(_TAG_SEARCH)))
            dpg.add_text("", tag=_TAG_FOUND, color=(180, 180, 100, 255))

        dpg.add_input_text(
            tag=_TAG_TEXT,
            default_value="",
            multiline=True,
            readonly=True,
            width=-1,
            height=-1,
            tab_input=False,
        )


# ── 외부 API ──────────────────────────────────────────────────

def append(msg: str, level: str = "INFO") -> None:
    """임의 스레드에서 안전하게 호출 가능.
    실제 DPG 갱신은 flush()가 담당 (메인 루프에서 프레임당 1회)."""
    ts = time.strftime("%H:%M:%S")
    _pending.append(f"[{ts}][{level}] {msg}")


def flush() -> None:
    """메인 루프에서 프레임당 1회 호출.
    pending 라인을 모두 소비하고 set_value 를 최대 1회 실행."""
    if not _pending:
        return
    if not dpg.does_item_exist(_TAG_TEXT):
        return

    at_bottom = _is_at_bottom()

    # pending 전체 소비
    while _pending:
        _lines.append(_pending.popleft())
    if len(_lines) > _MAX_LINES:
        del _lines[:len(_lines) - _MAX_LINES]

    # 표시 갱신 (set_value 1회)
    if _search_kw:
        matched = [t for t in _lines if _search_kw.lower() in t.lower()]
        dpg.set_value(_TAG_TEXT, "\n".join(matched))
        dpg.set_value(_TAG_FOUND, f"{len(matched)} match(es)")
    else:
        dpg.set_value(_TAG_TEXT, "\n".join(_lines[-_DISPLAY_LINES:]))

    if at_bottom:
        _scroll_to_end()


# ── 내부 ──────────────────────────────────────────────────────

def _is_at_bottom() -> bool:
    if not dpg.does_item_exist(_TAG_TEXT):
        return True
    s_y   = dpg.get_y_scroll(_TAG_TEXT)
    s_max = dpg.get_y_scroll_max(_TAG_TEXT)
    return s_max <= 0 or (s_max - s_y) < _BOTTOM_THRESHOLD


def _scroll_to_end() -> None:
    if dpg.does_item_exist(_TAG_TEXT):
        dpg.set_y_scroll(_TAG_TEXT, -1.0)


def _go_to_end() -> None:
    _scroll_to_end()


def _refresh_display() -> None:
    """검색 종료·Clear 후 전체 재빌드 (드문 케이스)."""
    if not dpg.does_item_exist(_TAG_TEXT):
        return
    dpg.set_value(_TAG_TEXT, "\n".join(_lines[-_DISPLAY_LINES:]))


def _on_search(keyword: str) -> None:
    global _search_kw
    _search_kw = keyword.strip()
    _rebuild_view()


def _rebuild_view() -> None:
    if not dpg.does_item_exist(_TAG_TEXT):
        return

    if _search_kw:
        matched = [t for t in _lines if _search_kw.lower() in t.lower()]
        dpg.set_value(_TAG_TEXT, "\n".join(matched))
        dpg.set_value(_TAG_FOUND, f"{len(matched)} match(es)")
    else:
        dpg.set_value(_TAG_FOUND, "")
        _refresh_display()

    _scroll_to_end()


def _clear() -> None:
    _lines.clear()
    _pending.clear()
    if dpg.does_item_exist(_TAG_TEXT):
        dpg.set_value(_TAG_TEXT, "")
    if dpg.does_item_exist(_TAG_FOUND):
        dpg.set_value(_TAG_FOUND, "")
