# panels/log.py
import time
import dearpygui.dearpygui as dpg
import ui_queue

_MAX_LINES  = 500
_TAG_CHILD  = "log_child"
_TAG_SEARCH = "log_search"
_TAG_FOUND  = "log_found"
_auto_scroll = True
_lines: list[tuple] = []   # (text, color)
_search_kw  = ""


def build(parent: int | str) -> None:
    with dpg.group(parent=parent):
        # toolbar
        with dpg.group(horizontal=True):
            dpg.add_checkbox(
                label="Auto Scroll",
                default_value=True,
                callback=lambda s, v: _set_auto_scroll(v),
            )
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

        # 로그 텍스트를 담는 child_window — 스크롤은 여기서 처리
        dpg.add_child_window(tag=_TAG_CHILD, width=-1, height=-1, border=False)


def append(msg: str, level: str = "INFO") -> None:
    ts    = time.strftime("%H:%M:%S")
    text  = f"[{ts}][{level}] {msg}"
    color = _level_color(level)
    ui_queue.post(lambda t=text, c=color: _add_line(t, c))


def _add_line(text: str, color: tuple) -> None:
    if not dpg.does_item_exist(_TAG_CHILD):
        return

    _lines.append((text, color))
    if len(_lines) > _MAX_LINES:
        _lines.pop(0)

    if _search_kw:
        # 검색 중: 키워드 포함 라인만 표시
        if _search_kw.lower() not in text.lower():
            return
    else:
        # 오래된 아이템 정리
        children = dpg.get_item_children(_TAG_CHILD, slot=1) or []
        if len(children) >= _MAX_LINES:
            for old in children[:len(children) - _MAX_LINES + 1]:
                dpg.delete_item(old)

    dpg.add_text(text, color=color, parent=_TAG_CHILD)

    if _auto_scroll:
        dpg.set_y_scroll(_TAG_CHILD, dpg.get_y_scroll_max(_TAG_CHILD))


def _go_to_end() -> None:
    if dpg.does_item_exist(_TAG_CHILD):
        dpg.set_y_scroll(_TAG_CHILD, dpg.get_y_scroll_max(_TAG_CHILD))


def _on_search(keyword: str) -> None:
    global _search_kw
    _search_kw = keyword.strip()
    _rebuild_view()


def _rebuild_view() -> None:
    if not dpg.does_item_exist(_TAG_CHILD):
        return
    dpg.delete_item(_TAG_CHILD, children_only=True)

    if _search_kw:
        matched = [(t, c) for t, c in _lines if _search_kw.lower() in t.lower()]
        for t, c in matched:
            dpg.add_text(t, color=c, parent=_TAG_CHILD)
        dpg.set_value(_TAG_FOUND, f"{len(matched)} match(es)")
    else:
        for t, c in _lines:
            dpg.add_text(t, color=c, parent=_TAG_CHILD)
        dpg.set_value(_TAG_FOUND, "")

    _go_to_end()


def _clear() -> None:
    _lines.clear()
    if dpg.does_item_exist(_TAG_CHILD):
        dpg.delete_item(_TAG_CHILD, children_only=True)
    if dpg.does_item_exist(_TAG_FOUND):
        dpg.set_value(_TAG_FOUND, "")


def _set_auto_scroll(val: bool) -> None:
    global _auto_scroll
    _auto_scroll = val


def _level_color(level: str) -> tuple:
    return {
        "SEND":  (100, 200, 255, 255),
        "RECV":  (100, 255, 150, 255),
        "WARN":  (255, 220,  50, 255),
        "ERROR": (255,  80,  80, 255),
        "AUTO":  (200, 150, 255, 255),
    }.get(level, (220, 220, 220, 255))