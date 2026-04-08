# ui_queue.py
"""
백그라운드 스레드(Receiver, AutoCaller 등)에서 DPG UI를 안전하게 업데이트하기 위한 큐.

사용법:
  - 백그라운드 스레드: post(lambda: dpg.set_value("tag", val))
  - 메인 루프:        drain() 을 매 프레임 호출
"""
import queue
from typing import Callable

_q: queue.SimpleQueue[Callable] = queue.SimpleQueue()


def post(fn: Callable) -> None:
    """백그라운드 스레드에서 UI 업데이트 람다를 등록."""
    _q.put(fn)


def drain() -> None:
    """DPG render loop에서 매 프레임 호출 — 큐에 쌓인 UI 업데이트를 소비."""
    while not _q.empty():
        try:
            _q.get_nowait()()
        except Exception as e:
            # UI 업데이트 실패가 메인 루프를 죽이지 않도록
            print(f"[ui_queue] drain error: {e}")