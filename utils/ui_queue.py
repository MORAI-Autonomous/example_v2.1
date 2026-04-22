# ui_queue.py
"""
백그라운드 스레드(Receiver, AutoCaller 등)에서 DPG UI를 안전하게 업데이트하기 위한 큐.

사용법:
  - 백그라운드 스레드: post(lambda: dpg.set_value("tag", val))
  - 메인 루프:        drain() 을 매 프레임 호출
"""
import queue
import time
from typing import Callable

_q: queue.Queue[Callable] = queue.Queue()


def post(fn: Callable) -> None:
    """백그라운드 스레드에서 UI 업데이트 람다를 등록."""
    _q.put(fn)


_DRAIN_CAP       = 200    # 한 프레임당 최대 처리 항목 수
_WARN_BACKLOG    = 50     # 이 이상 쌓이면 경고
_WARN_ITEM_MS    = 50.0   # 단일 항목이 이 시간 초과 시 경고


def drain() -> int:
    """DPG render loop에서 매 프레임 호출 — 큐에 쌓인 UI 업데이트를 소비.
    처리한 항목 수를 반환."""
    backlog = _q.qsize()
    if backlog > _WARN_BACKLOG:
        print(f"[ui_queue] backlog={backlog}")

    count = 0
    for _ in range(_DRAIN_CAP):
        if _q.empty():
            break
        try:
            fn = _q.get_nowait()
            count += 1
            t0 = time.perf_counter()
            fn()
            ms = (time.perf_counter() - t0) * 1000.0
            if ms > _WARN_ITEM_MS:
                print(f"[ui_queue] slow item {ms:.1f}ms: {fn}")
        except Exception as e:
            print(f"[ui_queue] drain error: {e}")
    return count