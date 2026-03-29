# autocaller.py
import threading
import time

import protocol_defs as proto
import tcp_transport as tcp
class AutoCaller(threading.Thread):
    """
    FixedStep <-> SaveData를 max_calls 만큼 반복 호출.
    - pending dict의 (request_id, msg_type) 이벤트를 기다려서 동기화
    """

    def __init__(
        self,
        tcp_sock,
        pending: dict,
        lock: threading.Lock,
        request_id_ref: dict,
        max_calls: int,
        pending_add_fn,
        pending_pop_fn,
        step_count: int = 1,
        timeout_sec: float = 3.0,
        delay_sec: float = 0.0,
        progress_every: int = 50,
    ):
        super().__init__(daemon=True)
        self.tcp_sock = tcp_sock
        self.pending = pending
        self.lock = lock
        self.request_id_ref = request_id_ref  # {"value": int}
        self.max_calls = max_calls

        # 외부 함수 주입(메인에 pending_*가 남아 있어도 분리 가능)
        self.pending_add = pending_add_fn
        self.pending_pop = pending_pop_fn

        self.step_count = step_count
        self.timeout_sec = timeout_sec
        self.delay_sec = delay_sec
        self.progress_every = progress_every

        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    def _next_rid(self) -> int:
        return self.request_id_ref.next()

    # def _next_rid(self) -> int:
    #     with self.lock:
    #         rid = self.request_id_ref["value"]
    #         self.request_id_ref["value"] += 1
    #     return rid

    def _wait_or_stop(self, ev: threading.Event) -> bool:
        if self._stop.is_set():
            return False
        return ev.wait(self.timeout_sec)

    def run(self):
        print(f"[AUTO] started. target_steps={self.max_calls}")

        for i in range(self.max_calls):
            if self._stop.is_set():
                break

            # ---- FixedStep ----
            rid_step = self._next_rid()
            ev_step = self.pending_add(self.pending, self.lock, rid_step, proto.MSG_TYPE_FIXED_STEP)
            tcp.send_fixed_step(self.tcp_sock, rid_step, step_count=self.step_count)

            if not self._wait_or_stop(ev_step):
                self.pending_pop(self.pending, self.lock, rid_step, proto.MSG_TYPE_FIXED_STEP)
                print(f"[AUTO][TIMEOUT/STOP] FixedStep. i={i} rid={rid_step}")
                break
            self.pending_pop(self.pending, self.lock, rid_step, proto.MSG_TYPE_FIXED_STEP)

            if self.delay_sec > 0.0:
                time.sleep(self.delay_sec)

            if self._stop.is_set():
                break

            # ---- SaveData ----
            rid_save = self._next_rid()
            ev_save = self.pending_add(self.pending, self.lock, rid_save, proto.MSG_TYPE_SAVE_DATA)
            tcp.send_save_data(self.tcp_sock, rid_save)

            if not self._wait_or_stop(ev_save):
                self.pending_pop(self.pending, self.lock, rid_save, proto.MSG_TYPE_SAVE_DATA)
                print(f"[AUTO][TIMEOUT/STOP] SaveData. i={i} rid={rid_save}")
                break
            self.pending_pop(self.pending, self.lock, rid_save, proto.MSG_TYPE_SAVE_DATA)

            if self.delay_sec > 0.0:
                time.sleep(self.delay_sec)

            if self.progress_every > 0 and (i + 1) % self.progress_every == 0:
                print(f"[AUTO] progress: {i+1}/{self.max_calls}")

        print("[AUTO] stopped.")