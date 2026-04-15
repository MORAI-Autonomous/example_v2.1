from __future__ import annotations
# panels/monitor_receiver.py
# UDP 수신 스레드 (monitor.py 에서 분리)

import threading


class UDPThread(threading.Thread):
    """UDP 소켓에서 패킷을 수신하고 파싱 결과를 콜백으로 전달하는 데몬 스레드."""

    def __init__(self, sock, parse_fn, on_data, on_error):
        super().__init__(daemon=True)
        self.sock     = sock
        self.parse_fn = parse_fn
        self.on_data  = on_data
        self.on_error = on_error
        self.running  = True

    def stop(self) -> None:
        self.running = False

    def run(self) -> None:
        while self.running:
            try:
                data, _ = self.sock.recvfrom(65535)
                parsed  = self.parse_fn(data)
                if parsed is not None:
                    self.on_data(parsed)
            except OSError:
                if self.running:
                    self.on_error()
                break
