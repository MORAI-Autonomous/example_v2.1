from __future__ import annotations

import argparse
import collections
import csv
import json
import logging
import socket
import struct
import sys
import threading
import time
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable, List, Optional


HDR_FMT = "<HBH"
PL_FMT = "<QHIfHfffBH"
HDR_SIZE = struct.calcsize(HDR_FMT)
PL_SIZE = struct.calcsize(PL_FMT)


@dataclass
class Payload:
    timestamp: int
    region_id: int
    vehicle_id: int
    key_type: int
    speed: float
    heading: float
    lat: float
    lon: float
    alt: float
    vehicle_class: int


@dataclass
class Packet:
    total_size: int
    type: int
    count: int
    payloads: List[Payload]


def parse_packet(data: bytes) -> Packet:
    if len(data) < HDR_SIZE:
        raise ValueError("packet too short for header")

    total_size, msg_type, count = struct.unpack_from(HDR_FMT, data, 0)
    offset = HDR_SIZE
    payloads: List[Payload] = []

    for _ in range(count):
        if offset + PL_SIZE > len(data):
            break
        tup = struct.unpack_from(PL_FMT, data, offset)
        payloads.append(
            Payload(
                timestamp=int(tup[0]),
                region_id=int(tup[1]),
                vehicle_id=int(tup[2]),
                speed=float(tup[3]),
                heading=float(tup[4]),
                lat=float(tup[5]),
                lon=float(tup[6]),
                alt=float(tup[7]),
                key_type=int(tup[8]),
                vehicle_class=int(tup[9]),
            )
        )
        offset += PL_SIZE

    return Packet(total_size=total_size, type=msg_type, count=len(payloads), payloads=payloads)


def parse_targets(items: Iterable[str]) -> list[tuple[str, int]]:
    targets: list[tuple[str, int]] = []
    for item in items:
        host, port = item.rsplit(":", 1)
        targets.append((host, int(port)))
    return targets


def wait_for_quit(stop_event: threading.Event) -> None:
    print("\nPress 'q' to quit.\n")
    if sys.platform.startswith("win"):
        try:
            import msvcrt
        except ImportError:
            _busy_wait_quit(stop_event)
            return
        while not stop_event.is_set():
            if msvcrt.kbhit():
                ch = msvcrt.getwch()
                if ch.lower() == "q":
                    stop_event.set()
                    break
            time.sleep(0.05)
        return

    try:
        import select
        import termios
        import tty

        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while not stop_event.is_set():
                ready, _, _ = select.select([sys.stdin], [], [], 0.1)
                if ready:
                    ch = sys.stdin.read(1)
                    if ch.lower() == "q":
                        stop_event.set()
                        break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
    except Exception:
        _busy_wait_quit(stop_event)


def _busy_wait_quit(stop_event: threading.Event) -> None:
    print("Fallback mode: type 'q' then Enter or press Ctrl+C to quit.")
    while not stop_event.is_set():
        try:
            line = sys.stdin.readline()
        except Exception:
            break
        if not line:
            time.sleep(0.1)
            continue
        if line.strip().lower() == "q":
            stop_event.set()
            break


class ThroughputCounter:
    def __init__(self, window: float = 5.0):
        self.window = window
        self._lock = threading.Lock()
        self._buf = collections.deque()
        self.total_pkts = 0
        self.total_payloads = 0
        self.total_bytes = 0
        self._start = time.monotonic()
        self._last_arrival: Optional[float] = None
        self._intervals = collections.deque(maxlen=200)

    def record(self, pkts: int, payloads: int, nbytes: int) -> None:
        now = time.monotonic()
        with self._lock:
            self.total_pkts += pkts
            self.total_payloads += payloads
            self.total_bytes += nbytes
            if self.window > 0:
                self._buf.append((now, pkts, payloads, nbytes))
                cutoff = now - self.window
                while self._buf and self._buf[0][0] < cutoff:
                    self._buf.popleft()
            if self._last_arrival is not None:
                self._intervals.append((now - self._last_arrival) * 1000.0)
            self._last_arrival = now

    def rates(self) -> tuple[float, float, float, float]:
        now = time.monotonic()
        with self._lock:
            elapsed_total = now - self._start
            if self.window <= 0 or not self._buf:
                return 0.0, 0.0, 0.0, elapsed_total
            valid = list(self._buf)
            if len(valid) < 2:
                span = min(self.window, elapsed_total) or 1e-9
            else:
                span = valid[-1][0] - valid[0][0] or 1e-9
            return (
                sum(v[1] for v in valid) / span,
                sum(v[2] for v in valid) / span,
                sum(v[3] for v in valid) / span,
                elapsed_total,
            )

    def interval_stats(self) -> tuple[float, float, float, int]:
        with self._lock:
            intervals = list(self._intervals)
        if not intervals:
            return 0.0, 0.0, 0.0, 0
        return min(intervals), max(intervals), sum(intervals) / len(intervals), len(intervals)


class FileLogger:
    def __init__(self, mode: str = "csv", directory: str = "."):
        self.mode = mode.lower()
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        base = Path(directory)
        base.mkdir(parents=True, exist_ok=True)
        if self.mode == "csv":
            self.path = base / f"rsa_udp_log_{timestamp}.csv"
            self.fp = self.path.open("w", newline="", encoding="utf-8")
            self.writer = csv.writer(self.fp)
            self.writer.writerow(
                [
                    "recv_time_iso",
                    "remote_ip",
                    "remote_port",
                    "total_size",
                    "type",
                    "count",
                    "idx",
                    "timestamp",
                    "region_id",
                    "vehicle_id",
                    "speed",
                    "heading",
                    "lat",
                    "lon",
                    "alt",
                    "key_type",
                    "vehicle_class",
                ]
            )
        else:
            self.path = base / f"rsa_udp_log_{timestamp}.jsonl"
            self.fp = self.path.open("w", encoding="utf-8")
            self.writer = None

    def write_packet(self, packet: Packet, remote: tuple[str, int]) -> None:
        now_iso = datetime.now().isoformat(timespec="milliseconds")
        ip, port = remote
        if self.mode == "csv":
            for idx, payload in enumerate(packet.payloads):
                self.writer.writerow(
                    [
                        now_iso,
                        ip,
                        port,
                        packet.total_size,
                        packet.type,
                        packet.count,
                        idx,
                        payload.timestamp,
                        payload.region_id,
                        payload.vehicle_id,
                        f"{payload.speed:.6f}",
                        payload.heading,
                        f"{payload.lat:.6f}",
                        f"{payload.lon:.6f}",
                        f"{payload.alt:.6f}",
                        payload.key_type,
                        payload.vehicle_class,
                    ]
                )
            self.fp.flush()
            return

        base = {
            "recv_time_iso": now_iso,
            "remote_ip": ip,
            "remote_port": port,
            "total_size": packet.total_size,
            "type": packet.type,
            "count": packet.count,
        }
        for idx, payload in enumerate(packet.payloads):
            self.fp.write(json.dumps({**base, "idx": idx, **asdict(payload)}, ensure_ascii=False) + "\n")
        self.fp.flush()

    def close(self) -> None:
        try:
            self.fp.close()
        except Exception:
            pass


def print_packet(packet: Packet) -> None:
    print(f"Header -> total_size={packet.total_size}, type={packet.type}, count={packet.count}")
    for idx, payload in enumerate(packet.payloads):
        data = asdict(payload)
        try:
            ts_utc = datetime.fromtimestamp(data["timestamp"] / 1000.0, tz=timezone.utc)
            ts_utc_str = ts_utc.strftime("%Y-%m-%d %H:%M:%S.%f %Z")
        except Exception:
            ts_utc_str = "invalid"
        print(
            f"  [#{idx}] ts={data['timestamp']} (UTC: {ts_utc_str}) region={data['region_id']} "
            f"vid={data['vehicle_id']} speed={data['speed']:.6f} heading={data['heading']} "
            f"lat={data['lat']:.6f} lon={data['lon']:.6f} alt={data['alt']:.6f} "
            f"key_type={data['key_type']} class={data['vehicle_class']}"
        )


def stats_printer(stop_event: threading.Event, counter: ThroughputCounter, interval: float, label: str = "RSA", kbps_bias: float = 0.0) -> None:
    while not stop_event.is_set():
        time.sleep(interval)
        if stop_event.is_set():
            break
        pps, plps, bps, elapsed = counter.rates()
        kbps = (bps / 1024.0) + kbps_bias
        imin, imax, iavg, samples = counter.interval_stats()
        print(
            f"\n[{label} Stats | elapsed {elapsed:>7.1f}s | window {counter.window:.1f}s] "
            f"{pps:>8.1f} pkt/s  {plps:>9.1f} payload/s  {kbps:>8.2f} KB/s"
        )
        if samples:
            print(f"  inter-arrival(last {samples}): avg={iavg:>7.2f}ms  min={imin:>7.2f}ms  max={imax:>7.2f}ms")


def run_parse(args: argparse.Namespace) -> int:
    stop_event = threading.Event()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.listen_ip, args.port))
    sock.settimeout(0.5)

    print(f"UDP listening on {args.listen_ip}:{args.port}")
    recv_thread = threading.Thread(target=wait_for_quit, args=(stop_event,), daemon=True)
    recv_thread.start()

    try:
        while not stop_event.is_set():
            try:
                data, addr = sock.recvfrom(args.bufsize)
            except socket.timeout:
                continue
            except OSError:
                break
            print(f"\nReceived {len(data)} bytes from {addr[0]}:{addr[1]}")
            try:
                packet = parse_packet(data)
            except Exception as exc:
                print(f"Parse error: {exc}")
                continue
            print_packet(packet)
    except KeyboardInterrupt:
        stop_event.set()
    finally:
        sock.close()
        print("UDP socket closed.")
    return 0


def run_record(args: argparse.Namespace) -> int:
    stop_event = threading.Event()
    counter = ThroughputCounter(window=args.stats_window)
    logger = FileLogger(mode=args.log_mode, directory=args.log_dir)
    print(f"Logging to: {logger.path}")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.listen_ip, args.port))
    sock.settimeout(0.5)

    threading.Thread(target=wait_for_quit, args=(stop_event,), daemon=True).start()
    threading.Thread(
        target=stats_printer,
        args=(stop_event, counter, args.stats_interval, "RSA-Record", 0.0),
        daemon=True,
    ).start()

    print(f"UDP listening on {args.listen_ip}:{args.port}")
    try:
        while not stop_event.is_set():
            try:
                data, addr = sock.recvfrom(args.bufsize)
            except socket.timeout:
                continue
            except OSError:
                break
            print(f"\nReceived {len(data)} bytes from {addr[0]}:{addr[1]}")
            try:
                packet = parse_packet(data)
            except Exception as exc:
                print(f"Parse error: {exc}")
                continue
            counter.record(1, packet.count, len(data))
            print_packet(packet)
            logger.write_packet(packet, addr)
    except KeyboardInterrupt:
        stop_event.set()
    finally:
        sock.close()
        logger.close()
        print("UDP socket closed.")
    return 0


def _make_logger() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s", datefmt="%H:%M:%S")


def _stdin_quit_watcher(running_flag: dict[str, bool]) -> None:
    while running_flag["running"]:
        line = sys.stdin.readline()
        if not line:
            time.sleep(0.1)
            continue
        if line.strip().lower() == "q":
            logging.info("quit requested by user")
            running_flag["running"] = False
            break


def run_bypass(args: argparse.Namespace) -> int:
    _make_logger()
    running = {"running": True}
    recv_count = 0
    bytes_received = 0
    targets = parse_targets(args.target)
    target_success = {target: 0 for target in targets}
    target_fail = {target: 0 for target in targets}

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    send_sock: Optional[socket.socket] = None
    sock.bind((args.listen_ip, args.port))
    sock.settimeout(1.0)

    def print_stats() -> None:
        nonlocal recv_count, bytes_received
        last_recv = 0
        last_bytes = 0
        while running["running"]:
            time.sleep(args.stats_interval)
            if not running["running"]:
                break
            delta = recv_count - last_recv
            last_recv = recv_count
            delta_bytes = bytes_received - last_bytes
            last_bytes = bytes_received
            kbps = (delta_bytes / (1024.0 * args.stats_interval)) + args.kbps_bias
            logging.info("%s data rate (%.1fs): recv=%d (+%d) %.2f KB/s", args.label, args.stats_interval, recv_count, delta, kbps)

    threading.Thread(target=_stdin_quit_watcher, args=(running,), daemon=True).start()
    threading.Thread(target=print_stats, daemon=True).start()

    logging.info("listening on %s:%d, forwarding to %s", args.listen_ip, args.port, ", ".join(f"{h}:{p}" for h, p in targets))
    try:
        while running["running"]:
            try:
                data, _ = sock.recvfrom(args.bufsize)
            except socket.timeout:
                continue
            except Exception as exc:
                logging.exception("receive error: %s", exc)
                continue

            recv_count += 1
            bytes_received += len(data)
            if send_sock is None:
                send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            for target in targets:
                try:
                    send_sock.sendto(data, target)
                    target_success[target] += 1
                except Exception as exc:
                    logging.error("forward error to %s:%d: %s", target[0], target[1], exc)
                    target_fail[target] += 1
    except KeyboardInterrupt:
        running["running"] = False
    finally:
        sock.close()
        if send_sock is not None:
            send_sock.close()
        logging.info("final stats: recv=%d bytes=%d", recv_count, bytes_received)
        for target in targets:
            logging.info("target %s:%d success=%d fail=%d", target[0], target[1], target_success[target], target_fail[target])
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="RSA UDP debug tool")
    sub = parser.add_subparsers(dest="command", required=True)

    parse_cmd = sub.add_parser("parse", help="Parse and print RSA UDP packets")
    parse_cmd.add_argument("--listen-ip", default="0.0.0.0")
    parse_cmd.add_argument("--port", type=int, default=50002)
    parse_cmd.add_argument("--bufsize", type=int, default=65000)
    parse_cmd.set_defaults(func=run_parse)

    record_cmd = sub.add_parser("record", help="Parse, print, and record RSA UDP packets")
    record_cmd.add_argument("--listen-ip", default="0.0.0.0")
    record_cmd.add_argument("--port", type=int, default=50002)
    record_cmd.add_argument("--bufsize", type=int, default=65000)
    record_cmd.add_argument("--log-mode", choices=("csv", "jsonl"), default="csv")
    record_cmd.add_argument("--log-dir", default=".")
    record_cmd.add_argument("--stats-interval", type=float, default=1.0)
    record_cmd.add_argument("--stats-window", type=float, default=5.0)
    record_cmd.set_defaults(func=run_record)

    bypass_cmd = sub.add_parser("bypass", help="Forward RSA UDP packets to one or more targets")
    bypass_cmd.add_argument("--listen-ip", default="0.0.0.0")
    bypass_cmd.add_argument("--port", type=int, default=50002)
    bypass_cmd.add_argument("--bufsize", type=int, default=65535)
    bypass_cmd.add_argument("--stats-interval", type=float, default=1.0)
    bypass_cmd.add_argument("--kbps-bias", type=float, default=0.0)
    bypass_cmd.add_argument("--label", default="RSA")
    bypass_cmd.add_argument("--target", action="append", required=True, help="host:port, repeatable")
    bypass_cmd.set_defaults(func=run_bypass)

    return parser


def main(argv: Optional[list[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
