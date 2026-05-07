from __future__ import annotations

import argparse
import logging
import socket
import struct
import sys
import threading
import time
from dataclasses import asdict, dataclass
from typing import List, Optional


HDR_FMT = "<HBH"
HDR_SIZE = struct.calcsize(HDR_FMT)
PL2_FMT = "<16sQffffHBH"
PL2_SIZE = struct.calcsize(PL2_FMT)


@dataclass
class PayloadV2:
    id: str
    timestamp: int
    key_type: int
    lat: float
    lon: float
    alt: float
    speed: float
    heading: int
    vehicle_class: int


@dataclass
class Packet:
    total_size: int
    type: int
    count: int
    payloads: List[PayloadV2]


def _decode_id(raw16: bytes) -> str:
    value = raw16.split(b"\x00", 1)[0]
    try:
        return value.decode("utf-8", errors="ignore")
    except Exception:
        return value.decode("latin1", errors="ignore")


def parse_packet_v2(data: bytes) -> Packet:
    if len(data) < HDR_SIZE:
        raise ValueError(f"buffer too small for header: {len(data)} bytes")

    total_size, msg_type, count = struct.unpack_from(HDR_FMT, data, 0)
    offset = HDR_SIZE
    payloads: List[PayloadV2] = []

    for _ in range(count):
        if offset + PL2_SIZE > len(data):
            break

        raw_id, ts, lat, lon, alt, speed, heading, key_type, vclass = struct.unpack_from(PL2_FMT, data, offset)
        offset += PL2_SIZE
        payloads.append(
            PayloadV2(
                id=_decode_id(raw_id),
                timestamp=ts,
                lat=lat,
                lon=lon,
                alt=alt,
                speed=speed,
                heading=heading,
                key_type=key_type,
                vehicle_class=vclass,
            )
        )

    return Packet(total_size=total_size, type=msg_type, count=count, payloads=payloads)


def parse_targets(items: List[str]) -> list[tuple[str, int]]:
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
            while not stop_event.is_set():
                if msvcrt.kbhit():
                    ch = msvcrt.getwch()
                    if ch and ch.lower() == "q":
                        stop_event.set()
                        break
                time.sleep(0.05)
            return
        except Exception:
            pass

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
                    if ch and ch.lower() == "q":
                        stop_event.set()
                        break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
    except Exception:
        print("Fallback mode: type 'q' then Enter to quit.")
        while not stop_event.is_set():
            line = sys.stdin.readline()
            if not line:
                time.sleep(0.1)
                continue
            if line.strip().lower() == "q":
                stop_event.set()
                break


def run_parse(args: argparse.Namespace) -> int:
    stop_event = threading.Event()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.listen_ip, args.port))
    sock.settimeout(0.5)

    print(f"UDP listening on {args.listen_ip}:{args.port}")
    threading.Thread(target=wait_for_quit, args=(stop_event,), daemon=True).start()

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
                pkt = parse_packet_v2(data)
            except Exception as exc:
                print(f"parse error: {exc}")
                continue

            print(f"Header -> total_size={pkt.total_size}, type={pkt.type}, count={pkt.count}")
            for idx, payload in enumerate(pkt.payloads):
                data_dict = asdict(payload)
                print(
                    f"  [#{idx}] id='{data_dict['id']}' ts={data_dict['timestamp']}"
                    f" lat={data_dict['lat']:.6f} lon={data_dict['lon']:.6f} alt={data_dict['alt']:.6f}"
                    f" speed={data_dict['speed']:.6f} heading={data_dict['heading']}"
                    f" key_type={data_dict['key_type']} class={data_dict['vehicle_class']}"
                )
    except KeyboardInterrupt:
        stop_event.set()
    finally:
        sock.close()
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

    def stats_printer() -> None:
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
            kbps = delta_bytes / (1024.0 * args.stats_interval)
            logging.info("PVD data rate (%.1fs): recv=%d (+%d) %.2f KB/s", args.stats_interval, recv_count, delta, kbps)

    threading.Thread(target=_stdin_quit_watcher, args=(running,), daemon=True).start()
    threading.Thread(target=stats_printer, daemon=True).start()

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
    parser = argparse.ArgumentParser(description="PVD UDP debug tool")
    sub = parser.add_subparsers(dest="command", required=True)

    parse_cmd = sub.add_parser("parse", help="Parse and print PVD UDP packets")
    parse_cmd.add_argument("--listen-ip", default="0.0.0.0")
    parse_cmd.add_argument("--port", type=int, default=50001)
    parse_cmd.add_argument("--bufsize", type=int, default=65000)
    parse_cmd.set_defaults(func=run_parse)

    bypass_cmd = sub.add_parser("bypass", help="Forward PVD UDP packets to one or more targets")
    bypass_cmd.add_argument("--listen-ip", default="0.0.0.0")
    bypass_cmd.add_argument("--port", type=int, default=50001)
    bypass_cmd.add_argument("--bufsize", type=int, default=65535)
    bypass_cmd.add_argument("--stats-interval", type=float, default=1.0)
    bypass_cmd.add_argument("--target", action="append", required=True, help="host:port, repeatable")
    bypass_cmd.set_defaults(func=run_bypass)

    return parser


def main(argv: Optional[list[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
