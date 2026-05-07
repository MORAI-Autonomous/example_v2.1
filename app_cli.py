import socket
import threading
import time

from transport.protocol_defs import *
import transport.tcp_transport as tcp
import transport.tcp_thread as tcp_thread
import automation.automation as ac
import utils.key_input as key_input
import transport.commands as commands
import utils.input_helper as prompt


# ============================================================
# RequestIdCounter
# ============================================================

class RequestIdCounter:
    """thread-safe request_id 발급기."""
    def __init__(self, start: int = 1):
        self._lock = threading.Lock()
        self._value = start

    def next(self) -> int:
        with self._lock:
            rid = self._value
            self._value += 1
        return rid


# ============================================================
# Pending (request sync)
# key: (request_id, msg_type) -> {"t": float, "ev": Event}
# ============================================================

def pending_add(pending: dict, lock: threading.Lock, request_id: int, msg_type: int) -> threading.Event:
    ev = threading.Event()
    with lock:
        pending[(request_id, msg_type)] = {"t": time.time(), "ev": ev}
    return ev

def pending_pop(pending: dict, lock: threading.Lock, request_id: int, msg_type: int):
    with lock:
        pending.pop((request_id, msg_type), None)


# ============================================================
# Connection helpers
# ============================================================

def _close_socket(sock: socket.socket):
    try:
        sock.shutdown(socket.SHUT_RDWR)
    except Exception:
        pass
    try:
        sock.close()
    except Exception:
        pass


def _make_tcp_socket() -> socket.socket:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    return s


def connect_and_start_receiver(pending: dict, lock: threading.Lock):
    sock = _make_tcp_socket()
    while True:
        try:
            print("[INFO] Attempting to connect...")
            sock.connect((TCP_SERVER_IP, TCP_SERVER_PORT))
            print(f"[INFO] Connected. IP: {TCP_SERVER_IP}, Port: {TCP_SERVER_PORT}")
            receiver = tcp_thread.Receiver(sock, pending, lock)
            receiver.start()
            return sock, receiver
        except Exception as e:
            print(f"[ERROR] IP: {TCP_SERVER_IP}, Port: {TCP_SERVER_PORT} Connect failed: {e}. Retrying in 5 seconds...")
            _close_socket(sock)
            time.sleep(5)
            sock = _make_tcp_socket()


# ============================================================
# Key bindings help
# ============================================================

def print_key_bindings():
    print("---- Simulation Time Mode ----")
    print("  [1] GetSimulationTimeStatus      (TCP 0x1101)")
    print("  [2] SetSimulationTimeModeCommand (TCP 0x1102)")
    print("---- Fixed Step Control ------")
    print("  [3] FixedStep                    (TCP 0x1201)")
    print("  [4] SaveData                     (TCP 0x1202)")
    print("---- Object Control ----------")
    print("  [5] CreateObject                 (TCP 0x1301)")
    print("  [6] ManualControlById            (TCP 0x1302)")
    print("  [7] TransformControlById         (TCP 0x1303)")
    print("  [8] SetTrajectory                (TCP 0x1304)")
    print("---- Scenario Control --------")
    print("  [a] ScenarioStatus               (TCP 0x1504)")
    print("  [b] ScenarioControl              (TCP 0x1505)")
    print("---- Suite Control -----------")
    print("  [c] ActiveSuiteStatus            (TCP 0x1401)")
    print("  [d] LoadSuite                    (TCP 0x1402)")
    print("---- ETC ---------------------")
    print(f" [W] Toggle AutoCall (FixedStep <-> SaveData) x {MAX_CALL_NUM}")
    print("  [Q] Quit\n")


# ============================================================
# Main
# ============================================================

def main():
    pending      = {}
    lock         = threading.Lock()
    rid_counter  = RequestIdCounter()
    receiver     = None
    auto_caller  = None

    tcp_sock, receiver = connect_and_start_receiver(pending, lock)
    print_key_bindings()

    def dispatch(msg_type: int, send_fn):
        """rid 발급 → pending 등록 → send_fn(rid) 호출."""
        rid = rid_counter.next()
        pending_add(pending, lock, rid, msg_type)
        send_fn(rid)

    def toggle_auto_caller():
        nonlocal auto_caller
        if auto_caller is None or not auto_caller.is_alive():
            auto_caller = ac.AutoCaller(
                tcp_sock=tcp_sock,
                pending=pending,
                lock=lock,
                request_id_ref=rid_counter,
                max_calls=MAX_CALL_NUM,
                pending_add_fn=pending_add,
                pending_pop_fn=pending_pop,
                step_count=1,
                timeout_sec=AUTO_TIMEOUT_SEC,
                delay_sec=AUTO_DELAY_BETWEEN_CMDS_SEC,
            )
            auto_caller.start()
        else:
            auto_caller.stop()
            auto_caller = None

    def stop_auto_caller():
        nonlocal auto_caller
        if auto_caller is not None and auto_caller.is_alive():
            auto_caller.stop()
            auto_caller = None

    def reconnect():
        nonlocal tcp_sock, receiver
        stop_auto_caller()
        if receiver is not None:
            try:
                receiver.stop()
            except Exception:
                pass
            receiver = None
        _close_socket(tcp_sock)
        tcp_sock, receiver = connect_and_start_receiver(pending, lock)
        print_key_bindings()

    try:
        while True:
            key = key_input.get_key()
            if key in ("q", "Q"):
                break

            try:
                if key == "1":
                    dispatch(MSG_TYPE_GET_SIMULATION_TIME_STATUS,
                             lambda rid: tcp.send_get_status(tcp_sock, rid))

                elif key == "2":
                    dispatch(MSG_TYPE_SET_SIMULATION_TIME_MODE_COMMAND,
                             lambda rid: tcp.send_simulation_time_mode_command(
                                 tcp_sock, rid,
                                 mode=TIME_MODE_FIXED,
                                 simulation_delta_time=20,
                                 physics_delta_time=10,
                                 rtf=1,
                                 user_control=0))

                elif key == "3":
                    dispatch(MSG_TYPE_FIXED_STEP,
                             lambda rid: tcp.send_fixed_step(tcp_sock, rid, step_count=1))

                elif key == "4":
                    dispatch(MSG_TYPE_SAVE_DATA,
                             lambda rid: tcp.send_save_data(tcp_sock, rid))

                elif key == "5":
                    params = prompt.prompt_create_object()
                    dispatch(MSG_TYPE_CREATE_OBJECT,
                             lambda rid: tcp.send_create_object(tcp_sock, rid, **params))

                elif key == "6":
                    params = prompt.prompt_manual_control_by_id()
                    dispatch(MSG_TYPE_MANUAL_CONTROL_BY_ID_COMMAND,
                             lambda rid: tcp.send_manual_control_by_id(tcp_sock, rid, **params))

                elif key == "7":
                    params = prompt.prompt_transform_control_by_id()
                    dispatch(MSG_TYPE_TRANSFORM_CONTROL_BY_ID_COMMAND,
                             lambda rid: tcp.send_transform_control_by_id(tcp_sock, rid, **params))

                elif key == "8":
                    dispatch(MSG_TYPE_SET_TRAJECTORY_COMMAND,
                             lambda rid: tcp.send_set_trajectory(
                                 tcp_sock, rid,
                                 entity_id="Car_1",
                                 follow_mode=2,
                                 trajectory_name="Route_1",
                                 points=[
                                     (237.4360, -299.4899, 0.0210, 2.0),
                                     (199.6393, -280.8129, 0.1524, 4.0),
                                 ]))

                elif key in ("a", "A"):
                    dispatch(MSG_TYPE_SCENARIO_STATUS,
                             lambda rid: tcp.send_scenario_status(tcp_sock, rid))

                elif key in ("b", "B"):
                    params = prompt.prompt_scenario_control()
                    dispatch(MSG_TYPE_SCENARIO_CONTROL,
                             lambda rid, p=params: tcp.send_scenario_control(
                                 tcp_sock, rid,
                                 command=p["command"],
                                 scenario_name=p["scenario_name"]))

                elif key in ("c", "C"):
                    dispatch(MSG_TYPE_ACTIVE_SUITE_STATUS,
                             lambda rid: tcp.send_active_suite_status(tcp_sock, rid))

                elif key in ("d", "D"):
                    dispatch(MSG_TYPE_LOAD_SUITE,
                             lambda rid: tcp.send_load_suite(
                                 tcp_sock, rid,
                                 suite_path=r"C:\\Users\\user\\Desktop\\TotalTest\\TotalTest.msuite"))

                elif key in ("w", "W"):
                    toggle_auto_caller()

            except (ConnectionError, OSError) as e:
                print(f"[ERROR] Connection lost: {e}")
                reconnect()

            time.sleep(0.001)

    finally:
        stop_auto_caller()
        if receiver is not None:
            try:
                receiver.stop()
            except Exception:
                pass
        _close_socket(tcp_sock)
        print("Disconnected.")


if __name__ == "__main__":
    main()
