import socket
import threading
import time
from protocol_defs import *
import tcp_transport as tcp
import tcp_thread as tcp_thread
import automation as ac
import key_input as key_input
import commands as commands
import input_helper as prompt

# =========================
# Pending (request sync)
# key: (request_id, msg_type) -> {"t": float, "ev": Event}
# =========================
def pending_add(pending: dict, lock: threading.Lock, request_id: int, msg_type: int) -> threading.Event:
    ev = threading.Event()
    with lock:
        pending[(request_id, msg_type)] = {"t": time.time(), "ev": ev}
    return ev

def pending_pop(pending: dict, lock: threading.Lock, request_id: int, msg_type: int):
    with lock:
        pending.pop((request_id, msg_type), None)

def connect_and_start_receiver(pending, lock):
    tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    while True:
        try:
            print("[INFO] Attempting to connect...")
            tcp_sock.connect((TCP_SERVER_IP, TCP_SERVER_PORT))
            print("[INFO] Connected.")
            receiver = tcp_thread.Receiver(tcp_sock, pending, lock)
            receiver.start()
            return tcp_sock, receiver
        except Exception as e:
            print(f"[ERROR] Connect failed: {e}. Retrying in 5 seconds...")
            try:
                tcp_sock.close()
            except Exception:
                pass
            time.sleep(5)
            tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            tcp_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

def print_key_bindings():
    print("Press [1] : Send ManualCommand (UDP)")
    print("Press [2] : Send GetStatus (TCP 0x1201)")
    print("Press [3] : Send FixedStep (TCP 0x1200)")
    print("Press [4] : Send SaveData (TCP 0x1101)")
    print("Press [5] : Send Create Object (TCP 0x1103)")
    print("Press [6] : Send ManualControlById (TCP 0x1104)")
    print("Press [7] : Send TransformControlById (TCP 0x1105)")    
    print("Press [8] : Send TransformControl (UDP)")
    #print(f"Press [9] : Toggle AutoCall (FixedStep <-> SaveData) x {MAX_CALL_NUM}")
    print("Press [Q] : Quit\n")

# =========================
# Main
# =========================
def main():
    udp_send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    pending = {}
    lock = threading.Lock()
    request_id_ref = {"value": 1}

    receiver = None
    auto_caller = None  # auto 변수명 충돌 방지

    # ✅ 초기 연결: 실패해도 계속 재시도
    tcp_sock, receiver = connect_and_start_receiver(pending, lock)

    print_key_bindings()

    manual_throttle = 0.3
    manual_brake = 0.0
    manual_steer = 0.0

    try:
        while True:
            key = key_input.get_key()
            if key in ("q", "Q"):
                break

            try:
                if key == "1":
                    commands.send_manual_udp(
                        udp_send_sock, manual_throttle, manual_brake, manual_steer
                    )

                elif key == "2":
                    with lock:
                        rid = request_id_ref["value"]
                        request_id_ref["value"] += 1
                    pending_add(pending, lock, rid, MSG_TYPE_GET_STATUS)
                    tcp.send_get_status(tcp_sock, rid)

                elif key == "3":
                    with lock:
                        rid = request_id_ref["value"]
                        request_id_ref["value"] += 1
                    pending_add(pending, lock, rid, MSG_TYPE_FIXED_STEP)
                    tcp.send_fixed_step(tcp_sock, rid, step_count=1)

                elif key == "4":
                    with lock:
                        rid = request_id_ref["value"]
                        request_id_ref["value"] += 1
                    pending_add(pending, lock, rid, MSG_TYPE_SAVE_DATA)
                    tcp.send_save_data(tcp_sock, rid)

                elif key == "5":
                    params = prompt.prompt_create_object()
                    with lock:
                        rid = request_id_ref["value"]
                        request_id_ref["value"] += 1
                    pending_add(pending, lock, rid, MSG_TYPE_CREATE_OBJECT)
                    tcp.send_create_object(tcp_sock, rid, **params)

                elif key == "6":
                    params = prompt.prompt_manual_control_by_id()
                    with lock:
                        rid = request_id_ref["value"]
                        request_id_ref["value"] += 1
                    pending_add(pending, lock, rid, MSG_TYPE_MANUAL_CONTROL_BY_ID_COMMAND)
                    tcp.send_manual_control_by_id(tcp_sock, rid, **params)

                elif key == "7":
                    params = prompt.prompt_transform_control_by_id()
                    with lock:
                        rid = request_id_ref["value"]
                        request_id_ref["value"] += 1
                    pending_add(pending, lock, rid, MSG_TYPE_TRANSFORM_CONTROL_BY_ID_COMMAND)
                    tcp.send_transform_control_by_id(tcp_sock, rid, **params)

                elif key == "8":
                    params = prompt.prompt_transform_control()
                    commands.send_transform_control_udp(udp_send_sock, **params)

                elif key == "9":
                    # Toggle AutoCaller
                    if auto_caller is None or not auto_caller.is_alive():
                        auto_caller = ac.AutoCaller(
                            tcp_sock=tcp_sock,
                            pending=pending,
                            lock=lock,
                            request_id_ref=request_id_ref,
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
                
                

            except (ConnectionError, OSError) as e:
                print(f"[ERROR] Connection lost: {e}")

                # stop auto on disconnect
                if auto_caller is not None and auto_caller.is_alive():
                    auto_caller.stop()
                    auto_caller = None

                # stop old receiver
                if receiver is not None:
                    try:
                        receiver.stop()
                    except Exception:
                        pass
                    receiver = None

                # close tcp socket
                try:
                    tcp_sock.shutdown(socket.SHUT_RDWR)
                except Exception:
                    pass
                try:
                    tcp_sock.close()
                except Exception:
                    pass

                # ✅ 여기서도 동일 함수로 재연결 + receiver 재시작
                tcp_sock, receiver = connect_and_start_receiver(pending, lock)
                print_key_bindings()

            time.sleep(0.001)

    finally:
        if auto_caller is not None and auto_caller.is_alive():
            auto_caller.stop()

        if receiver is not None:
            try:
                receiver.stop()
            except Exception:
                pass

        try:
            tcp_sock.shutdown(socket.SHUT_RDWR)
        except Exception:
            pass
        try:
            tcp_sock.close()
        except Exception:
            pass

        try:
            udp_send_sock.close()
        except Exception:
            pass

        print("Disconnected.")

if __name__ == "__main__":
    main()