# tcp_thread.py
import socket
import threading

import transport.protocol_defs as proto
import transport.tcp_transport as tcp
import utils.input_helper as prompt
import panels.log as log


def result_to_string(code: int):
    return proto.RESULT_CODE_MAP.get(code, f"UNKNOWN({code})")

def time_mode_to_string(mode: int):
    if mode == proto.TIME_MODE_VARIABLE:    return "VARIABLE"
    if mode == proto.TIME_MODE_FIXED:       return "FIXED"
    if mode == proto.TIME_MODE_FIXED_STEP:  return "FIXED_STEP_LEGACY"
    return f"UNKNOWN({mode})"


class Receiver(threading.Thread):
    def __init__(self, sock, pending: dict, lock: threading.Lock, on_disconnect=None):
        super().__init__(daemon=True)
        self.sock          = sock
        self.pending       = pending
        self.lock          = lock
        self.running       = True
        self.on_disconnect = on_disconnect

    def stop(self):
        self.running = False

    def run(self):
        while self.running:
            # recv_packet만 별도 try — socket.timeout(recv 주기 만료)은 재연결 없이 continue
            try:
                msg_class, msg_type, payload_size, request_id, flag, payload = \
                    tcp.recv_packet(self.sock)
            except socket.timeout:
                continue
            except (ConnectionError, OSError) as e:
                err_code = getattr(e, 'errno', None) or getattr(e, 'winerror', None)
                log.append(f"Receiver stopped: errno={err_code}", "ERROR")
                self.running = False
                if self.on_disconnect:
                    self.on_disconnect()
                return
            except Exception as e:
                import traceback
                log.append(f"Receiver unexpected: {type(e).__name__}: {e}", "ERROR")
                log.append(traceback.format_exc(), "ERROR")
                self.running = False
                if self.on_disconnect:
                    self.on_disconnect()
                return

            # SetSimulationTimeModeCommand (0x1102)
            if msg_class == proto.MSG_CLASS_RESP \
                    and msg_type == proto.MSG_TYPE_SET_SIMULATION_TIME_MODE_COMMAND:
                parsed = tcp.parse_set_simulation_time_mode_payload(payload)
                if parsed:
                    log.append(
                        f"SetSimulationTimeMode rid={request_id} "
                        f"result={parsed['result_code']}({result_to_string(parsed['result_code'])}) "
                        f"mode={time_mode_to_string(parsed['mode'])} "
                        f"fixed_delta={parsed['fixed_delta']:.3f}",
                        "RECV"
                    )
                else:
                    log.append(f"SetSimulationTimeMode parse_failed rid={request_id}", "WARN")

            # GetSimulationTimeStatus (0x1101)
            elif msg_class == proto.MSG_CLASS_RESP \
                    and msg_type == proto.MSG_TYPE_GET_SIMULATION_TIME_STATUS:
                parsed = tcp.parse_get_status_payload(payload)
                if parsed:
                    if parsed["mode"] == proto.TIME_MODE_VARIABLE:
                        mode_detail = (
                            f"target_fps={parsed['target_fps']} "
                            f"physics_dt={parsed['physics_delta_time']}ms "
                            f"speed={parsed['simulation_speed']:.2f}"
                        )
                    elif parsed["mode"] == proto.TIME_MODE_FIXED:
                        mode_detail = (
                            f"sim_dt={parsed['simulation_delta_time']}ms "
                            f"physics_dt={parsed['physics_delta_time']}ms "
                            f"rtf={parsed['rtf']} user_control={parsed['user_control']}"
                        )
                    else:
                        mode_detail = ""
                    log.append(
                        f"GetStatus rid={request_id} "
                        f"result={parsed['result_code']}({result_to_string(parsed['result_code'])}) "
                        f"mode={time_mode_to_string(parsed['mode'])} "
                        f"{mode_detail} "
                        f"step={parsed['step_index']} "
                        f"sim={parsed['seconds']}s {parsed['nanos']}ns",
                        "RECV"
                    )
                else:
                    log.append(f"GetStatus parse_failed rid={request_id}", "WARN")

            # CreateObject (0x1301)
            elif msg_class == proto.MSG_CLASS_RESP \
                    and msg_type == proto.MSG_TYPE_CREATE_OBJECT:
                parsed = tcp.parse_create_object_payload(payload)
                if parsed:
                    log.append(
                        f"CreateObject rid={request_id} "
                        f"result={parsed['result_code']}({result_to_string(parsed['result_code'])}) "
                        f"object_id={parsed['object_id']}",
                        "RECV"
                    )
                else:
                    log.append(f"CreateObject parse_failed rid={request_id}", "WARN")

            # ActiveSuiteStatus (0x1401)
            elif msg_class == proto.MSG_CLASS_RESP \
                    and msg_type == proto.MSG_TYPE_ACTIVE_SUITE_STATUS:
                parsed = tcp.parse_active_suite_status_payload(payload)
                if parsed:
                    prompt.update_scenario_list(parsed["scenario_list"])
                    scenarios = ", ".join(parsed["scenario_list"]) or "(empty)"
                    log.append(
                        f"ActiveSuiteStatus rid={request_id} "
                        f"suite={parsed['active_suite_name']!r} "
                        f"scenario={parsed['active_scenario_name']!r} "
                        f"list=[{scenarios}]",
                        "RECV"
                    )
                else:
                    log.append(f"ActiveSuiteStatus parse_failed rid={request_id}", "WARN")

            # ScenarioStatus (0x1504)
            elif msg_class == proto.MSG_CLASS_RESP \
                    and msg_type == proto.MSG_TYPE_SCENARIO_STATUS:
                parsed = tcp.parse_scenario_status_payload(payload)
                if parsed:
                    state_str = {1:"PLAY", 2:"PAUSE", 3:"STOP"}.get(
                        parsed["state"], f"UNKNOWN({parsed['state']})")
                    log.append(
                        f"ScenarioStatus rid={request_id} "
                        f"result={parsed['result_code']}({result_to_string(parsed['result_code'])}) "
                        f"state={state_str}",
                        "RECV"
                    )
                else:
                    log.append(f"ScenarioStatus parse_failed rid={request_id}", "WARN")

            # General RESP — 오류만 로그
            elif msg_class == proto.MSG_CLASS_RESP:
                if payload_size >= proto.RESULT_SIZE:
                    parsed = tcp.parse_result_code(payload)
                    if parsed:
                        result_code, detail_code = parsed
                        if result_code != 0:
                            log.append(
                                f"0x{msg_type:04X} rid={request_id} "
                                f"result={result_code}({result_to_string(result_code)}) "
                                f"detail={detail_code}",
                                "ERROR"
                            )
                    else:
                        log.append(
                            f"0x{msg_type:04X} rid={request_id} result=parse_failed",
                            "WARN"
                        )

            # pending event set + cleanup
            if msg_class == proto.MSG_CLASS_RESP:
                with self.lock:
                    item = self.pending.pop((request_id, msg_type), None)
                    if item:
                        item["ev"].set()
