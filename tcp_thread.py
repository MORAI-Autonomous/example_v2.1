# tcp_thread.py
import threading

import protocol_defs as proto
import tcp_transport as tcp
import input_helper as prompt
import panels.log as log


def result_to_string(code: int):
    return proto.RESULT_CODE_MAP.get(code, f"UNKNOWN({code})")

def time_mode_to_string(mode: int):
    if mode == proto.TIME_MODE_VARIABLE:    return "VARIABLE"
    if mode == proto.TIME_MODE_FIXED_DELTA: return "FIXED_DELTA"
    if mode == proto.TIME_MODE_FIXED_STEP:  return "FIXED_STEP"
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
        try:
            while self.running:
                msg_class, msg_type, payload_size, request_id, flag, payload = \
                    tcp.recv_packet(self.sock)

                log.append(
                    f"class=0x{msg_class:02X} type=0x{msg_type:04X} "
                    f"payload={payload_size}B rid={request_id} flag={flag}",
                    "RECV"
                )

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
                        log.append(
                            f"GetStatus rid={request_id} "
                            f"result={parsed['result_code']}({result_to_string(parsed['result_code'])}) "
                            f"mode={time_mode_to_string(parsed['mode'])} "
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

                # General RESP
                elif msg_class == proto.MSG_CLASS_RESP:
                    if payload_size >= proto.RESULT_SIZE:
                        parsed = tcp.parse_result_code(payload)
                        if parsed:
                            result_code, detail_code = parsed
                            log.append(
                                f"General 0x{msg_type:04X} rid={request_id} "
                                f"result={result_code}({result_to_string(result_code)}) "
                                f"detail={detail_code}",
                                "RECV"
                            )
                        else:
                            log.append(
                                f"General 0x{msg_type:04X} rid={request_id} result=parse_failed",
                                "WARN"
                            )
                    else:
                        log.append(
                            f"General 0x{msg_type:04X} rid={request_id} "
                            f"payload_size={payload_size}",
                            "RECV"
                        )

                # pending event set
                if msg_class == proto.MSG_CLASS_RESP:
                    with self.lock:
                        item = self.pending.get((request_id, msg_type))
                        if item:
                            item["ev"].set()

        except (ConnectionError, OSError) as e:
            err_code = getattr(e, 'errno', None) or getattr(e, 'winerror', None)
            log.append(f"Receiver stopped: errno={err_code}", "ERROR")
            self.running = False
            if self.on_disconnect:
                self.on_disconnect()
        except Exception as e:
            import traceback
            log.append(f"Receiver unexpected: {type(e).__name__}: {e}", "ERROR")
            log.append(traceback.format_exc(), "ERROR")
            self.running = False
            if self.on_disconnect:
                self.on_disconnect()