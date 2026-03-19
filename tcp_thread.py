import threading

import protocol_defs as proto
import tcp_transport as tcp
import input_helper as prompt


def result_to_string(code: int):
    return proto.RESULT_CODE_MAP.get(code, f"UNKNOWN({code})")

def time_mode_to_string(mode: int):
    if mode == proto.TIME_MODE_VARIABLE:
        return "VARIABLE"
    if mode == proto.TIME_MODE_FIXED_DELTA:
        return "FIXED_DELTA"
    if mode == proto.TIME_MODE_FIXED_STEP:
        return "FIXED_STEP"
    return f"UNKNOWN({mode})"

class Receiver(threading.Thread):
    """
    TCP 응답 수신 스레드.
    - tcp.recv_packet()으로 스트림 동기화 포함 수신
    - GetStatus / CreateObject / SetSimulationTimeMode 응답은 전용 parse 후 출력
    - 그 외 메시지는 General 로그로 출력
    - pending dict에 (request_id, msg_type) event를 set 해서 동기화 신호 제공
    """
    def __init__(self, sock, pending: dict, lock: threading.Lock):
        super().__init__(daemon=True)
        self.sock = sock
        self.pending = pending
        self.lock = lock
        self.running = True

    def stop(self):
        self.running = False

    def run(self):
        try:
            while self.running:
                msg_class, msg_type, payload_size, request_id, flag, payload = tcp.recv_packet(self.sock)

                print(
                    f"[RAW][TCP] class=0x{msg_class:02X} type=0x{msg_type:04X} "
                    f"payload_size={payload_size} rid={request_id} flag={flag}"
                )

                # =========================
                # SetSimulationTimeModeCommand (0x1102)
                # =========================
                if (
                    msg_class == proto.MSG_CLASS_RESP
                    and msg_type == proto.MSG_TYPE_SET_SIMULATION_TIME_MODE_COMMAND
                ):

                    parsed = tcp.parse_set_simulation_time_mode_payload(payload)

                    if parsed is not None:
                        result_str = result_to_string(parsed["result_code"])
                        mode_str = time_mode_to_string(parsed["mode"])

                        print(
                            f"[RECV][TCP][SetSimulationTimeMode] rid={request_id} "
                            f"result={parsed['result_code']}({result_str}) "
                            f"detail={parsed['detail_code']} "
                            f"mode={parsed['mode']}({mode_str}) "
                            f"fixed_delta={parsed['fixed_delta']:.6f}"
                            f" simulation_speed={parsed['simulation_speed']:.2f}"
                        )
                    else:
                        print(
                            f"[RECV][TCP][SetSimulationTimeMode] parse failed "
                            f"rid={request_id} payload_size={payload_size}"
                        )

                # =========================
                # Get Simulation Time Status (0x1101)
                # =========================
                elif msg_class == proto.MSG_CLASS_RESP and msg_type == proto.MSG_TYPE_GET_SIMULATION_TIME_STATUS:

                    parsed = tcp.parse_get_status_payload(payload)

                    if parsed is not None:
                        result_str = result_to_string(parsed["result_code"])
                        mode_str = time_mode_to_string(parsed["mode"])

                        print(
                            f"[RECV][TCP][GetStatus] rid={request_id} "
                            f"result={parsed['result_code']}({result_str}) "
                            f"detail={parsed['detail_code']} "
                            f"mode={parsed['mode']}({mode_str}) "
                            f"fixed_delta={parsed['fixed_delta']:.6f} "
                            f"simulation_speed={parsed['simulation_speed']:.2f} "
                            f"step_index={parsed['step_index']} "
                            f"sim_time={parsed['seconds']}s {parsed['nanos']}ns"
                        )
                    else:
                        print(
                            f"[RECV][TCP][GetStatus] parse failed "
                            f"rid={request_id} payload_size={payload_size}"
                        )

                # =========================
                # CreateObject (0x1301)
                # =========================
                elif msg_class == proto.MSG_CLASS_RESP and msg_type == proto.MSG_TYPE_CREATE_OBJECT:

                    parsed = tcp.parse_create_object_payload(payload)

                    if parsed is not None:
                        result_str = result_to_string(parsed["result_code"])

                        print(
                            f"[RECV][TCP][CreateObject] rid={request_id} "
                            f"result={parsed['result_code']}({result_str}) "
                            f"detail={parsed['detail_code']} "
                            f"object_id={parsed['object_id']}"
                        )
                    else:
                        print(
                            f"[RECV][TCP][CreateObject] parse failed "
                            f"rid={request_id} payload_size={payload_size}"
                        )

                # =========================
                # ActiveSuiteStatus (0x1401)
                # =========================
                elif msg_class == proto.MSG_CLASS_RESP and msg_type == proto.MSG_TYPE_ACTIVE_SUITE_STATUS:

                    parsed = tcp.parse_active_suite_status_payload(payload)

                    if parsed is not None:
                        # 시나리오 목록 캐시 갱신
                        prompt.update_scenario_list(parsed["scenario_list"])

                        scenario_list_str = (
                            ", ".join(parsed["scenario_list"])
                            if parsed["scenario_list"]
                            else "(empty)"
                        )
                        print(
                            f"[RECV][TCP][ActiveSuiteStatus] rid={request_id} "
                            f"result={parsed['result_code']}({result_to_string(parsed['result_code'])}) "
                            f"suite={parsed['active_suite_name']!r} "
                            f"scenario={parsed['active_scenario_name']!r} "
                            f"scenario_count={len(parsed['scenario_list'])} "
                            f"scenarios=[{scenario_list_str}]"
                        )
                    else:
                        print(
                            f"[RECV][TCP][ActiveSuiteStatus] parse failed "
                            f"rid={request_id} payload_size={payload_size}"
                        )

                # =========================
                # ScenarioStatus (0x1504)
                # =========================
                elif msg_class == proto.MSG_CLASS_RESP and msg_type == proto.MSG_TYPE_SCENARIO_STATUS:
                    parsed = tcp.parse_scenario_status_payload(payload)

                    if parsed is not None:
                        STATE_MAP = {
                            1: "PLAY",
                            2: "PAUSE",
                            3: "STOP",
                        }
                        state_str = STATE_MAP.get(parsed["state"], f"UNKNOWN({parsed['state']})")

                        print(
                            f"[RECV][TCP][ScenarioStatus] rid={request_id} "
                            f"result={parsed['result_code']}({result_to_string(parsed['result_code'])}) "
                            f"detail={parsed['detail_code']} "
                            f"state={parsed['state']}({state_str})"
                        )
                    else:
                        print(
                            f"[RECV][TCP][ScenarioStatus] parse failed "
                            f"rid={request_id} payload_size={payload_size}"
                        )

                # =========================
                # General RESP
                # =========================
                elif msg_class == proto.MSG_CLASS_RESP:

                    if payload_size >= proto.RESULT_SIZE:

                        parsed = tcp.parse_result_code(payload)

                        if parsed is not None:
                            result_code, detail_code = parsed
                            result_str = result_to_string(result_code)

                            print(
                                f"[RECV][TCP][General] type=0x{msg_type:04X} "
                                f"rid={request_id} "
                                f"result={result_code}({result_str}) "
                                f"detail={detail_code}"
                            )
                        else:
                            print(
                                f"[RECV][TCP][General] type=0x{msg_type:04X} "
                                f"rid={request_id} result=parse_failed"
                            )

                    else:
                        print(
                            f"[RECV][TCP][General] type=0x{msg_type:04X} "
                            f"rid={request_id} payload_size={payload_size}"
                        )

                # =========================
                # pending sync
                # =========================
                if msg_class == proto.MSG_CLASS_RESP:
                    key = (request_id, msg_type)
                    with self.lock:
                        item = self.pending.get(key)
                        if item is not None:
                            item["ev"].set()

        except (ConnectionError, OSError) as e:
            print(f"[RECV-THREAD] stopped: {e}")
            self.running = False

        except Exception as e:
            print(f"[RECV-THREAD][UNEXPECTED] {type(e).__name__}: {e}")
            self.running = False