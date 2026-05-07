from __future__ import annotations

import struct
import unittest

from transport.message_schema import pack_message_payload
import transport.protocol_defs as proto
import transport.tcp_transport as tcp


class TcpPayloadGoldenTests(unittest.TestCase):
    def test_set_simulation_time_mode_variable_payload(self) -> None:
        expected = struct.pack("<iiif", 1, 60, 10, 1.25)
        actual = pack_message_payload(
            proto.MSG_TYPE_SET_SIMULATION_TIME_MODE_COMMAND,
            {
                "mode": proto.TIME_MODE_VARIABLE,
                "target_fps": 60,
                "physics_delta_time": 10,
                "simulation_speed": 1.25,
            },
        )
        self.assertEqual(actual, expected)

    def test_set_simulation_time_mode_fixed_payload(self) -> None:
        expected = struct.pack("<iiiii", 2, 33, 10, 1, 0)
        actual = pack_message_payload(
            proto.MSG_TYPE_SET_SIMULATION_TIME_MODE_COMMAND,
            {
                "mode": proto.TIME_MODE_FIXED,
                "simulation_delta_time": 33,
                "physics_delta_time": 10,
                "rtf": 1,
                "user_control": 0,
            },
        )
        self.assertEqual(actual, expected)

    def test_manual_control_by_id_payload(self) -> None:
        entity_id = "Car_1"
        expected = (
            struct.pack("<I", len(entity_id.encode("utf-8")))
            + entity_id.encode("utf-8")
            + struct.pack("<ddd", 0.4, 0.0, 12.5)
        )
        actual = tcp.build_manual_control_by_id_payload(entity_id, 0.4, 0.0, 12.5)
        self.assertEqual(actual, expected)

    def test_transform_control_by_id_payload(self) -> None:
        entity_id = "Car_2"
        expected = (
            struct.pack("<I", len(entity_id.encode("utf-8")))
            + entity_id.encode("utf-8")
            + struct.pack("<fffffffd", -1.0, 2.5, 3.0, 10.0, 20.0, 30.0, 4.5, 6.75)
        )
        actual = tcp.build_transform_control_by_id_payload(
            entity_id,
            -1.0,
            2.5,
            3.0,
            10.0,
            20.0,
            30.0,
            4.5,
            6.75,
        )
        self.assertEqual(actual, expected)

    def test_set_trajectory_payload(self) -> None:
        entity_id = "Car_1"
        trajectory_name = "lane_change"
        points = [
            (1.0, 2.0, 3.0, 0.0),
            (4.0, 5.0, 6.0, 0.5),
        ]
        expected = (
            struct.pack("<I", len(entity_id.encode("utf-8")))
            + entity_id.encode("utf-8")
            + struct.pack("<i", 1)
            + struct.pack("<I", len(trajectory_name.encode("utf-8")))
            + trajectory_name.encode("utf-8")
            + struct.pack("<I", len(points))
            + b"".join(struct.pack("<dddd", *point) for point in points)
        )
        actual = tcp.build_set_trajectory_payload(
            entity_id=entity_id,
            follow_mode=1,
            trajectory_name=trajectory_name,
            points=points,
        )
        self.assertEqual(actual, expected)

    def test_load_suite_payload(self) -> None:
        suite_path = "C:/Suite/Test.suite"
        expected = struct.pack("<I", len(suite_path.encode("utf-8"))) + suite_path.encode("utf-8")
        actual = pack_message_payload(
            proto.MSG_TYPE_LOAD_SUITE,
            {"suite_path": suite_path},
        )
        self.assertEqual(actual, expected)

    def test_scenario_control_payload(self) -> None:
        expected = struct.pack("<I", 3) + struct.pack("<I", 0)
        actual = pack_message_payload(
            proto.MSG_TYPE_SCENARIO_CONTROL,
            {
                "command": 3,
                "scenario_name": "",
            },
        )
        self.assertEqual(actual, expected)

    def test_parse_get_status_variable_payload(self) -> None:
        payload = struct.pack(
            proto.GET_STATUS_VARIABLE_FMT,
            0,
            0,
            proto.TIME_MODE_VARIABLE,
            60,
            10,
            1.25,
            123,
            45,
            678,
        )
        parsed = tcp.parse_get_status_payload(payload)
        self.assertIsNotNone(parsed)
        self.assertEqual(parsed["result_code"], 0)
        self.assertEqual(parsed["detail_code"], 0)
        self.assertEqual(parsed["mode"], proto.TIME_MODE_VARIABLE)
        self.assertEqual(parsed["target_fps"], 60)
        self.assertEqual(parsed["physics_delta_time"], 10)
        self.assertAlmostEqual(parsed["simulation_speed"], 1.25)
        self.assertEqual(parsed["step_index"], 123)
        self.assertEqual(parsed["seconds"], 45)
        self.assertEqual(parsed["nanos"], 678)

    def test_parse_get_status_fixed_payload(self) -> None:
        payload = struct.pack(
            proto.GET_STATUS_FIXED_FMT,
            0,
            0,
            proto.TIME_MODE_FIXED,
            33,
            10,
            2,
            1,
            456,
            78,
            901,
        )
        parsed = tcp.parse_get_status_payload(payload)
        self.assertIsNotNone(parsed)
        self.assertEqual(parsed["mode"], proto.TIME_MODE_FIXED)
        self.assertEqual(parsed["simulation_delta_time"], 33)
        self.assertEqual(parsed["physics_delta_time"], 10)
        self.assertEqual(parsed["rtf"], 2)
        self.assertEqual(parsed["user_control"], 1)
        self.assertEqual(parsed["step_index"], 456)
        self.assertEqual(parsed["seconds"], 78)
        self.assertEqual(parsed["nanos"], 901)

    def test_parse_create_object_payload(self) -> None:
        object_id = "Car_9"
        payload = (
            struct.pack(proto.RESULT_FMT, 0, 0)
            + struct.pack("<I", len(object_id.encode("utf-8")))
            + object_id.encode("utf-8")
        )
        parsed = tcp.parse_create_object_payload(payload)
        self.assertIsNotNone(parsed)
        self.assertEqual(parsed["object_id"], object_id)

    def test_parse_active_suite_status_payload(self) -> None:
        suite_name = "SuiteA"
        scenario_name = "Scenario01"
        scenario_list = ["Scenario01", "Scenario02"]
        payload = (
            struct.pack(proto.RESULT_FMT, 0, 0)
            + struct.pack("<I", len(suite_name.encode("utf-8")))
            + suite_name.encode("utf-8")
            + struct.pack("<I", len(scenario_name.encode("utf-8")))
            + scenario_name.encode("utf-8")
            + struct.pack("<I", len(scenario_list))
            + b"".join(
                struct.pack("<I", len(name.encode("utf-8"))) + name.encode("utf-8")
                for name in scenario_list
            )
        )
        parsed = tcp.parse_active_suite_status_payload(payload)
        self.assertIsNotNone(parsed)
        self.assertEqual(parsed["active_suite_name"], suite_name)
        self.assertEqual(parsed["active_scenario_name"], scenario_name)
        self.assertEqual(parsed["scenario_list"], scenario_list)

    def test_parse_scenario_status_payload(self) -> None:
        payload = struct.pack("<III", 0, 0, 1)
        parsed = tcp.parse_scenario_status_payload(payload)
        self.assertIsNotNone(parsed)
        self.assertEqual(parsed["result_code"], 0)
        self.assertEqual(parsed["detail_code"], 0)
        self.assertEqual(parsed["state"], 1)


if __name__ == "__main__":
    unittest.main()
