import time
import unittest
from dataclasses import dataclass, field

from skye_leader_bridge.mapping import (
    ArmMappingConfig,
    BridgeSafetyConfig,
    GripperMappingConfig,
    LeaderBridgeState,
    fill_joint_command_message,
    map_arm_positions,
    map_gripper_value,
)
from skye_leader_bridge.prepare import find_first_available_service, is_service_available, set_int_request_value
from skye_leader_bridge.node import build_arm_runtime


class MappingTest(unittest.TestCase):
    def test_map_arm_positions_applies_order_sign_offset_and_limits(self):
        cfg = ArmMappingConfig(
            joint_order=[2, 0, 1],
            signs=[1.0, -1.0, 1.0],
            offsets=[0.1, 0.2, -0.3],
            limits_min=[-1.0, -1.0, -1.0],
            limits_max=[1.0, 1.0, 1.0],
        )

        self.assertEqual(map_arm_positions([0.5, 2.0, -3.0], cfg), [-1.0, -0.3, 1.0])

    def test_map_arm_positions_rejects_missing_or_bad_dimensions(self):
        cfg = ArmMappingConfig(joint_order=[0, 1, 2])

        with self.assertRaisesRegex(ValueError, "at least 3"):
            map_arm_positions([0.0, 1.0], cfg)

        bad_cfg = ArmMappingConfig(joint_order=[0, 1], signs=[1.0])
        with self.assertRaisesRegex(ValueError, "signs"):
            map_arm_positions([0.0, 1.0], bad_cfg)

    def test_gripper_mapping_supports_inversion_deadband_and_output_range(self):
        cfg = GripperMappingConfig(
            input_min=0.0,
            input_max=1.0,
            output_min=0.0,
            output_max=90.0,
            invert=True,
            deadband=0.02,
        )

        self.assertEqual(map_gripper_value(0.0, cfg), 90.0)
        self.assertEqual(map_gripper_value(1.0, cfg), 0.0)
        self.assertEqual(map_gripper_value(0.5, cfg), 45.0)
        self.assertEqual(map_gripper_value(0.01, cfg), 90.0)
        self.assertEqual(map_gripper_value(0.99, cfg), 0.0)

    def test_bridge_state_gates_publish_by_enable_timeout_and_delta(self):
        safety = BridgeSafetyConfig(
            leader_timeout_s=0.2,
            max_delta_per_cycle=0.1,
            require_enable=True,
        )
        state = LeaderBridgeState(safety=safety)
        now = time.monotonic()

        state.update_leader("left", [0.0, 0.0, 0.0], now=now)
        self.assertIsNone(state.command_for_arm("left", [0.0, 0.0, 0.0], now=now))

        state.set_enabled(True)
        self.assertEqual(
            state.command_for_arm("left", [0.05, -0.05, 0.0], now=now),
            [0.05, -0.05, 0.0],
        )
        limited = state.command_for_arm("left", [0.5, 0.0, 0.0], now=now)
        self.assertEqual([round(value, 6) for value in limited], [0.15, 0.0, 0.0])
        self.assertIsNone(state.command_for_arm("left", [0.0, 0.0, 0.0], now=now + 1.0))

    def test_fill_joint_command_message_sets_configured_field(self):
        msg = FakeJointCmd()

        fill_joint_command_message(msg, [1.0, 2.0, 3.0], ["joint_pos"])

        self.assertEqual(msg.joint_pos, [1.0, 2.0, 3.0])

    def test_fill_joint_command_message_supports_numbered_fields(self):
        msg = FakeNumberedJointCmd()

        fill_joint_command_message(msg, [1.0, 2.0, 3.0], ["missing"])

        self.assertEqual([msg.joint_0, msg.joint_1, msg.joint_2], [1.0, 2.0, 3.0])

    def test_set_int_request_value_uses_data_or_first_slot(self):
        data_request = FakeDataRequest()
        slot_request = FakeSlotRequest()

        set_int_request_value(data_request, 10)
        set_int_request_value(slot_request, 3)

        self.assertEqual(data_request.data, 10)
        self.assertEqual(slot_request.value, 3)

    def test_find_first_available_service_prefers_first_matching_candidate(self):
        available = [
            ("/control/set_mode", ["marvin_msgs/srv/Int"]),
            ("/control/set_acc_ratio", ["marvin_msgs/srv/Int"]),
            ("/control/set_accel_ratio", ["marvin_msgs/srv/Int"]),
        ]

        self.assertEqual(
            find_first_available_service(
                available,
                ["/control/set_accel_ratio", "/control/set_acc_ratio"],
            ),
            "/control/set_accel_ratio",
        )

    def test_is_service_available_checks_service_names(self):
        available = [
            ("/control/set_mode", ["marvin_msgs/srv/Int"]),
            ("/control/set_ready", ["std_srvs/srv/Trigger"]),
        ]

        self.assertTrue(is_service_available(available, "/control/set_mode"))
        self.assertFalse(is_service_available(available, "/control/clear_fault"))

    def test_gento_arm_output_uses_joint_state(self):
        runtime = build_arm_runtime(
            "right",
            {
                "input_topic": "/right_joint_control",
                "output_topic": "/gento/right_joint_control",
                "output_type": "joint_state",
            },
        )

        self.assertEqual(runtime.output_type, "joint_state")


@dataclass
class FakeJointCmd:
    joint_pos: list[float] = field(default_factory=list)


class FakeNumberedJointCmd:
    def __init__(self):
        self.joint_0 = 0.0
        self.joint_1 = 0.0
        self.joint_2 = 0.0


class FakeDataRequest:
    def __init__(self):
        self.data = 0


class FakeSlotRequest:
    __slots__ = ("value",)

    def __init__(self):
        self.value = 0


if __name__ == "__main__":
    unittest.main()
