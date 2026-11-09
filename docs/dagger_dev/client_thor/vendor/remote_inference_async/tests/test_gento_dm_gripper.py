"""Offline protocol regressions for the direct DM4310 gripper backend."""

import unittest
from unittest import mock

from remote_inference_async.gento_dm_gripper import (
    CHANNEL_CANFD,
    DM_ENABLED,
    GentoDmGripper,
    decode_dm_feedback,
    float_to_uint,
    terminal_can_frame,
)


def _feedback(*, slave_id: int, status: int = DM_ENABLED, position: float = 0.7) -> bytes:
    encoded_position = float_to_uint(position, -12.5, 12.5, 16)
    return bytes(
        (
            (status << 4) | slave_id,
            encoded_position >> 8,
            encoded_position & 0xFF,
            0,
            0,
            0,
            30,
            31,
        )
    )


class DecodeDmFeedbackTest(unittest.TestCase):
    def test_accepts_all_known_firmware_response_ids(self) -> None:
        slave_id = 1
        master_id = 0x11
        payload = _feedback(slave_id=slave_id)

        for can_id in (slave_id, master_id, 0):
            with self.subTest(can_id=can_id):
                decoded = decode_dm_feedback(
                    can_id,
                    payload,
                    slave_id=slave_id,
                    master_id=master_id,
                )
                self.assertIsNotNone(decoded)
                self.assertEqual(decoded["status"], DM_ENABLED)

    def test_rejects_another_motor_response(self) -> None:
        self.assertIsNone(
            decode_dm_feedback(
                2,
                _feedback(slave_id=2),
                slave_id=1,
                master_id=0x11,
            )
        )

    def test_initialization_accepts_enabled_feedback_on_slave_id(self) -> None:
        class FakeTerminalRobot:
            def __init__(self) -> None:
                self.received: list[tuple[int, bytes, int]] = []

            def terminal_clear(self, terminal_type: int) -> int:
                return 0

            def terminal_set(
                self,
                terminal_type: int,
                channel: int,
                data: bytes,
                timeout: int = 1000,
            ) -> tuple[int, int]:
                can_id = int.from_bytes(data[:4], "little")
                if data[4:12] == b"\xff" * 7 + b"\xfc":
                    self.received.append(
                        (
                            CHANNEL_CANFD,
                            terminal_can_frame(
                                can_id, _feedback(slave_id=can_id)
                            ),
                            0,
                        )
                    )
                return 0, 0

            def terminal_get(
                self, terminal_type: int, timeout: int = 1000
            ) -> tuple[int, bytes, int]:
                if self.received:
                    return self.received.pop(0)
                return 0, b"", 0

        # Skip the fixed startup drain delays. The test exercises the actual
        # enable/read/accept path which used to time out on CAN ID == SlaveID.
        with mock.patch.object(GentoDmGripper, "_drain_for", return_value=False):
            motor = GentoDmGripper(
                FakeTerminalRobot(),
                limits=(0.0, 1.6),
                terminal_type=0,
                arm_label="left",
                slave_id=1,
                master_id=0x11,
                initialization_timeout_s=0.05,
                trace_interval_s=0.0,
            )
        try:
            self.assertTrue(motor.diagnostics()["enabled"])
        finally:
            motor.close()

    def test_initialization_retries_enable_after_dropped_feedback(self) -> None:
        class FakeTerminalRobot:
            def __init__(self) -> None:
                self.received: list[tuple[int, bytes, int]] = []
                self.enable_attempts = 0

            def terminal_clear(self, terminal_type: int) -> int:
                return 0

            def terminal_set(
                self,
                terminal_type: int,
                channel: int,
                data: bytes,
                timeout: int = 1000,
            ) -> tuple[int, int]:
                can_id = int.from_bytes(data[:4], "little")
                if data[4:12] == b"\xff" * 7 + b"\xfc":
                    self.enable_attempts += 1
                    if self.enable_attempts == 2:
                        self.received.append(
                            (
                                CHANNEL_CANFD,
                                terminal_can_frame(
                                    0x11, _feedback(slave_id=can_id)
                                ),
                                0,
                            )
                        )
                return 0, 0

            def terminal_get(
                self, terminal_type: int, timeout: int = 1000
            ) -> tuple[int, bytes, int]:
                if self.received:
                    return self.received.pop(0)
                return 0, b"", 0

        robot = FakeTerminalRobot()
        with mock.patch.object(GentoDmGripper, "_drain_for", return_value=False):
            motor = GentoDmGripper(
                robot,
                limits=(0.0, 1.6),
                terminal_type=0,
                arm_label="left",
                slave_id=1,
                master_id=0x11,
                initialization_timeout_s=0.25,
                trace_interval_s=0.0,
            )
        try:
            self.assertGreaterEqual(robot.enable_attempts, 2)
            self.assertTrue(motor.diagnostics()["enabled"])
        finally:
            motor.close()


if __name__ == "__main__":
    unittest.main()
