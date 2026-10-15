# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

from skye_factr_teleop.friction_test_monitor import AbsoluteCommandCurrent


def test_absolute_command_current_tracks_selected_joint_without_sign():
    display = AbsoluteCommandCurrent(2)
    assert "等待命令电流消息" in display.line(0)
    display.update([0, 3, 0, 0, 0, 0, 0, 0], 1)
    assert display.line(1) == "J2 |命令电流| = 3.00 mA"
    display.update([0, -4, 0, 0, 0, 0, 0, 0], 1.1)
    assert display.line(1.1) == "J2 |命令电流| = 4.00 mA"
    display.update([0, 0, 0, 0, 0, 0, 0, 0], 1.2)
    assert display.line(1.2) == "J2 |命令电流| = 0.00 mA"


def test_absolute_command_current_marks_stale_and_ignores_invalid_data():
    display = AbsoluteCommandCurrent(1)
    display.update([2] * 8, 1)
    display.update([float("nan")] * 8, 1.5)
    assert display.line(1.5) == "J1 |命令电流| = 2.00 mA"
    assert "超过 1 s 无新消息" in display.line(2.1)
