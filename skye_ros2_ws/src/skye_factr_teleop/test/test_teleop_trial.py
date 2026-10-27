# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from skye_factr_teleop.teleop_trial import TeleopSummary, joint_packet, main
from test_sync_trial import sample


def packet(position=None, stamp=100.0):
    return {"name": [f"l_j{i}" for i in range(1, 8)], "stamp_s": stamp,
            "position": position or [0.0] * 7, "velocity": [], "effort": []}


def test_teleop_is_recorded_and_post_stop_window_is_not_cut_at_idle():
    summary = TeleopSummary(3.0)
    for seq, mode in enumerate(("IDLE", "TELEOP_SYNCING", "HOLD_SETTLING", "SYNCED", "TELEOP"), 1):
        summary.diagnostic(sample(mode, seq), 100.0 + seq * 0.01)
    for name in ("command", "applied", "feedback"):
        summary.stream(name, packet(), 100.1)
        summary.stream(name, packet([0.02] + [0.0] * 6, 100.2), 100.2)
    summary.diagnostic(sample("IDLE", 6, current_after_clip_ma=[0.0] * 8), 101.0)
    assert not summary.post_stop_complete(103.99)
    assert summary.result(103.99)["assessment"] == "INCOMPLETE"
    result = summary.result(104.0)
    assert result["assessment"] == "RECORDED"
    assert result["teleop_observed"]
    assert not result["hardware_torque_disable_verified"]
    assert result["streams"]["command"]["receive_hz"] == pytest.approx(10.0)
    assert result["streams"]["applied"]["max_abs_position_step_rad"][0] == 0.02
    assert result["transitions"][-1]["from"] == "TELEOP"


def test_missing_applied_or_no_teleop_does_not_count_as_complete():
    summary = TeleopSummary(1.0)
    summary.diagnostic(sample("TELEOP_SYNCING", 1), 100.0)
    summary.diagnostic(sample("SYNCED", 2), 100.1)
    summary.diagnostic(sample("IDLE", 3, current_after_clip_ma=[0.0] * 8), 100.2)
    for name in ("command", "feedback"):
        summary.stream(name, packet(), 100.1)
    assert summary.result(102.0)["assessment"] == "INCOMPLETE"
    summary.teleop_observed = True
    assert summary.result(102.0)["assessment"] == "INCOMPLETE"


def test_new_motion_resets_post_stop_window_and_starting_in_teleop_is_supported():
    summary = TeleopSummary(1.0)
    summary.diagnostic(sample("TELEOP", 1), 100.0)
    summary.diagnostic(sample("IDLE", 2, current_after_clip_ma=[0.0] * 8), 100.1)
    # A recording started during TELEOP must still recognize the stop.
    assert summary.idle_received_at == 100.1
    summary.diagnostic(sample("TELEOP_SYNCING", 3), 100.2)
    assert not summary.post_stop_complete(102.0)


def test_stream_gap_steps_and_timestamp_regressions_are_evidence_not_interpolated():
    summary = TeleopSummary(1.0)
    summary.stream("feedback", packet(), 100.0)
    summary.stream("feedback", packet([0.1] * 7, 99.0), 100.5)
    stats = summary.result(100.5)["streams"]["feedback"]
    assert stats["nonincreasing_stamp_count"] == 1
    assert stats["max_receive_gap_s"] == 0.5
    np.testing.assert_allclose(stats["max_abs_position_step_rad"], 0.1)


def test_idle_with_nonzero_output_does_not_reuse_previous_stop_evidence():
    summary = TeleopSummary(1.0)
    summary.diagnostic(sample("TELEOP", 1), 100.0)
    summary.diagnostic(sample("IDLE", 2, current_after_clip_ma=[0.0] * 8), 100.1)
    summary.diagnostic(sample("IDLE", 3, current_after_clip_ma=[1.0] * 8), 100.2)
    assert not summary.post_stop_complete(102.0)


@pytest.mark.parametrize("field,value", [
    ("name", [f"r_j{i}" for i in range(1, 8)]),
    ("position", [0.0] * 14),
    ("position", [float("nan")] * 7),
    ("velocity", [0.0]),
])
def test_joint_packet_rejects_invalid_axis_data(field, value):
    message = SimpleNamespace(**{key: val for key, val in packet().items() if key != "stamp_s"},
                              header=SimpleNamespace(stamp=SimpleNamespace(sec=100, nanosec=0)))
    setattr(message, field, value)
    with pytest.raises(ValueError):
        joint_packet(message, "left")


def test_joint_packet_preserves_ros_timestamp_names_and_vectors():
    message = SimpleNamespace(**{key: val for key, val in packet().items() if key != "stamp_s"},
                              header=SimpleNamespace(stamp=SimpleNamespace(sec=100, nanosec=500_000_000)))
    assert joint_packet(message, "left") == packet(stamp=100.5)
    message.header.stamp.sec = message.header.stamp.nanosec = 0
    with pytest.raises(ValueError, match="nonzero"):
        joint_packet(message, "left")


@pytest.mark.parametrize("flag,value", [("--duration", "0"), ("--post-stop-seconds", "nan")])
def test_cli_rejects_bad_timing_before_ros_is_initialized(flag, value, capsys):
    with pytest.raises(SystemExit) as exc:
        main(["--side", "left", "--config", "unused.yaml", "--driver-config", "unused.yaml",
              "--output", "unused.json", "--operator", "test", flag, value])
    assert exc.value.code == 2
    assert "finite and positive" in capsys.readouterr().err


def test_cli_rejects_wrong_side_before_ros_is_initialized(capsys):
    source = Path(__file__).parents[1] / "config/grav_comp_m6_left.yaml"
    with pytest.raises(SystemExit) as exc:
        main(["--side", "right", "--config", str(source), "--driver-config", "unused.yaml",
              "--output", "unused.json", "--operator", "test"])
    assert exc.value.code == 2
    assert "side does not match" in capsys.readouterr().err
