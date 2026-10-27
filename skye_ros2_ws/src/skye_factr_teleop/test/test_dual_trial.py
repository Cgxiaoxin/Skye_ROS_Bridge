# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from skye_factr_teleop.dual_trial import DualRecorder
from test_sync_trial import sample


@pytest.fixture
def args(tmp_path):
    root = Path(__file__).parents[1]
    return SimpleNamespace(
        phase="sync", left_config=root / "config/grav_comp_m6_left.yaml",
        right_config=root / "config/grav_comp_m6_right.yaml",
        left_output=tmp_path / "left.json", right_output=tmp_path / "right.json",
        driver_config=root.parent / "skye_robot_driver/config/skye_robot.yaml",
        operator="test", duration=180.0, post_stop_seconds=3.0,
    )


def diagnostic(side, mode, seq, now=100.0):
    packet = sample(mode, seq, stamp_s=now)
    packet["side"] = side
    if mode == "IDLE":
        packet["current_before_clip_ma"] = packet["current_after_clip_ma"] = [0.0] * 8
    return packet


def joint_message(side, value):
    return SimpleNamespace(
        name=[f"{'l' if side == 'left' else 'r'}_j{i}" for i in range(1, 8)],
        position=[value] * 7, velocity=[], effort=[],
        header=SimpleNamespace(stamp=SimpleNamespace(sec=100, nanosec=0)),
    )


def test_sync_waits_for_both_sides_and_preserves_separate_results(args, capsys):
    with DualRecorder(args) as recorder:
        for side in ("left", "right"):
            recorder.diagnostic(side, diagnostic(side, "TELEOP_SYNCING", 1), 100.0)
        recorder.diagnostic("left", diagnostic("left", "IDLE", 2), 100.0)
        assert not recorder.ready_to_finish(100.0, 100.0)
        recorder.diagnostic("right", diagnostic("right", "SYNCED", 2), 100.0)
        recorder.diagnostic("right", diagnostic("right", "IDLE", 3), 100.1)
        assert recorder.ready_to_finish(100.1, 100.0)
        recorder.save("both_stopped", 100.1)
    left = json.loads(args.left_output.read_text())
    right = json.loads(args.right_output.read_text())
    assert left["states"] == ["TELEOP_SYNCING", "IDLE"]
    assert right["states"] == ["TELEOP_SYNCING", "SYNCED", "IDLE"]
    assert left["assessment"] == right["assessment"] == "RECORDED"
    for side, output in (("left", args.left_output), ("right", args.right_output)):
        packets = [json.loads(line)["data"] for line in output.with_suffix(".trace.jsonl").read_text().splitlines()]
        assert all(packet["side"] == side for packet in packets)
    text = capsys.readouterr().out
    assert "[left] IDLE" in text and "[right] SYNCED" in text


def test_teleop_waits_for_later_stop_tail_and_records_both_command_streams(args):
    args.phase = "teleop"
    with DualRecorder(args) as recorder:
        for side, value in (("left", 0.1), ("right", -0.2)):
            recorder.diagnostic(side, diagnostic(side, "TELEOP", 1), 100.0)
            for event in ("command", "applied", "feedback"):
                recorder.joint(side, event, joint_message(side, value), 100.0)
        recorder.diagnostic("left", diagnostic("left", "IDLE", 2), 101.0)
        assert not recorder.ready_to_finish(104.0, 100.0)
        recorder.diagnostic("right", diagnostic("right", "IDLE", 2), 103.0)
        # Fresh IDLE samples must not restart the per-side three-second tail.
        for side in ("left", "right"):
            recorder.diagnostic(side, diagnostic(side, "IDLE", 3), 105.9)
        assert not recorder.ready_to_finish(105.9, 100.0)
        assert recorder.ready_to_finish(106.0, 100.0)
        recorder.save("post_stop_window", 106.0)
    left = json.loads(args.left_output.read_text())
    right = json.loads(args.right_output.read_text())
    assert left["streams"]["command"]["first"]["position"] == [0.1] * 7
    assert left["streams"]["other_command"]["first"]["position"] == [-0.2] * 7
    assert right["streams"]["command"]["first"]["position"] == [-0.2] * 7
    assert right["streams"]["other_command"]["first"]["position"] == [0.1] * 7
    assert left["assessment"] == right["assessment"] == "RECORDED"


def test_missing_side_saves_partial_and_never_reports_both_finished(args):
    with DualRecorder(args) as recorder:
        recorder.diagnostic("left", diagnostic("left", "TELEOP_SYNCING", 1), 100.0)
        recorder.diagnostic("left", diagnostic("left", "IDLE", 2), 100.0)
        assert not recorder.ready_to_finish(100.0, 100.0)
        recorder.save("interrupted", 100.0)
    right = json.loads(args.right_output.read_text())
    assert right["assessment"] == "INCOMPLETE"
    assert right["states"] == []
    assert right["ended_by"] == "interrupted"


def test_output_collision_is_rejected_before_creating_any_trace(args):
    args.right_output.write_text("existing record")
    with pytest.raises(FileExistsError):
        DualRecorder(args)
    assert not args.left_output.exists()
    assert not args.left_output.with_suffix(".trace.jsonl").exists()
    assert args.right_output.read_text() == "existing record"
    args.right_output.unlink()
    args.right_output = args.left_output.with_suffix(".trace.jsonl")
    with pytest.raises(ValueError, match="distinct"):
        DualRecorder(args)


def test_sync_does_not_finish_on_stale_or_resumed_side(args):
    with DualRecorder(args) as recorder:
        for side in ("left", "right"):
            recorder.diagnostic(side, diagnostic(side, "TELEOP_SYNCING", 1), 100.0)
            recorder.diagnostic(side, diagnostic(side, "IDLE", 2), 100.0)
        assert not recorder.ready_to_finish(102.0, 100.0)
        recorder.diagnostic("left", diagnostic("left", "TELEOP_SYNCING", 3), 102.0)
        recorder.diagnostic("right", diagnostic("right", "IDLE", 3), 102.0)
        assert not recorder.ready_to_finish(102.0, 100.0)


def test_automatic_sessions_are_unique_and_print_matching_readback_commands(args, capsys, tmp_path):
    import shlex

    args.left_output = args.right_output = None
    args.output_dir = tmp_path / "sessions with spaces"
    sessions = []
    for _ in range(2):
        with DualRecorder(args) as recorder:
            recorder.save("interrupted", 100.0)
            sessions.append(recorder.session_dir)
        output = capsys.readouterr().out
        commands = [shlex.split(line) for line in output.splitlines() if line.startswith("ros2 run ")]
        assert len(commands) == 2
        for side, command in zip(("left", "right"), commands):
            record_path = sessions[-1] / f"{side}.json"
            assert json.loads(record_path.read_text())["side"] == side
            assert command[command.index("--output") + 1] == str(sessions[-1] / f"{side}_torque_disable.json")
            assert command[command.index("--config") + 1] == str(getattr(args, f"{side}_config").resolve())
            assert not (sessions[-1] / f"{side}_torque_disable.json").exists()
    assert sessions[0] != sessions[1]
    assert all((session / "left.trace.jsonl").exists() for session in sessions)


@pytest.mark.parametrize("partial", [False, True])
def test_auto_directory_and_explicit_outputs_cannot_be_mixed(args, partial):
    args.output_dir = args.left_output.parent / "sessions"
    if partial:
        args.right_output = None
    with pytest.raises(ValueError):
        DualRecorder(args)
    assert not args.output_dir.exists()


@pytest.mark.parametrize("phase", ["sync", "teleop"])
def test_one_ros_recorder_receives_both_arms_and_waits_for_both_stops(args, phase, monkeypatch):
    """Real DDS messages, no leader/follower hardware or motion requests."""
    import os
    import subprocess
    import time

    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    monkeypatch.setenv("ROS_DOMAIN_ID", "194")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ROS_LOG_DIR", str(args.left_output.parent / "ros_logs"))
    context = Context()
    rclpy.init(context=context)
    node = Node("dual_recorder_test_source", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    best_effort = QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT)
    reliable = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
    diagnostics = {side: node.create_publisher(String, f"/leader_arm/{side}/sync_diagnostics", best_effort)
                   for side in ("left", "right")}
    publishers = {}
    if phase == "teleop":
        for side in ("left", "right"):
            for event, suffix, qos in (("command", "joint_control", best_effort),
                                       ("applied", "joint_action_applied", reliable),
                                       ("feedback", "joint_states", reliable)):
                publishers[side, event] = node.create_publisher(JointState, f"/gento/{side}_{suffix}", qos)
    modes = {"left": "IDLE", "right": "IDLE"}
    sequence = {"left": 0, "right": 0}
    output_dir = args.left_output.parent / "sessions"
    command = ["/usr/bin/python3", "-m", "skye_factr_teleop.dual_trial", "--phase", phase,
               "--left-config", str(args.left_config), "--right-config", str(args.right_config),
               "--output-dir", str(output_dir),
               "--driver-config", str(args.driver_config), "--operator", "fake-ROS",
               "--duration", "15", "--post-stop-seconds", "0.15"]
    log_path = args.left_output.parent / "recorder.log"

    def spin_for(seconds, until=lambda: False):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and not until():
            stamp = node.get_clock().now()
            for side in ("left", "right"):
                sequence[side] += 1
                packet = diagnostic(side, modes[side], sequence[side], stamp.nanoseconds * 1e-9)
                packet["monotonic_s"] = time.monotonic()
                diagnostics[side].publish(String(data=json.dumps(packet)))
                if phase == "teleop":
                    for event in ("command", "applied", "feedback"):
                        if event != "feedback" and modes[side] != "TELEOP":
                            continue
                        msg = JointState()
                        msg.header.stamp = stamp.to_msg()
                        msg.name = [f"{'l' if side == 'left' else 'r'}_j{i}" for i in range(1, 8)]
                        msg.position = [0.1 if side == "left" else -0.2] * 7
                        publishers[side, event].publish(msg)
            executor.spin_once(timeout_sec=0.005)

    with log_path.open("w") as log:
        process = subprocess.Popen(command, env=os.environ.copy(), stdout=log, stderr=subprocess.STDOUT)
        try:
            ready = lambda: all(p.get_subscription_count() > 0
                                for p in [*diagnostics.values(), *publishers.values()])
            spin_for(8, ready)
            assert ready(), log_path.read_text()
            assert not node.get_publishers_info_by_topic("/mode/switch_sync")
            assert not node.get_publishers_info_by_topic("/mode/switch_teleop")
            assert not node.get_publishers_info_by_topic("/mode/switch_stop")
            modes.update(left="TELEOP_SYNCING", right="TELEOP_SYNCING")
            spin_for(0.15)
            if phase == "teleop":
                modes.update(left="TELEOP", right="TELEOP")
                spin_for(0.2)
            modes["left"] = "IDLE"
            if phase == "sync":
                modes["right"] = "SYNCED"
            spin_for(0.3)
            assert process.poll() is None, log_path.read_text()
            assert not list(output_dir.glob("*/left.json")) and not list(output_dir.glob("*/right.json"))
            modes["right"] = "IDLE"
            spin_for(8, lambda: process.poll() is not None)
            assert process.poll() == 0, log_path.read_text()
            sessions = list(output_dir.iterdir())
            assert len(sessions) == 1
            for side in ("left", "right"):
                output = sessions[0] / f"{side}.json"
                result = json.loads(output.read_text())
                assert result["side"] == side
                assert result["assessment"] == "RECORDED"
                assert result["missing_samples"] == 0
                assert result["idle_zero_output_and_cached_disable_observed"]
                if phase == "teleop":
                    assert result["post_stop_window_complete"]
                    assert result["streams"]["command"]["count"] > 0
                    assert result["streams"]["other_command"]["count"] > 0
                else:
                    assert result["follower_command_count"] == 0
            text = log_path.read_text()
            assert "[left] IDLE" in text and "[right] IDLE" in text
        finally:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=5)
            executor.remove_node(node)
            executor.shutdown()
            node.destroy_node()
            context.shutdown()
