"""Operator feedback uses ROS only; these tests never access robot hardware."""

import json
import subprocess
import sys
import time
from types import SimpleNamespace

import pytest

from skye_factr_teleop import sync_monitor as module


ARGS = ["topic", "pub", "--once", "/mode/switch_sync", "std_msgs/msg/String",
        "{data: switch_sync}"]


def feedback(side, outcome, stamp=101.0, text="progress"):
    return SimpleNamespace(data=json.dumps({
        "side": side, "outcome": outcome, "stamp_s": stamp, "text": text,
    }))


def test_dual_feedback_waits_for_both_sides_and_ignores_old_attempts():
    output = []
    state = module.SyncFeedback(["left", "right"], 100.0,
                                output=lambda text, **kw: output.append(text))
    state.receive(feedback("right", "synced", stamp=99.0, text="old success"))
    state.receive(feedback("left", "synced"))
    assert not state.done
    state.receive(feedback("right", "active"))
    assert not state.done
    state.receive(feedback("right", "failed", text="J7 error; settle_timeout"))
    assert state.done and not state.succeeded
    assert output == ["progress", "progress", "J7 error; settle_timeout"]


def test_single_side_success_and_stop():
    state = module.SyncFeedback(["right"], 100.0, output=lambda *a, **kw: None)
    state.receive(feedback("left", "failed"))
    assert not state.done
    state.receive(feedback("right", "synced"))
    assert state.done and state.succeeded
    state.receive(feedback("right", "stopped"))
    assert state.done and not state.succeeded


def test_entrypoint_uses_selected_sides_and_forwards_other_payloads(monkeypatch):
    monkeypatch.setenv("RIGHT_CONFIG", "/config/right.yaml")
    monkeypatch.delenv("LEFT_CONFIG", raising=False)
    monkeypatch.setattr(module.shutil, "which", lambda name: "/opt/ros/humble/bin/ros2")
    calls = []
    monkeypatch.setattr(module, "monitor", lambda cmd, sides: calls.append((cmd, sides)) or 0)
    assert module.main(ARGS) == 0
    assert calls == [(["/opt/ros/humble/bin/ros2", *ARGS], ["right"])]
    forwarded = []
    monkeypatch.setattr(module.subprocess, "call", lambda cmd: forwarded.append(cmd) or 7)
    args = [*ARGS[:-1], "{data: other}"]
    assert module.main(args) == 7
    assert forwarded == [["/opt/ros/humble/bin/ros2", *args]]


@pytest.mark.parametrize("outcome", ["synced", "failed"])
def test_real_cli_receives_feedback_in_operator_process(monkeypatch, tmp_path, outcome):
    rclpy = pytest.importorskip("rclpy")
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from std_msgs.msg import String

    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_logs"))
    monkeypatch.setenv("ROS_DOMAIN_ID", "197")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("LEFT_CONFIG", "/fake/left.yaml")
    monkeypatch.setenv("RIGHT_CONFIG", "/fake/right.yaml")
    context = Context()
    context.init(args=[], domain_id=197)
    node = Node("fake_sync_feedback", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pubs = {side: node.create_publisher(String, f"/leader_arm/{side}/sync_progress", qos)
            for side in ("left", "right")}
    received = []

    def on_sync(msg):
        received.append(msg.data)
        for side, pub in pubs.items():
            # Even an immediate reply must be visible in terminal B.
            pub.publish(String(data=feedback(side, outcome, stamp=time.time(),
                                            text=f"[{side}] {outcome}").data))

    node.create_subscription(String, "/mode/switch_sync", on_sync, 10)
    # Retained feedback from an earlier attempt must never complete this one.
    for side, pub in pubs.items():
        pub.publish(String(data=feedback(side, "synced", stamp=1.0, text="old attempt").data))
    operator = subprocess.Popen([sys.executable, "-m", "skye_factr_teleop.sync_monitor", *ARGS],
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        deadline = time.monotonic() + 12
        while operator.poll() is None and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        assert operator.poll() is not None, "operator monitor did not finish"
        output = operator.communicate()[0]
        assert operator.returncode == (0 if outcome == "synced" else 1), output
        assert received == ["switch_sync"]
        assert f"[left] {outcome}" in output and f"[right] {outcome}" in output
        assert "old attempt" not in output
    finally:
        if operator.poll() is None:
            operator.terminate()
            operator.wait(timeout=5)
        executor.shutdown()
        node.destroy_node()
        context.try_shutdown()
