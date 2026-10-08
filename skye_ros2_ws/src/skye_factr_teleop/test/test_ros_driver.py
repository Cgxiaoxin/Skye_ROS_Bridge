# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

"""Task 5.1: actual rclpy nodes with injected devices; no robot access."""

from pathlib import Path
from types import SimpleNamespace
import os
import subprocess
import time

import numpy as np
import pytest
import yaml

pytest.importorskip("rclpy")
from rclpy.context import Context
from rclpy.parameter import Parameter
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from conftest import FakeDynamixelIO
from test_control_core import FakeDynamics
from test_sync_hold import FakeTrajectory
from skye_factr_teleop import ros_driver as module
from skye_factr_teleop.state_machine import ControlMode


class Hardware(FakeDynamixelIO):
    def __init__(self):
        super().__init__(np.zeros(8), np.zeros(8), torque_enabled=False)
        self.open_calls = []
        self.close_calls = 0

    def open(self, *, enable_torque=True):
        self.open_calls.append(enable_torque)
        self.set_torque_enabled(enable_torque)

    def close(self):
        self.close_calls += 1
        super().close()


class Recorder:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def packet(side="left", *, stamp=100.0):
    message = JointState()
    message.name = [f'{"l" if side == "left" else "r"}_j{i}' for i in range(1, 8)]
    message.position = [0.1] * 7
    message.header.stamp.sec = int(stamp)
    message.header.stamp.nanosec = int(round((stamp - int(stamp)) * 1e9))
    return message


def settle(rig, *, follower=True):
    rig.hardware.inject_state(rig.node.core._target * rig.node.core.joint_signs,
                              np.zeros(8))
    rig.clock.now += 0.002
    if follower:
        rig.node._on_follower(packet(rig.node.side, stamp=rig.clock.now))
    rig.node._tick()
    rig.clock.now += rig.node.config.controller.hold_ruckig_control.settle_duration + 0.002
    if follower:
        rig.node._on_follower(packet(rig.node.side, stamp=rig.clock.now))
    rig.node._tick()


@pytest.fixture
def context(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_logs"))
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    context = Context()
    context.init(args=[], domain_id=193)
    yield context
    context.try_shutdown()


@pytest.fixture
def make_node(context, monkeypatch, tmp_path):
    nodes = []

    def make(side="left", *, parameters=None, config_path_override=None,
             hardware=None, record_publishers=True, **kwargs):
        source = Path(__file__).parents[1] / "config" / f"grav_comp_m6_{side}.yaml"
        document = yaml.safe_load(source.read_text(encoding="utf-8"))
        document["runtime"] = {
            "joint_offsets": [0.0] * 8, "current_limit_ma": 50.0,
            "max_velocity": [0.5] * 8, "max_acceleration": [1.0] * 8,
            "max_jerk": [4.0] * 8,
        }
        document["runtime"].update(parameters or {})
        config_path = tmp_path / f"{side}_{len(nodes)}.yaml"
        config_path.write_text(yaml.safe_dump(document), encoding="utf-8")
        hardware = hardware or Hardware()
        trajectories = []
        clock = SimpleNamespace(now=100.0)

        def factory():
            trajectory = FakeTrajectory()
            trajectories.append(trajectory)
            return trajectory

        node = module.SkyeFactrTeleopNode(
            side, context=context, use_global_arguments=False,
            parameter_overrides=[Parameter("config_path", value=str(config_path_override or config_path))],
            hardware_factory=lambda config: hardware,
            dynamics_factory=lambda config: FakeDynamics(),
            trajectory_factory=factory, monotonic=lambda: clock.now, **kwargs,
        )
        nodes.append(node)
        monkeypatch.setattr(node, "_ros_now", lambda: clock.now)
        pubs = list(node.publishers)
        if record_publishers:
            for name in ("_state_pub", "_current_pub", "_target_pub", "_command_pub"):
                setattr(node, name, Recorder())
        return SimpleNamespace(node=node, hardware=hardware, trajectories=trajectories,
                               clock=clock, publishers=pubs)

    yield make
    for node in nodes:
        node.destroy_node()


@pytest.mark.parametrize("side", ["left", "right"])
def test_interfaces_qos_and_disabled_startup(make_node, side):
    rig = make_node(side)
    node = rig.node
    assert node.get_name() == f"skye_factr_teleop_{side}"
    assert rig.hardware.open_calls == [False]
    assert not rig.hardware.torque_enabled
    assert node._published_state == "IDLE"
    assert node._timer.timer_period_ns == 2_000_000
    pubs = {pub.topic_name: pub for pub in rig.publishers}
    assert {"/teleop/state", "/leader_arm/current_state",
            "/leader_arm/target_joint_state", "/joint_control"} <= set(pubs)
    assert pubs["/joint_control"].qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
    assert pubs["/joint_control"].qos_profile.depth == 1
    assert pubs["/teleop/state"].qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL
    for topic in ("/leader_arm/current_state", "/leader_arm/target_joint_state"):
        assert pubs[topic].qos_profile.reliability == ReliabilityPolicy.RELIABLE
        assert pubs[topic].qos_profile.durability == DurabilityPolicy.VOLATILE
    subs = {sub.topic_name: sub for sub in node.subscriptions}
    assert {"/joint_state", "/joint_move", "/mode/switch_sync",
            "/mode/switch_teleop", "/mode/switch_stop"} <= set(subs)
    assert subs["/joint_state"].qos_profile.reliability == ReliabilityPolicy.RELIABLE
    assert subs["/joint_state"].qos_profile.depth == 1


@pytest.mark.parametrize("side", ["left", "right"])
def test_sync_hold_teleop_stop_and_published_shapes(make_node, side):
    rig = make_node(side)
    node = rig.node
    node._on_sync(String(data="switch_sync"))
    assert not rig.hardware.torque_enabled
    node._on_follower(packet(side))
    node._on_sync(String(data="switch_sync"))
    assert node.core.mode == ControlMode.TELEOP_SYNCING
    node._on_teleop(String(data="switch_teleop"))
    assert node.core.mode == ControlMode.TELEOP_SYNCING
    rig.trajectories[0].finished = True
    rig.clock.now += 0.002
    node._tick()
    assert node.core.mode == ControlMode.HOLD_SETTLING
    assert len(node._target_pub.messages[-1].position) == 7
    assert len(node._current_pub.messages[-1].velocity) == 7
    node._on_teleop(String(data="switch_teleop"))
    assert node.core.mode == ControlMode.HOLD_SETTLING
    settle(rig)
    assert node.core.mode == ControlMode.SYNCED
    node._on_teleop(String(data="switch_teleop"))
    rig.clock.now += 0.002
    target_count = len(node._target_pub.messages)
    node._tick()
    command = node._command_pub.messages[-1]
    assert command.name == packet(side).name
    assert len(command.position) == 7 and np.all(np.isfinite(command.position))
    assert len(command.velocity) == len(command.effort) == 0
    assert len(node._target_pub.messages) == target_count
    node._on_stop(String(data="switch_stop"))
    rig.clock.now += 0.002
    node._tick()
    assert len(node._command_pub.messages) == 1
    assert not rig.hardware.torque_enabled
    assert [msg.data for msg in node._state_pub.messages] == [
        "TELEOP_SYNCING", "HOLD_SETTLING", "SYNCED", "TELEOP", "IDLE",
    ]
    np.testing.assert_array_equal(rig.hardware.last_currents_ma, np.zeros(8))


def test_joint_move_holds_but_does_not_grant_teleop(make_node):
    rig = make_node()
    node = rig.node
    node._on_joint_move(packet())
    assert node.core.mode == ControlMode.TELEOP_SYNCING
    rig.trajectories[0].finished = True
    rig.clock.now += 0.002
    node._tick()
    assert node.core.mode == ControlMode.HOLD_SETTLING
    settle(rig, follower=False)
    assert node.core.mode == ControlMode.SYNCED
    node._on_follower(packet(stamp=rig.clock.now))
    node._on_teleop(String(data="switch_teleop"))
    assert node.core.mode == ControlMode.SYNCED
    node._on_joint_move(packet(stamp=rig.clock.now))
    assert len(rig.trajectories) == 1
    node._on_stop(None)
    node._on_sync(String(data="switch_sync"))
    assert len(rig.trajectories) == 2


@pytest.mark.parametrize("mode", ["sync", "hold", "teleop"])
def test_stale_follower_stops_all_follower_modes(make_node, mode):
    rig = make_node()
    node = rig.node
    node._on_follower(packet())
    node._on_sync(String(data="switch_sync"))
    if mode != "sync":
        rig.trajectories[0].finished = True
        rig.clock.now += 0.002
        node._tick()
    if mode == "teleop":
        settle(rig)
        node._on_teleop(String(data="switch_teleop"))
    rig.clock.now += 0.3
    node._tick()
    assert node.core.mode == ControlMode.IDLE
    assert not rig.hardware.torque_enabled
    assert node._command_pub.messages == []


def test_receive_timeout_survives_paused_ros_clock(make_node, monkeypatch):
    rig = make_node()
    rig.node._on_follower(packet())
    rig.node._on_sync(String(data="switch_sync"))
    monkeypatch.setattr(rig.node, "_ros_now", lambda: 100.0)
    rig.clock.now += 0.3
    rig.node._tick()
    assert rig.node.core.mode == ControlMode.IDLE


@pytest.mark.parametrize("field,value", [
    ("position", [0.0] * 14), ("position", [float("nan")] * 7),
    ("velocity", [0.0]), ("effort", [float("inf")] * 7),
    ("name", [f"r_j{i}" for i in range(1, 8)]),
    ("name", [f"l_j{i}" for i in range(7, 0, -1)]), ("name", []),
])
def test_invalid_follower_packet_stops_active_sync(make_node, field, value):
    rig = make_node()
    rig.node._on_follower(packet())
    rig.node._on_sync(String(data="switch_sync"))
    bad = packet()
    setattr(bad, field, value)
    rig.node._on_follower(bad)
    assert rig.node.core.mode == ControlMode.IDLE
    assert rig.node._follower is None
    assert not rig.hardware.torque_enabled


@pytest.mark.parametrize("stamp", [0.0, 99.0, 101.0])
def test_joint_move_timestamp_rejection_never_enables_torque(make_node, stamp):
    rig = make_node()
    rig.node._on_joint_move(packet(stamp=stamp))
    assert rig.node.core.mode == ControlMode.IDLE
    assert not rig.hardware.torque_enabled
    assert rig.trajectories == []


def test_unnamed_fixed_order_requires_explicit_parameter(make_node):
    rig = make_node(parameters={"allow_unnamed_joint_states": True})
    message = packet()
    message.name = []
    rig.node._on_follower(message)
    rig.node._on_sync(String(data="switch_sync"))
    assert rig.node.core.mode == ControlMode.TELEOP_SYNCING


def test_input_snapshot_is_copied(make_node):
    rig = make_node()
    message = packet()
    rig.node._on_follower(message)
    message.position[0] = 2.0
    rig.node._on_sync(String(data="switch_sync"))
    np.testing.assert_allclose(rig.trajectories[0].target[:7], 0.1)


@pytest.mark.parametrize("fault", ["read", "zero_write", "publish"])
def test_shutdown_closes_even_when_cleanup_or_callback_fails(make_node, fault):
    rig = make_node()
    rig.node._on_joint_move(packet())
    if fault == "read":
        rig.hardware.fail_next_read(RuntimeError("read failed"))
        rig.node._tick()
    elif fault == "zero_write":
        rig.hardware.fail_next_write(RuntimeError("zero failed"))
        rig.node.shutdown_control()
    else:
        rig.node._current_pub.publish = lambda message: (_ for _ in ()).throw(RuntimeError("publish failed"))
        rig.node._tick()
    assert rig.node._closed
    assert rig.node.core.mode == ControlMode.IDLE
    assert rig.node._timer.is_canceled()
    assert rig.hardware.close_calls == 1
    assert not rig.hardware.torque_enabled
    rig.node.shutdown_control()
    assert rig.hardware.close_calls == 1


@pytest.mark.parametrize("key,value", [
    ("joint_offsets", [0.0] * 7), ("current_limit_ma", 0.0),
    ("max_jerk", [-1.0] * 8), ("state_timeout", -0.1),
])
def test_invalid_runtime_values_fail_before_hardware_open(make_node, key, value):
    hardware = Hardware()
    with pytest.raises(ValueError):
        make_node(parameters={key: value}, hardware=hardware)
    assert hardware.open_calls == []


def test_missing_runtime_fails_before_hardware_open(make_node):
    hardware = Hardware()
    template = Path(__file__).parents[1] / "config/grav_comp_m6_left.yaml"
    with pytest.raises(ValueError, match="runtime is required"):
        make_node(config_path_override=template, hardware=hardware)
    assert hardware.open_calls == []


def test_parameter_changes_are_rejected(make_node):
    node = make_node().node
    result = node.set_parameters([Parameter("config_path", value="/other.yaml")])
    assert not result[0].successful


@pytest.mark.parametrize("failure", [KeyboardInterrupt, RuntimeError])
def test_main_cleans_up_on_signal_or_exception(monkeypatch, failure):
    events = []
    fake = SimpleNamespace(destroy_node=lambda: events.append("destroy"))
    monkeypatch.setattr(module.rclpy, "init", lambda **kwargs: events.append("init"))
    monkeypatch.setattr(module.rclpy, "ok", lambda: True)
    monkeypatch.setattr(module.rclpy, "shutdown", lambda: events.append("shutdown"))
    monkeypatch.setattr(module, "SkyeFactrTeleopNode", lambda side: fake)
    monkeypatch.setattr(module.rclpy, "spin", lambda node: (_ for _ in ()).throw(failure()))
    if failure is RuntimeError:
        with pytest.raises(RuntimeError):
            module.main(["--side", "right"])
    else:
        module.main(["--side", "right"])
    assert events == ["init", "destroy", "shutdown"]

def test_timer_is_driven_by_executor(make_node, context):
    from rclpy.executors import SingleThreadedExecutor

    rig = make_node()
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(rig.node)
    try:
        for _ in range(20):
            executor.spin_once(timeout_sec=0.05)
            if rig.node._current_pub.messages:
                break
        assert rig.node._current_pub.messages
        assert rig.node._command_pub.messages == []
    finally:
        executor.remove_node(rig.node)
        executor.shutdown()


def test_sync_timeout_with_fresh_input_publishes_idle(make_node):
    rig = make_node()
    rig.node._on_follower(packet())
    rig.node._on_sync(String(data="switch_sync"))
    rig.clock.now += rig.node.config.controller.teleop_sync_timeout
    rig.node._on_follower(packet(stamp=rig.clock.now))
    rig.node._tick()
    assert rig.node.core.mode == ControlMode.IDLE
    assert rig.node.core.last_transition.reason == "sync_timeout"
    assert rig.node._state_pub.messages[-1].data == "IDLE"
    assert not rig.hardware.torque_enabled
    assert rig.trajectories[0].calls == 0


def test_constructor_failure_closes_partially_open_device(make_node):
    hardware = Hardware()

    def fail_open(**kwargs):
        hardware.set_torque_enabled(True)
        raise RuntimeError("open failed")

    hardware.open = fail_open
    with pytest.raises(RuntimeError, match="open failed"):
        make_node(hardware=hardware)
    assert hardware.close_calls == 1
    assert not hardware.torque_enabled


def test_config_path_resolution(monkeypatch, tmp_path):
    monkeypatch.setattr(module, "get_package_share_directory", lambda package: str(tmp_path))
    assert module.resolve_config_path("", "right") == tmp_path / "config/grav_comp_m6_right.yaml"
    assert module.resolve_config_path("custom.yaml", "left") == tmp_path / "config/custom.yaml"
    absolute = tmp_path / "elsewhere.yaml"
    assert module.resolve_config_path(str(absolute), "left") == absolute


def _spin_until(executor, predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        executor.spin_once(timeout_sec=0.02)
    assert predicate(), "ROS graph or message delivery did not reach the expected state"


def test_dual_ros_graph_types_remaps_and_isolation(make_node, context):
    """Exercise both remapped adapters and CLI modes through real DDS."""
    from rclpy.executors import SingleThreadedExecutor

    def remaps(side):
        return ["--ros-args", "-r", f"/joint_state:=/gento/{side}_joint_states",
                "-r", f"/joint_control:=/gento/{side}_joint_control",
                "-r", f"/joint_move:=/{side}_joint_move",
                "-r", f"/leader_arm/current_state:=/{side}_leader_arm/current_state",
                "-r", f"/leader_arm/target_joint_state:=/{side}_leader_arm/target_joint_state"]

    rigs = {side: make_node(side, record_publishers=False, cli_args=remaps(side))
            for side in ("left", "right")}
    probe = module.rclpy.create_node("ros_interface_probe", context=context,
                                     use_global_arguments=False)
    executor = SingleThreadedExecutor(context=context)
    commands = {side: [] for side in rigs}
    current = {side: [] for side in rigs}
    target = {side: [] for side in rigs}
    states = []
    try:
        for rig in rigs.values():
            executor.add_node(rig.node)
        executor.add_node(probe)
        follower_pubs = {}
        for side, rig in rigs.items():
            node = rig.node
            expected_pubs = {
                "/teleop/state": "std_msgs/msg/String",
                f"/{side}_leader_arm/current_state": "sensor_msgs/msg/JointState",
                f"/{side}_leader_arm/target_joint_state": "sensor_msgs/msg/JointState",
                f"/gento/{side}_joint_control": "sensor_msgs/msg/JointState",
            }
            expected_subs = {
                f"/gento/{side}_joint_states": "sensor_msgs/msg/JointState",
                f"/{side}_joint_move": "sensor_msgs/msg/JointState",
                "/mode/switch_sync": "std_msgs/msg/String",
                "/mode/switch_teleop": "std_msgs/msg/String",
                "/mode/switch_stop": "std_msgs/msg/String",
            }
            assert {pub.topic_name for pub in node.publishers} - {"/parameter_events"} == set(expected_pubs)
            assert {sub.topic_name for sub in node.subscriptions} == set(expected_subs)
            for topic, msg_type in expected_pubs.items():
                assert [info.topic_type for info in node.get_publishers_info_by_topic(topic)
                        if info.node_name == node.get_name()] == [msg_type]
            for topic, msg_type in expected_subs.items():
                assert [info.topic_type for info in node.get_subscriptions_info_by_topic(topic)
                        if info.node_name == node.get_name()] == [msg_type]
            command_info = node.get_publishers_info_by_topic(
                f"/gento/{side}_joint_control")[0]
            follower_info = node.get_subscriptions_info_by_topic(
                f"/gento/{side}_joint_states")[0]
            assert command_info.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
            assert follower_info.qos_profile.reliability == ReliabilityPolicy.RELIABLE
            assert next(pub for pub in node.publishers if pub.topic_name ==
                        f"/gento/{side}_joint_control").qos_profile.depth == 1
            assert next(sub for sub in node.subscriptions if sub.topic_name ==
                        f"/gento/{side}_joint_states").qos_profile.depth == 1
            follower_pubs[side] = probe.create_publisher(
                JointState, f"/gento/{side}_joint_states", module.state_qos())
            probe.create_subscription(
                JointState, f"/gento/{side}_joint_control",
                lambda message, side=side: commands[side].append(message),
                module.command_qos())
            probe.create_subscription(
                JointState, f"/{side}_leader_arm/current_state",
                lambda message, side=side: current[side].append(message),
                module.state_qos())
            probe.create_subscription(
                JointState, f"/{side}_leader_arm/target_joint_state",
                lambda message, side=side: target[side].append(message),
                module.state_qos())
        def publish_mode(mode):
            environment = {**os.environ, "ROS_DOMAIN_ID": "193"}
            subprocess.run(
                ["ros2", "topic", "pub", "--once", f"/mode/switch_{mode}",
                 "std_msgs/msg/String", f"{{data: switch_{mode}}}"],
                check=True, timeout=20, capture_output=True, text=True,
                env=environment,
            )
        probe.create_subscription(String, "/teleop/state",
                                  lambda message: states.append(message.data),
                                  module.latched_state_qos())
        _spin_until(executor, lambda: all(
            probe.count_subscribers(f"/gento/{side}_joint_states") == 1
            and probe.count_publishers(f"/gento/{side}_joint_control") == 1
            for side in rigs))
        _spin_until(executor, lambda: "IDLE" in states)
        _spin_until(executor, lambda: all(current[side] for side in rigs))
        assert all(current[side][-1].name == packet(side).name for side in rigs)
        assert all(probe.count_subscribers(f"/mode/switch_{mode}") == 2
                   for mode in ("sync", "teleop", "stop"))

        follower_pubs["left"].publish(packet("left"))
        _spin_until(executor, lambda: rigs["left"].node._follower is not None)
        publish_mode("sync")
        _spin_until(executor, lambda: rigs["left"].node.core.mode == ControlMode.TELEOP_SYNCING)
        assert rigs["right"].node.core.mode == ControlMode.IDLE
        assert rigs["right"].node._follower is None

        follower_pubs["right"].publish(packet("right"))
        _spin_until(executor, lambda: rigs["right"].node._follower is not None)
        publish_mode("sync")
        _spin_until(executor, lambda: rigs["right"].node.core.mode == ControlMode.TELEOP_SYNCING)
        for rig in rigs.values():
            rig.trajectories[0].finished = True
        _spin_until(executor, lambda: all(
            rig.node.core.mode == ControlMode.HOLD_SETTLING for rig in rigs.values()))
        for rig in rigs.values():
            rig.hardware.inject_state(rig.node.core._target * rig.node.core.joint_signs,
                                      np.zeros(8))
        for _ in range(4):
            for side, rig in rigs.items():
                rig.clock.now += 0.1
                follower_pubs[side].publish(packet(side, stamp=rig.clock.now))
            _spin_until(executor, lambda: all(
                rig.node._follower.received_at == rig.clock.now
                for rig in rigs.values()))
            for _ in range(5):
                executor.spin_once(timeout_sec=0.01)
        _spin_until(executor, lambda: all(
            rig.node.core.mode == ControlMode.SYNCED for rig in rigs.values()))
        _spin_until(executor, lambda: all(target[side] for side in rigs))
        assert all(target[side][-1].name == packet(side).name for side in rigs)
        publish_mode("teleop")
        _spin_until(executor, lambda: all(
            rig.node.core.mode == ControlMode.TELEOP for rig in rigs.values()))
        _spin_until(executor, lambda: all(commands[side] for side in rigs))
        for side in rigs:
            command = commands[side][-1]
            assert command.name == packet(side).name
            assert len(command.position) == 7
            assert np.all(np.isfinite(command.position))
            assert not command.velocity and not command.effort

        publish_mode("stop")
        _spin_until(executor, lambda: all(
            rig.node.core.mode == ControlMode.IDLE for rig in rigs.values()))
        _spin_until(executor, lambda: states and states[-1] == "IDLE")
        counts = {side: len(messages) for side, messages in commands.items()}
        for _ in range(10):
            executor.spin_once(timeout_sec=0.01)
        assert {side: len(messages) for side, messages in commands.items()} == counts
        for state in ("TELEOP_SYNCING", "HOLD_SETTLING", "SYNCED", "TELEOP", "IDLE"):
            assert state in states
        assert all(not rig.hardware.torque_enabled for rig in rigs.values())
    finally:
        for rig in rigs.values():
            executor.remove_node(rig.node)
        executor.remove_node(probe)
        executor.shutdown()
        probe.destroy_node()
