# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

"""Task 5.1: actual rclpy nodes with injected devices; no robot access."""

from dataclasses import replace
import json
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


def test_leader_dynamics_loads_when_gravity_comp_is_disabled(monkeypatch):
    config = module.load_config(Path(__file__).parents[1] / "config/grav_comp_m6_left.yaml")
    gravity = replace(config.controller.joint_gravity_comp, enable=False)
    config = replace(config, controller=replace(config.controller, joint_gravity_comp=gravity))
    monkeypatch.setattr(module, "get_package_share_directory", lambda package: f"/share/{package}")
    monkeypatch.setattr(module, "PinocchioDynamics", lambda path: path)

    assert module._make_pinocchio_dynamics(config) == Path(
        "/share/skye_factr_description/urdf/m6left_isomorphism.urdf"
    )


class Hardware(FakeDynamixelIO):
    def __init__(self):
        super().__init__(np.zeros(8), np.zeros(8), torque_enabled=False)
        self.open_calls = []
        self.close_calls = 0
        self.present_currents_ma = np.zeros(8)

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
    rig.clock.now += rig.node.config.controller.sync_hold_control.hold.settle_duration + 0.002
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
             hardware=None, record_publishers=True, gravity_test=None, limit_test=None,
             friction_test=None, diagnostics=False, use_default_factory=False, **kwargs):
        source = Path(__file__).parents[1] / "config" / f"grav_comp_m6_{side}.yaml"
        document = yaml.safe_load(source.read_text(encoding="utf-8"))
        document["dynamixel"].update({
            "joint_offsets": [0.0] * 8, "current_limit_ma": [50.0] * 8,
        })
        if gravity_test is None and limit_test is None and friction_test is None:
            document["controller"]["sync_hold_control"]["ruckig"].update({
                "max_velocity": [0.5] * 8, "max_acceleration": [1.0] * 8,
                "max_jerk": [4.0] * 8,
            })
        for key, value in (parameters or {}).items():
            target = document["dynamixel"] if key in ("joint_offsets", "current_limit_ma") else document.setdefault("runtime", {})
            target[key] = value
        config_path = tmp_path / f"{side}_{len(nodes)}.yaml"
        config_path.write_text(yaml.safe_dump(document), encoding="utf-8")
        hardware = hardware or Hardware()
        trajectories = []
        clock = SimpleNamespace(now=100.0)

        def factory():
            trajectory = FakeTrajectory()
            trajectories.append(trajectory)
            return trajectory

        overrides = [Parameter("config_path", value=str(config_path_override or config_path))]
        overrides.append(Parameter("sync_diagnostics", value=diagnostics))
        if gravity_test is not None:
            overrides.append(Parameter("gravity_test_only", value=True))
            overrides.extend(Parameter(f"gravity_test_{key}", value=value)
                             for key, value in gravity_test.items())
        if limit_test is not None:
            overrides.append(Parameter("limit_test_only", value=True))
            overrides.extend(Parameter(f"limit_test_{key}", value=value)
                             for key, value in limit_test.items())
        if friction_test is not None:
            overrides.append(Parameter("friction_test_only", value=True))
            overrides.extend(Parameter(f"friction_test_{key}", value=value)
                             for key, value in friction_test.items())
        node = module.SkyeFactrTeleopNode(
            side, context=context, use_global_arguments=False,
            parameter_overrides=overrides,
            hardware_factory=None if use_default_factory else lambda config: hardware,
            dynamics_factory=lambda config: FakeDynamics(),
            trajectory_factory=factory, monotonic=lambda: clock.now, **kwargs,
        )
        nodes.append(node)
        monkeypatch.setattr(node, "_ros_now", lambda: clock.now)
        pubs = list(node.publishers)
        if record_publishers:
            for name in ("_state_pub", "_current_pub", "_target_pub", "_command_pub"):
                setattr(node, name, Recorder())
            if diagnostics:
                node._diagnostic_pub = Recorder()
            if gravity_test is not None or limit_test is not None or friction_test is not None:
                node._test_current_pub = Recorder()
                node._test_present_current_pub = Recorder()
            if limit_test is not None:
                node._test_gripper_pub = Recorder()
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
def test_gravity_test_node_has_isolated_interface_and_stops(make_node, side):
    rig = make_node(side, gravity_test={
        "joint_id": 6,
        "gain_scale": 0.5,
        "current_limit_ma": 20.0,
        "ramp_seconds": 2.0,
        "max_duration_seconds": 5.0,
    })
    node = rig.node
    subs = {sub.topic_name for sub in node.subscriptions}
    assert subs == {f"/mode/{side}/switch_gravity_test", "/mode/switch_stop"}
    assert rig.trajectories == []
    assert rig.hardware.open_calls == [False]
    assert not rig.hardware.torque_enabled
    assert f"/leader_arm/{side}/gravity_test_present_current_ma" in {
        pub.topic_name for pub in rig.publishers
    }

    node._on_gravity_test(String(data="switch_gravity_test"))
    assert node.core.mode == ControlMode.GRAVITY_TEST
    assert rig.hardware.torque_enabled
    rig.hardware.present_currents_ma[5] = -3.0
    rig.clock.now += 1.0
    node._tick()
    assert rig.hardware.last_currents_ma[5] * node.core.joint_signs[5] > 0
    np.testing.assert_array_equal(np.delete(rig.hardware.last_currents_ma, 5), np.zeros(7))
    assert node._test_current_pub.messages[-1].data[5] * node.core.joint_signs[5] > 0
    assert node._test_present_current_pub.messages[-1].data[5] == -3.0
    assert not node._command_pub.messages
    assert not node._target_pub.messages
    node._on_stop(String(data="switch_stop"))
    assert node.core.mode == ControlMode.IDLE
    assert not rig.hardware.torque_enabled
    np.testing.assert_array_equal(rig.hardware.last_currents_ma, np.zeros(8))
    node._on_gravity_test(String(data="switch_gravity_test"))
    rig.clock.now += 5.0
    node._tick()
    assert node.core.mode == ControlMode.IDLE
    assert node.core.last_transition.reason == "gravity_test_timeout"
    assert not rig.hardware.torque_enabled


def test_gravity_test_destroy_disables_torque(make_node):
    rig = make_node(gravity_test={
        "joint_id": 1,
        "gain_scale": 0.1,
        "current_limit_ma": 20.0,
        "ramp_seconds": 2.0,
        "max_duration_seconds": 5.0,
    })
    rig.node._on_gravity_test(String(data="switch_gravity_test"))
    assert rig.hardware.torque_enabled
    rig.node.destroy_node()
    assert not rig.hardware.torque_enabled
    np.testing.assert_array_equal(rig.hardware.last_currents_ma, np.zeros(8))


@pytest.mark.parametrize("side", ["left", "right"])
def test_limit_test_node_isolated_and_stops(make_node, side):
    rig = make_node(side, limit_test={
        "joint_id": 1, "gain_scale": 0.5, "current_limit_ma": 20.0,
        "ramp_seconds": 2.0, "max_duration_seconds": 5.0,
    })
    node = rig.node
    assert {sub.topic_name for sub in node.subscriptions} == {
        f"/mode/{side}/switch_limit_test", "/mode/switch_stop",
    }
    assert rig.trajectories == []
    assert not rig.hardware.torque_enabled
    assert f"/leader_arm/{side}/limit_test_gripper_state" in {
        pub.topic_name for pub in rig.publishers
    }
    node._on_limit_test(String(data="switch_limit_test"))
    assert node.core.mode == ControlMode.LIMIT_TEST
    rig.clock.now += 1
    node._tick()
    assert not node._command_pub.messages
    assert not node._target_pub.messages
    assert len(node._test_gripper_pub.messages[-1].data) == 2
    node._on_stop(String(data="switch_stop"))
    assert node.core.mode == ControlMode.IDLE
    assert not rig.hardware.torque_enabled
    np.testing.assert_array_equal(rig.hardware.last_currents_ma, np.zeros(8))
    node._on_limit_test(String(data="switch_limit_test"))
    rig.clock.now += 5
    node._tick()
    assert node.core.last_transition.reason == "limit_test_timeout"
    assert not rig.hardware.torque_enabled


def test_limit_test_destroy_disables_torque(make_node):
    rig = make_node(limit_test={
        "joint_id": 8, "gain_scale": 0.5, "current_limit_ma": 20.0,
        "ramp_seconds": 2.0, "max_duration_seconds": 5.0,
    })
    rig.node._on_limit_test(String(data="switch_limit_test"))
    rig.node.destroy_node()
    assert not rig.hardware.torque_enabled
    np.testing.assert_array_equal(rig.hardware.last_currents_ma, np.zeros(8))


@pytest.mark.parametrize("side", ["left", "right"])
def test_friction_test_node_isolated_and_stops(make_node, side):
    settings = {"joint_id": 1, "gain_scale": 0.5, "current_limit_ma": 20.0,
                "ramp_seconds": 2.0, "max_duration_seconds": 5.0, "enabled": True}
    rig = make_node(side, friction_test=settings)
    node = rig.node
    assert {sub.topic_name for sub in node.subscriptions} == {
        f"/mode/{side}/switch_friction_test",
        f"/mode/{side}/friction_test_output", "/mode/switch_stop",
    }
    assert rig.trajectories == []
    assert not rig.hardware.torque_enabled
    assert f"/leader_arm/{side}/friction_test_current_ma" in {
        pub.topic_name for pub in rig.publishers
    }
    node._on_friction_test(String(data="switch_friction_test"))
    assert node.core.mode == ControlMode.FRICTION_TEST
    assert node._published_friction_phase == "on"
    rig.clock.now += 1
    node._tick()
    assert not node._command_pub.messages
    assert not node._target_pub.messages
    assert abs(rig.hardware.last_currents_ma[0]) <= 10
    np.testing.assert_array_equal(rig.hardware.last_currents_ma[1:], np.zeros(7))
    node._on_stop(String(data="switch_stop"))
    assert node.core.mode == ControlMode.IDLE
    assert not rig.hardware.torque_enabled
    np.testing.assert_array_equal(rig.hardware.last_currents_ma, np.zeros(8))
    node._on_friction_test(String(data="switch_friction_test"))
    rig.clock.now += 5
    node._tick()
    assert node.core.last_transition.reason == "friction_test_timeout"
    assert not rig.hardware.torque_enabled


def test_friction_test_node_toggles_with_one_launch(make_node):
    rig = make_node(friction_test={
        "joint_id": 1, "gain_scale": 0.5, "current_limit_ma": 20.0,
        "ramp_seconds": 2.0, "max_duration_seconds": 5.0, "enabled": False,
    })
    node = rig.node
    node._on_friction_test(String(data="switch_friction_test"))
    assert node._published_friction_phase == "off"
    rig.clock.now += 1
    node._tick()
    np.testing.assert_array_equal(rig.hardware.last_currents_ma, np.zeros(8))
    node._on_friction_output(String(data="on"))
    assert node._published_friction_phase == "on"
    rig.clock.now += 1
    node._tick()
    assert rig.hardware.last_currents_ma[0] != 0
    node._on_friction_output(String(data="off"))
    assert node._published_friction_phase == "off"
    np.testing.assert_array_equal(rig.hardware.last_currents_ma, np.zeros(8))
    node._on_stop(String(data="switch_stop"))
    assert node._published_friction_phase == "idle"


def test_friction_test_destroy_disables_torque(make_node):
    rig = make_node(friction_test={
        "joint_id": 1, "gain_scale": 0.1, "current_limit_ma": 20.0,
        "ramp_seconds": 2.0, "max_duration_seconds": 5.0, "enabled": False,
    })
    rig.node._on_friction_test(String(data="switch_friction_test"))
    rig.node.destroy_node()
    assert not rig.hardware.torque_enabled
    np.testing.assert_array_equal(rig.hardware.last_currents_ma, np.zeros(8))


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
    ("joint_offsets", [0.0] * 7), ("current_limit_ma", 50.0),
    ("state_timeout", -0.1),
])
def test_invalid_config_values_fail_before_hardware_open(make_node, key, value):
    hardware = Hardware()
    with pytest.raises(ValueError):
        make_node(parameters={key: value}, hardware=hardware)
    assert hardware.open_calls == []


def test_missing_current_limit_fails_before_hardware_open(make_node, tmp_path):
    hardware = Hardware()
    template = Path(__file__).parents[1] / "config/grav_comp_m6_left.yaml"
    document = yaml.safe_load(template.read_text())
    document["dynamixel"].pop("current_limit_ma")
    config = tmp_path / "without_current_limit.yaml"
    config.write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError, match="dynamixel.current_limit_ma is required"):
        make_node(config_path_override=config, hardware=hardware)
    assert hardware.open_calls == []


def test_missing_ruckig_limits_fail_before_hardware_open(make_node, tmp_path):
    hardware = Hardware()
    template = Path(__file__).parents[1] / "config/grav_comp_m6_left.yaml"
    document = yaml.safe_load(template.read_text())
    document["dynamixel"].update({"joint_offsets": [0.0] * 8, "current_limit_ma": [50.0] * 8})
    document["controller"]["sync_hold_control"]["ruckig"].pop("max_velocity")
    config = tmp_path / "without_limits.yaml"
    config.write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError, match="ruckig.max_velocity is required"):
        make_node(config_path_override=config, hardware=hardware)
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
    rig.clock.now += rig.node.config.controller.sync_hold_control.ruckig.trajectory_timeout
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


@pytest.mark.parametrize("side", ["left", "right"])
def test_sync_diagnostics_cover_motion_hold_clipping_and_stop(make_node, side):
    from skye_factr_teleop.sync_trial import TrialSummary, move_message
    rig = make_node(side, diagnostics=True)
    node = rig.node
    node._tick()
    idle = json.loads(node._diagnostic_pub.messages[-1].data)
    assert idle["mode"] == "IDLE" and not idle["torque_enabled_cached"]
    assert idle["cycle_dt_s"] is None
    stamp = packet(side).header.stamp
    node._on_joint_move(move_message(side, [0.1] * 7 + [0.4], stamp))
    rig.clock.now += 0.002
    node._tick()
    moving = json.loads(node._diagnostic_pub.messages[-1].data)
    assert moving["mode"] == "TELEOP_SYNCING"
    assert moving["cycle_dt_s"] == pytest.approx(0.002)
    assert len(moving["position_rad"]) == len(moving["current_before_clip_ma"]) == 8
    np.testing.assert_allclose(moving["current_after_clip_ma"],
                               np.clip(moving["current_before_clip_ma"], -50, 50))
    np.testing.assert_allclose(moving["current_after_clip_ma"], rig.hardware.last_currents_ma)
    np.testing.assert_allclose(moving["logical_torque_nm"], sum(
        np.asarray(moving[name + "_torque_nm"])
        for name in ("gravity", "tracking", "friction", "joint_limit", "nullspace")))
    rig.trajectories[0].finished = True
    rig.clock.now += 0.002
    node._tick()
    settle(rig, follower=False)
    assert node.core.mode == ControlMode.SYNCED
    node._on_teleop(String(data="switch_teleop"))
    assert node.core.mode == ControlMode.SYNCED
    node._on_stop(None)
    rig.clock.now += 0.002
    node._tick()
    summary = TrialSummary()
    for msg in node._diagnostic_pub.messages:
        summary.observe(json.loads(msg.data))
    assert summary.result()["states"] == ["IDLE", "TELEOP_SYNCING", "HOLD_SETTLING", "SYNCED", "IDLE"]
    assert summary.result()["idle_zero_output_and_cached_disable_observed"]
    assert not rig.hardware.torque_enabled
    np.testing.assert_array_equal(rig.hardware.last_currents_ma, np.zeros(8))
    assert node.core.hold_integral.tolist() == [0.0] * 8
    assert node.core._target is None
    assert not node._command_pub.messages


def test_diagnostics_are_optional_and_enable_present_current_reads(make_node, monkeypatch):
    rig = make_node()
    assert rig.node._diagnostic_pub is None
    calls = []
    hardware = Hardware()
    monkeypatch.setattr(module, "_hardware_factory", lambda config, **kw: calls.append(kw) or hardware)
    make_node(diagnostics=True, use_default_factory=True, hardware=hardware)
    assert calls == [{"read_present_current": True}]


@pytest.mark.parametrize("side", ["left", "right"])
def test_trial_cli_over_ros_with_fake_hardware(make_node, side, tmp_path, monkeypatch):
    """Exercise discovery, stamped publication, recording and stop through DDS."""
    from rclpy.executors import SingleThreadedExecutor
    rig = make_node(side, diagnostics=True, record_publishers=False,
                    cli_args=["--ros-args", "-r", f"/joint_move:=/{side}_joint_move",
                              "-r", f"/joint_control:=/gento/{side}_joint_control"])
    node = rig.node
    monkeypatch.setattr(node, "_ros_now", lambda: node.get_clock().now().nanoseconds * 1e-9)
    executor = SingleThreadedExecutor(context=node.context)
    executor.add_node(node)
    config_path = node.get_parameter("config_path").value
    output = tmp_path / f"{side}_trial.json"
    env = dict(os.environ, ROS_DOMAIN_ID="193", ROS_LOCALHOST_ONLY="1")
    base = ["/usr/bin/python3", "-m", "skye_factr_teleop.sync_trial"]
    shared = ["--side", side, "--config", config_path]
    processes = []

    def spawn(arguments):
        process = subprocess.Popen(base + arguments, env=env, stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, text=True)
        processes.append(process)
        return process

    def spin_until(condition, timeout=10):
        deadline = time.monotonic() + timeout
        while not condition() and time.monotonic() < deadline:
            rig.clock.now = time.monotonic()
            if node.core._target is not None:
                rig.hardware.inject_state(node.core._target * node.core.joint_signs, np.zeros(8))
                rig.trajectories[-1].finished = True
            executor.spin_once(timeout_sec=0.005)
        assert condition(), "ROS trial test timed out"

    try:
        recorder = spawn(["record", *shared, "--output", str(output),
                          "--operator", "automated-fake-hardware", "--duration", "20"])
        spin_until(lambda: node._diagnostic_pub.get_subscription_count() > 0)
        preview = spawn(["move", *shared, "--joint-id", "1", "--delta", "0.05"])
        spin_until(lambda: preview.poll() is not None)
        preview_text = preview.communicate()[0]
        assert preview.returncode == 0, preview_text
        assert "Preview only" in preview_text
        assert not rig.hardware.torque_enabled
        command = spawn(["move", *shared, "--joint-id", "1", "--delta", "0.05", "--execute"])
        spin_until(lambda: command.poll() is not None)
        command_text = command.communicate()[0]
        assert command.returncode == 0, command_text
        assert "Observed mode:" in command_text
        spin_until(lambda: node.core.mode == ControlMode.SYNCED)
        stop = node.create_publisher(String, "/mode/switch_stop", module.event_qos())
        spin_until(lambda: stop.get_subscription_count() > 0)
        stop.publish(String(data="switch_stop"))
        spin_until(lambda: recorder.poll() is not None)
        recorder_text = recorder.communicate()[0]
        assert recorder.returncode == 0, recorder_text
        record = json.loads(output.read_text())
        assert record["assessment"] == "RECORDED"
        assert record["synced_observed"]
        assert record["follower_command_count"] == 0
        assert record["side"] == side
        assert record["config"]["controller"]["sync_hold_control"]["gripper_setpoint"] == 0.4
        assert output.with_suffix(".trace.jsonl").stat().st_size > 0
        assert not rig.hardware.torque_enabled
        np.testing.assert_array_equal(rig.hardware.last_currents_ma, np.zeros(8))
    finally:
        for process in processes:
            if process.poll() is None:
                process.terminate()
                process.communicate(timeout=5)
        executor.remove_node(node)
        executor.shutdown()
