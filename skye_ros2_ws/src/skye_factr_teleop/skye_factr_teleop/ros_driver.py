# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

"""Single-arm ROS adapter. Hardware and model construction support injection."""

import argparse
from dataclasses import dataclass
import json
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

from .config import load_config
from .state_machine import ControlMode
from .control_core import (
    ControlCore, GravityTestSettings, LimitTestSettings, FrictionTestSettings,
    gripper_command_norm, transform_leader_state,
)
from .dynamics import PinocchioDynamics
from .trajectory import RuckigTrajectory
from .dynamixel_driver import DynamixelDriver
from .sync_trial import preview_move, _sync_conditions


def state_qos():
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE)


def command_qos():
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                      reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE)


def event_qos():
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE)


def latched_state_qos():
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


def _vector(value, size, label):
    result = np.array(value, dtype=float, copy=True)
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{label} must contain {size} finite values")
    return result


def resolve_config_path(value, side):
    path = Path(value) if value else Path(f"config/grav_comp_m6_{side}.yaml")
    if not path.is_absolute():
        if len(path.parts) == 1:
            path = Path("config") / path
        path = Path(get_package_share_directory("skye_factr_teleop")) / path
    return path


@dataclass(frozen=True)
class JointSample:
    position: np.ndarray
    stamp: float
    received_at: float


def validate_joint_state(
    message, expected_names, *, ros_now, received_at, max_age,
    future_tolerance, allow_unnamed=False,
):
    """Validate a seven-axis packet; never reorder or slice a 14-axis message."""
    position = _vector(message.position, 7, "position")
    for label in ("velocity", "effort"):
        values = getattr(message, label)
        if len(values):
            _vector(values, 7, label)
    names = list(message.name)
    if names != list(expected_names) and not (allow_unnamed and not names):
        raise ValueError("joint names/order do not match this arm")
    sec, nanosec = message.header.stamp.sec, message.header.stamp.nanosec
    if sec < 0 or not 0 <= nanosec < 1_000_000_000:
        raise ValueError("invalid JointState timestamp")
    stamp = sec + nanosec * 1e-9
    if stamp == 0:
        raise ValueError("JointState requires a nonzero timestamp")
    age = ros_now - stamp
    if age > max_age or -age > future_tolerance:
        raise ValueError(
            "JointState timestamp is stale or in the future "
            f"(age={age:+.4f}s max_age={max_age:.3f}s "
            f"future_tol={future_tolerance:.3f}s)"
        )
    return JointSample(position, stamp, received_at)


def _hardware_factory(config, *, read_present_current=False, read_present_temperature=False):
    return DynamixelDriver(
        range(1, 9), config.dynamixel.servo_types,
        config.dynamixel.dynamixel_port, baudrate=config.dynamixel.baudrate,
        read_present_current=read_present_current,
        read_present_temperature=read_present_temperature,
    )


def _make_pinocchio_dynamics(config):
    dynamics = config.controller.joint_gravity_comp.dynamics
    path = Path(get_package_share_directory(dynamics.leader_urdf_package)) / dynamics.leader_urdf
    return PinocchioDynamics(path)


class SkyeFactrTeleopNode(Node):
    """All callbacks use the node's default mutually-exclusive callback group."""

    def __init__(
        self, side="left", *, hardware_factory=None, dynamics_factory=None,
        trajectory_factory=None, monotonic=time.monotonic, **node_kwargs,
    ):
        if side not in ("left", "right"):
            raise ValueError("side must be left or right")
        super().__init__(f"skye_factr_teleop_{side}", **node_kwargs)
        self.side = side
        self._monotonic = monotonic
        self._hardware = self.core = self._timer = None
        self._closed = False
        self._leader_gated_off = False
        self._follower = None
        # True from follower synchronization through teleop; joint_move does not set it.
        self._follower_session_active = False
        self._published_state = None
        self._published_friction_phase = None
        self._diagnostic_sequence = 0
        self._previous_cycle_time = None
        self._sync_goal = None
        self._sync_progress_mode = None
        self._sync_progress_at = None
        self._last_temperature_publish_time = None
        try:
            self._configure()
            self._create_ros_interfaces()
            self._start_control(
                hardware_factory=hardware_factory,
                dynamics_factory=dynamics_factory,
                trajectory_factory=trajectory_factory,
            )
        except BaseException:
            self.shutdown_control()
            super().destroy_node()
            raise

    def _configure(self):
        config_path = self._parameter("config_path", "")
        self._sync_diagnostics = self._parameter("sync_diagnostics", False)
        if type(self._sync_diagnostics) is not bool:
            raise ValueError("sync_diagnostics must be a bool")
        rate = self._parameter("temperature_publish_rate_hz", 1.0)
        if (isinstance(rate, bool) or not isinstance(rate, (int, float))
                or not np.isfinite(rate) or rate < 0):
            raise ValueError("temperature_publish_rate_hz must be a finite non-negative number")
        self._temperature_publish_rate_hz = float(rate)
        self._gravity_test_only = self._parameter("gravity_test_only", False)
        self._limit_test_only = self._parameter("limit_test_only", False)
        self._friction_test_only = self._parameter("friction_test_only", False)
        if type(self._gravity_test_only) is not bool:
            raise ValueError("gravity_test_only must be a bool")
        if (type(self._limit_test_only) is not bool
                or type(self._friction_test_only) is not bool
                or sum((self._gravity_test_only, self._limit_test_only,
                        self._friction_test_only)) > 1):
            raise ValueError("exactly one test mode may be selected")
        self._test_only = (self._gravity_test_only or self._limit_test_only
                           or self._friction_test_only)
        self.config = load_config(
            resolve_config_path(config_path, self.side),
            require_runtime=not self._test_only,
        )
        runtime = self.config.runtime
        self._gravity_test = None
        if self._gravity_test_only:
            if (self.config.dynamixel.joint_offsets is None
                    or self.config.dynamixel.current_limit_ma is None):
                raise ValueError("gravity test requires joint_offsets and current_limit_ma")
            self._gravity_test = GravityTestSettings(
                joint_id=self._parameter("gravity_test_joint_id", 0),
                gain_scale=self._parameter("gravity_test_gain_scale", 0.0),
                current_limit_ma=self._parameter("gravity_test_current_limit_ma", 0.0),
                ramp_seconds=self._parameter("gravity_test_ramp_seconds", 0.0),
                max_duration_seconds=self._parameter("gravity_test_max_duration_seconds", 0.0),
            )
            if self._gravity_test.current_limit_ma > min(self.config.dynamixel.current_limit_ma):
                raise ValueError("gravity test current cap exceeds configured limit")
            joint_gain = self.config.controller.joint_gravity_comp.gain[
                self._gravity_test.joint_id - 1
            ]
            if not self.config.controller.joint_gravity_comp.enable or joint_gain <= 0:
                raise ValueError("gravity test requires a positive enabled joint gain")
        self._limit_test = None
        if self._limit_test_only:
            if (self.config.dynamixel.joint_offsets is None
                    or self.config.dynamixel.current_limit_ma is None):
                raise ValueError("limit test requires joint_offsets and current_limit_ma")
            self._limit_test = LimitTestSettings(
                joint_id=self._parameter("limit_test_joint_id", 0),
                gain_scale=self._parameter("limit_test_gain_scale", 0.0),
                current_limit_ma=self._parameter("limit_test_current_limit_ma", 0.0),
                ramp_seconds=self._parameter("limit_test_ramp_seconds", 0.0),
                max_duration_seconds=self._parameter("limit_test_max_duration_seconds", 0.0),
            )
            if self._limit_test.current_limit_ma > min(self.config.dynamixel.current_limit_ma):
                raise ValueError("limit test current cap exceeds configured limit")
            if not self.config.controller.limit_barrier_control.enable:
                raise ValueError("limit test requires enabled limit barrier")
            if self.side not in Path(
                    self.config.controller.joint_gravity_comp.dynamics.leader_urdf).stem:
                raise ValueError("limit test side does not match leader URDF")
            barrier = self.config.controller.limit_barrier_control
            kp = barrier.gripper.kp if self._limit_test.joint_id == 8 else barrier.joint.kp
            if kp <= 0:
                raise ValueError("limit test requires positive position gain")
        self._friction_test = None
        if self._friction_test_only:
            if (self.config.dynamixel.joint_offsets is None
                    or self.config.dynamixel.current_limit_ma is None):
                raise ValueError("friction test requires joint_offsets and current_limit_ma")
            self._friction_test = FrictionTestSettings(
                joint_id=self._parameter("friction_test_joint_id", 0),
                gain_scale=self._parameter("friction_test_gain_scale", 0.0),
                current_limit_ma=self._parameter("friction_test_current_limit_ma", 0.0),
                ramp_seconds=self._parameter("friction_test_ramp_seconds", 0.0),
                max_duration_seconds=self._parameter("friction_test_max_duration_seconds", 0.0),
                enabled=self._parameter("friction_test_enabled", True),
            )
            if self._friction_test.current_limit_ma > min(self.config.dynamixel.current_limit_ma):
                raise ValueError("friction test current cap exceeds configured limit")
            if self.side not in Path(
                    self.config.controller.joint_gravity_comp.dynamics.leader_urdf).stem:
                raise ValueError("friction test side does not match leader URDF")
            index = self._friction_test.joint_id - 1
            if (self.config.controller.joint_gravity_comp.gain[index] <= 0
                    or self.config.controller.static_friction_comp.gain[index] <= 0
                    or self.config.controller.static_friction_comp.enable_speed[index] <= 0):
                raise ValueError("friction test requires positive selected joint gains and speed threshold")
            if not self.config.controller.static_friction_comp.enable:
                raise ValueError("friction test requires enabled static friction in configuration")
        self._offsets = np.asarray(self.config.dynamixel.joint_offsets)
        self._current_limit = self.config.dynamixel.current_limit_ma
        if not self._test_only:
            self._limits = {
                name: np.asarray(getattr(self.config.controller.sync_hold_control.ruckig, name))
                for name in ("max_velocity", "max_acceleration", "max_jerk")
            }
        self._max_age = runtime.state_timeout
        self._future_tolerance = runtime.future_tolerance
        self._allow_unnamed = runtime.allow_unnamed_joint_states
        prefix = "l" if self.side == "left" else "r"
        self._names = [f"{prefix}_j{i}" for i in range(1, 8)]

    def _parameter(self, name, default):
        descriptor = ParameterDescriptor(read_only=True, dynamic_typing=True)
        return self.declare_parameter(name, default, descriptor).value

    def _create_ros_interfaces(self):
        self._state_pub = self.create_publisher(String, "/teleop/state", latched_state_qos())
        self._sync_progress_pub = None
        if not self._test_only:
            self._sync_progress_pub = self.create_publisher(
                String, f"/leader_arm/{self.side}/sync_progress",
                QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL),
            )
        self._current_pub = self.create_publisher(JointState, "/leader_arm/current_state", state_qos())
        self._target_pub = self.create_publisher(JointState, "/leader_arm/target_joint_state", state_qos())
        self._command_pub = self.create_publisher(JointState, "/joint_control", command_qos())
        self._gripper_pub = None
        self._temperature_pub = None
        if self._temperature_publish_rate_hz > 0:
            self._temperature_pub = self.create_publisher(
                Float64MultiArray, f"/leader_arm/{self.side}/temperature_c", state_qos(),
            )
        self._diagnostic_pub = None
        if self._sync_diagnostics and not self._test_only:
            self._diagnostic_pub = self.create_publisher(
                String, f"/leader_arm/{self.side}/sync_diagnostics", command_qos(),
            )
        self._test_current_pub = self._test_present_current_pub = None
        if self._test_only:
            name = ("gravity_test" if self._gravity_test_only else
                    "limit_test" if self._limit_test_only else "friction_test")
            self._test_current_pub = self.create_publisher(
                Float64MultiArray, f"/leader_arm/{self.side}/{name}_current_ma",
                state_qos(),
            )
            self._test_present_current_pub = self.create_publisher(
                Float64MultiArray,
                f"/leader_arm/{self.side}/{name}_present_current_ma",
                state_qos(),
            )
        self._test_gripper_pub = None
        self._friction_phase_pub = None
        if self._friction_test_only:
            self._friction_phase_pub = self.create_publisher(
                String, f"/leader_arm/{self.side}/friction_test_phase", latched_state_qos(),
            )
        if self._limit_test_only:
            self._test_gripper_pub = self.create_publisher(
                Float64MultiArray, f"/leader_arm/{self.side}/limit_test_gripper_state",
                state_qos(),
            )
        self.create_subscription(String, "/mode/switch_stop", self._on_stop, event_qos())
        if not self._test_only:
            self.create_subscription(
                String, f"/mode/{self.side}/leader_gate",
                self._on_leader_gate, event_qos(),
            )
        if self._gravity_test_only:
            self.create_subscription(
                String, f"/mode/{self.side}/switch_gravity_test",
                self._on_gravity_test, event_qos(),
            )
        elif self._limit_test_only:
            self.create_subscription(
                String, f"/mode/{self.side}/switch_limit_test",
                self._on_limit_test, event_qos(),
            )
        elif self._friction_test_only:
            self.create_subscription(
                String, f"/mode/{self.side}/switch_friction_test",
                self._on_friction_test, event_qos(),
            )
            self.create_subscription(
                String, f"/mode/{self.side}/friction_test_output",
                self._on_friction_output, event_qos(),
            )
        else:
            self._gripper_pub = self.create_publisher(
                JointState, "/gripper/ctrl", command_qos(),
            )
            self.create_subscription(JointState, "/joint_state", self._on_follower, state_qos())
            self.create_subscription(JointState, "/joint_move", self._on_joint_move, state_qos())
            self.create_subscription(String, "/mode/switch_sync", self._on_sync, event_qos())
            self.create_subscription(String, "/mode/switch_teleop", self._on_teleop, event_qos())

    def _start_control(self, *, hardware_factory, dynamics_factory, trajectory_factory):
        dynamics = None if self._limit_test_only else (dynamics_factory or _make_pinocchio_dynamics)(self.config)
        make_trajectory = None if self._test_only else (trajectory_factory or self._make_ruckig_trajectory)
        if not self._test_only and trajectory_factory is None:
            make_trajectory()  # Check Ruckig before opening the hardware.
        self._hardware = (hardware_factory(self.config) if hardware_factory is not None
                          else _hardware_factory(
                              self.config, read_present_current=(
                                  self._test_only or self._sync_diagnostics),
                              read_present_temperature=self._temperature_publish_rate_hz > 0,
                          ))
        self._hardware.open(enable_torque=False)
        self.core = ControlCore(
            self.config, self._hardware, dynamics, self._offsets,
            self._current_limit, trajectory_factory=make_trajectory,
            gravity_test=self._gravity_test, now=self._monotonic(),
            limit_test=self._limit_test,
            friction_test=self._friction_test,
        )
        self._publish_state()
        self._publish_friction_phase()
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._timer = self.create_timer(
            1 / self.config.controller.frequency, self._tick, clock=self._steady_clock,
        )

    def _make_ruckig_trajectory(self):
        return RuckigTrajectory(frequency=self.config.controller.frequency, **self._limits)

    def _tick(self):
        if self._closed:
            return
        phase = self.core.mode.name
        try:
            if self._follower_session_active and not self._fresh_follower():
                self.get_logger().warning("Follower state expired; stopping")
                self._on_stop(None)
                return
            cycle_time = self._monotonic()
            result = self.core.step(now=cycle_time)
            if result.mode == ControlMode.IDLE:
                self._follower_session_active = False
            self._publish_cycle(result)
            self._publish_temperatures(cycle_time)
            self._publish_sync_progress(result, cycle_time)
            self._publish_diagnostics(result, cycle_time)
            self._publish_friction_phase()
        except Exception as exc:
            self._fault(exc, phase=phase)

    def _ros_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _validate(self, message):
        return validate_joint_state(
            message, self._names, ros_now=self._ros_now(), received_at=self._monotonic(),
            max_age=self._max_age, future_tolerance=self._future_tolerance,
            allow_unnamed=self._allow_unnamed,
        )

    def _fresh_follower(self):
        sample = self._follower
        if sample is None:
            return False
        age = self._monotonic() - sample.received_at
        stamp_age = self._ros_now() - sample.stamp
        return (0 <= age <= self._max_age
                and -self._future_tolerance <= stamp_age <= self._max_age)

    def _on_follower(self, message):
        if self._closed:
            return
        try:
            self._follower = self._validate(message)
        except (TypeError, ValueError) as exc:
            self._follower = None
            self.get_logger().warning(f"Rejected follower state: {exc}")
            if self._follower_session_active:
                self._on_stop(None)

    def _on_leader_gate(self, message):
        """Soft mute one side without killing the dual-launch process tree."""
        if self._closed or message.data not in ("off", "on"):
            return
        try:
            if message.data == "off":
                if not self._leader_gated_off:
                    self._follower_session_active = False
                    self.core.stop()
                    self._leader_gated_off = True
                    self.get_logger().info(
                        f"[{self.side}] leader gate OFF：忽略 sync/teleop，力矩已关闭")
                self._publish_state()
                return
            if self._leader_gated_off:
                self._leader_gated_off = False
                self.get_logger().info(
                    f"[{self.side}] leader gate ON：可再次接收 sync/teleop")
            self._publish_state()
        except Exception as exc:
            self._fault(exc)

    def _on_sync(self, message):
        if self._closed or message.data != "switch_sync":
            return
        if self._leader_gated_off:
            self.get_logger().info(f"[{self.side}] sync ignored：leader gate OFF")
            return
        if self.core.mode != ControlMode.IDLE:
            if self.core.mode == ControlMode.SYNCED:
                self._report_sync(f"[{self.side}] SYNCED：已同步，继续保持目标姿态", "synced")
            elif self.core.mode not in (ControlMode.TELEOP_SYNCING, ControlMode.HOLD_SETTLING):
                self._report_sync(f"[{self.side}] 同步被拒绝：当前状态 {self.core.mode.name}，请先 stop", "failed")
            return
        if not self._fresh_follower():
            self.get_logger().warning("Synchronization requires fresh follower state")
            self._report_sync(f"[{self.side}] 同步被拒绝：缺少新鲜的大臂关节状态", "failed")
            return
        target = self._follower.position.copy()
        try:
            position, velocity = self._hardware.read_state()
            state = transform_leader_state(
                position, velocity, self.core.joint_offsets, self.core.joint_signs,
            )
        except Exception as exc:
            self._fault(exc, phase="sync_preflight")
            return
        try:
            plan = preview_move(
                self.config, np.append(state.arm_position, state.gripper_position),
                np.append(state.arm_velocity, state.gripper_velocity), target=target,
            )
        except Exception as exc:
            text = f"Synchronization rejected [side={self.side}, phase=sync_preflight]: {exc}"
            self.get_logger().error(text)
            self._report_sync(text, "failed")
            return
        self._report_sync(
            f"Synchronization preflight passed [side={self.side}]: "
            f"trajectory={plan['trajectory_duration_s']:.3f}s, "
            f"joint_delta_rad={(target - state.arm_position).tolist()}"
        )
        self._start_motion(target, follower_session=True)

    def _on_gravity_test(self, message):
        if self._closed or message.data != "switch_gravity_test":
            return
        if self.core.mode != ControlMode.IDLE:
            return
        try:
            self._hardware.set_torque_enabled(True)
            self.core.handle("switch_gravity_test", now=self._monotonic())
            self._publish_state()
        except Exception as exc:
            self._fault(exc)

    def _on_limit_test(self, message):
        if self._closed or message.data != "switch_limit_test":
            return
        if self.core.mode != ControlMode.IDLE:
            return
        try:
            self._hardware.set_torque_enabled(True)
            self.core.handle("switch_limit_test", now=self._monotonic())
            self._publish_state()
        except Exception as exc:
            self._fault(exc)

    def _on_friction_test(self, message):
        if self._closed or message.data != "switch_friction_test":
            return
        if self.core.mode != ControlMode.IDLE:
            return
        try:
            self._hardware.set_torque_enabled(True)
            self.core.handle("switch_friction_test", now=self._monotonic())
            self._publish_state()
            self._publish_friction_phase()
        except Exception as exc:
            self._fault(exc)

    def _on_friction_output(self, message):
        if self._closed or message.data not in ("on", "off"):
            return
        if self.core.mode != ControlMode.FRICTION_TEST:
            return
        try:
            self.core.set_friction_test_enabled(message.data == "on", now=self._monotonic())
            self._publish_state()
            self._publish_friction_phase()
        except Exception as exc:
            self._fault(exc)

    def _on_joint_move(self, message):
        if self._closed:
            return
        if self._leader_gated_off:
            self.get_logger().info(f"[{self.side}] joint_move ignored：leader gate OFF")
            return
        try:
            target = self._validate(message)
        except (TypeError, ValueError) as exc:
            self.get_logger().warning(f"Rejected joint_move: {exc}")
            return
        if self.core.mode != ControlMode.IDLE:
            self.get_logger().warning("joint_move requires IDLE; stop before a new move")
            return
        self._start_motion(target.position, follower_session=False)

    def _start_motion(self, target, *, follower_session):
        try:
            self._hardware.set_torque_enabled(True)
            self.core.handle("switch_sync", now=self._monotonic(), follower_position=target)
            self._sync_goal = self.core.sync_target_position
            self._sync_progress_mode = None
            self._sync_progress_at = None
            self._follower_session_active = follower_session
            self._publish_state()
        except Exception as exc:
            self._fault(exc)

    def _on_teleop(self, message):
        if self._closed or message.data != "switch_teleop":
            return
        if self._leader_gated_off:
            self.get_logger().info(f"[{self.side}] teleop ignored：leader gate OFF")
            return
        if self.core.mode != ControlMode.SYNCED or not self._follower_session_active:
            self.get_logger().warning("Teleoperation requires successful follower synchronization")
            return
        if not self._fresh_follower():
            self._on_stop(None)
            return
        try:
            self.core.handle("switch_teleop", now=self._monotonic())
            self._publish_state()
        except Exception as exc:
            self._fault(exc)

    def _on_stop(self, message):
        if self._closed:
            return
        try:
            self._follower_session_active = False
            self.core.stop()
            self._publish_state()
            self._publish_friction_phase()
        except Exception as exc:
            self._fault(exc)

    def _joint_message(self, position, velocity=None):
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = self._names
        message.position = _vector(position, 7, "published position").tolist()
        if velocity is not None:
            message.velocity = _vector(velocity, 7, "published velocity").tolist()
        return message

    def _publish_state(self):
        state = self.core.mode.name
        if state != self._published_state and self.context.ok():
            previous = self._published_state
            self._state_pub.publish(String(data=state))
            self._published_state = state
            if not self._closed and not self._test_only:
                descriptions = {
                    "IDLE": "空闲，等待同步指令",
                    "TELEOP_SYNCING": "同步轨迹运动中",
                    "HOLD_SETTLING": "轨迹完成，检查到位及持续稳定时间",
                    "SYNCED": "同步成功（八轴已连续满足位置和速度阈值）",
                    "TELEOP": "遥操中",
                }
                reason = self.core.sync_status["transition_reason"]
                if state == "IDLE" and reason in ("sync_timeout", "settle_timeout"):
                    self._report_sync(
                        f"[{self.side}] 同步失败：{reason}；已停止并关闭力矩"
                    )
                elif state in ("TELEOP_SYNCING", "HOLD_SETTLING", "SYNCED"):
                    self._report_sync(
                        f"[{self.side}] {state}：{descriptions[state]}",
                        "synced" if state == "SYNCED" else "active",
                    )
                elif state == "IDLE" and previous in ("TELEOP_SYNCING", "HOLD_SETTLING", "SYNCED"):
                    self._report_sync(f"[{self.side}] IDLE：同步已停止，已关闭力矩", "stopped")
                else:
                    self.get_logger().info(
                        f"[{self.side}] {state}：{descriptions.get(state, state)}"
                    )

    def _report_sync(self, text, outcome="active"):
        if self._sync_progress_pub is not None and self.context.ok():
            self._sync_progress_pub.publish(String(data=json.dumps({
                "side": self.side, "text": text, "outcome": outcome,
                "stamp_s": time.time(),
            }, ensure_ascii=False)))

    def _publish_sync_progress(self, result, now):
        """Report final-goal blockers at 1 Hz, immediately on phase change/timeout."""
        if self._sync_goal is None:
            return
        mode = result.mode.name
        timed_out = (mode == "IDLE" and self.core.sync_status["transition_reason"]
                     in ("sync_timeout", "settle_timeout"))
        if mode not in ("TELEOP_SYNCING", "HOLD_SETTLING") and not timed_out:
            if mode == "IDLE":
                self._sync_goal = None
            return
        if (not timed_out and mode == self._sync_progress_mode
                and self._sync_progress_at is not None
                and now - self._sync_progress_at < 1.0):
            return
        hold = self.config.controller.sync_hold_control.hold
        position = np.append(result.state.arm_position, result.state.gripper_position)
        velocity = np.append(result.state.arm_velocity, result.state.gripper_velocity)
        within = ((np.abs(self._sync_goal - position) <= hold.sync_position_tolerance)
                  & (np.abs(velocity) <= hold.sync_velocity_tolerance))
        packet = {
            "position_rad": position, "velocity_rad_s": velocity,
            "mode": mode, "sync_status": self.core.sync_status,
        }
        details = _sync_conditions(packet, self._sync_goal, hold)
        lines = [f"[{self.side}] {mode}：{np.count_nonzero(within)}/8 轴当前达标（相对最终同步目标）"]
        lines.extend(f"[{self.side}] {line}" for line in details.splitlines())
        if timed_out:
            reason = self.core.sync_status["transition_reason"]
            lines.insert(0, f"[{self.side}] 同步失败：{reason}；已停止并关闭力矩")
        self._report_sync("\n".join(lines), "failed" if timed_out else "active")
        self._sync_progress_mode = mode
        self._sync_progress_at = now
        if timed_out:
            self._sync_goal = None

    def _publish_friction_phase(self):
        if self._friction_phase_pub is None or not self.context.ok():
            return
        phase = ("idle" if self.core.mode != ControlMode.FRICTION_TEST else
                 "on" if self.core.friction_test_enabled else "off")
        if phase != self._published_friction_phase:
            self._friction_phase_pub.publish(String(data=phase))
            self._published_friction_phase = phase

    def _publish_cycle(self, result):
        self._current_pub.publish(self._joint_message(
            result.state.arm_position, result.state.arm_velocity,
        ))
        if result.target_position is not None:
            self._target_pub.publish(self._joint_message(
                result.target_position[:7], result.target_velocity[:7],
            ))
        if result.mode == ControlMode.TELEOP:
            self._command_pub.publish(self._joint_message(result.state.arm_position))
            if self._gripper_pub is not None:
                gripper = self.config.controller.limit_barrier_control.gripper
                norm = gripper_command_norm(
                    result.state.gripper_position, gripper.limit_min, gripper.limit_max,
                )
                if norm is None:
                    self.get_logger().warning(
                        "Skipping gripper command: non-finite logical gripper position",
                    )
                else:
                    msg = JointState()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.position = [norm]
                    self._gripper_pub.publish(msg)
        if self._test_current_pub is not None:
            self._test_current_pub.publish(Float64MultiArray(data=result.physical_current_ma.tolist()))
            present_current = _vector(
                self._hardware.present_currents_ma, 8, "present current",
            )
            self._test_present_current_pub.publish(
                Float64MultiArray(data=present_current.tolist())
            )
        if self._test_gripper_pub is not None:
            self._test_gripper_pub.publish(Float64MultiArray(data=[
                result.state.gripper_position, result.state.gripper_velocity,
            ]))
        self._publish_state()

    def _publish_temperatures(self, cycle_time):
        if self._temperature_pub is None:
            return
        if (self._last_temperature_publish_time is not None
                and cycle_time - self._last_temperature_publish_time
                < 1.0 / self._temperature_publish_rate_hz):
            return
        temperatures = self._hardware.present_temperatures_c
        if temperatures is None:
            return
        temperatures = _vector(temperatures, 8, "present temperature")
        self._temperature_pub.publish(Float64MultiArray(data=temperatures.tolist()))
        self._last_temperature_publish_time = cycle_time

    def _fault(self, exc, *, phase=None):
        phase = phase or self.core.mode.name
        text = f"Control stopped after failure [side={self.side}, phase={phase}]: {exc}"
        self.get_logger().error(text)
        self._report_sync(text, "failed")
        if exc.__cause__ is not None:
            self.get_logger().error(f"Additional error context: {exc.__cause__}")
        self.shutdown_control()

    def _publish_diagnostics(self, result, cycle_time):
        if self._diagnostic_pub is None:
            return
        from .sync_trial import cycle_packet

        packet = cycle_packet(
            result, side=self.side, sequence=self._diagnostic_sequence,
            stamp_s=self._ros_now(), monotonic_s=cycle_time,
            cycle_dt_s=(None if self._previous_cycle_time is None else
                        cycle_time - self._previous_cycle_time),
            torque_enabled_cached=self._hardware.torque_enabled,
            present_current_ma=self._hardware.present_currents_ma,
            sync_status=self.core.sync_status,
            current_limit_ma=self.core.current_limit_ma,
        )
        self._diagnostic_pub.publish(String(data=json.dumps(packet, allow_nan=False)))
        self._diagnostic_sequence += 1
        self._previous_cycle_time = cycle_time

    def shutdown_control(self):
        """Attempt every cleanup step, even if zeroing/disable/publish fails."""
        if self._closed:
            return
        self._closed = True
        self._follower_session_active = False
        if self._timer is not None:
            try:
                self._timer.cancel()
            except Exception as exc:
                self.get_logger().error(f"Timer cancellation failed: {exc}")
        try:
            if self.core is not None:
                try:
                    self.core.stop()
                finally:
                    self._publish_state()
        except Exception as exc:
            self.get_logger().error(f"Control stop failed: {exc}")
        finally:
            if self._hardware is not None:
                try:
                    self._hardware.close()
                except Exception as exc:
                    self.get_logger().error(f"Hardware close failed: {exc}")

    def destroy_node(self):
        self.shutdown_control()
        return super().destroy_node()

def main(args=None):
    parser = argparse.ArgumentParser(description="Skye FACTR single-arm teleoperation")
    parser.add_argument("--side", choices=("left", "right"), default="left")
    argv = sys.argv if args is None else ["skye_factr_teleop_driver", *args]
    options = parser.parse_args(remove_ros_args(args=argv)[1:])
    rclpy.init(args=argv)
    node = None
    try:
        node = SkyeFactrTeleopNode(options.side)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node is not None:
                node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
