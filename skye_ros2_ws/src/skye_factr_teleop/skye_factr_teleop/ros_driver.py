# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

"""Single-arm ROS adapter. Hardware and model construction support injection."""

import argparse
from dataclasses import dataclass
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
from std_msgs.msg import String

from .config import load_config
from .control_core import ControlCore, RuckigTrajectory
from .dynamixel_driver import DynamixelDriver
from .dynamics import PinocchioDynamics
from .state_machine import ControlMode


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


def _positive(value, label):
    value = float(value)
    if not np.isfinite(value) or value <= 0:
        raise ValueError(f"{label} must be finite and positive")
    return value


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
    if ros_now - stamp > max_age or stamp - ros_now > future_tolerance:
        raise ValueError("JointState timestamp is stale or in the future")
    return JointSample(position, stamp, received_at)


def _hardware_factory(config):
    return DynamixelDriver(
        range(1, 9), config.dynamixel.servo_types,
        config.dynamixel.dynamixel_port, baudrate=config.arm_teleop.baudrate,
    )


def _dynamics_factory(config):
    arm = config.arm_teleop
    path = Path(get_package_share_directory(arm.leader_urdf_package)) / arm.leader_urdf
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
        self._follower = None
        self._follower_sync = False
        self._published_state = None
        try:
            self._configure()
            self._state_pub = self.create_publisher(String, "/teleop/state", latched_state_qos())
            self._current_pub = self.create_publisher(JointState, "/leader_arm/current_state", state_qos())
            self._target_pub = self.create_publisher(JointState, "/leader_arm/target_joint_state", state_qos())
            self._command_pub = self.create_publisher(JointState, "/joint_control", command_qos())
            self.create_subscription(JointState, "/joint_state", self._on_follower, state_qos())
            self.create_subscription(JointState, "/joint_move", self._on_joint_move, state_qos())
            self.create_subscription(String, "/mode/switch_sync", self._on_sync, event_qos())
            self.create_subscription(String, "/mode/switch_teleop", self._on_teleop, event_qos())
            self.create_subscription(String, "/mode/switch_stop", self._on_stop, event_qos())

            dynamics = (dynamics_factory or _dynamics_factory)(self.config)
            factory = trajectory_factory or self._make_trajectory
            if trajectory_factory is None:
                factory()  # Fail on missing Ruckig before hardware access.
            self._hardware = (hardware_factory or _hardware_factory)(self.config)
            self._hardware.open(enable_torque=False)
            self.core = ControlCore(
                self.config, self._hardware, dynamics, self._offsets,
                self._current_limit, trajectory_factory=factory, now=self._monotonic(),
            )
            self._publish_state()
            self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self._timer = self.create_timer(
                1 / self.config.controller.frequency, self._tick, clock=self._steady_clock,
            )
        except BaseException:
            self.shutdown_control()
            super().destroy_node()
            raise

    def _parameter(self, name, default):
        descriptor = ParameterDescriptor(read_only=True, dynamic_typing=True)
        return self.declare_parameter(name, default, descriptor).value

    def _configure(self):
        config_path = self._parameter("config_path", "")
        self.config = load_config(resolve_config_path(config_path, self.side))
        self._offsets = _vector(self._parameter("joint_offsets", []), 8, "joint_offsets")
        self._current_limit = _positive(self._parameter("current_limit_ma", 0.0), "current_limit_ma")
        self._limits = {}
        for name in ("max_velocity", "max_acceleration", "max_jerk"):
            value = _vector(self._parameter(name, []), 8, name)
            if np.any(value <= 0):
                raise ValueError(f"{name} must be positive")
            self._limits[name] = value
        self._max_age = _positive(self._parameter("state_timeout", 0.25), "state_timeout")
        self._future_tolerance = _positive(self._parameter("future_tolerance", 0.05), "future_tolerance")
        self._allow_unnamed = self._parameter("allow_unnamed_joint_states", False)
        if not isinstance(self._allow_unnamed, bool):
            raise ValueError("allow_unnamed_joint_states must be bool")
        prefix = "l" if self.side == "left" else "r"
        self._names = [f"{prefix}_j{i}" for i in range(1, 8)]

    def _make_trajectory(self):
        return RuckigTrajectory(frequency=self.config.controller.frequency, **self._limits)

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
            if self._follower_sync:
                self._on_stop(None)

    def _on_sync(self, message):
        if self._closed or message.data != "switch_sync":
            return
        if self.core.mode != ControlMode.IDLE:
            return
        if not self._fresh_follower():
            self.get_logger().warning("Synchronization requires fresh follower state")
            return
        self._start_motion(self._follower.position, follower_sync=True)

    def _on_joint_move(self, message):
        if self._closed:
            return
        try:
            target = self._validate(message)
        except (TypeError, ValueError) as exc:
            self.get_logger().warning(f"Rejected joint_move: {exc}")
            return
        if self.core.mode != ControlMode.IDLE:
            self.get_logger().warning("joint_move requires IDLE; stop before a new move")
            return
        self._start_motion(target.position, follower_sync=False)

    def _start_motion(self, target, *, follower_sync):
        try:
            self._hardware.set_torque_enabled(True)
            self.core.handle("switch_sync", now=self._monotonic(), follower_position=target)
            self._follower_sync = follower_sync
            self._publish_state()
        except Exception as exc:
            self._fault(exc)

    def _on_teleop(self, message):
        if self._closed or message.data != "switch_teleop":
            return
        if self.core.mode != ControlMode.SYNCED or not self._follower_sync:
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
            self._follower_sync = False
            self.core.stop()
            self._publish_state()
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
            self._state_pub.publish(String(data=state))
            self._published_state = state

    def _tick(self):
        if self._closed:
            return
        try:
            if self._follower_sync and not self._fresh_follower():
                self.get_logger().warning("Follower state expired; stopping")
                self._on_stop(None)
                return
            result = self.core.step(now=self._monotonic())
            if result.mode == ControlMode.IDLE:
                self._follower_sync = False
            self._current_pub.publish(self._joint_message(
                result.state.arm_position, result.state.arm_velocity,
            ))
            if result.target_position is not None:
                self._target_pub.publish(self._joint_message(
                    result.target_position[:7], result.target_velocity[:7],
                ))
            if result.mode == ControlMode.TELEOP:
                self._command_pub.publish(self._joint_message(result.state.arm_position))
            self._publish_state()
        except Exception as exc:
            self._fault(exc)

    def _fault(self, exc):
        self.get_logger().error(f"Teleoperation stopped after failure: {exc}")
        self.shutdown_control()

    def shutdown_control(self):
        """Attempt every cleanup step, even if zeroing/disable/publish fails."""
        if self._closed:
            return
        self._closed = True
        self._follower_sync = False
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
