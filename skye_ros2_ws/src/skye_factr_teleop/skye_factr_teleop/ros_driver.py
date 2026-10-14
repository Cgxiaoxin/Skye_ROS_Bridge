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
from std_msgs.msg import Float64MultiArray, String

from .config import load_config
from .state_machine import ControlMode
from .control_core import ControlCore, GravityTestSettings, LimitTestSettings
from .dynamics import PinocchioDynamics
from .trajectory import RuckigTrajectory
from .dynamixel_driver import DynamixelDriver


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
    if ros_now - stamp > max_age or stamp - ros_now > future_tolerance:
        raise ValueError("JointState timestamp is stale or in the future")
    return JointSample(position, stamp, received_at)


def _hardware_factory(config, *, read_present_current=False):
    return DynamixelDriver(
        range(1, 9), config.dynamixel.servo_types,
        config.dynamixel.dynamixel_port, baudrate=config.dynamixel.baudrate,
        read_present_current=read_present_current,
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
        self._follower = None
        # True from follower synchronization through teleop; joint_move does not set it.
        self._follower_session_active = False
        self._published_state = None
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
        self._gravity_test_only = self._parameter("gravity_test_only", False)
        self._limit_test_only = self._parameter("limit_test_only", False)
        if type(self._gravity_test_only) is not bool:
            raise ValueError("gravity_test_only must be a bool")
        if type(self._limit_test_only) is not bool or (self._gravity_test_only and self._limit_test_only):
            raise ValueError("exactly one test mode may be selected")
        self._test_only = self._gravity_test_only or self._limit_test_only
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
        self._current_pub = self.create_publisher(JointState, "/leader_arm/current_state", state_qos())
        self._target_pub = self.create_publisher(JointState, "/leader_arm/target_joint_state", state_qos())
        self._command_pub = self.create_publisher(JointState, "/joint_control", command_qos())
        self._test_current_pub = self._test_present_current_pub = None
        if self._test_only:
            name = "gravity_test" if self._gravity_test_only else "limit_test"
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
        if self._limit_test_only:
            self._test_gripper_pub = self.create_publisher(
                Float64MultiArray, f"/leader_arm/{self.side}/limit_test_gripper_state",
                state_qos(),
            )
        self.create_subscription(String, "/mode/switch_stop", self._on_stop, event_qos())
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
        else:
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
                              self.config, read_present_current=self._test_only,
                          ))
        self._hardware.open(enable_torque=False)
        self.core = ControlCore(
            self.config, self._hardware, dynamics, self._offsets,
            self._current_limit, trajectory_factory=make_trajectory,
            gravity_test=self._gravity_test, now=self._monotonic(),
            limit_test=self._limit_test,
        )
        self._publish_state()
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._timer = self.create_timer(
            1 / self.config.controller.frequency, self._tick, clock=self._steady_clock,
        )

    def _make_ruckig_trajectory(self):
        return RuckigTrajectory(frequency=self.config.controller.frequency, **self._limits)

    def _tick(self):
        if self._closed:
            return
        try:
            if self._follower_session_active and not self._fresh_follower():
                self.get_logger().warning("Follower state expired; stopping")
                self._on_stop(None)
                return
            result = self.core.step(now=self._monotonic())
            if result.mode == ControlMode.IDLE:
                self._follower_session_active = False
            self._publish_cycle(result)
        except Exception as exc:
            self._fault(exc)

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

    def _on_sync(self, message):
        if self._closed or message.data != "switch_sync":
            return
        if self.core.mode != ControlMode.IDLE:
            return
        if not self._fresh_follower():
            self.get_logger().warning("Synchronization requires fresh follower state")
            return
        self._start_motion(self._follower.position, follower_session=True)

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
        self._start_motion(target.position, follower_session=False)

    def _start_motion(self, target, *, follower_session):
        try:
            self._hardware.set_torque_enabled(True)
            self.core.handle("switch_sync", now=self._monotonic(), follower_position=target)
            self._follower_session_active = follower_session
            self._publish_state()
        except Exception as exc:
            self._fault(exc)

    def _on_teleop(self, message):
        if self._closed or message.data != "switch_teleop":
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

    def _fault(self, exc):
        self.get_logger().error(f"Control stopped after failure: {exc}")
        self.shutdown_control()

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
