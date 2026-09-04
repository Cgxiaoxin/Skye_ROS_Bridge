#!/usr/bin/env python3
"""ROS 2 node: align big-arm followers to leader arms after FACTR sync."""

from __future__ import annotations

import time
from typing import Optional, Sequence

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from skye_robot_driver.srv import SetMotionRates
from std_msgs.msg import String
from std_srvs.srv import Trigger

from skye_follower_align.align_logic import (
    AlignPhase,
    AlignSession,
    combine_phase,
    leader_positions_for_abs_command,
)

DOF = 7
PHASE_TO_STATUS = {
    AlignPhase.IDLE: "IDLE",
    AlignPhase.ALIGNING: "ALIGNING",
    AlignPhase.ALIGNED: "ALIGNED",
    AlignPhase.TIMEOUT_WARN: "TIMEOUT_WARN",
}
DEFAULT_SIGNS = [1.0] * DOF


def joint_positions(msg: JointState, count: int = DOF) -> Optional[list[float]]:
    if len(msg.position) < count:
        return None
    return [float(v) for v in msg.position[:count]]


class FollowerAlignNode(Node):
    def __init__(self) -> None:
        super().__init__("follower_align")

        self.declare_parameter("align_threshold_rad", 0.05)
        self.declare_parameter("align_hold_frames", 5)
        self.declare_parameter("align_timeout_s", 10.0)
        self.declare_parameter("align_rate_hz", 50.0)
        self.declare_parameter("align_vel_ratio", 10)
        self.declare_parameter("align_acc_ratio", 10)
        self.declare_parameter("restore_left_vel_ratio", 30)
        self.declare_parameter("restore_left_acc_ratio", 30)
        self.declare_parameter("restore_right_vel_ratio", 30)
        self.declare_parameter("restore_right_acc_ratio", 30)
        self.declare_parameter("leader_freshness_s", 0.5)
        self.declare_parameter("big_freshness_s", 0.5)
        self.declare_parameter("left_joint_signs", DEFAULT_SIGNS)
        self.declare_parameter("right_joint_signs", DEFAULT_SIGNS)

        threshold = float(self.get_parameter("align_threshold_rad").value)
        hold_frames = int(self.get_parameter("align_hold_frames").value)
        timeout_s = float(self.get_parameter("align_timeout_s").value)
        self._align_vel = int(self.get_parameter("align_vel_ratio").value)
        self._align_acc = int(self.get_parameter("align_acc_ratio").value)
        self._restore_rates = (
            int(self.get_parameter("restore_left_vel_ratio").value),
            int(self.get_parameter("restore_left_acc_ratio").value),
            int(self.get_parameter("restore_right_vel_ratio").value),
            int(self.get_parameter("restore_right_acc_ratio").value),
        )
        self._leader_freshness_s = float(
            self.get_parameter("leader_freshness_s").value)
        self._big_freshness_s = float(self.get_parameter("big_freshness_s").value)
        self._left_signs = [
            float(v) for v in self.get_parameter("left_joint_signs").value]
        self._right_signs = [
            float(v) for v in self.get_parameter("right_joint_signs").value]

        self._left_session = AlignSession(threshold, hold_frames, timeout_s)
        self._right_session = AlignSession(threshold, hold_frames, timeout_s)
        self._status: Optional[str] = None
        self._streaming = False

        self._left_leader: Optional[list[float]] = None
        self._right_leader: Optional[list[float]] = None
        self._left_leader_at: Optional[float] = None
        self._right_leader_at: Optional[float] = None
        self._big_left: Optional[list[float]] = None
        self._big_right: Optional[list[float]] = None
        self._big_at: Optional[float] = None

        cmd_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self._left_cmd_pub = self.create_publisher(
            JointState, "/gento/left_joint_control_abs", cmd_qos)
        self._right_cmd_pub = self.create_publisher(
            JointState, "/gento/right_joint_control_abs", cmd_qos)
        self._status_pub = self.create_publisher(String, "/align/status", status_qos)

        self.create_subscription(
            String, "/mode/align_follower", self._align_callback, 10)
        self.create_subscription(
            String, "/mode/align_cancel", self._cancel_callback, 10)
        self.create_subscription(
            JointState, "/left_leader_arm/current_state",
            self._left_leader_callback, state_qos)
        self.create_subscription(
            JointState, "/right_leader_arm/current_state",
            self._right_leader_callback, state_qos)
        self.create_subscription(
            JointState, "/gento/joint_states", self._big_callback, state_qos)

        # Reentrant group + MultiThreadedExecutor: spin_until_future_complete in
        # callbacks would deadlock with the default MutuallyExclusive group.
        service_cb_group = ReentrantCallbackGroup()
        self._rates_client = self.create_client(
            SetMotionRates, "/gento/set_motion_rates",
            callback_group=service_cb_group)
        self._hold_client = self.create_client(
            Trigger, "/gento/hold_current", callback_group=service_cb_group)

        rate_hz = max(1.0, float(self.get_parameter("align_rate_hz").value))
        self.create_timer(1.0 / rate_hz, self._timer_callback)
        self._publish_status("IDLE")

    def _publish_status(self, status: str) -> None:
        if status == self._status:
            return
        self._status = status
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)
        self.get_logger().info(f"align status: {status}")

    def _is_fresh(self, received_at: Optional[float], max_age_s: float) -> bool:
        if received_at is None:
            return False
        return (time.monotonic() - received_at) < max_age_s

    def _align_callback(self, msg: String) -> None:
        if msg.data != "align_follower":
            return
        if not self._left_session.start():
            self.get_logger().info("align ignored: already aligning")
            return
        self._right_session.start()
        if not self._call_set_motion_rates(
                self._align_vel, self._align_acc,
                self._align_vel, self._align_acc):
            self.get_logger().error("failed to set align motion rates")
            if not self._call_set_motion_rates(*self._restore_rates):
                self.get_logger().error(
                    "failed to restore motion rates after align rate failure")
            self._left_session.cancel()
            self._right_session.cancel()
            return
        self._streaming = True
        self._publish_status("ALIGNING")

    def _cancel_callback(self, msg: String) -> None:
        if msg.data != "align_cancel":
            return
        if self._phase() == AlignPhase.ALIGNING:
            self._abort_align("cancelled")
        else:
            self._left_session.cancel()
            self._right_session.cancel()
            self._streaming = False
            self._publish_status("IDLE")

    def _left_leader_callback(self, msg: JointState) -> None:
        positions = joint_positions(msg)
        if positions is None:
            return
        self._left_leader = positions
        self._left_leader_at = time.monotonic()

    def _right_leader_callback(self, msg: JointState) -> None:
        positions = joint_positions(msg)
        if positions is None:
            return
        self._right_leader = positions
        self._right_leader_at = time.monotonic()

    def _big_callback(self, msg: JointState) -> None:
        if len(msg.position) < DOF * 2:
            return
        self._big_left = [float(v) for v in msg.position[:DOF]]
        self._big_right = [float(v) for v in msg.position[DOF:DOF * 2]]
        self._big_at = time.monotonic()

    def _phase(self) -> AlignPhase:
        return combine_phase(self._left_session.phase, self._right_session.phase)

    def _timer_callback(self) -> None:
        if self._phase() != AlignPhase.ALIGNING or not self._streaming:
            return

        leaders_ok = (
            self._is_fresh(self._left_leader_at, self._leader_freshness_s)
            and self._is_fresh(self._right_leader_at, self._leader_freshness_s))
        if not leaders_ok:
            self._abort_align("leader feedback stale or missing")
            return

        if (self._left_leader is None or self._right_leader is None
                or self._big_left is None or self._big_right is None):
            return

        big_fresh = self._is_fresh(self._big_at, self._big_freshness_s)
        if big_fresh:
            # skye_robot_driver applies joint signs on *_joint_control_abs.
            self._publish_abs(
                self._left_cmd_pub,
                leader_positions_for_abs_command(self._left_leader))
            self._publish_abs(
                self._right_cmd_pub,
                leader_positions_for_abs_command(self._right_leader))

        # Keep advancing sessions (timeout) even when big feedback is stale.
        left_phase = self._left_session.on_tick(
            self._left_leader, self._big_left, self._left_signs)
        right_phase = self._right_session.on_tick(
            self._right_leader, self._big_right, self._right_signs)
        phase = combine_phase(left_phase, right_phase)

        if phase in (AlignPhase.ALIGNED, AlignPhase.TIMEOUT_WARN):
            self._finish_align(phase)

    def _publish_abs(self, publisher, positions: Sequence[float]) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = list(positions)
        publisher.publish(msg)

    def _finish_align(self, phase: AlignPhase) -> None:
        self._streaming = False
        self._call_hold()
        if not self._call_set_motion_rates(*self._restore_rates):
            self.get_logger().error(
                "failed to restore motion rates; arms may remain at align speed")
        self._publish_status(PHASE_TO_STATUS[phase])
        self._left_session.cancel()
        self._right_session.cancel()

    def _abort_align(self, reason: str) -> None:
        self.get_logger().error(f"align aborted: {reason}")
        self._streaming = False
        self._left_session.cancel()
        self._right_session.cancel()
        self._call_hold()
        if not self._call_set_motion_rates(*self._restore_rates):
            self.get_logger().error(
                "failed to restore motion rates after abort")
        self._publish_status("IDLE")

    def _shutdown_cleanup(self) -> None:
        if not self._streaming and self._phase() != AlignPhase.ALIGNING:
            return
        self.get_logger().warn("shutdown during align; holding and restoring rates")
        self._streaming = False
        self._call_hold()
        if not self._call_set_motion_rates(*self._restore_rates):
            self.get_logger().error("failed to restore motion rates on shutdown")
        self._left_session.cancel()
        self._right_session.cancel()

    def _call_set_motion_rates(
            self, left_vel: int, left_acc: int,
            right_vel: int, right_acc: int) -> bool:
        if not self._rates_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("set_motion_rates service unavailable")
            return False
        request = SetMotionRates.Request()
        request.left_vel_ratio = int(left_vel)
        request.left_acc_ratio = int(left_acc)
        request.right_vel_ratio = int(right_vel)
        request.right_acc_ratio = int(right_acc)
        future = self._rates_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done():
            self.get_logger().error("set_motion_rates call timed out")
            return False
        response = future.result()
        if response is None or not response.success:
            message = response.message if response else "no response"
            self.get_logger().error(f"set_motion_rates failed: {message}")
            return False
        return True

    def _call_hold(self) -> bool:
        if not self._hold_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("hold_current service unavailable")
            return False
        future = self._hold_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done():
            self.get_logger().error("hold_current call timed out")
            return False
        response = future.result()
        if response is None or not response.success:
            message = response.message if response else "no response"
            self.get_logger().error(f"hold_current failed: {message}")
            return False
        return True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FollowerAlignNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._shutdown_cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
