"""ROS-only robot/gripper facades for HITL DAgger (no Gento SDK link)."""

from __future__ import annotations

import threading
import time
from typing import Optional

import numpy as np


class NullGripper:
    """Satisfies gripper calls used by async_robot_client in dagger mode."""

    def wait_for_feedback(self, *_args, **_kwargs) -> dict[int, float]:
        return {0: 0.0, 1: 0.0}

    def state(self, *_args, **_kwargs) -> np.ndarray:
        return np.zeros(2, dtype=np.float32)

    def diagnostics(self) -> dict:
        # Match DualGentoDmGripper.diagnostics() shape used by async_robot_client.
        empty = {"contact_latched": False, "backend": "dagger_null_gripper"}
        return {"left": dict(empty), "right": dict(empty)}

    def command(self, *_args, **_kwargs) -> None:
        return None

    def close(self) -> None:
        return None


class DaggerRosRobotFacade:
    """Read /gento/joint_states (rad) and expose get_rt_dict() in degree units."""

    def __init__(
        self,
        *,
        joint_states_topic: str = "/gento/joint_states",
        stale_s: float = 2.0,
    ) -> None:
        self.joint_states_topic = joint_states_topic
        self.stale_s = float(stale_s)
        self._connected = False
        self._lock = threading.Lock()
        self._left = np.zeros(7, dtype=np.float64)
        self._right = np.zeros(7, dtype=np.float64)
        self._stamp_mono: Optional[float] = None
        self._node = None
        self._sub = None

    def link(self, *_args, **_kwargs) -> int:
        import rclpy
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import JointState

        if not rclpy.ok():
            rclpy.init(args=None)
        self._node = rclpy.create_node("unijepa_dagger_joint_state")
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._sub = self._node.create_subscription(
            JointState, self.joint_states_topic, self._on_joint_state, qos
        )
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.1)
            with self._lock:
                if self._stamp_mono is not None:
                    self._connected = True
                    return 0
        raise RuntimeError(
            f"timed out waiting for {self.joint_states_topic}; "
            "start skye_robot_driver / confirm ROS_DOMAIN_ID"
        )

    def _on_joint_state(self, msg) -> None:
        positions = list(msg.position)
        if len(positions) < 14:
            return
        left = np.asarray(positions[:7], dtype=np.float64)
        right = np.asarray(positions[7:14], dtype=np.float64)
        if not np.isfinite(left).all() or not np.isfinite(right).all():
            return
        # Client state_from_rt expects degrees (SDK convention).
        with self._lock:
            self._left = np.rad2deg(left)
            self._right = np.rad2deg(right)
            self._stamp_mono = time.monotonic()

    def spin_once(self, timeout_sec: float = 0.0) -> None:
        if self._node is None:
            return
        import rclpy
        rclpy.spin_once(self._node, timeout_sec=timeout_sec)

    def get_rt_dict(self) -> dict:
        # Pump DDS briefly so a busy control loop does not false-trip stale.
        deadline = time.monotonic() + min(0.05, max(0.0, self.stale_s * 0.1))
        while time.monotonic() < deadline:
            self.spin_once(0.01)
            with self._lock:
                if self._stamp_mono is not None and (
                    time.monotonic() - self._stamp_mono
                ) <= self.stale_s:
                    break
        with self._lock:
            if self._stamp_mono is None:
                return {"error": f"no {self.joint_states_topic} yet"}
            age = time.monotonic() - self._stamp_mono
            if age > self.stale_s:
                return {
                    "error": (
                        f"{self.joint_states_topic} stale: age={age:.3f}s "
                        f"> {self.stale_s:.3f}s"
                    )
                }
            left = self._left.copy()
            right = self._right.copy()
        return {
            "arms": [
                {"fb": {"fb_pos": left.tolist(), "fb_ext_torque": [0.0] * 7},
                 "state": {"cur": 0, "cmd": 0}, "cmd": {}},
                {"fb": {"fb_pos": right.tolist(), "fb_ext_torque": [0.0] * 7},
                 "state": {"cur": 0, "cmd": 0}, "cmd": {}},
            ]
        }

    def get_robot_type(self) -> str:
        return "dagger-ros-joint-states"

    def get_controller_version(self) -> str:
        return "n/a"

    def get_sdk_version(self) -> str:
        return "n/a"

    def unlink(self) -> None:
        self.close()

    def close(self) -> None:
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
            self._sub = None
        self._connected = False
