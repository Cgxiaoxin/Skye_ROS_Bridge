"""ROS-only robot/gripper facades for HITL DAgger (no Gento SDK link)."""

from __future__ import annotations

import logging
import threading
import time
from typing import Optional

import numpy as np

LOGGER = logging.getLogger("tianji_dagger_ros_facade")


class NullGripper:
    """Satisfies gripper calls used by async_robot_client in dagger mode.

    Observations are zeros (OOD for the policy). Prefer RosGripperFeedback.
    """

    def wait_for_feedback(self, *_args, **_kwargs) -> np.ndarray:
        return np.zeros(2, dtype=np.float32)

    def state(self, *_args, **_kwargs) -> np.ndarray:
        return np.zeros(2, dtype=np.float32)

    def diagnostics(self) -> dict:
        empty = {
            "contact_latched": False,
            "feedback_normalized": 0.0,
            "command_normalized": 0.0,
        }
        return {
            "backend": "dagger_null_gripper",
            "left": dict(empty),
            "right": dict(empty),
        }

    def command(self, *_args, **_kwargs) -> None:
        return None

    def close(self) -> None:
        return None


class RosGripperFeedback:
    """Subscribe to driver ``/left|right_gripper/state`` (FACTR [0,1]).

    Interface matches ``DualGentoDmGripper.state()``: shape (2,) float32,
    order [left, right], calibrated 0=open / 1=closed.
    Contact latch is not ported yet (always False); command is a no-op
    because arbiter/driver own gripper actuation.
    """

    def __init__(
        self,
        *,
        left_topic: str = "/left_gripper/state",
        right_topic: str = "/right_gripper/state",
        warn_age_s: float = 0.05,
        node_name: str = "unijepa_dagger_gripper_state",
    ) -> None:
        self.left_topic = left_topic
        self.right_topic = right_topic
        self.warn_age_s = float(warn_age_s)
        self.node_name = node_name
        self._lock = threading.Lock()
        self._values = np.zeros(2, dtype=np.float32)
        self._stamp_mono: list[Optional[float]] = [None, None]
        self._node = None
        self._subs: list = []
        self._started = False

    def start(self, timeout_s: float = 5.0) -> np.ndarray:
        import rclpy
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import JointState

        if self._started:
            return self.state(timeout_s, {0, 1})

        if not rclpy.ok():
            rclpy.init(args=None)
        self._node = rclpy.create_node(self.node_name)
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._subs = [
            self._node.create_subscription(
                JointState,
                self.left_topic,
                lambda msg: self._on_state(0, msg),
                qos,
            ),
            self._node.create_subscription(
                JointState,
                self.right_topic,
                lambda msg: self._on_state(1, msg),
                qos,
            ),
        ]
        deadline = time.monotonic() + float(timeout_s)
        while time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.1)
            with self._lock:
                if self._stamp_mono[0] is not None and self._stamp_mono[1] is not None:
                    self._started = True
                    values = self._values.copy()
                    break
        else:
            raise RuntimeError(
                f"timed out waiting for {self.left_topic} / {self.right_topic}; "
                "enable gripper on skye_robot_driver"
            )

        self._validate_startup_range(values)
        LOGGER.info(
            "RosGripperFeedback ready: left=%.3f right=%.3f "
            "(topics %s, %s)",
            float(values[0]),
            float(values[1]),
            self.left_topic,
            self.right_topic,
        )
        return values

    def _validate_startup_range(self, values: np.ndarray) -> None:
        """Fail closed if values are clearly unusable (no pi05 stats available).

        Without pi05 dataset stats we only enforce finite values in [0, 1].
        Fully-open (≈0) is a valid physical reading; log a warning so operators
        notice if both sides look stuck at NullGripper-like zeros.
        """
        if values.shape != (2,) or not np.isfinite(values).all():
            raise RuntimeError(f"invalid gripper startup values: {values}")
        if np.any(values < -1e-3) or np.any(values > 1.0 + 1e-3):
            raise RuntimeError(
                f"gripper state outside [0,1]: left={values[0]:.4f} "
                f"right={values[1]:.4f}"
            )
        if float(np.max(np.abs(values))) < 1e-6:
            LOGGER.warning(
                "gripper state is ~0/0 at startup (fully open or missing "
                "motion); continuing because topics are live — compare with "
                "pi05 stats when available"
            )

    def _on_state(self, arm: int, msg) -> None:
        if not msg.position:
            return
        value = float(msg.position[0])
        if not np.isfinite(value):
            return
        value = float(np.clip(value, 0.0, 1.0))
        with self._lock:
            self._values[arm] = np.float32(value)
            self._stamp_mono[arm] = time.monotonic()

    def spin_once(self, timeout_sec: float = 0.0) -> None:
        if self._node is None:
            return
        import rclpy
        rclpy.spin_once(self._node, timeout_sec=timeout_sec)

    def age_s(self, enabled_arms: set[int] | None = None) -> float:
        arms = {0, 1} if enabled_arms is None else enabled_arms
        self.spin_once(0.0)
        now = time.monotonic()
        with self._lock:
            ages = []
            for arm in arms:
                stamp = self._stamp_mono[arm]
                if stamp is None:
                    return float("inf")
                ages.append(now - stamp)
        return max(ages) if ages else float("inf")

    def wait_for_feedback(
        self,
        timeout_s: float,
        max_age_s: float,
        enabled_arms: set[int] | None = None,
    ) -> np.ndarray:
        if not self._started:
            return self.start(timeout_s=timeout_s)
        deadline = time.monotonic() + float(timeout_s)
        while time.monotonic() < deadline:
            try:
                return self.state(max_age_s, enabled_arms)
            except RuntimeError:
                time.sleep(0.02)
        raise RuntimeError("RosGripperFeedback wait_for_feedback timed out")

    def state(
        self,
        max_age_s: float,
        enabled_arms: set[int] | None = None,
    ) -> np.ndarray:
        if not self._started:
            raise RuntimeError("RosGripperFeedback.start() was not called")
        self.spin_once(0.0)
        arms = {0, 1} if enabled_arms is None else enabled_arms
        now = time.monotonic()
        with self._lock:
            result = np.zeros(2, dtype=np.float32)
            for arm in arms:
                stamp = self._stamp_mono[arm]
                if stamp is None:
                    raise RuntimeError(
                        f"waiting for gripper arm {arm} feedback "
                        f"({self.left_topic if arm == 0 else self.right_topic})"
                    )
                age = now - stamp
                if age > self.warn_age_s:
                    LOGGER.warning(
                        "gripper arm %d feedback age=%.3fs > warn=%.3fs",
                        arm,
                        age,
                        self.warn_age_s,
                    )
                if age > float(max_age_s):
                    raise RuntimeError(
                        f"gripper arm {arm} feedback stale: age={age:.3f}s "
                        f"> {float(max_age_s):.3f}s"
                    )
                result[arm] = self._values[arm]
        return result

    def diagnostics(self) -> dict:
        self.spin_once(0.0)
        now = time.monotonic()
        with self._lock:
            def side(arm: int) -> dict:
                stamp = self._stamp_mono[arm]
                age = None if stamp is None else now - stamp
                return {
                    "contact_latched": False,
                    "feedback_normalized": float(self._values[arm]),
                    "command_normalized": None,
                    "feedback_age_seconds": age,
                }

            return {
                "backend": "ros_gripper_feedback",
                "left": side(0),
                "right": side(1),
            }

    def command(self, *_args, **_kwargs) -> None:
        # Actuation stays on arbiter → driver path.
        return None

    def close(self) -> None:
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
            self._subs = []
        self._started = False


class DaggerRosRobotFacade:
    """Read /gento/joint_states (rad) and expose get_rt_dict() in degree units."""

    def __init__(
        self,
        *,
        joint_states_topic: str = "/gento/joint_states",
        stale_s: float = 0.5,
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
        self.spin_once(0.0)
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
