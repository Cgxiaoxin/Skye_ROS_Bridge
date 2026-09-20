"""ROS2 subscriptions, health probes, and whitelisted command dispatch."""

from __future__ import annotations

import time
from typing import TYPE_CHECKING, Any

from skye_operator_ui.commands import validate_op
from skye_operator_ui.session_state import UiMode

if TYPE_CHECKING:
    from rclpy.node import Node

FRESHNESS_S = 1.0
ALIGN_FRESHNESS_S = 2.0
TRIGGER_TIMEOUT_S = 2.0
TRIGGER_POLL_S = 0.01

_STRING_OPS: dict[str, tuple[str, str]] = {
    "switch_sync": ("/mode/switch_sync", "switch_sync"),
    "switch_teleop": ("/mode/switch_teleop", "switch_teleop"),
    "switch_stop": ("/mode/switch_stop", "switch_stop"),
    "align_start": ("/mode/align_follower", "align_follower"),
    "align_cancel": ("/mode/align_cancel", "align_cancel"),
    "takeover": ("/skye/intervention_cmd", "takeover"),
    "enter_teleop": ("/skye/intervention_cmd", "enter_teleop"),
    "return": ("/skye/intervention_cmd", "return"),
}

_TRIGGER_OPS: dict[str, str] = {
    "emergency_stop": "/gento/emergency_stop",
    "hold_current": "/gento/hold_current",
    "stop_motion": "/gento/stop_motion",
}

_RECORDER_SERVICES: dict[UiMode, dict[str, str]] = {
    UiMode.teleop_record: {
        "recorder_start": "/skye/data_recorder/start",
        "recorder_stop": "/skye/data_recorder/stop",
    },
    UiMode.dagger: {
        "recorder_start": "/skye/recorder/start",
        "recorder_stop": "/skye/recorder/stop",
    },
}


def dispatch_plan(op: str, ui_mode: UiMode | None) -> tuple[str, str, str] | None:
    """Pure dispatch mapping: (kind, target, payload). kind is string|trigger."""
    if op in _STRING_OPS:
        topic, payload = _STRING_OPS[op]
        return ("string", topic, payload)
    if op in _TRIGGER_OPS:
        return ("trigger", _TRIGGER_OPS[op], "")
    if ui_mode is not None and op in _RECORDER_SERVICES.get(ui_mode, {}):
        return ("trigger", _RECORDER_SERVICES[ui_mode][op], "")
    return None


class RosBridge:
    """Composable ROS bridge: subscribe to state, dispatch whitelisted ops."""

    def __init__(self, node: Node) -> None:
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Int16MultiArray, String

        try:
            from skye_hitl_dagger.msg import ControlMode
        except ImportError:
            ControlMode = None  # type: ignore[misc, assignment]

        self._node = node
        self._ui_mode: UiMode | None = None
        self._cb_group = ReentrantCallbackGroup()

        self._joint_states_stamp: float | None = None
        self._align_stamp: float | None = None
        self._control_mode_stamp: float | None = None
        self._teleop_state: str | None = None
        self._align_status: str | None = None
        self._hitl_mode: str | None = None
        self._hitl_source: str | None = None
        self._left_joints: list[float] = []
        self._right_joints: list[float] = []
        self._left_gripper: float | None = None
        self._right_gripper: float | None = None
        self._robot_state: list[int] | None = None
        self._recording_active = False

        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        teleop_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        node.create_subscription(
            JointState,
            "/gento/joint_states",
            self._joint_states_callback,
            state_qos,
            callback_group=self._cb_group,
        )
        node.create_subscription(
            Int16MultiArray,
            "/gento/robot_state",
            self._robot_state_callback,
            state_qos,
            callback_group=self._cb_group,
        )
        node.create_subscription(
            JointState,
            "/left_gripper/state",
            lambda msg: self._gripper_callback(msg, "left"),
            state_qos,
            callback_group=self._cb_group,
        )
        node.create_subscription(
            JointState,
            "/right_gripper/state",
            lambda msg: self._gripper_callback(msg, "right"),
            state_qos,
            callback_group=self._cb_group,
        )
        node.create_subscription(
            String,
            "/teleop/state",
            self._teleop_state_callback,
            teleop_qos,
            callback_group=self._cb_group,
        )
        node.create_subscription(
            String,
            "/align/status",
            self._align_status_callback,
            teleop_qos,
            callback_group=self._cb_group,
        )
        if ControlMode is not None:
            node.create_subscription(
                ControlMode,
                "/skye/control_mode",
                self._control_mode_callback,
                state_qos,
                callback_group=self._cb_group,
            )

        self._string_publishers: dict[str, Any] = {}
        self._trigger_clients: dict[str, Any] = {}

    def set_session_mode(self, mode: UiMode | None) -> None:
        self._ui_mode = mode
        if mode is None:
            self.clear_session_cache()

    def clear_session_cache(self) -> None:
        """Drop latched session state so a stopped session shows no stale values."""
        self._teleop_state = None
        self._align_status = None
        self._hitl_mode = None
        self._hitl_source = None
        self._align_stamp = None
        self._control_mode_stamp = None
        self._recording_active = False

    def recording_active(self) -> bool:
        return self._recording_active

    def stop_recorder_if_active(self) -> tuple[bool, str]:
        """Best-effort recorder stop; safe to call during teardown or degrade."""
        if not self._recording_active:
            return True, ""
        try:
            return self.dispatch("recorder_stop")
        except Exception as exc:  # noqa: BLE001 - teardown must never raise
            return False, str(exc)

    def available(self) -> bool:
        import rclpy

        return self._node is not None and rclpy.ok()

    def mailbox(self) -> dict[str, Any]:
        return {
            "teleop_state": self._teleop_state,
            "align_status": self._align_status,
            "hitl_mode": self._hitl_mode,
            "hitl_source": self._hitl_source,
            "left_joints": list(self._left_joints),
            "right_joints": list(self._right_joints),
            "left_gripper": self._left_gripper,
            "right_gripper": self._right_gripper,
            "robot_state": list(self._robot_state) if self._robot_state is not None else None,
            "health": self._health_map(),
            "recording_active": self._recording_active,
        }

    def health(self, key: str) -> bool:
        if key == "driver":
            return self._is_fresh(self._joint_states_stamp, FRESHNESS_S)
        if key == "align":
            return self._align_status is not None and self._is_fresh(
                self._align_stamp, ALIGN_FRESHNESS_S
            )
        if key == "arbiter":
            return self._is_fresh(self._control_mode_stamp, FRESHNESS_S)
        if key == "marvin":
            return self._teleop_state is not None
        if key == "recorder":
            return self._service_ready(self._recorder_service_name())
        if key == "policy":
            return self._is_fresh(self._control_mode_stamp, FRESHNESS_S)
        return False

    def dispatch(self, op: str) -> tuple[bool, str]:
        if not validate_op(op):
            return False, "未知命令，已拒绝"

        plan = dispatch_plan(op, self._ui_mode)
        if plan is None:
            return False, "命令当前不可用"

        kind, target, payload = plan
        if kind == "string":
            return self._publish_string(target, payload)
        if kind == "trigger":
            return self._call_trigger(target, op)
        return False, "内部错误：未知派发类型"

    def _health_map(self) -> dict[str, bool]:
        return {
            "driver": self.health("driver"),
            "align": self.health("align"),
            "arbiter": self.health("arbiter"),
            "marvin": self.health("marvin"),
            "recorder": self.health("recorder"),
            "policy": self.health("policy"),
        }

    def _recorder_service_name(self) -> str | None:
        if self._ui_mode is None:
            return None
        services = _RECORDER_SERVICES.get(self._ui_mode)
        if not services:
            return None
        return services["recorder_start"]

    def _is_fresh(self, stamp: float | None, max_age_s: float) -> bool:
        if stamp is None:
            return False
        return (time.monotonic() - stamp) <= max_age_s

    def _touch_stamp(self, msg: Any) -> float:
        stamp = time.monotonic()
        header = getattr(msg, "header", None)
        if header is not None and hasattr(header, "stamp"):
            ros_stamp = header.stamp
            if hasattr(ros_stamp, "sec"):
                if ros_stamp.sec or ros_stamp.nanosec:
                    return time.monotonic()
        return stamp

    def _joint_states_callback(self, msg: Any) -> None:
        self._joint_states_stamp = self._touch_stamp(msg)
        positions = list(msg.position)
        if len(positions) >= 14:
            self._left_joints = positions[:7]
            self._right_joints = positions[7:14]
        elif len(positions) >= 7:
            self._left_joints = positions[:7]

    def _robot_state_callback(self, msg: Any) -> None:
        self._robot_state = list(msg.data)

    def _gripper_callback(self, msg: Any, side: str) -> None:
        if msg.position:
            if side == "left":
                self._left_gripper = float(msg.position[0])
            else:
                self._right_gripper = float(msg.position[0])

    def _teleop_state_callback(self, msg: Any) -> None:
        self._teleop_state = msg.data

    def _align_status_callback(self, msg: Any) -> None:
        self._align_status = msg.data
        self._align_stamp = time.monotonic()

    def _control_mode_callback(self, msg: Any) -> None:
        self._control_mode_stamp = self._touch_stamp(msg)
        self._hitl_mode = msg.mode
        self._hitl_source = msg.source

    def _get_string_publisher(self, topic: str) -> Any:
        if topic not in self._string_publishers:
            from std_msgs.msg import String

            self._string_publishers[topic] = self._node.create_publisher(
                String, topic, 10
            )
        return self._string_publishers[topic]

    def _get_trigger_client(self, service: str) -> Any:
        if service not in self._trigger_clients:
            from std_srvs.srv import Trigger

            self._trigger_clients[service] = self._node.create_client(
                Trigger, service, callback_group=self._cb_group
            )
        return self._trigger_clients[service]

    def _publish_string(self, topic: str, data: str) -> tuple[bool, str]:
        from std_msgs.msg import String

        pub = self._get_string_publisher(topic)
        msg = String()
        msg.data = data
        pub.publish(msg)
        return True, ""

    def _call_trigger(self, service: str, op: str) -> tuple[bool, str]:
        client = self._get_trigger_client(service)
        if not client.service_is_ready():
            if not client.wait_for_service(timeout_sec=0.5):
                return False, f"服务不可用：{service}"

        from std_srvs.srv import Trigger

        # The node is owned by a MultiThreadedExecutor spinning in its own thread;
        # spinning it here would steal callbacks from that executor, so just poll.
        future = client.call_async(Trigger.Request())
        if not self._await_future(future, TRIGGER_TIMEOUT_S):
            future.cancel()
            return False, f"服务调用超时：{service}"

        result = future.result()
        if result is None:
            return False, f"服务调用失败：{service}"
        if not result.success:
            message = result.message or "服务返回失败"
            return False, message

        if op in ("recorder_start", "recorder_stop"):
            self._recording_active = op == "recorder_start"
        return True, result.message or ""

    @staticmethod
    def _await_future(future: Any, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while not future.done():
            if time.monotonic() >= deadline:
                return False
            time.sleep(TRIGGER_POLL_S)
        return True

    def _service_ready(self, service: str | None) -> bool:
        if not service:
            return False
        client = self._get_trigger_client(service)
        return client.service_is_ready()
