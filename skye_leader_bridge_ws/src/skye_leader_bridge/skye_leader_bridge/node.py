from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .mapping import (
    ArmMappingConfig,
    BridgeSafetyConfig,
    GripperMappingConfig,
    LeaderBridgeState,
    fill_joint_command_message,
    map_arm_positions,
    map_gripper_value,
)


@dataclass
class ArmRuntime:
    name: str
    input_topic: str
    output_topic: str
    mapping: ArmMappingConfig
    output_type: str = "marvin_jointcmd"
    latest_target: list[float] | None = None


@dataclass
class GripperRuntime:
    name: str
    input_topic: str
    output_topic: str
    mapping: GripperMappingConfig
    output_type: str = "float32"


def load_config(config_file: str | Path) -> dict[str, Any]:
    text = Path(config_file).read_text(encoding="utf-8")
    try:
        import yaml
    except ImportError:
        return _load_simple_yaml(text)
    return yaml.safe_load(text) or {}


def _load_simple_yaml(text: str) -> dict[str, Any]:
    """Small YAML subset parser for this package's default config.

    It supports nested maps by indentation plus scalar and inline-list values.
    PyYAML is still preferred in a ROS environment.
    """
    root: dict[str, Any] = {}
    stack: list[tuple[int, dict[str, Any]]] = [(-1, root)]
    for raw_line in text.splitlines():
        line = raw_line.split("#", 1)[0].rstrip()
        if not line.strip():
            continue
        indent = len(line) - len(line.lstrip(" "))
        key, sep, value = line.strip().partition(":")
        if not sep:
            raise ValueError(f"Unsupported config line: {raw_line}")
        while stack and indent <= stack[-1][0]:
            stack.pop()
        parent = stack[-1][1]
        if value.strip() == "":
            child: dict[str, Any] = {}
            parent[key] = child
            stack.append((indent, child))
        else:
            parent[key] = _parse_simple_yaml_value(value.strip())
    return root


def _parse_simple_yaml_value(value: str) -> Any:
    if value.startswith("[") and value.endswith("]"):
        inner = value[1:-1].strip()
        if not inner:
            return []
        return [_parse_simple_yaml_value(part.strip()) for part in inner.split(",")]
    lowered = value.lower()
    if lowered == "true":
        return True
    if lowered == "false":
        return False
    try:
        if any(ch in value for ch in (".", "e", "E")):
            return float(value)
        return int(value)
    except ValueError:
        return value.strip("'\"")


def build_arm_runtime(name: str, data: dict[str, Any]) -> ArmRuntime:
    output_type = str(data.get("output_type", "marvin_jointcmd"))
    if output_type not in {"marvin_jointcmd", "joint_state"}:
        raise ValueError(
            f"arm '{name}' output_type must be 'marvin_jointcmd' or 'joint_state'"
        )
    return ArmRuntime(
        name=name,
        input_topic=str(data["input_topic"]),
        output_topic=str(data["output_topic"]),
        mapping=ArmMappingConfig(
            joint_order=tuple(data.get("joint_order", [0, 1, 2, 3, 4, 5, 6])),
            signs=tuple(data["signs"]) if "signs" in data else None,
            offsets=tuple(data["offsets"]) if "offsets" in data else None,
            limits_min=tuple(data["limits_min"]) if "limits_min" in data else None,
            limits_max=tuple(data["limits_max"]) if "limits_max" in data else None,
        ),
        output_type=output_type,
    )


def build_gripper_runtime(name: str, data: dict[str, Any]) -> GripperRuntime:
    return GripperRuntime(
        name=name,
        input_topic=str(data["input_topic"]),
        output_topic=str(data["output_topic"]),
        output_type=str(data.get("output_type", "float32")),
        mapping=GripperMappingConfig(
            input_min=float(data.get("input_min", 0.0)),
            input_max=float(data.get("input_max", 1.0)),
            output_min=float(data.get("output_min", 0.0)),
            output_max=float(data.get("output_max", 1.0)),
            invert=bool(data.get("invert", False)),
            deadband=float(data.get("deadband", 0.0)),
        ),
    )


def main(args: list[str] | None = None) -> None:
    import rclpy

    rclpy.init(args=args)
    bridge = SkyeLeaderBridgeNode()
    try:
        rclpy.spin(bridge.ros_node)
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


class SkyeLeaderBridgeNode:
    def __init__(self) -> None:
        import rclpy
        from rclpy.node import Node

        class _Node(Node):
            pass

        self._node = _Node("skye_leader_bridge")
        self._node.declare_parameter("config_file", "")
        config_file = self._node.get_parameter("config_file").get_parameter_value().string_value
        if not config_file:
            raise RuntimeError("Parameter 'config_file' is required")

        self.config = load_config(config_file)
        self.joint_fields = tuple(
            self.config.get(
                "message",
                {},
            ).get(
                "joint_command_fields",
                ["joint_pos", "joint_position", "joint_positions", "positions", "position", "pos", "data"],
            )
        )
        self.state = LeaderBridgeState(
            safety=BridgeSafetyConfig(
                leader_timeout_s=float(self.config.get("safety", {}).get("leader_timeout_s", 0.2)),
                max_delta_per_cycle=float(self.config.get("safety", {}).get("max_delta_per_cycle", 0.05)),
                require_enable=bool(self.config.get("safety", {}).get("require_enable", True)),
            )
        )
        self.publish_rate_hz = float(self.config.get("safety", {}).get("publish_rate_hz", 250.0))
        self.allow_existing_joint_publishers = bool(
            self.config.get("safety", {}).get("allow_existing_joint_publishers", False)
        )
        self.existing_publisher_check_delay_s = float(
            self.config.get("safety", {}).get("existing_publisher_check_delay_s", 1.0)
        )

        self.arms = {
            name: build_arm_runtime(name, arm_cfg)
            for name, arm_cfg in self.config.get("arms", {}).items()
            if arm_cfg.get("enabled", True)
        }
        self.grippers = {
            name: build_gripper_runtime(name, gripper_cfg)
            for name, gripper_cfg in self.config.get("grippers", {}).items()
            if gripper_cfg.get("enabled", True)
        }
        self.JointState, self.String, self.Float32, self.JointcmdArm = self._import_ros_messages(
            needs_marvin_messages=any(
                arm.output_type == "marvin_jointcmd" for arm in self.arms.values()
            )
        )

        self._check_existing_joint_publishers()
        self.arm_publishers = {
            name: self._node.create_publisher(
                self.JointState if arm.output_type == "joint_state" else self.JointcmdArm,
                arm.output_topic,
                10,
            )
            for name, arm in self.arms.items()
        }
        self.gripper_publishers = {
            name: self._create_gripper_publisher(gripper)
            for name, gripper in self.grippers.items()
        }
        self._subscriptions = []
        for name, arm in self.arms.items():
            self._subscriptions.append(
                self._node.create_subscription(
                    self.JointState,
                    arm.input_topic,
                    self._make_arm_callback(name),
                    10,
                )
            )
        for name, gripper in self.grippers.items():
            self._subscriptions.append(
                self._node.create_subscription(
                    self.JointState,
                    gripper.input_topic,
                    self._make_gripper_callback(name),
                    10,
                )
            )
        self._setup_mode_subscriptions()
        period = 1.0 / max(self.publish_rate_hz, 1e-6)
        self._timer = self._node.create_timer(period, self._publish_arm_commands)
        self._node.get_logger().info(
            f"Skye leader bridge started with {len(self.arms)} arms and {len(self.grippers)} grippers"
        )

    def __getattr__(self, name: str) -> Any:
        return getattr(self._node, name)

    @property
    def ros_node(self) -> Any:
        return self._node

    def destroy_node(self) -> None:
        self._node.destroy_node()

    def _check_existing_joint_publishers(self) -> None:
        if self.allow_existing_joint_publishers:
            self._node.get_logger().warn(
                "Existing joint command publisher check is disabled. "
                "Only use this if another publisher is intentionally muxed."
            )
            return

        if self.existing_publisher_check_delay_s > 0.0:
            time.sleep(self.existing_publisher_check_delay_s)

        conflicts: list[str] = []
        for arm in self.arms.values():
            infos = self._node.get_publishers_info_by_topic(arm.output_topic)
            external_publishers = [
                f"{info.node_namespace.rstrip('/')}/{info.node_name}".replace("//", "/")
                for info in infos
                if info.node_name != self._node.get_name()
            ]
            if external_publishers:
                conflicts.append(f"{arm.output_topic}: {', '.join(external_publishers)}")

        if conflicts:
            detail = "; ".join(conflicts)
            raise RuntimeError(
                "Refusing to start route-1 direct joint command bridge because target "
                "joint command topics already have publishers. Stop qp_controller or "
                f"launch only marvin_robot_node before starting this bridge. Conflicts: {detail}"
            )

    def _import_ros_messages(self, needs_marvin_messages: bool) -> tuple[Any, Any, Any, Any | None]:
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float32, String

        if not needs_marvin_messages:
            return JointState, String, Float32, None

        try:
            from marvin_msgs.msg import JointcmdArm
        except ImportError as exc:
            raise RuntimeError(
                "Cannot import marvin_msgs.msg.JointcmdArm. Source the Skye ROS workspace "
                "before launching skye_leader_bridge."
            ) from exc
        return JointState, String, Float32, JointcmdArm

    def _setup_mode_subscriptions(self) -> None:
        mode_cfg = self.config.get("mode", {})
        sync_topic = mode_cfg.get("sync_topic", "/mode/switch_sync")
        teleop_topic = mode_cfg.get("teleop_topic", "/mode/switch_teleop")
        stop_topic = mode_cfg.get("stop_topic", "/mode/switch_stop")
        self._subscriptions.extend(
            [
                self._node.create_subscription(self.String, sync_topic, self._sync_callback, 10),
                self._node.create_subscription(self.String, teleop_topic, self._teleop_callback, 10),
                self._node.create_subscription(self.String, stop_topic, self._stop_callback, 10),
            ]
        )
        if not self.state.safety.require_enable:
            self.state.set_enabled(True)

    def _make_arm_callback(self, arm_name: str):
        def callback(msg: Any) -> None:
            arm = self.arms[arm_name]
            if len(msg.position) == 0:
                self._node.get_logger().warn(f"{arm_name} leader JointState has no position")
                return
            try:
                target = map_arm_positions(msg.position, arm.mapping)
            except ValueError as exc:
                self._node.get_logger().error(f"Invalid {arm_name} leader joint command: {exc}")
                return
            arm.latest_target = target
            self.state.update_leader(arm_name, target)

        return callback

    def _publish_arm_commands(self) -> None:
        for name, arm in self.arms.items():
            if arm.latest_target is None:
                continue
            command = self.state.command_for_arm(name, arm.latest_target)
            if command is None:
                continue
            msg = self._make_arm_message(arm, command)
            if msg is None:
                continue
            self.arm_publishers[name].publish(msg)

    def _make_arm_message(self, arm: ArmRuntime, command: list[float]) -> Any | None:
        if arm.output_type == "joint_state":
            msg = self.JointState()
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.name = [f"{arm.name}_j{index}" for index in range(1, len(command) + 1)]
            msg.position = command
            return msg

        if self.JointcmdArm is None:
            self._node.get_logger().error("marvin_jointcmd output requested without marvin_msgs")
            return None
        msg = self.JointcmdArm()
        if hasattr(msg, "header"):
            msg.header.stamp = self._node.get_clock().now().to_msg()
        try:
            fill_joint_command_message(msg, command, self.joint_fields)
        except AttributeError as exc:
            self._node.get_logger().error(str(exc))
            return None
        return msg

    def _make_gripper_callback(self, gripper_name: str):
        def callback(msg: Any) -> None:
            if len(msg.position) == 0:
                self._node.get_logger().warn(f"{gripper_name} leader gripper JointState has no position")
                return
            if self.state.safety.require_enable and not self.state.enabled:
                return
            gripper = self.grippers[gripper_name]
            output_value = map_gripper_value(float(msg.position[0]), gripper.mapping)
            publisher = self.gripper_publishers[gripper_name]
            publisher.publish(self._make_gripper_msg(gripper, output_value))

        return callback

    def _create_gripper_publisher(self, gripper: GripperRuntime) -> Any:
        msg_type = self.JointState if gripper.output_type == "joint_state" else self.Float32
        return self._node.create_publisher(msg_type, gripper.output_topic, 10)

    def _make_gripper_msg(self, gripper: GripperRuntime, value: float) -> Any:
        if gripper.output_type == "joint_state":
            msg = self.JointState()
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.name = ["gripper_joint"]
            msg.position = [float(value)]
            return msg
        msg = self.Float32()
        msg.data = float(value)
        return msg

    def _sync_callback(self, msg: Any) -> None:
        del msg
        self.state.set_enabled(False)
        for arm in self.arms.values():
            arm.latest_target = None
        self._node.get_logger().info("Bridge sync requested; teleop output disabled until /mode/switch_teleop")

    def _teleop_callback(self, msg: Any) -> None:
        del msg
        self.state.set_enabled(True)
        self._node.get_logger().info("Bridge teleop enabled")

    def _stop_callback(self, msg: Any) -> None:
        del msg
        self.state.set_enabled(False)
        self._node.get_logger().info("Bridge teleop stopped")
