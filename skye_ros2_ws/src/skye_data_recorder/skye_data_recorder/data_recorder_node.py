#!/usr/bin/env python3
"""MCAP episode recorder for applied joint/gripper actions."""

from __future__ import annotations

import gc
from pathlib import Path
from typing import Dict, Sequence

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.serialization import serialize_message
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

import rosbag2_py


DEFAULT_TOPICS = (
    "/gento/left_joint_action_applied",
    "/gento/right_joint_action_applied",
    "/gento/left_gripper_action_applied",
    "/gento/right_gripper_action_applied",
    "/gento/joint_states",
    "/left_gripper/state",
    "/right_gripper/state",
)

TOPIC_MESSAGE_TYPES = {t: JointState for t in DEFAULT_TOPICS}

APPLIED_TOPICS = {
    "/gento/left_joint_action_applied",
    "/gento/right_joint_action_applied",
    "/gento/left_gripper_action_applied",
    "/gento/right_gripper_action_applied",
}


def next_episode_path(output_dir: str, existing: Sequence[Path] = ()) -> Path:
    """Return the first unused rosbag2 episode directory URI."""
    root = Path(output_dir)
    used = {path.name for path in existing}
    index = 0
    while f"episode_{index:04d}" in used or (
            root / f"episode_{index:04d}").exists():
        index += 1
    return root / f"episode_{index:04d}"


class DataRecorderNode(Node):
    """Record configured topics only while an episode is active."""

    def __init__(self) -> None:
        super().__init__("skye_data_recorder")
        self.declare_parameter("output_dir", "/tmp/skye_data_bags")
        self.declare_parameter("topics", list(DEFAULT_TOPICS))
        self.declare_parameter("storage_id", "mcap")
        self.declare_parameter("applied_qos_depth", 20)
        self._output_dir = str(self.get_parameter("output_dir").value)
        self._topics = tuple(self.get_parameter("topics").value)
        self._storage_id = str(self.get_parameter("storage_id").value)
        applied_qos_depth = int(self.get_parameter("applied_qos_depth").value)
        unknown = [topic for topic in self._topics
                   if topic not in TOPIC_MESSAGE_TYPES]
        if unknown:
            raise ValueError(f"unsupported recorder topics: {unknown}")

        self._writer = None
        self._episode_path: Path | None = None
        self._topic_types: Dict[str, str] = {}
        self._subscriptions = []
        applied_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=max(10, applied_qos_depth),
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        for topic in self._topics:
            message_type = TOPIC_MESSAGE_TYPES[topic]
            qos = applied_qos if topic in APPLIED_TOPICS else state_qos
            self._subscriptions.append(
                self.create_subscription(
                    message_type, topic,
                    lambda msg, topic=topic: self._record(topic, msg), qos))

        self.create_service(Trigger, "/skye/data_recorder/start", self._start)
        self.create_service(Trigger, "/skye/data_recorder/stop", self._stop)

    def _start(self, _request, response):
        if self._writer is not None:
            response.success = False
            response.message = "recording already active"
            return response

        output_dir = Path(self._output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        self._episode_path = next_episode_path(self._output_dir)
        writer = rosbag2_py.SequentialWriter()
        try:
            writer.open(
                rosbag2_py.StorageOptions(
                    uri=str(self._episode_path),
                    storage_id=self._storage_id),
                rosbag2_py.ConverterOptions(
                    input_serialization_format="cdr",
                    output_serialization_format="cdr"))
        except Exception as exc:
            self._episode_path = None
            if self._storage_id == "mcap":
                message = (
                    "failed to open MCAP writer; install the plugin with "
                    "`sudo apt install ros-humble-rosbag2-storage-mcap`: "
                    f"{exc}")
            else:
                message = (
                    f"failed to open rosbag2 writer for storage_id="
                    f"'{self._storage_id}': {exc}")
            self.get_logger().error(message)
            response.success = False
            response.message = message
            return response
        self._topic_types = {}
        for topic in self._topics:
            message_type = TOPIC_MESSAGE_TYPES[topic]
            type_name = (
                f"{message_type.__module__.split('.')[0]}/msg/"
                f"{message_type.__name__}")
            writer.create_topic(rosbag2_py.TopicMetadata(
                name=topic,
                type=type_name,
                serialization_format="cdr",
                offered_qos_profiles=""))
            self._topic_types[topic] = type_name
        self._writer = writer
        response.success = True
        response.message = f"started {self._episode_path}"
        self.get_logger().info(response.message)
        return response

    def _stop(self, _request, response):
        if self._writer is None:
            response.success = False
            response.message = "recording is not active"
            return response
        path = self._episode_path
        self._writer = None
        self._topic_types = {}
        self._episode_path = None
        gc.collect()
        response.success = True
        response.message = f"stopped {path}"
        self.get_logger().info(response.message)
        return response

    def _record(self, topic: str, message) -> None:
        writer = self._writer
        if writer is None:
            return
        stamp = message.header.stamp
        timestamp = stamp.sec * 1_000_000_000 + stamp.nanosec
        writer.write(topic, serialize_message(message), timestamp)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DataRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._writer = None
        node.destroy_node()
        rclpy.shutdown()
