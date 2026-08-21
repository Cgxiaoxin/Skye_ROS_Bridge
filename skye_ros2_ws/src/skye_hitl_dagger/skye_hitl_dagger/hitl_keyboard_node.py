#!/usr/bin/env python3
"""HITL keyboard bridge: q/w keys to /skye/intervention_cmd."""

from __future__ import annotations

import select
import sys
import termios
import threading
import tty
from typing import Callable, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

KEY_TO_CMD = {
    "q": "takeover",
    "w": "return",
}


def map_key(key: str) -> Optional[str]:
    """Map a single key or line to an intervention command."""
    normalized = key.strip().lower()
    if not normalized:
        return None
    return KEY_TO_CMD.get(normalized)


class HitlKeyboardNode(Node):
    def __init__(self) -> None:
        super().__init__("hitl_keyboard")
        self._pub = self.create_publisher(String, "/skye/intervention_cmd", 10)
        self._reader = KeyboardReader(self._publish_cmd)
        self._reader.start()

    def _publish_cmd(self, cmd: str) -> None:
        msg = String()
        msg.data = cmd
        self._pub.publish(msg)
        self.get_logger().info(f"published intervention_cmd: {cmd}")

    def destroy_node(self) -> bool:
        self._reader.stop()
        return super().destroy_node()


class KeyboardReader:
    """Read q/w from stdin; tty cbreak when available, else line input."""

    def __init__(self, on_cmd: Callable[[str], None]) -> None:
        self._on_cmd = on_cmd
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._old_term_attrs = None
        self._use_tty = False

    def start(self) -> None:
        if self._thread is not None:
            return
        self._use_tty = sys.stdin.isatty()
        if self._use_tty:
            self._old_term_attrs = termios.tcgetattr(sys.stdin.fileno())
            tty.setcbreak(sys.stdin.fileno())
        self._running = True
        self._thread = threading.Thread(
            target=self._loop_tty if self._use_tty else self._loop_line,
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._old_term_attrs is not None:
            termios.tcsetattr(
                sys.stdin.fileno(), termios.TCSADRAIN, self._old_term_attrs)
            self._old_term_attrs = None

    def _emit(self, key: str) -> None:
        cmd = map_key(key)
        if cmd is not None:
            self._on_cmd(cmd)

    def _loop_tty(self) -> None:
        while self._running and rclpy.ok():
            readable, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not readable:
                continue
            self._emit(sys.stdin.read(1))

    def _loop_line(self) -> None:
        while self._running and rclpy.ok():
            line = sys.stdin.readline()
            if not line:
                break
            self._emit(line)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HitlKeyboardNode()
    if not sys.stdin.isatty():
        node.get_logger().warn(
            "stdin is not a TTY; using line mode (type q or w then Enter)")
    else:
        node.get_logger().info(
            "tty mode: q=takeover, w=return (no Enter required)")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
