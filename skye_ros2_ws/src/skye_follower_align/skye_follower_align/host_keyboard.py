#!/usr/bin/env python3
"""Host keyboard bridge: s/x keys to align mode topics; q quits."""

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

from skye_follower_align.align_keys import map_key


class KeyboardReader:
    """Read s/x/q from stdin; tty cbreak when available, else line input."""

    def __init__(self, on_action: Callable[[str], None]) -> None:
        self._on_action = on_action
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

    def request_stop(self) -> None:
        self._running = False

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            # Quit ('q') runs on the reader thread; joining self deadlocks.
            if threading.current_thread() is not self._thread:
                self._thread.join(timeout=1.0)
            self._thread = None
        if self._old_term_attrs is not None:
            termios.tcsetattr(
                sys.stdin.fileno(), termios.TCSADRAIN, self._old_term_attrs)
            self._old_term_attrs = None

    def _emit(self, key: str) -> None:
        action = map_key(key)
        if action is not None:
            self._on_action(action)

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


class HostKeyboardAlignNode(Node):
    def __init__(self) -> None:
        super().__init__("host_keyboard_align")
        self._align_pub = self.create_publisher(
            String, "/mode/align_follower", 10)
        self._cancel_pub = self.create_publisher(
            String, "/mode/align_cancel", 10)
        self._reader = KeyboardReader(self._handle_action)
        self._reader.start()

    def _handle_action(self, action: str) -> None:
        if action == "quit":
            self.get_logger().info("quit requested")
            self._reader.request_stop()
            rclpy.shutdown()
            return
        if action == "align_follower":
            msg = String()
            msg.data = "align_follower"
            self._align_pub.publish(msg)
            self.get_logger().info("published align_follower")
            return
        if action == "align_cancel":
            msg = String()
            msg.data = "align_cancel"
            self._cancel_pub.publish(msg)
            self.get_logger().info("published align_cancel")

    def destroy_node(self) -> bool:
        self._reader.stop()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HostKeyboardAlignNode()
    if not sys.stdin.isatty():
        node.get_logger().warn(
            "stdin is not a TTY; using line mode (type s, x, or q then Enter)")
    else:
        node.get_logger().info("s=align x=cancel q=quit")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
