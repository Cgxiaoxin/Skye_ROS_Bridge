# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

"""ROS 2 driver entry-point skeleton."""

from typing import Optional, Sequence


def main(args: Optional[Sequence[str]] = None) -> None:
    """Start the skeleton ROS node without commanding any hardware."""
    import rclpy

    rclpy.init(args=args)
    node = rclpy.create_node("skye_factr_teleop_driver")
    node.get_logger().info(
        "skye_factr_teleop package skeleton started; control is not implemented"
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
