from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory("skye_robot_driver"),
        "config",
        "skye_robot.yaml",
    )
    return LaunchDescription(
        [
            Node(
                package="skye_robot_driver",
                executable="skye_robot_driver",
                name="skye_robot_driver",
                output="screen",
                parameters=[cfg],
            )
        ]
    )
