import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory("gento_robot_driver"),
        "config",
        "gento_robot.yaml",
    )
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="Full path to the Gento driver parameter file",
            ),
            Node(
                package="gento_robot_driver",
                executable="gento_robot_driver",
                name="gento_robot_driver",
                parameters=[params_file],
                output="screen",
            ),
        ]
    )
