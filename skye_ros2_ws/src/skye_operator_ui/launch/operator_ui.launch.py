from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("repo_root", default_value=""),
        DeclareLaunchArgument("config_path", default_value=""),
        Node(
            package="skye_operator_ui",
            executable="operator_ui",
            name="operator_ui",
            output="screen",
            parameters=[{
                "repo_root": LaunchConfiguration("repo_root"),
                "config_path": LaunchConfiguration("config_path"),
            }],
        ),
    ])
