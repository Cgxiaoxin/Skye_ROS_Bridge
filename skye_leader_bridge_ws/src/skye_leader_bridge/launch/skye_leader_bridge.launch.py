from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [FindPackageShare("skye_leader_bridge"), "config", "skye_leader_bridge.yaml"]
    )
    config_file = LaunchConfiguration("config_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Path to skye_leader_bridge YAML config.",
            ),
            Node(
                package="skye_leader_bridge",
                executable="leader_to_skye_bridge",
                name="skye_leader_bridge",
                output="screen",
                parameters=[{"config_file": config_file}],
            ),
        ]
    )
