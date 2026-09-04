from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("output_dir", default_value="/tmp/skye_data_bags"),
        DeclareLaunchArgument("applied_qos_depth", default_value="20"),
        Node(
            package="skye_data_recorder",
            executable="data_recorder",
            name="skye_data_recorder",
            output="screen",
            parameters=[{
                "output_dir": LaunchConfiguration("output_dir"),
                "applied_qos_depth": LaunchConfiguration("applied_qos_depth"),
                "storage_id": "mcap",
            }],
        ),
    ])
