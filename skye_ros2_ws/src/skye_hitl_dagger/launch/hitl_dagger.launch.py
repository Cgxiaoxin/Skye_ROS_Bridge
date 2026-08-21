"""HITL DAgger stack: control arbiter + keyboard intervention bridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    gripper_invert_on_driver = LaunchConfiguration("gripper_invert_on_driver")
    enable_recorder = LaunchConfiguration("enable_recorder")
    recorder_output_dir = LaunchConfiguration("recorder_output_dir")
    recorder_storage_id = LaunchConfiguration("recorder_storage_id")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gripper_invert_on_driver",
                default_value="true",
                description=(
                    "Invert FACTR trigger gripper semantics for driver motor space"
                ),
            ),
            DeclareLaunchArgument(
                "enable_recorder",
                default_value="false",
                description="Start the optional side-channel MCAP recorder",
            ),
            DeclareLaunchArgument(
                "recorder_output_dir",
                default_value="/tmp/hitl_bags",
                description="Directory for recorded MCAP episodes",
            ),
            DeclareLaunchArgument(
                "recorder_storage_id",
                default_value="mcap",
                description="rosbag2 storage plugin for recorded episodes",
            ),
            Node(
                package="skye_hitl_dagger",
                executable="control_arbiter",
                name="control_arbiter",
                output="screen",
                parameters=[
                    {
                        "gripper_invert_on_driver": ParameterValue(
                            gripper_invert_on_driver, value_type=bool
                        ),
                    }
                ],
            ),
            Node(
                package="skye_hitl_dagger",
                executable="hitl_keyboard",
                name="hitl_keyboard",
                output="screen",
            ),
            Node(
                package="skye_hitl_dagger",
                executable="episode_recorder",
                name="episode_recorder",
                output="screen",
                condition=IfCondition(enable_recorder),
                parameters=[{
                    "output_dir": recorder_output_dir,
                    "storage_id": recorder_storage_id,
                }],
            ),
        ]
    )
