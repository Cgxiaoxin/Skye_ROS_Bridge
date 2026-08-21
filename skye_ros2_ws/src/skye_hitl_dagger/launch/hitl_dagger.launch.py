"""HITL DAgger stack: control arbiter + keyboard intervention bridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    gripper_invert_on_driver = LaunchConfiguration("gripper_invert_on_driver")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gripper_invert_on_driver",
                default_value="true",
                description=(
                    "Invert FACTR trigger gripper semantics for driver motor space"
                ),
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
            # episode_recorder: optional; add when mcap path is ready (P6.3)
        ]
    )
