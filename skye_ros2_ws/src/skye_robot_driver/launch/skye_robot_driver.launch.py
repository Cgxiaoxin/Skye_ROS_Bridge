import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory("skye_robot_driver"),
        "config",
        "skye_robot.yaml",
    )
    params_file = LaunchConfiguration("params_file")
    connect_on_startup = LaunchConfiguration("connect_on_startup")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="Full path to the Skye driver parameter file",
            ),
            DeclareLaunchArgument(
                "connect_on_startup",
                default_value="true",
                description="Link SDK and enter PD on startup",
            ),
            Node(
                package="skye_robot_driver",
                executable="skye_robot_driver",
                name="skye_robot_driver",
                parameters=[
                    params_file,
                    {
                        "connect_on_startup": ParameterValue(
                            connect_on_startup, value_type=bool
                        )
                    },
                ],
                remappings=[
                    ("/joint_states", "/gento/joint_states"),
                    ("/left_joint_control", "/gento/left_joint_control"),
                    ("/right_joint_control", "/gento/right_joint_control"),
                    ("/left_joint_control_abs", "/gento/left_joint_control_abs"),
                    ("/right_joint_control_abs", "/gento/right_joint_control_abs"),
                    ("/robot_state", "/gento/robot_state"),
                    ("/set_mode", "/gento/set_mode"),
                    ("/hold_current", "/gento/hold_current"),
                    ("/stop_motion", "/gento/stop_motion"),
                    ("/emergency_stop", "/gento/emergency_stop"),
                ],
                output="screen",
            ),
        ]
    )
