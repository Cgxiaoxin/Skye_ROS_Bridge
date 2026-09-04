import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


_REMAPS = [
    ("/joint_states", "/gento/joint_states"),
    ("/left_joint_states", "/gento/left_joint_states"),
    ("/right_joint_states", "/gento/right_joint_states"),
    ("/left_joint_control", "/gento/left_joint_control"),
    ("/right_joint_control", "/gento/right_joint_control"),
    ("/left_joint_control_abs", "/gento/left_joint_control_abs"),
    ("/right_joint_control_abs", "/gento/right_joint_control_abs"),
    ("/robot_state", "/gento/robot_state"),
    ("/set_mode", "/gento/set_mode"),
    ("/hold_current", "/gento/hold_current"),
    ("/stop_motion", "/gento/stop_motion"),
    ("/emergency_stop", "/gento/emergency_stop"),
    ("/left_joint_action_applied", "/gento/left_joint_action_applied"),
    ("/right_joint_action_applied", "/gento/right_joint_action_applied"),
]


def _launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("skye_robot_driver")
    params_file = LaunchConfiguration("params_file").perform(context)
    connect_on_startup = LaunchConfiguration("connect_on_startup").perform(context)
    robot_profile = LaunchConfiguration("robot_profile").perform(context).strip().lower()
    robotiq_dual = LaunchConfiguration("robotiq_dual_gripper").perform(context)
    robotiq_right = LaunchConfiguration("robotiq_right_gripper").perform(context)

    if robot_profile not in ("thor", "orin"):
        raise RuntimeError(
            f"robot_profile must be 'thor' or 'orin', got: {robot_profile!r}"
        )

    # Legacy flags map to orin/partial overlays if profile still thor.
    node_params = [
        params_file,
        os.path.join(pkg_share, "config", "profiles", f"{robot_profile}.yaml"),
    ]
    if robot_profile == "thor" and robotiq_dual.lower() in ("1", "true", "yes"):
        node_params.append(
            os.path.join(pkg_share, "config", "skye_robot_robotiq_dual.yaml")
        )
    elif robot_profile == "thor" and robotiq_right.lower() in ("1", "true", "yes"):
        node_params.append(
            os.path.join(pkg_share, "config", "skye_robot_robotiq_right.yaml")
        )

    node_params.append(
        {
            "connect_on_startup": ParameterValue(
                connect_on_startup.lower() in ("1", "true", "yes"), value_type=bool
            )
        }
    )

    return [
        Node(
            package="skye_robot_driver",
            executable="skye_robot_driver",
            name="skye_robot_driver",
            parameters=node_params,
            remappings=_REMAPS,
            output="screen",
        )
    ]


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory("skye_robot_driver"),
        "config",
        "skye_robot.yaml",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="Base Skye driver parameter file",
            ),
            DeclareLaunchArgument(
                "connect_on_startup",
                default_value="true",
                description="Link SDK and enter control mode on startup",
            ),
            DeclareLaunchArgument(
                "robot_profile",
                default_value="thor",
                description="Machine profile: thor (DM4310) | orin (Robotiq)",
            ),
            DeclareLaunchArgument(
                "robotiq_dual_gripper",
                default_value="false",
                description="Legacy: if true with robot_profile:=thor, load robotiq_dual overlay (not orin signs)",
            ),
            DeclareLaunchArgument(
                "robotiq_right_gripper",
                default_value="false",
                description="Legacy: right-only Robotiq overlay (prefer robot_profile:=orin)",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
