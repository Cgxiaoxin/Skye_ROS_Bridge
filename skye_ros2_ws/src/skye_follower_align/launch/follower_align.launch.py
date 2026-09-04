from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_DEFAULT_SIGNS = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
_ORIN_RIGHT_SIGNS = [1.0, 1.0, 1.0, 1.0, 1.0, -1.0, -1.0]


def _launch_setup(context, *args, **kwargs):
    robot_profile = LaunchConfiguration("robot_profile").perform(
        context).strip().lower()
    if robot_profile not in ("thor", "orin"):
        raise RuntimeError(
            f"robot_profile must be thor|orin (got: {robot_profile!r})")
    left_signs = list(_DEFAULT_SIGNS)
    right_signs = (
        list(_ORIN_RIGHT_SIGNS) if robot_profile == "orin"
        else list(_DEFAULT_SIGNS))

    return [
        Node(
            package="skye_follower_align",
            executable="follower_align_node",
            name="follower_align",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "left_joint_signs": left_signs,
                "right_joint_signs": right_signs,
            }],
        ),
    ]


def generate_launch_description():
    enable_keyboard = LaunchConfiguration("enable_keyboard")
    return LaunchDescription([
        DeclareLaunchArgument("enable_keyboard", default_value="true"),
        DeclareLaunchArgument("robot_profile", default_value="thor"),
        OpaqueFunction(function=_launch_setup),
        Node(
            package="skye_follower_align",
            executable="host_keyboard_align",
            name="host_keyboard_align",
            output="screen",
            condition=IfCondition(enable_keyboard),
        ),
    ])
