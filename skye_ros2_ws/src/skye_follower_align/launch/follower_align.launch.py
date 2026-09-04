from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_DEFAULT_SIGNS = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
_ORIN_RIGHT_SIGNS = [1.0, 1.0, 1.0, 1.0, 1.0, -1.0, -1.0]


def _launch_setup(context, *args, **kwargs):
    robot_profile = LaunchConfiguration("robot_profile").perform(
        context).strip().lower()
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
    return LaunchDescription([
        DeclareLaunchArgument("enable_keyboard", default_value="true"),
        DeclareLaunchArgument("robot_profile", default_value="thor"),
        OpaqueFunction(function=_launch_setup),
    ])
