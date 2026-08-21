"""FACTR dual-arm teleop remapped for HITL (teleop branch → /skye/teleop_*).

Use with control_arbiter on the host (same ROS_DOMAIN_ID).
Do NOT use for plain teleop — use start_teleop_m6_dual_gento.launch.py instead.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("factr_teleop")

    existing_pythonpath = os.environ.get("PYTHONPATH", "")
    pinocchio_path = "/opt/openrobots/lib/python3.10/site-packages"
    set_pythonpath = SetEnvironmentVariable(
        "PYTHONPATH", existing_pythonpath + ":" + pinocchio_path
    )

    use_keyboard_arg = DeclareLaunchArgument(
        "use_keyboard",
        default_value=EnvironmentVariable("USE_KEYBOARD", default_value="true"),
    )
    use_keyboard = LaunchConfiguration("use_keyboard")

    left_config = os.path.join(pkg_share, "configs", "grav_comp_m6_left.yaml")
    right_config = os.path.join(pkg_share, "configs", "grav_comp_m6_right.yaml")

    factr_left = Node(
        package="factr_teleop",
        executable="factr_teleop_robot_driver.py",
        name="factr_teleop_left",
        output="screen",
        parameters=[
            {
                "config_file": left_config,
                "debug_state_print": True,
                "print_joint_states": True,
                "print_period": 0.5,
                "cb_print_period": 1.0,
            }
        ],
        remappings=[
            ("/joint_control", "/skye/teleop_action_left"),
            ("/joint_state", "/gento/joint_states"),
            ("/joint_move", "/left_joint_move"),
            ("/gripper/ctrl", "/skye/teleop_gripper_left"),
            ("/gripper/state", "/left_gripper/state"),
        ],
    )

    factr_right = Node(
        package="factr_teleop",
        executable="factr_teleop_robot_driver.py",
        name="factr_teleop_right",
        output="screen",
        parameters=[
            {
                "config_file": right_config,
                "debug_state_print": True,
                "print_joint_states": True,
                "print_period": 0.5,
                "cb_print_period": 1.0,
            }
        ],
        remappings=[
            ("/joint_control", "/skye/teleop_action_right"),
            ("/joint_state", "/gento/joint_states"),
            ("/joint_move", "/right_joint_move"),
            ("/gripper/ctrl", "/skye/teleop_gripper_right"),
            ("/gripper/state", "/right_gripper/state"),
        ],
    )

    keyboard_node = Node(
        package="factr_teleop",
        executable="keyboard_gripper.py",
        name="keyboard_gripper",
        output="screen",
        parameters=[{"debug_state_print": True}],
        condition=IfCondition(use_keyboard),
    )

    return LaunchDescription(
        [
            set_pythonpath,
            use_keyboard_arg,
            factr_left,
            factr_right,
            keyboard_node,
        ]
    )
