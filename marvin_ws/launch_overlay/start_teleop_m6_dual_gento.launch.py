"""FACTR dual-arm teleop remapped to Skye /gento/* (bridge-less).

Use with skye_robot_driver on the host (same ROS_DOMAIN_ID).
Do NOT start marvin gento_robot_driver at the same time.
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

    # FACTR sync bug: offset ignored on 14-DOF topic → each arm uses 7-DOF side topic.
    # yaml follower_joint_offset (0 / 7) unchanged for 14-DOF URDF gravity model.
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
            ("/joint_control", "/gento/left_joint_control"),
            ("/joint_state", "/gento/left_joint_states"),
            ("/joint_move", "/left_joint_move"),
            ("/leader_arm/current_state", "/left_leader_arm/current_state"),
            ("/leader_arm/target_joint_state", "/left_leader_arm/target_joint_state"),
            ("/gripper/ctrl", "/left_teleop_gripper/ctrl"),
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
            ("/joint_control", "/gento/right_joint_control"),
            ("/joint_state", "/gento/right_joint_states"),
            ("/joint_move", "/right_joint_move"),
            ("/leader_arm/current_state", "/right_leader_arm/current_state"),
            ("/leader_arm/target_joint_state", "/right_leader_arm/target_joint_state"),
            ("/gripper/ctrl", "/right_teleop_gripper/ctrl"),
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
