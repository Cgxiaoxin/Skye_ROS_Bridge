"""HITL DAgger stack: control arbiter + keyboard intervention bridge.

Pass ``robot_profile:=thor|orin`` so abs mapping signs match
``skye_robot_driver`` on the same machine. Keep the same profile as the
leader HITL launch (``start_teleop_m6_dual_hitl.launch.py``).
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _default_robot_profile() -> str:
    env = os.environ.get("ROBOT_PROFILE", os.environ.get("MARVIN_PROFILE", "")).strip().lower()
    if env in ("thor", "orin"):
        return env
    return "thor"


def _dds_profile_actions():
    """Load the same Fast DDS profile as driver / FACTR teleop.

    ``fastrtps_teleop.xml`` (SHM + 4 MB UDP) must match across participants;
    otherwise data falls back to builtin UDP and overflows under ~250 Hz
    joint-state load (``sequence size exceeds remaining buffer``).
    Prefer ``skye_robot_driver`` then ``skye_factr_teleop`` share copies.
    """
    if os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE"):
        return []
    for pkg in ("skye_robot_driver", "skye_factr_teleop"):
        try:
            path = os.path.join(
                get_package_share_directory(pkg), "config", "fastrtps_teleop.xml"
            )
        except PackageNotFoundError:
            continue
        if os.path.isfile(path):
            return [SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", path)]
    return []


def _launch_setup(context, *args, **kwargs):
    robot_profile = LaunchConfiguration("robot_profile").perform(context).strip().lower()
    if robot_profile not in ("thor", "orin"):
        raise RuntimeError(
            f"robot_profile must be 'thor' or 'orin', got: {robot_profile!r}"
        )

    pkg_share = get_package_share_directory("skye_hitl_dagger")
    profile_params = os.path.join(
        pkg_share, "config", "profiles", f"{robot_profile}.yaml"
    )
    gripper_invert_on_driver = LaunchConfiguration("gripper_invert_on_driver")
    enable_recorder = LaunchConfiguration("enable_recorder")
    recorder_output_dir = LaunchConfiguration("recorder_output_dir")
    recorder_storage_id = LaunchConfiguration("recorder_storage_id")

    return [
        Node(
            package="skye_hitl_dagger",
            executable="control_arbiter",
            name="control_arbiter",
            output="screen",
            parameters=[
                profile_params,
                {
                    "gripper_invert_on_driver": ParameterValue(
                        gripper_invert_on_driver, value_type=bool
                    ),
                },
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


def generate_launch_description():
    return LaunchDescription(
        _dds_profile_actions()
        + [
            DeclareLaunchArgument(
                "robot_profile",
                default_value=_default_robot_profile(),
                description=(
                    "Machine profile thor|orin — must match skye_robot_driver "
                    "and skye_factr_teleop HITL launch. Default: ROBOT_PROFILE "
                    "env, else thor."
                ),
            ),
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
            OpaqueFunction(function=_launch_setup),
        ]
    )
