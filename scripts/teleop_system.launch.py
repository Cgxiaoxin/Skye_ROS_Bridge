"""One ROS launch supervisor for the follower and explicitly selected leaders."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    profile, side = value("robot_profile"), value("side")
    sides = ("left", "right") if side == "dual" else (side,)
    configs = {f"{arm}_config": value(f"{arm}_config") for arm in sides}
    # Validate source YAMLs before returning any action that could connect hardware.
    from skye_factr_teleop.config import load_config
    for path in configs.values():
        load_config(path, require_runtime=True)
    driver = Path(get_package_share_directory("skye_robot_driver"))
    leader = Path(get_package_share_directory("skye_factr_teleop"))
    # Soft gate: do NOT register OnProcessExit→Shutdown. Killing one leader
    # (or a soft-mute path) must not tear down the sibling arm + robot driver.
    # Session stop is owned by the operator UI process-group terminate.
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(driver / "launch/skye_robot_driver.launch.py")),
            launch_arguments={"robot_profile": profile, "params_file": value("driver_config")}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(leader / f"launch/start_teleop_m6_{side}.launch.py")),
            launch_arguments={"robot_profile": profile, "sync_diagnostics": "true", **configs}.items()),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_profile", choices=["thor", "orin"]),
        DeclareLaunchArgument("side", default_value="dual", choices=["dual", "left", "right"]),
        DeclareLaunchArgument("driver_config"),
        DeclareLaunchArgument("left_config", default_value=""),
        DeclareLaunchArgument("right_config", default_value=""),
        OpaqueFunction(function=setup),
    ])
