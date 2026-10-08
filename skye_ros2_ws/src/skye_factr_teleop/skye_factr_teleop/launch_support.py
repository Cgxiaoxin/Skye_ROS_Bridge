# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

"""Shared launch construction for the left, right, and dual M6 leaders."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from .config import ConfigError, load_config


def _existing_file(value, share, argument):
    if not value:
        raise RuntimeError(f"{argument} requires a file path")
    path = Path(value)
    if not path.is_absolute():
        path = share / path
    if not path.is_file():
        raise RuntimeError(f"{argument} file does not exist: {path}")
    return str(path)


def _launch_setup(context, *, sides):
    share = Path(get_package_share_directory("skye_factr_teleop"))
    profile = LaunchConfiguration("robot_profile").perform(context)
    config_dir = share / "config"
    if profile == "orin":
        config_dir /= "orin"

    # Resolve every requested arm before returning any executable action.
    files = []
    for side in sides:
        config = LaunchConfiguration(f"{side}_config").perform(context)
        config = config or str(config_dir / f"grav_comp_m6_{side}.yaml")
        config = _existing_file(config, share, f"{side}_config")
        try:
            load_config(config, require_runtime=True)
        except ConfigError as exc:
            raise RuntimeError(f"{side}_config invalid: {exc}") from exc
        files.append((side, config))

    return [
        Node(
            package="skye_factr_teleop",
            executable="skye_factr_teleop_driver",
            name=f"skye_factr_teleop_{side}",
            namespace="",
            arguments=["--side", side],
            parameters=[{"config_path": config}],
            remappings=[
                ("/joint_state", f"/gento/{side}_joint_states"),
                ("/joint_control", f"/gento/{side}_joint_control"),
                ("/joint_move", f"/{side}_joint_move"),
                ("/leader_arm/current_state", f"/{side}_leader_arm/current_state"),
                ("/leader_arm/target_joint_state", f"/{side}_leader_arm/target_joint_state"),
            ],
            output="screen",
        )
        for side, config in files
    ]


def leader_launch_description(sides):
    """Declare profile and one validated configuration YAML per arm."""
    actions = [DeclareLaunchArgument(
        "robot_profile", default_value="thor", choices=["thor", "orin"],
        description="Select the packaged leader hardware/control configuration",
    )]
    for side in sides:
        actions.append(DeclareLaunchArgument(
            f"{side}_config", default_value="",
            description=(f"{side} unified control and runtime YAML with measured calibration; "
                         "empty selects the robot_profile template (which lacks calibration); "
                         "absolute or relative to the package share"),
        ))
    actions.append(OpaqueFunction(function=_launch_setup, kwargs={"sides": sides}))
    return LaunchDescription(actions)
