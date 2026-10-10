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
from .control_core import GravityTestSettings


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
            description=(f"{side} unified control YAML with measured dynamixel calibration; "
                         "empty selects the robot_profile template (which lacks calibration); "
                         "absolute or relative to the package share"),
        ))
    actions.append(OpaqueFunction(function=_launch_setup, kwargs={"sides": sides}))
    return LaunchDescription(actions)


def _gravity_test_setup(context, *, side):
    share = Path(get_package_share_directory("skye_factr_teleop"))
    config_path = _existing_file(
        LaunchConfiguration("config_path").perform(context), share, "config_path",
    )
    try:
        config = load_config(config_path)
        if config.dynamixel.joint_offsets is None or config.dynamixel.current_limit_ma is None:
            raise ValueError("calibrated joint_offsets and current_limit_ma are required")
        settings = GravityTestSettings(
            joint_id=int(LaunchConfiguration("joint_id").perform(context)),
            gain_scale=float(LaunchConfiguration("gain_scale").perform(context)),
            current_limit_ma=float(LaunchConfiguration("current_limit_ma").perform(context)),
            ramp_seconds=float(LaunchConfiguration("ramp_seconds").perform(context)),
            max_duration_seconds=float(LaunchConfiguration("max_duration_seconds").perform(context)),
        )
        if settings.current_limit_ma > min(config.dynamixel.current_limit_ma):
            raise ValueError("gravity test current cap exceeds configured limit")
        joint_gain = config.controller.joint_gravity_comp.gain[settings.joint_id - 1]
        if not config.controller.joint_gravity_comp.enable or joint_gain <= 0:
            raise ValueError("gravity test requires a positive enabled joint gain")
    except (ConfigError, ValueError) as exc:
        raise RuntimeError(f"gravity test configuration invalid: {exc}") from exc
    return [Node(
        package="skye_factr_teleop",
        executable="skye_factr_teleop_driver",
        name=f"skye_factr_teleop_{side}",
        namespace="",
        arguments=["--side", side],
        parameters=[{
            "config_path": config_path,
            "gravity_test_only": True,
            "gravity_test_joint_id": settings.joint_id,
            "gravity_test_gain_scale": settings.gain_scale,
            "gravity_test_current_limit_ma": settings.current_limit_ma,
            "gravity_test_ramp_seconds": settings.ramp_seconds,
            "gravity_test_max_duration_seconds": settings.max_duration_seconds,
        }],
        output="screen",
    )]


def gravity_test_launch_description(side):
    return LaunchDescription([
        DeclareLaunchArgument("config_path", default_value=""),
        DeclareLaunchArgument("joint_id", default_value=""),
        DeclareLaunchArgument("gain_scale", default_value=""),
        DeclareLaunchArgument("current_limit_ma", default_value=""),
        DeclareLaunchArgument("ramp_seconds", default_value=""),
        DeclareLaunchArgument("max_duration_seconds", default_value=""),
        OpaqueFunction(function=_gravity_test_setup, kwargs={"side": side}),
    ])
