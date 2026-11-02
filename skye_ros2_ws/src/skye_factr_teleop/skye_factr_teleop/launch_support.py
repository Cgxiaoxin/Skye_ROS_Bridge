# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

"""Shared launch construction for the left, right, and dual M6 leaders."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from .config import ConfigError, load_config
from .control_core import GravityTestSettings, LimitTestSettings, FrictionTestSettings


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
            parameters=[{
                "config_path": config,
                "sync_diagnostics": ParameterValue(
                    LaunchConfiguration("sync_diagnostics"), value_type=bool),
            }],
            remappings=[
                ("/joint_state", f"/gento/{side}_joint_states"),
                ("/joint_control", f"/gento/{side}_joint_control"),
                ("/joint_move", f"/{side}_joint_move"),
                ("/leader_arm/current_state", f"/{side}_leader_arm/current_state"),
                ("/leader_arm/target_joint_state", f"/{side}_leader_arm/target_joint_state"),
                ("/gripper/ctrl", f"/{side}_teleop_gripper/ctrl"),
            ],
            output="screen",
        )
        for side, config in files
    ]


def _dds_profile_actions():
    """Select the packaged Fast DDS profile unless the caller already has one."""
    if os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE"):
        return []
    path = (Path(get_package_share_directory("skye_factr_teleop"))
            / "config" / "fastrtps_teleop.xml")
    if not path.is_file():
        return []
    return [SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", str(path))]


def leader_launch_description(sides):
    """Declare profile and one validated configuration YAML per arm."""
    actions = [DeclareLaunchArgument(
        "robot_profile", default_value="thor", choices=["thor", "orin"],
        description="Select the packaged leader hardware/control configuration",
    )]
    actions.append(DeclareLaunchArgument(
        "sync_diagnostics", default_value="false", choices=["true", "false"],
        description="Publish eight-axis cycle diagnostics and read present current",
    ))
    for side in sides:
        actions.append(DeclareLaunchArgument(
            f"{side}_config", default_value="",
            description=(f"{side} unified control YAML with measured dynamixel calibration; "
                         "empty selects the robot_profile configuration; "
                         "absolute or relative to the package share"),
        ))
    actions.extend(_dds_profile_actions())
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


def _limit_test_setup(context, *, side):
    share = Path(get_package_share_directory("skye_factr_teleop"))
    config_path = _existing_file(
        LaunchConfiguration("config_path").perform(context), share, "config_path",
    )
    try:
        config = load_config(config_path)
        if config.dynamixel.joint_offsets is None or config.dynamixel.current_limit_ma is None:
            raise ValueError("calibrated joint_offsets and current_limit_ma are required")
        if side not in Path(config.controller.joint_gravity_comp.dynamics.leader_urdf).stem:
            raise ValueError("selected side does not match leader URDF")
        if not config.controller.limit_barrier_control.enable:
            raise ValueError("limit barrier must be enabled")
        settings = LimitTestSettings(
            joint_id=int(LaunchConfiguration("joint_id").perform(context)),
            gain_scale=float(LaunchConfiguration("gain_scale").perform(context)),
            current_limit_ma=float(LaunchConfiguration("current_limit_ma").perform(context)),
            ramp_seconds=float(LaunchConfiguration("ramp_seconds").perform(context)),
            max_duration_seconds=float(LaunchConfiguration("max_duration_seconds").perform(context)),
        )
        if settings.current_limit_ma > min(config.dynamixel.current_limit_ma):
            raise ValueError("limit test current cap exceeds configured limit")
        barrier = config.controller.limit_barrier_control
        kp = barrier.gripper.kp if settings.joint_id == 8 else barrier.joint.kp
        if kp <= 0:
            raise ValueError("limit test requires positive position gain")
    except (ConfigError, ValueError) as exc:
        raise RuntimeError(f"limit test configuration invalid: {exc}") from exc
    return [Node(
        package="skye_factr_teleop",
        executable="skye_factr_teleop_driver",
        name=f"skye_factr_teleop_{side}",
        namespace="",
        arguments=["--side", side],
        parameters=[{
            "config_path": config_path,
            "limit_test_only": True,
            "limit_test_joint_id": settings.joint_id,
            "limit_test_gain_scale": settings.gain_scale,
            "limit_test_current_limit_ma": settings.current_limit_ma,
            "limit_test_ramp_seconds": settings.ramp_seconds,
            "limit_test_max_duration_seconds": settings.max_duration_seconds,
        }],
        output="screen",
    )]


def limit_test_launch_description(side):
    return LaunchDescription([
        DeclareLaunchArgument("config_path", default_value=""),
        DeclareLaunchArgument("joint_id", default_value=""),
        DeclareLaunchArgument("gain_scale", default_value=""),
        DeclareLaunchArgument("current_limit_ma", default_value=""),
        DeclareLaunchArgument("ramp_seconds", default_value=""),
        DeclareLaunchArgument("max_duration_seconds", default_value=""),
        OpaqueFunction(function=_limit_test_setup, kwargs={"side": side}),
    ])


def _friction_test_setup(context, *, side):
    share = Path(get_package_share_directory("skye_factr_teleop"))
    config_path = _existing_file(
        LaunchConfiguration("config_path").perform(context), share, "config_path",
    )
    try:
        config = load_config(config_path)
        if config.dynamixel.joint_offsets is None or config.dynamixel.current_limit_ma is None:
            raise ValueError("calibrated joint_offsets and current_limit_ma are required")
        if side not in Path(config.controller.joint_gravity_comp.dynamics.leader_urdf).stem:
            raise ValueError("selected side does not match leader URDF")
        enabled_text = LaunchConfiguration("enabled").perform(context).lower()
        if enabled_text not in ("true", "false"):
            raise ValueError("enabled must be true or false")
        settings = FrictionTestSettings(
            joint_id=int(LaunchConfiguration("joint_id").perform(context)),
            gain_scale=float(LaunchConfiguration("gain_scale").perform(context)),
            current_limit_ma=float(LaunchConfiguration("current_limit_ma").perform(context)),
            ramp_seconds=float(LaunchConfiguration("ramp_seconds").perform(context)),
            max_duration_seconds=float(LaunchConfiguration("max_duration_seconds").perform(context)),
            enabled=enabled_text == "true",
        )
        if settings.current_limit_ma > min(config.dynamixel.current_limit_ma):
            raise ValueError("friction test current cap exceeds configured limit")
        index = settings.joint_id - 1
        if (config.controller.joint_gravity_comp.gain[index] <= 0
                or config.controller.static_friction_comp.gain[index] <= 0
                or config.controller.static_friction_comp.enable_speed[index] <= 0):
            raise ValueError("selected joint gravity/friction gain and speed threshold must be positive")
        if not config.controller.static_friction_comp.enable:
            raise ValueError("static friction must be enabled in the configuration")
    except (ConfigError, ValueError) as exc:
        raise RuntimeError(f"friction test configuration invalid: {exc}") from exc
    return [Node(
        package="skye_factr_teleop",
        executable="skye_factr_teleop_driver",
        name=f"skye_factr_teleop_{side}",
        namespace="",
        arguments=["--side", side],
        parameters=[{
            "config_path": config_path,
            "friction_test_only": True,
            "friction_test_joint_id": settings.joint_id,
            "friction_test_gain_scale": settings.gain_scale,
            "friction_test_current_limit_ma": settings.current_limit_ma,
            "friction_test_ramp_seconds": settings.ramp_seconds,
            "friction_test_max_duration_seconds": settings.max_duration_seconds,
            "friction_test_enabled": settings.enabled,
        }],
        output="screen",
    )]


def friction_test_launch_description(side):
    return LaunchDescription([
        DeclareLaunchArgument("config_path", default_value=""),
        DeclareLaunchArgument("joint_id", default_value=""),
        DeclareLaunchArgument("gain_scale", default_value=""),
        DeclareLaunchArgument("current_limit_ma", default_value=""),
        DeclareLaunchArgument("ramp_seconds", default_value=""),
        DeclareLaunchArgument("max_duration_seconds", default_value=""),
        DeclareLaunchArgument("enabled", default_value="false", choices=["true", "false"]),
        OpaqueFunction(function=_friction_test_setup, kwargs={"side": side}),
    ])
