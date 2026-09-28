# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

"""Expand real launch actions and inspect commands without starting processes."""

from pathlib import Path
import runpy

import pytest
import yaml

pytest.importorskip("launch_ros")
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.substitutions import ExecutableInPackage

from skye_factr_teleop import launch_support
from skye_factr_teleop.config import load_config


PACKAGE_ROOT = Path(__file__).parents[1]


@pytest.fixture
def expand_launch(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_logs"))
    monkeypatch.setattr(launch_support, "get_package_share_directory",
                        lambda package: str(PACKAGE_ROOT))
    monkeypatch.setattr(ExecutableInPackage, "perform",
                        lambda self, context: "/test-bin/skye_factr_teleop_driver")
    captured = []

    def capture_process(node, context):
        command = [perform_substitutions(context, part) for part in node.cmd]
        parameter_files = [Path(command[i + 1]) for i, arg in enumerate(command)
                           if arg == "--params-file"]
        parameters = [yaml.safe_load(path.read_text()) for path in parameter_files]
        # launch_ros creates the final config_path override as a temporary YAML.
        parameter_files[-1].unlink()
        captured.append((node, command, parameter_files, parameters))
        return []

    monkeypatch.setattr(ExecuteProcess, "execute", capture_process)
    runtime = tmp_path / "runtime.yaml"
    runtime.write_text(yaml.safe_dump({"/**": {"ros__parameters": {
        "joint_offsets": [0.0] * 8, "current_limit_ma": 50.0,
        "max_velocity": [0.5] * 8, "max_acceleration": [1.0] * 8,
        "max_jerk": [4.0] * 8,
        "config_path": "/ignored-by-launch.yaml",
    }}}))

    def expand(kind, **arguments):
        context = LaunchContext()
        context.launch_configurations.update(arguments)
        path = PACKAGE_ROOT / "launch" / f"start_teleop_m6_{kind}.launch.py"
        description = runpy.run_path(str(path))["generate_launch_description"]()
        for action in description.entities:
            # Reject unexpected actions such as keyboard processes or environment edits.
            assert isinstance(action, (DeclareLaunchArgument, OpaqueFunction))
            children = action.execute(context)
            for node in children or []:
                assert isinstance(node, Node)
                node.execute(context)
        return captured

    expand.runtime = str(runtime)
    expand.captured = captured
    return expand


@pytest.mark.parametrize("profile", ["thor", "orin"])
@pytest.mark.parametrize("kind,sides", [
    ("left", ("left",)), ("right", ("right",)), ("dual", ("left", "right")),
])
def test_launch_profiles_sides_and_remaps(expand_launch, profile, kind, sides):
    result = expand_launch(kind, robot_profile=profile, **{
        f"{side}_params_file": expand_launch.runtime for side in sides
    })
    assert len(result) == len(sides)
    assert len({node.node_name for node, *_ in result}) == len(sides)
    for side, (node, command, parameter_files, parameters) in zip(sides, result):
        assert node.node_package == "skye_factr_teleop"
        assert node.node_executable == "skye_factr_teleop_driver"
        assert node.node_name == f"/skye_factr_teleop_{side}"
        assert command[command.index("--side") + 1] == side
        assert node.expanded_remapping_rules == [
            ("/joint_state", f"/gento/{side}_joint_states"),
            ("/joint_control", f"/gento/{side}_joint_control"),
            ("/joint_move", f"/{side}_joint_move"),
            ("/leader_arm/current_state", f"/{side}_leader_arm/current_state"),
            ("/leader_arm/target_joint_state", f"/{side}_leader_arm/target_joint_state"),
        ]
        assert parameter_files[0] == Path(expand_launch.runtime)
        assert parameters[0]["/**"]["ros__parameters"]["joint_offsets"] == [0.0] * 8
        config_dir = PACKAGE_ROOT / "config"
        if profile == "orin":
            config_dir /= "orin"
        config = config_dir / f"grav_comp_m6_{side}.yaml"
        assert parameters[-1][node.node_name]["ros__parameters"] == {"config_path": str(config)}
        assert load_config(config).arm_teleop.follower_joint_offset == (0 if side == "left" else 7)


@pytest.mark.parametrize("absolute", [False, True])
def test_config_override(expand_launch, absolute):
    config = Path("config/orin/grav_comp_m6_left.yaml")
    if absolute:
        config = PACKAGE_ROOT / config
    result = expand_launch("left", left_params_file=expand_launch.runtime, left_config=str(config))
    node, _, _, parameters = result[0]
    assert parameters[-1][node.node_name]["ros__parameters"]["config_path"] == str(
        PACKAGE_ROOT / "config/orin/grav_comp_m6_left.yaml")


def test_default_profile_is_thor(expand_launch):
    node, _, _, parameters = expand_launch("left", left_params_file=expand_launch.runtime)[0]
    assert parameters[-1][node.node_name]["ros__parameters"]["config_path"] == str(
        PACKAGE_ROOT / "config/grav_comp_m6_left.yaml")


def test_relative_runtime_path_uses_package_share(expand_launch, monkeypatch):
    share = Path(expand_launch.runtime).parent
    monkeypatch.setattr(launch_support, "get_package_share_directory", lambda package: str(share))
    result = expand_launch(
        "left", left_params_file="runtime.yaml",
        left_config=str(PACKAGE_ROOT / "config/grav_comp_m6_left.yaml"),
    )
    assert result[0][2][0] == Path(expand_launch.runtime)


@pytest.mark.parametrize("arguments,match", [
    ({"robot_profile": "invalid"}, "is not valid"),
    ({}, "left_params_file"),
    ({"left_params_file": ""}, "left_params_file requires"),
    ({"left_params_file": "missing.yaml"}, "left_params_file file does not exist"),
])
def test_invalid_arguments_never_start_processes(expand_launch, arguments, match):
    with pytest.raises(RuntimeError, match=match):
        expand_launch("left", **arguments)
    assert not expand_launch.captured


def test_dual_checks_both_files_before_starting_either_arm(expand_launch):
    with pytest.raises(RuntimeError, match="right_config file does not exist"):
        expand_launch("dual", left_params_file=expand_launch.runtime,
                      right_params_file=expand_launch.runtime, right_config="missing.yaml")
    assert not expand_launch.captured


def test_orin_configuration_preserves_hardware_and_gains():
    left, right = [load_config(PACKAGE_ROOT / "config/orin" / f"grav_comp_m6_{side}.yaml")
                   for side in ("left", "right")]
    assert left.dynamixel.dynamixel_port == "usb-FTDI_USB__-__Serial_Converter_FTBEO9L1-if00-port0"
    assert right.dynamixel.dynamixel_port == "usb-FTDI_USB__-__Serial_Converter_FTBEO8WL-if00-port0"
    assert right.dynamixel.joint_signs == (1, -1, 1, -1, 1, -1, -1, -1)
    assert left.controller.gravity_comp.gain == (0.25, 0.2, 0.7, 0.6, 1.0, 0.8, 1.0)
    assert right.controller.gravity_comp.gain == (0.25, 0.2, 1.0, 1.0, 1.0, 0.8, 1.0)
    assert not left.arm_teleop.arm_limit_barrier_enable
    assert not right.arm_teleop.arm_limit_barrier_enable
