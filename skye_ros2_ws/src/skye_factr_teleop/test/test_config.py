# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

from copy import deepcopy
from dataclasses import FrozenInstanceError
from pathlib import Path

import pytest
import yaml

from skye_factr_teleop.config import ConfigError, TeleopConfig, load_config


CONFIG_DIR = Path(__file__).parents[1] / "config"
LEFT_CONFIG = CONFIG_DIR / "grav_comp_m6_left.yaml"
RIGHT_CONFIG = CONFIG_DIR / "grav_comp_m6_right.yaml"


def _document(path=LEFT_CONFIG):
    with path.open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def _write(tmp_path, document):
    path = tmp_path / "config.yaml"
    path.write_text(yaml.safe_dump(document), encoding="utf-8")
    return path


def test_left_and_right_configs_load_and_preserve_differences():
    left = load_config(LEFT_CONFIG)
    right = load_config(RIGHT_CONFIG)

    assert left.arm_teleop.num_arm_joints == 7
    assert len(left.dynamixel.servo_types) == 8
    assert left.arm_teleop.follower_joint_offset == 0
    assert right.arm_teleop.follower_joint_offset == 7
    assert left.dynamixel.dynamixel_port != right.dynamixel.dynamixel_port
    assert left.dynamixel.joint_signs != right.dynamixel.joint_signs
    assert left.arm_teleop.leader_urdf_package == "skye_factr_description"
    assert not Path(left.arm_teleop.leader_urdf).is_absolute()


def test_result_is_deeply_immutable():
    config = load_config(LEFT_CONFIG)
    with pytest.raises(FrozenInstanceError):
        config.name = "changed"
    with pytest.raises(TypeError):
        config.controller.gravity_comp.gain[0] = 0.0


@pytest.mark.parametrize(
    "mutation, match",
    [
        (lambda d: d.pop("controller"), "controller is required"),
        (lambda d: d["dynamixel"].pop("servo_types"), "servo_types is required"),
        (lambda d: d["arm_teleop"]["initialization"].pop("calibration_joint_pos"), "calibration_joint_pos is required"),
    ],
)
def test_missing_required_fields_fail(mutation, match):
    document = _document()
    mutation(document)
    with pytest.raises(ConfigError, match=match):
        TeleopConfig.from_mapping(document)


@pytest.mark.parametrize(
    "path, value, match",
    [
        (("dynamixel", "servo_types"), ["XM430-W350"] * 7, "8 values"),
        (("dynamixel", "joint_signs"), [1] * 7, "8 values"),
        (("arm_teleop", "arm_joint_limits_max"), [1.0] * 6, "7 values"),
        (("controller", "gravity_comp", "gain"), [1.0] * 6, "7 values"),
        (("controller", "hold_ruckig_control", "hold_kp"), [1.0] * 7, "8 values"),
        (("controller", "hold_ruckig_control", "hold_gravity_comp_gain"), [1.0] * 8, "7 values"),
    ],
)
def test_wrong_array_lengths_fail(path, value, match):
    document = _document()
    target = document
    for key in path[:-1]:
        target = target[key]
    target[path[-1]] = value
    with pytest.raises(ConfigError, match=match):
        TeleopConfig.from_mapping(document)


@pytest.mark.parametrize("bad_value", [float("nan"), float("inf"), -float("inf")])
def test_non_finite_values_fail(tmp_path, bad_value):
    document = _document()
    document["controller"]["gravity_comp"]["gain"][3] = bad_value
    with pytest.raises(ConfigError, match="finite"):
        load_config(_write(tmp_path, document))


def test_reversed_joint_limits_fail():
    document = _document()
    document["arm_teleop"]["arm_joint_limits_min"][2] = 4.0
    with pytest.raises(ConfigError, match="lower limit"):
        TeleopConfig.from_mapping(document)


def test_safety_margin_must_leave_a_valid_range():
    document = _document()
    document["arm_teleop"]["arm_joint_limits_safety_margin"] = 2.0
    with pytest.raises(ConfigError, match="safety margin"):
        TeleopConfig.from_mapping(document)


def test_scalar_and_per_joint_gains_normalize_to_tuples():
    document = _document()
    document["controller"]["gravity_comp"]["gain"] = 0.25
    document["controller"]["torque_feedback"]["gain"] = list(range(7))
    config = TeleopConfig.from_mapping(document)

    assert config.controller.gravity_comp.gain == (0.25,) * 7
    assert config.controller.torque_feedback.gain == tuple(float(i) for i in range(7))
    assert config.controller.null_space_regulation.kp == (0.1,) * 7
    assert config.controller.static_friction_comp.enable_speed == (0.05,) * 7


def test_arm_and_gripper_hold_dimensions_are_distinct():
    config = load_config(RIGHT_CONFIG)
    hold = config.controller.hold_ruckig_control
    assert len(hold.ruckig_kp) == 8
    assert len(hold.hold_kp) == 8
    assert len(hold.hold_i_max) == 8
    assert len(hold.ruckig_gravity_comp_gain) == 7
    assert len(hold.hold_gravity_comp_gain) == 7


@pytest.mark.parametrize(
    "mutation, match",
    [
        (lambda d: d["dynamixel"]["servo_types"].__setitem__(0, "unknown"), "unsupported"),
        (lambda d: d["dynamixel"]["joint_signs"].__setitem__(0, 0), "-1 or 1"),
        (lambda d: d["controller"].__setitem__("frequency", 0), "frequency must be positive"),
        (lambda d: d["controller"].__setitem__("teleop_sync_timeout", -1), "timeout must be positive"),
        (lambda d: d["controller"]["hold_ruckig_control"]["hold_i_max"].__setitem__(0, 0), "must be positive"),
        (lambda d: d["arm_teleop"].__setitem__("leader_urdf", "/legacy_ws/model.urdf"), "package-relative"),
    ],
)
def test_invalid_types_signs_control_and_resources_fail(mutation, match):
    document = deepcopy(_document())
    mutation(document)
    with pytest.raises(ConfigError, match=match):
        TeleopConfig.from_mapping(document)
