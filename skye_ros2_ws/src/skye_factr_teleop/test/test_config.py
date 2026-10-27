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


def _calibrated_document():
    document = _document()
    document["dynamixel"].update({
        "joint_offsets": [0.0] * 8,
        "current_limit_ma": [50.0] * 8,
    })
    document["controller"]["sync_hold_control"]["ruckig"].update({
        "max_velocity": [0.5] * 8,
        "max_acceleration": [1.0] * 8,
        "max_jerk": [4.0] * 8,
    })
    return document


def test_device_a_trial_config_and_explicit_calibration_load(tmp_path):
    assert load_config(LEFT_CONFIG).dynamixel.joint_offsets == (0.0,) * 8
    assert load_config(LEFT_CONFIG).dynamixel.current_limit_ma == (
        500.0, 500.0, 450.0, 250.0, 180.0, 180.0, 180.0, 50.0)
    for path in (LEFT_CONFIG, RIGHT_CONFIG):
        trial = load_config(path, require_runtime=True)
        assert trial.controller.sync_hold_control.ruckig.max_velocity == (0.8,) * 8
        assert trial.controller.sync_hold_control.ruckig.max_acceleration == (1.0,) * 8
        assert trial.controller.sync_hold_control.ruckig.max_jerk == (5.0,) * 8
    config = load_config(_write(tmp_path, _calibrated_document()), require_runtime=True)
    assert config.dynamixel.joint_offsets == (0.0,) * 8
    assert config.dynamixel.current_limit_ma == (50.0,) * 8
    assert config.runtime.state_timeout == 0.25
    assert config.controller.sync_hold_control.ruckig.max_velocity == (0.5,) * 8


@pytest.mark.parametrize("field", [
    "joint_offsets", "current_limit_ma",
])
def test_missing_dynamixel_calibration_fails_startup_validation(tmp_path, field):
    document = _calibrated_document()
    document["dynamixel"].pop(field)
    with pytest.raises(ConfigError, match=f"dynamixel.{field} is required"):
        load_config(_write(tmp_path, document), require_runtime=True)


@pytest.mark.parametrize("field,value", [
    ("joint_offsets", [0.0] * 7),
    ("joint_offsets", [None] * 8),
    ("current_limit_ma", 50.0),
    ("current_limit_ma", [50.0] * 7),
    ("current_limit_ma", 0.0),
    ("current_limit_ma", None),
    ("current_limit_ma", [0.0] + [50.0] * 7),
    ("current_limit_ma", [3210.0] + [50.0] * 7),
    ("current_limit_ma", [50.0, 50.0, 911.0] + [50.0] * 5),
])
def test_invalid_dynamixel_calibration_fails_startup_validation(tmp_path, field, value):
    document = _calibrated_document()
    document["dynamixel"][field] = value
    with pytest.raises(ConfigError, match=f"dynamixel.{field}"):
        load_config(_write(tmp_path, document), require_runtime=True)


def test_current_limits_preserve_physical_servo_order(tmp_path):
    document = _calibrated_document()
    document["dynamixel"]["current_limit_ma"] = [100.0, 200.0] + [50.0] * 6
    config = load_config(_write(tmp_path, document), require_runtime=True)
    assert config.dynamixel.current_limit_ma == (100.0, 200.0) + (50.0,) * 6


def test_runtime_state_policy_remains_optional(tmp_path):
    document = _calibrated_document()
    document["runtime"] = {"state_timeout": 0.4, "future_tolerance": 0.1,
                           "allow_unnamed_joint_states": True}
    runtime = load_config(_write(tmp_path, document), require_runtime=True).runtime
    assert runtime.state_timeout == 0.4
    assert runtime.future_tolerance == 0.1
    assert runtime.allow_unnamed_joint_states


@pytest.mark.parametrize("field", ["max_velocity", "max_acceleration", "max_jerk"])
def test_missing_ruckig_limit_fails_teleop_but_template_loads(tmp_path, field):
    document = _calibrated_document()
    document["controller"]["sync_hold_control"]["ruckig"].pop(field)
    path = _write(tmp_path, document)
    assert getattr(load_config(path).controller.sync_hold_control.ruckig, field) is None
    with pytest.raises(ConfigError, match=f"ruckig.{field} is required"):
        load_config(path, require_runtime=True)


@pytest.mark.parametrize("field,value", [
    ("max_velocity", [0.5] * 7),
    ("max_acceleration", [1.0] * 7),
    ("max_jerk", [0.0] * 8),
])
def test_invalid_ruckig_limit_fails_validation(tmp_path, field, value):
    document = _calibrated_document()
    document["controller"]["sync_hold_control"]["ruckig"][field] = value
    with pytest.raises(ConfigError, match=f"ruckig.{field}"):
        load_config(_write(tmp_path, document), require_runtime=True)


def test_left_and_right_configs_load_and_preserve_differences():
    left = load_config(LEFT_CONFIG)
    right = load_config(RIGHT_CONFIG)

    assert len(left.dynamixel.servo_types) == 8
    assert left.controller.force_feedback_control.joint.dynamics.follower_joint_offset == 0
    assert right.controller.force_feedback_control.joint.dynamics.follower_joint_offset == 7
    assert left.controller.force_feedback_control.joint.follower_gravity_comp_enable
    assert left.controller.force_feedback_control.gripper.gain == 20.0
    assert left.dynamixel.dynamixel_port != right.dynamixel.dynamixel_port
    assert left.dynamixel.joint_signs != right.dynamixel.joint_signs
    assert left.dynamixel.baudrate == 4_000_000
    assert left.controller.joint_gravity_comp.dynamics.leader_urdf_package == "skye_factr_description"
    assert not Path(left.controller.joint_gravity_comp.dynamics.leader_urdf).is_absolute()
    assert left.controller.joint_gravity_comp.mode == "position_only"
    assert right.controller.joint_gravity_comp.mode == "position_only"
    assert left.controller.limit_barrier_control.gripper.kp == 0.301658022
    assert right.controller.limit_barrier_control.gripper.kp == 1.03561658022
    assert left.controller.sync_hold_control.gripper_setpoint == 0.4


@pytest.mark.parametrize("mode", ["position_velocity", "position_only"])
def test_gravity_modes_load_from_yaml(tmp_path, mode):
    document = _document()
    document["controller"]["joint_gravity_comp"]["mode"] = mode
    assert load_config(_write(tmp_path, document)).controller.joint_gravity_comp.mode == mode


def test_missing_gravity_mode_preserves_factr_default():
    document = _document()
    document["controller"]["joint_gravity_comp"].pop("mode")
    assert TeleopConfig.from_mapping(document).controller.joint_gravity_comp.mode == "position_velocity"


@pytest.mark.parametrize("mode", ["unknown", "", None, True, []])
def test_invalid_gravity_mode_fails(mode):
    document = _document()
    document["controller"]["joint_gravity_comp"]["mode"] = mode
    with pytest.raises(ConfigError, match="joint_gravity_comp.mode"):
        TeleopConfig.from_mapping(document)


def test_result_is_deeply_immutable():
    config = load_config(LEFT_CONFIG)
    with pytest.raises(FrozenInstanceError):
        config.name = "changed"
    with pytest.raises(TypeError):
        config.controller.joint_gravity_comp.gain[0] = 0.0


@pytest.mark.parametrize(
    "mutation, match",
    [
        (lambda d: d.pop("controller"), "controller is required"),
        (lambda d: d["dynamixel"].pop("servo_types"), "servo_types is required"),
        (lambda d: d["dynamixel"].pop("baudrate"), "dynamixel.baudrate is required"),
        (lambda d: d["controller"]["joint_gravity_comp"].pop("dynamics"), "controller.joint_gravity_comp.dynamics is required"),
        (lambda d: d["controller"]["force_feedback_control"]["joint"].pop("dynamics"), "controller.force_feedback_control.joint.dynamics is required"),
        (lambda d: d["controller"].pop("force_feedback_control"), "controller.force_feedback_control is required"),
        (lambda d: d["controller"]["force_feedback_control"].pop("joint"), "controller.force_feedback_control.joint is required"),
        (lambda d: d["controller"]["force_feedback_control"].pop("gripper"), "controller.force_feedback_control.gripper is required"),
        (lambda d: d["controller"].pop("limit_barrier_control"), "controller.limit_barrier_control is required"),
        (lambda d: d["controller"]["limit_barrier_control"].pop("joint"), "controller.limit_barrier_control.joint is required"),
        (lambda d: d["controller"]["limit_barrier_control"].pop("gripper"), "controller.limit_barrier_control.gripper is required"),
        (lambda d: d["controller"]["force_feedback_control"]["joint"].pop("follower_gravity_comp_enable"), "follower_gravity_comp_enable is required"),
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
        (("controller", "limit_barrier_control", "joint", "limit_max"), [1.0] * 6, "7 values"),
        (("controller", "joint_gravity_comp", "gain"), [1.0] * 6, "7 values"),
        (("controller", "sync_hold_control", "hold", "kp"), [1.0] * 7, "8 values"),
        (("controller", "sync_hold_control", "hold", "gravity_comp_gain"), [1.0] * 8, "7 values"),
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
    document["controller"]["joint_gravity_comp"]["gain"][3] = bad_value
    with pytest.raises(ConfigError, match="finite"):
        load_config(_write(tmp_path, document))


def test_reversed_joint_limits_fail():
    document = _document()
    document["controller"]["limit_barrier_control"]["joint"]["limit_min"][2] = 4.0
    with pytest.raises(ConfigError, match="lower limit"):
        TeleopConfig.from_mapping(document)


def test_safety_margin_must_leave_a_valid_range():
    document = _document()
    document["controller"]["limit_barrier_control"]["joint"]["safety_margin"] = 2.0
    with pytest.raises(ConfigError, match="safety margin"):
        TeleopConfig.from_mapping(document)


def test_scalar_and_per_joint_gains_normalize_to_tuples():
    document = _document()
    document["controller"]["joint_gravity_comp"]["gain"] = 0.25
    document["controller"]["force_feedback_control"]["joint"]["gain"] = list(range(7))
    config = TeleopConfig.from_mapping(document)

    assert config.controller.joint_gravity_comp.gain == (0.25,) * 7
    assert config.controller.force_feedback_control.joint.gain == tuple(float(i) for i in range(7))
    assert config.controller.null_space_regulation.kp == (0.1,) * 7
    assert config.controller.static_friction_comp.enable_speed == (0.05,) * 7


@pytest.mark.parametrize("gravity_enabled, friction_enabled", [
    (False, False), (False, True), (True, False), (True, True),
])
def test_friction_enable_is_independent_of_gravity(gravity_enabled, friction_enabled):
    document = _document()
    document["controller"]["joint_gravity_comp"]["enable"] = gravity_enabled
    document["controller"]["static_friction_comp"]["enable"] = friction_enabled
    config = TeleopConfig.from_mapping(document)
    assert config.controller.joint_gravity_comp.enable is gravity_enabled
    assert config.controller.static_friction_comp.enable is friction_enabled


@pytest.mark.parametrize("gravity_enabled", [False, True])
def test_legacy_friction_enable_follows_gravity(gravity_enabled):
    document = _document()
    document["controller"]["joint_gravity_comp"]["enable"] = gravity_enabled
    document["controller"]["static_friction_comp"].pop("enable")
    config = TeleopConfig.from_mapping(document)
    assert config.controller.static_friction_comp.enable is gravity_enabled


def test_friction_enable_rejects_non_boolean():
    document = _document()
    document["controller"]["static_friction_comp"]["enable"] = 1
    with pytest.raises(ConfigError, match="static_friction_comp.enable must be a boolean"):
        TeleopConfig.from_mapping(document)


def test_arm_and_gripper_hold_dimensions_are_distinct():
    config = load_config(RIGHT_CONFIG)
    ruckig = config.controller.sync_hold_control.ruckig
    hold = config.controller.sync_hold_control.hold
    assert len(ruckig.kp) == 8
    assert len(hold.kp) == 8
    assert len(hold.i_max) == 8
    assert len(hold.sync_position_tolerance) == 8
    assert len(hold.sync_velocity_tolerance) == 8
    assert len(ruckig.gravity_comp_gain) == 7
    assert len(hold.gravity_comp_gain) == 7


@pytest.mark.parametrize(
    "mutation, match",
    [
        (lambda d: d["dynamixel"]["servo_types"].__setitem__(0, "unknown"), "unsupported"),
        (lambda d: d["dynamixel"]["joint_signs"].__setitem__(0, 0), "-1 or 1"),
        (lambda d: d["controller"].__setitem__("frequency", 0), "frequency must be positive"),
        (lambda d: d["controller"]["sync_hold_control"]["ruckig"].__setitem__("trajectory_timeout", -1), "timeout must be positive"),
        (lambda d: d["controller"]["sync_hold_control"]["hold"]["i_max"].__setitem__(0, 0), "must be positive"),
        (lambda d: d["controller"]["sync_hold_control"]["hold"]["sync_position_tolerance"].__setitem__(0, 0), "must be positive"),
        (lambda d: d["controller"]["sync_hold_control"]["hold"].__setitem__("sync_velocity_tolerance", [0.1] * 7), "8 values"),
        (lambda d: d["controller"]["sync_hold_control"]["hold"].__setitem__("settle_duration", 0), "must be positive"),
        (lambda d: d["controller"]["sync_hold_control"]["hold"].__setitem__("settle_timeout", 0), "must be positive"),
        (lambda d: d["dynamixel"].__setitem__("baudrate", 0), "dynamixel.baudrate must be positive"),
        (lambda d: d["controller"]["limit_barrier_control"]["gripper"].__setitem__("limit_max", -1), "limit_min must be below limit_max"),
        (lambda d: d["controller"]["limit_barrier_control"]["gripper"].__setitem__("kp", -1), "gripper.kp must be non-negative"),
        (lambda d: d["controller"]["joint_gravity_comp"]["dynamics"].__setitem__("leader_urdf", "/legacy_ws/model.urdf"), "package-relative"),
        (lambda d: d["controller"]["force_feedback_control"]["joint"]["dynamics"].__setitem__("follower_joint_offset", 3), "follower_joint_offset must be 0 or 7"),
    ],
)
def test_invalid_types_signs_control_and_resources_fail(mutation, match):
    document = deepcopy(_document())
    mutation(document)
    with pytest.raises(ConfigError, match=match):
        TeleopConfig.from_mapping(document)
