"""Arbiter profile YAML must stay aligned with skye_robot_driver profiles."""

from pathlib import Path

import yaml

HITL_ROOT = Path(__file__).resolve().parents[1]
DRIVER_ROOT = HITL_ROOT.parent / "skye_robot_driver"


def _arm_signs(document, node_name, side):
    return document[node_name]["ros__parameters"][f"{side}_joint_signs"]


def test_arbiter_profiles_match_driver_joint_signs():
    for profile in ("thor", "orin"):
        arbiter = yaml.safe_load(
            (HITL_ROOT / "config" / "profiles" / f"{profile}.yaml").read_text(
                encoding="utf-8"
            )
        )
        driver = yaml.safe_load(
            (DRIVER_ROOT / "config" / "profiles" / f"{profile}.yaml").read_text(
                encoding="utf-8"
            )
        )
        for side in ("left", "right"):
            assert _arm_signs(arbiter, "control_arbiter", side) == _arm_signs(
                driver, "skye_robot_driver", side
            ), f"{profile} {side} joint_signs drifted"
