from skye_hitl_dagger.control_arbiter_node import (
    is_post_sync_teleop,
    policy_gripper_value,
)


def test_policy_gripper_inverts_motor_value_when_enabled():
    assert policy_gripper_value(0.2, True) == 0.8


def test_policy_gripper_preserves_motor_value_when_disabled():
    assert policy_gripper_value(0.2, False) == 0.2


def test_sync_requires_teleop_received_after_request():
    assert is_post_sync_teleop("TELEOP", 10.1, 10.0)
    assert not is_post_sync_teleop("TELEOP", 9.9, 10.0)
    assert not is_post_sync_teleop("STOP", 10.1, 10.0)
