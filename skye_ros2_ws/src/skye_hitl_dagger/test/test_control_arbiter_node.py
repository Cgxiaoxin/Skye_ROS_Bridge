from skye_hitl_dagger.control_arbiter_node import (
    policy_gripper_value,
    sync_ready,
)


def test_policy_gripper_inverts_motor_value_when_enabled():
    assert policy_gripper_value(0.2, True) == 0.8


def test_policy_gripper_preserves_motor_value_when_disabled():
    assert policy_gripper_value(0.2, False) == 0.2


def test_sync_ready_rejects_stale_transient_local_teleop():
    assert not sync_ready("TELEOP", True, False)


def test_sync_ready_requires_non_teleop_then_teleop():
    assert sync_ready("TELEOP", True, True)
    assert not sync_ready("SYNC", True, True)
    assert not sync_ready("TELEOP", False, True)
