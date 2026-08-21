from skye_hitl_dagger.control_arbiter_node import (
    chunk_is_fresh,
    policy_gripper_value,
)


def test_policy_gripper_inverts_motor_value_when_enabled():
    assert policy_gripper_value(0.2, True) == 0.8


def test_policy_gripper_preserves_motor_value_when_disabled():
    assert policy_gripper_value(0.2, False) == 0.2


def test_chunk_accepted_when_no_return_pending():
    assert chunk_is_fresh(0.0, None)
    assert chunk_is_fresh(12.0, None)


def test_chunk_rejected_when_stamped_before_return():
    assert not chunk_is_fresh(9.999, 10.0)


def test_chunk_accepted_when_stamped_at_or_after_return():
    assert chunk_is_fresh(10.0, 10.0)
    assert chunk_is_fresh(10.001, 10.0)


def test_unstamped_chunk_rejected_while_return_pending():
    assert not chunk_is_fresh(0.0, 10.0)
