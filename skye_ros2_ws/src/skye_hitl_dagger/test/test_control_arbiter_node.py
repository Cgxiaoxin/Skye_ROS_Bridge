from skye_hitl_dagger.control_arbiter_node import policy_gripper_value


def test_policy_gripper_inverts_motor_value_when_enabled():
    assert policy_gripper_value(0.2, True) == 0.8


def test_policy_gripper_preserves_motor_value_when_disabled():
    assert policy_gripper_value(0.2, False) == 0.2
