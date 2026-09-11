from skye_hitl_dagger.chunk_continuity import (
    chunk_is_fresh,
    max_step0_jump_rad,
    rebase_arm_joints_to_pose,
)


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


def test_unstamped_chunk_uses_receive_time():
    assert chunk_is_fresh(0.0, 10.0, 12.0, 11.0, 12.0)


def test_stale_stamp_uses_receive_time_after_fallback():
    assert chunk_is_fresh(9.0, 10.0, 13.1, 10.0, 12.0, 2.0)


def test_stale_stamp_rejected_before_fallback():
    assert not chunk_is_fresh(9.0, 10.0, 11.0, 10.0, 11.5, 2.0)


def test_max_step0_jump_reports_largest_joint_error():
    left = [0.0] * 7 + [0.1] * 7
    right = [0.0] * 7 + [0.2] * 7
    pose_l = [0.0] * 7
    pose_r = [0.05] * 7
    assert abs(max_step0_jump_rad(left, right, pose_l, pose_r) - 0.05) < 1e-9


def test_rebase_arm_joints_preserves_deltas_from_new_start():
    joints = [1.0] * 7 + [1.2] * 7
    pose = [0.5] * 7
    out = rebase_arm_joints_to_pose(joints, pose, steps=2)
    assert out[:7] == [0.5] * 7
    assert out[7:] == [0.7] * 7
