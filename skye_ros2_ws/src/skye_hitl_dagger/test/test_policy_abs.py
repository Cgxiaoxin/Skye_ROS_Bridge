from skye_hitl_dagger.policy_abs import (
    apply_joint_mapping,
    follower_pose_to_leader_abs,
)


def test_identity_signs_roundtrip():
    pose = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7]
    signs = [1.0] * 7
    order = list(range(7))
    leader = follower_pose_to_leader_abs(pose, signs, order)
    assert leader == pose
    assert apply_joint_mapping(leader, signs, order) == pose


def test_negative_signs_roundtrip():
    pose = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7]
    # Mirror thor/orin right arm: J6/J7 flipped for teleop/abs.
    signs = [1.0, 1.0, 1.0, 1.0, 1.0, -1.0, -1.0]
    order = list(range(7))
    leader = follower_pose_to_leader_abs(pose, signs, order)
    assert leader[5] == -pose[5]
    assert leader[6] == -pose[6]
    assert apply_joint_mapping(leader, signs, order) == pose


def test_offsets_roundtrip():
    pose = [0.0] * 7
    pose[0] = 1.0
    signs = [1.0] * 7
    order = list(range(7))
    offsets = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    leader = follower_pose_to_leader_abs(pose, signs, order, offsets)
    assert abs(leader[0] - 0.75) < 1e-9
    assert apply_joint_mapping(leader, signs, order, offsets) == pose
