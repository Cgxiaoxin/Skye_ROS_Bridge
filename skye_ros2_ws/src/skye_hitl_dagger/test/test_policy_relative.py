import math

from skye_hitl_dagger.policy_relative import (
    PI,
    PolicyRelativeSession,
    unwrap_frame_delta,
)


def test_unwrap_frame_delta_corrects_pi_crossing():
    assert abs(unwrap_frame_delta(-3.1, 2.6) - 0.5831853071795865) < 1e-9


def test_policy_session_first_frame_holds_feedback_pose():
    session = PolicyRelativeSession()
    feedback = [0.5, -0.2, 1.0, 0.1, -0.3, 0.0, 0.2]
    session.begin(feedback)
    leader = session.follower_target_to_leader(feedback)
    session.commit_published(leader)
    assert leader == feedback


def test_policy_session_tracks_follower_delta_with_identity_mapping():
    session = PolicyRelativeSession()
    session.begin([0.0] * 7)
    target = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    leader = session.follower_target_to_leader(target)
    session.commit_published(leader)
    assert abs(leader[0] - 0.1) < 1e-12
    assert all(abs(leader[i]) < 1e-12 for i in range(1, 7))


def test_policy_session_unwrap_avoids_spurious_full_turn():
    session = PolicyRelativeSession()
    session.begin([2.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    session.leader_prev[0] = 2.6
    session.leader_continuous[0] = 2.6
    session.leader_cont_ref[0] = 2.6
    target = [2.6 + 0.5831853071795865, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    leader = session.follower_target_to_leader(target)
    assert abs(leader[0] - (-3.1)) < 1e-6 or abs(leader[0] - target[0]) < 1e-6
    session.commit_published(leader)
    assert abs(session.leader_continuous[0] - target[0]) < 1e-9


def test_invalidate_requires_begin_before_publish():
    session = PolicyRelativeSession()
    session.invalidate()
    assert not session.active
