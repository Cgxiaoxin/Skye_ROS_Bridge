from skye_follower_align.align_logic import (
    AlignPhase,
    AlignSession,
    map_leader_to_follower,
)


def test_map_leader_applies_signs():
    leader = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
    signs = [1, 1, 1, 1, 1, -1, -1]
    out = map_leader_to_follower(leader, signs)
    assert out == [0.1, 0.2, 0.3, 0.4, 0.5, -0.6, -0.7]


def test_aligned_after_hold_frames():
    s = AlignSession(threshold_rad=0.05, hold_frames=3, timeout_s=10.0)
    s.start()
    leader = [0.0] * 7
    big = [0.01] * 7
    signs = [1.0] * 7
    for _ in range(2):
        assert s.on_tick(leader, big, signs) == AlignPhase.ALIGNING
    assert s.on_tick(leader, big, signs) == AlignPhase.ALIGNED


def test_timeout_warn_soft():
    s = AlignSession(threshold_rad=0.01, hold_frames=3, timeout_s=0.0)
    s.start(now=0.0)
    phase = s.on_tick([1.0] * 7, [0.0] * 7, [1.0] * 7, now=1.0)
    assert phase == AlignPhase.TIMEOUT_WARN


def test_second_start_ignored_while_aligning():
    s = AlignSession(threshold_rad=0.05, hold_frames=10, timeout_s=10.0)
    assert s.start() is True
    assert s.start() is False
    assert s.phase == AlignPhase.ALIGNING


def test_cancel_returns_idle():
    s = AlignSession(threshold_rad=0.05, hold_frames=10, timeout_s=10.0)
    s.start()
    s.cancel()
    assert s.phase == AlignPhase.IDLE
