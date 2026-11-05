"""Sync republish termination, backoff, and failure classification."""

from skye_hitl_dagger.sync_retry import (
    SideSyncOutcome,
    SyncRetryPolicy,
    classify_sync_failure,
    is_deterministic_sync_failure,
)


def test_deterministic_failure_detects_bounds_and_preflight():
    assert is_deterministic_sync_failure(
        "Synchronization rejected [side=right, phase=sync_preflight]: "
        "current pose is outside configured safe bounds")
    assert is_deterministic_sync_failure(
        "trajectory leaves configured safe bounds on axis 1")
    assert not is_deterministic_sync_failure(
        "Synchronization requires fresh follower state")
    assert not is_deterministic_sync_failure("sync_timeout")


def test_classify_failed_progress_outcome():
    assert classify_sync_failure(
        "failed",
        "Synchronization rejected [side=left, phase=sync_preflight]: "
        "current pose is outside configured safe bounds; reposition",
    ) is SideSyncOutcome.FAILED_DETERMINISTIC
    assert classify_sync_failure(
        "failed", "缺少新鲜的大臂关节状态"
    ) is SideSyncOutcome.FAILED_RETRYABLE
    assert classify_sync_failure("synced", "SYNCED") is SideSyncOutcome.SYNCED
    assert classify_sync_failure("active", "TELEOP_SYNCING") is SideSyncOutcome.SYNCING


def test_no_republish_once_aligned_ready():
    policy = SyncRetryPolicy(backoff_s=(5.0, 10.0, 20.0), max_attempts=3)
    policy.note_sync_published(now=0.0)
    assert policy.should_republish_sync(
        now=6.0, aligned_ready=False, dual_terminal=False) is True
    policy.note_sync_published(now=6.0)
    assert policy.should_republish_sync(
        now=100.0, aligned_ready=True, dual_terminal=False) is False


def test_backoff_then_exhausted_to_fault():
    policy = SyncRetryPolicy(backoff_s=(5.0, 10.0, 20.0), max_attempts=3)
    policy.note_sync_published(now=0.0)
    assert policy.attempts == 1
    assert policy.should_republish_sync(
        now=4.9, aligned_ready=False, dual_terminal=False) is False
    assert policy.should_republish_sync(
        now=5.0, aligned_ready=False, dual_terminal=False) is True
    policy.note_sync_published(now=5.0)
    assert policy.should_republish_sync(
        now=14.9, aligned_ready=False, dual_terminal=False) is False
    assert policy.should_republish_sync(
        now=15.0, aligned_ready=False, dual_terminal=False) is True
    policy.note_sync_published(now=15.0)
    # 3 publishes total (initial + 2 backoff retries) → exhausted
    assert policy.exhausted() is True
    assert policy.should_republish_sync(
        now=35.0, aligned_ready=False, dual_terminal=False) is False
    assert policy.should_republish_sync(
        now=100.0, aligned_ready=False, dual_terminal=False) is False


def test_dual_terminal_stops_republish_without_exhausting():
    policy = SyncRetryPolicy(backoff_s=(5.0, 10.0, 20.0), max_attempts=3)
    policy.note_sync_published(now=0.0)
    assert policy.should_republish_sync(
        now=10.0, aligned_ready=False, dual_terminal=True) is False
    assert policy.exhausted() is False


def test_reset_clears_attempts():
    policy = SyncRetryPolicy(backoff_s=(5.0,), max_attempts=1)
    policy.note_sync_published(now=0.0)
    assert policy.exhausted() is True
    policy.reset()
    assert policy.attempts == 0
    assert not policy.exhausted()
