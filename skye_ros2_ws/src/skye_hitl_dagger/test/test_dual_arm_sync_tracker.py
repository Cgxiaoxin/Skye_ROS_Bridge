from skye_hitl_dagger.sync_retry import DualArmSyncTracker, SideSyncOutcome


def test_synced_plus_deterministic_fail_is_terminal_without_dragging():
    tracker = DualArmSyncTracker()
    tracker.on_progress(
        "right", "synced", "[right] SYNCED：同步成功")
    tracker.on_progress(
        "left", "failed",
        "Synchronization rejected [side=left, phase=sync_preflight]: "
        "current pose is outside configured safe bounds")
    assert tracker.outcome("right") is SideSyncOutcome.SYNCED
    assert tracker.outcome("left") is SideSyncOutcome.FAILED_DETERMINISTIC
    assert tracker.dual_terminal() is True
    assert tracker.any_synced() is True
    assert not tracker.all_deterministic_failed()
    hints = tracker.operator_hints()
    assert len(hints) == 1
    assert hints[0].startswith("left:")


def test_both_deterministic_failed():
    tracker = DualArmSyncTracker()
    for side in ("left", "right"):
        tracker.on_progress(
            side, "failed",
            f"Synchronization rejected [side={side}, phase=sync_preflight]: "
            "current pose is outside configured safe bounds")
    assert tracker.all_deterministic_failed()
    assert tracker.dual_terminal()


def test_synced_arm_not_downgraded_by_later_reject_spam():
    tracker = DualArmSyncTracker()
    tracker.on_progress("right", "synced", "ok")
    tracker.on_progress(
        "right", "failed",
        "Synchronization rejected [side=right, phase=sync_preflight]: "
        "current pose is outside configured safe bounds")
    assert tracker.outcome("right") is SideSyncOutcome.SYNCED


def test_still_syncing_sibling_keeps_retry_open():
    tracker = DualArmSyncTracker()
    tracker.on_progress("right", "synced", "ok")
    tracker.on_progress("left", "active", "TELEOP_SYNCING")
    assert tracker.dual_terminal() is False
