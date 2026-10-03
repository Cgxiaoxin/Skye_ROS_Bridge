from skye_operator_ui.leader_arms import LeaderArmGate


def test_default_both_enabled_unlocked():
    g = LeaderArmGate()
    assert g.snapshot() == {
        "left_enabled": True, "right_enabled": True, "locked": False
    }


def test_disable_left_before_lock():
    g = LeaderArmGate()
    assert g.set_enabled("left", False)
    assert g.snapshot()["left_enabled"] is False
    assert g.align_payload() == "align_follower_right"


def test_lock_on_sync_blocks_toggle():
    g = LeaderArmGate()
    g.note_sync_dispatched()
    assert g.snapshot()["locked"] is True
    assert g.set_enabled("left", False) is False


def test_lock_on_teleop_state():
    g = LeaderArmGate()
    g.note_teleop_state("TELEOP_SYNCING")
    assert g.snapshot()["locked"] is True


def test_both_off_align_none():
    g = LeaderArmGate()
    g.set_enabled("left", False)
    g.set_enabled("right", False)
    assert g.any_enabled() is False
    assert g.align_payload() is None


def test_reset_clears_lock():
    g = LeaderArmGate()
    g.set_enabled("left", False)
    g.note_sync_dispatched()
    g.reset()
    assert g.snapshot() == {
        "left_enabled": True, "right_enabled": True, "locked": False
    }
