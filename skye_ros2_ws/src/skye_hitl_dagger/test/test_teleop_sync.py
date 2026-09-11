from skye_hitl_dagger.teleop_sync import (
    SyncPhase,
    TeleopHandshake,
    is_aligned_state,
    is_teleop_state,
)


class FakeClock:
    def __init__(self, t: float = 0.0) -> None:
        self.t = t

    def __call__(self) -> float:
        return self.t


def test_aligned_state_accepts_synced_only():
    assert is_aligned_state("SYNCED")
    assert is_aligned_state("synced")
    assert not is_aligned_state("TELEOP_SYNCING")
    assert not is_aligned_state("IDLE")
    assert not is_aligned_state("TELEOP")
    assert not is_aligned_state(None)


def test_teleop_state_detection():
    assert is_teleop_state("TELEOP")
    assert not is_teleop_state("TELEOP_SYNCING")
    assert not is_teleop_state(None)


def test_switch_teleop_requested_after_alignment_not_after_teleop():
    hs = TeleopHandshake()
    hs.start_sync()
    hs.on_state("TELEOP_SYNCING")
    assert not hs.aligned_ready()
    hs.on_state("SYNCED")
    assert hs.aligned_ready()
    assert hs.pending_command() == "switch_sync"


def test_latched_synced_before_takeover_is_ignored():
    """Old latched SYNCED must not satisfy a new handover sync."""
    hs = TeleopHandshake()
    hs.on_state("SYNCED")
    hs.start_sync()
    assert hs.state() is None
    assert not hs.aligned_ready()
    hs.on_state("SYNCED")  # still no progress through unaligned
    assert not hs.aligned_ready()


def test_requires_unaligned_progress_before_synced():
    hs = TeleopHandshake()
    hs.start_sync()
    hs.on_state("SYNCED")
    assert not hs.aligned_ready()
    hs.on_state("TELEOP_SYNCING")
    assert not hs.aligned_ready()
    hs.on_state("SYNCED")
    assert hs.aligned_ready()


def test_idle_or_stop_also_counts_as_unaligned_progress():
    hs = TeleopHandshake()
    hs.start_sync()
    hs.on_state("IDLE")
    hs.on_state("SYNCED")
    assert hs.aligned_ready()


def test_min_sync_hold_delays_aligned_ready():
    clock = FakeClock(10.0)
    hs = TeleopHandshake(min_sync_hold_s=1.5, clock=clock)
    hs.start_sync()
    hs.on_state("TELEOP_SYNCING")
    hs.on_state("SYNCED")
    assert not hs.aligned_ready()
    clock.t = 11.0
    assert not hs.aligned_ready()
    clock.t = 11.5
    assert hs.aligned_ready()


def test_leaving_synced_resets_hold_timer():
    clock = FakeClock(0.0)
    hs = TeleopHandshake(min_sync_hold_s=1.0, clock=clock)
    hs.start_sync()
    hs.on_state("TELEOP_SYNCING")
    hs.on_state("SYNCED")
    clock.t = 0.8
    assert not hs.aligned_ready()
    hs.on_state("TELEOP_SYNCING")
    hs.on_state("SYNCED")
    clock.t = 1.5  # only 0.7s since re-entry
    assert not hs.aligned_ready()
    clock.t = 1.9
    assert hs.aligned_ready()


def test_latched_teleop_cannot_satisfy_phase_two():
    hs = TeleopHandshake()
    hs.on_state("TELEOP")
    hs.start_sync()
    assert not hs.aligned_ready()
    hs.on_state("TELEOP_SYNCING")
    hs.on_state("SYNCED")
    hs.start_teleop()
    assert hs.phase() is SyncPhase.WAIT_TELEOP
    assert hs.state() is None
    assert not hs.teleop_ready()
    hs.on_state("TELEOP")
    assert hs.teleop_ready()


def test_reset_clears_phase_and_state():
    hs = TeleopHandshake()
    hs.start_sync()
    hs.on_state("TELEOP_SYNCING")
    hs.on_state("SYNCED")
    hs.reset()
    assert hs.phase() is SyncPhase.IDLE
    assert hs.state() is None
    assert not hs.aligned_ready()
    assert hs.pending_command() is None


def test_pending_command_tracks_phase():
    hs = TeleopHandshake()
    assert hs.pending_command() is None
    hs.start_sync()
    assert hs.pending_command() == "switch_sync"
    hs.start_teleop()
    assert hs.pending_command() == "switch_teleop"
