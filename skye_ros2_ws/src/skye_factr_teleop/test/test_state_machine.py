# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

"""Pure state-machine contracts with caller-supplied time and no hardware."""

import pytest

from skye_factr_teleop import state_machine as sm


def _machine_in(state):
    machine = sm.StateMachine(now=0.0)
    assert machine.state == sm.ControlMode.IDLE
    events = {
        "IDLE": (),
        "TELEOP_SYNCING": ("switch_sync",),
        "HOLD_SETTLING": ("switch_sync", "trajectory_complete"),
        "SYNCED": ("switch_sync", "trajectory_complete", "sync_complete"),
        "TELEOP": ("switch_sync", "trajectory_complete", "sync_complete", "switch_teleop"),
        "GRAVITY_TEST": ("switch_gravity_test",),
        "LIMIT_TEST": ("switch_limit_test",),
        "FRICTION_TEST": ("switch_friction_test",),
    }
    for now, event in enumerate(events[state], start=1):
        result = machine.handle(event, now=float(now))
        assert result.outcome == "accepted"
    assert machine.state == getattr(sm.ControlMode, state)
    return machine


@pytest.mark.parametrize(
    "initial,event,expected,outcome",
    [
        ("IDLE", "switch_sync", "TELEOP_SYNCING", "accepted"),
        ("IDLE", "switch_teleop", "IDLE", "rejected"),
        ("IDLE", "switch_stop", "IDLE", "accepted"),
        ("IDLE", "sync_complete", "IDLE", "ignored"),
        ("IDLE", "sync_timeout", "IDLE", "ignored"),
        ("TELEOP_SYNCING", "switch_sync", "TELEOP_SYNCING", "ignored"),
        ("TELEOP_SYNCING", "switch_teleop", "TELEOP_SYNCING", "rejected"),
        ("TELEOP_SYNCING", "switch_stop", "IDLE", "accepted"),
        ("TELEOP_SYNCING", "trajectory_complete", "HOLD_SETTLING", "accepted"),
        ("TELEOP_SYNCING", "sync_complete", "TELEOP_SYNCING", "ignored"),
        ("TELEOP_SYNCING", "sync_timeout", "IDLE", "accepted"),
        ("HOLD_SETTLING", "switch_sync", "HOLD_SETTLING", "ignored"),
        ("HOLD_SETTLING", "switch_teleop", "HOLD_SETTLING", "rejected"),
        ("HOLD_SETTLING", "sync_complete", "SYNCED", "accepted"),
        ("HOLD_SETTLING", "settle_timeout", "IDLE", "accepted"),
        ("HOLD_SETTLING", "switch_stop", "IDLE", "accepted"),
        ("SYNCED", "switch_sync", "SYNCED", "ignored"),
        ("SYNCED", "switch_teleop", "TELEOP", "accepted"),
        ("SYNCED", "switch_stop", "IDLE", "accepted"),
        ("SYNCED", "sync_complete", "SYNCED", "ignored"),
        ("SYNCED", "sync_timeout", "SYNCED", "ignored"),
        ("TELEOP", "switch_sync", "TELEOP", "rejected"),
        ("TELEOP", "switch_teleop", "TELEOP", "rejected"),
        ("TELEOP", "switch_stop", "IDLE", "accepted"),
        ("TELEOP", "sync_complete", "TELEOP", "ignored"),
        ("TELEOP", "sync_timeout", "TELEOP", "ignored"),
        ("IDLE", "switch_gravity_test", "GRAVITY_TEST", "accepted"),
        ("GRAVITY_TEST", "switch_gravity_test", "GRAVITY_TEST", "ignored"),
        ("GRAVITY_TEST", "switch_sync", "GRAVITY_TEST", "rejected"),
        ("GRAVITY_TEST", "switch_teleop", "GRAVITY_TEST", "rejected"),
        ("GRAVITY_TEST", "gravity_test_timeout", "IDLE", "accepted"),
        ("GRAVITY_TEST", "switch_stop", "IDLE", "accepted"),
        ("IDLE", "switch_limit_test", "LIMIT_TEST", "accepted"),
        ("LIMIT_TEST", "switch_limit_test", "LIMIT_TEST", "ignored"),
        ("LIMIT_TEST", "switch_sync", "LIMIT_TEST", "rejected"),
        ("LIMIT_TEST", "switch_teleop", "LIMIT_TEST", "rejected"),
        ("LIMIT_TEST", "limit_test_timeout", "IDLE", "accepted"),
        ("LIMIT_TEST", "switch_stop", "IDLE", "accepted"),
        ("IDLE", "switch_friction_test", "FRICTION_TEST", "accepted"),
        ("FRICTION_TEST", "switch_friction_test", "FRICTION_TEST", "ignored"),
        ("FRICTION_TEST", "switch_sync", "FRICTION_TEST", "rejected"),
        ("FRICTION_TEST", "switch_teleop", "FRICTION_TEST", "rejected"),
        ("FRICTION_TEST", "friction_test_timeout", "IDLE", "accepted"),
        ("FRICTION_TEST", "switch_stop", "IDLE", "accepted"),
    ],
)
def test_transition_contract(initial, event, expected, outcome):
    machine = _machine_in(initial)
    entered_at = machine.entered_at
    result = machine.handle(event, now=4.0)
    assert machine.state == getattr(sm.ControlMode, expected)
    assert result.previous_state == getattr(sm.ControlMode, initial)
    assert result.state == machine.state
    assert result.entered_at == machine.entered_at
    assert machine.entered_at == (4.0 if initial != expected else entered_at)
    assert result.outcome == outcome
    if outcome != "accepted":
        assert result.actions == ()
    if outcome == "rejected":
        assert isinstance(result.reason, str)
        assert result.reason.strip()
        assert machine.last_rejection_reason == result.reason


@pytest.mark.parametrize("exit_event", ["sync_timeout", "switch_stop"])
def test_sync_exit_requires_a_new_successful_sync(exit_event):
    machine = _machine_in("TELEOP_SYNCING")
    result = machine.handle(exit_event, now=2.0)
    assert result.outcome == "accepted"
    assert machine.state == sm.ControlMode.IDLE

    # A completion arriving after exit cannot grant teleoperation permission.
    result = machine.handle("sync_complete", now=3.0)
    assert result.outcome == "ignored"
    assert machine.state == sm.ControlMode.IDLE
    result = machine.handle("switch_teleop", now=4.0)
    assert result.outcome == "rejected"
    assert machine.state == sm.ControlMode.IDLE

    result = machine.handle("switch_sync", now=5.0)
    assert result.outcome == "accepted"
    assert machine.state == sm.ControlMode.TELEOP_SYNCING
    result = machine.handle("switch_teleop", now=6.0)
    assert result.outcome == "rejected"
    assert machine.state == sm.ControlMode.TELEOP_SYNCING
    result = machine.handle("trajectory_complete", now=7.0)
    assert result.state == sm.ControlMode.HOLD_SETTLING
    assert machine.handle("switch_teleop", now=7.0).outcome == "rejected"
    result = machine.handle("sync_complete", now=7.0)
    assert result.outcome == "accepted"
    assert machine.state == sm.ControlMode.SYNCED
    result = machine.handle("switch_teleop", now=8.0)
    assert result.outcome == "accepted"
    assert machine.state == sm.ControlMode.TELEOP


def test_base_control_mode_values():
    _machine_in("IDLE")
    assert sm.ControlMode.IDLE.value == 0
    assert sm.ControlMode.TELEOP_SYNCING.value == 1
    assert sm.ControlMode.SYNCED.value == 2
    assert sm.ControlMode.TELEOP.value == 3
    assert sm.ControlMode.HOLD_SETTLING.value == 4
    assert sm.ControlMode.GRAVITY_TEST.value == 6
    assert sm.ControlMode.LIMIT_TEST.value == 7
    assert sm.ControlMode.FRICTION_TEST.value == 8


@pytest.mark.parametrize(
    "initial,event,actions",
    [
        ("IDLE", "switch_sync", ("clear_sync", "start_sync")),
        ("TELEOP_SYNCING", "trajectory_complete", ("stop_motion", "start_hold")),
        ("HOLD_SETTLING", "sync_complete", ()),
        ("HOLD_SETTLING", "settle_timeout", ("stop_motion", "clear_sync")),
        ("SYNCED", "switch_teleop", ("stop_motion", "clear_sync", "start_teleop")),
        ("TELEOP_SYNCING", "sync_timeout", ("stop_motion", "clear_sync")),
        ("IDLE", "switch_stop", ("stop_motion", "clear_sync")),
        ("TELEOP_SYNCING", "switch_stop", ("stop_motion", "clear_sync")),
        ("SYNCED", "switch_stop", ("stop_motion", "clear_sync")),
        ("TELEOP", "switch_stop", ("stop_motion", "clear_sync")),
        ("IDLE", "switch_gravity_test", ("clear_sync", "start_gravity_test")),
        ("GRAVITY_TEST", "gravity_test_timeout", ("stop_motion", "clear_sync")),
        ("IDLE", "switch_limit_test", ("clear_sync", "start_limit_test")),
        ("LIMIT_TEST", "limit_test_timeout", ("stop_motion", "clear_sync")),
        ("IDLE", "switch_friction_test", ("clear_sync", "start_friction_test")),
        ("FRICTION_TEST", "friction_test_timeout", ("stop_motion", "clear_sync")),
    ],
)
def test_ordered_control_actions(initial, event, actions):
    machine = _machine_in(initial)
    result = machine.handle(event, now=4.0)
    assert result.actions == tuple(sm.Action(action) for action in actions)
    if event == "sync_timeout":
        assert result.reason == "sync_timeout"


@pytest.mark.parametrize("state", ["IDLE", "TELEOP_SYNCING", "HOLD_SETTLING", "SYNCED", "TELEOP"])
@pytest.mark.parametrize("event", ["switch_telepose", "unknown"])
def test_unsupported_events_are_rejected(state, event):
    machine = _machine_in(state)
    entered_at = machine.entered_at
    result = machine.handle(event, now=4.0)
    assert result.outcome == "rejected"
    assert result.reason == "unsupported_event"
    assert machine.last_rejection_reason == result.reason
    assert result.actions == ()
    assert machine.state == getattr(sm.ControlMode, state)
    assert machine.entered_at == entered_at


def test_explicit_events_and_recorded_rejection():
    machine = sm.StateMachine(now=10.0)
    assert machine.entered_at == 10.0
    assert machine.last_rejection_reason is None
    rejected = machine.handle(sm.Event.SWITCH_TELEOP, now=11.0)
    assert rejected.reason == "not_synced"
    machine.handle(sm.Event.SWITCH_SYNC, now=12.0)
    assert machine.entered_at == 12.0
    assert machine.last_rejection_reason == "not_synced"
    # Repeated sync requests must not restart the timeout window.
    machine.handle(sm.Event.SWITCH_SYNC, now=13.0)
    assert machine.entered_at == 12.0
    machine.handle(sm.Event.SYNC_TIMEOUT, now=14.0)
    assert machine.entered_at == 14.0
    assert rejected.state == sm.ControlMode.IDLE
    assert rejected.entered_at == 10.0


@pytest.mark.parametrize("now", [float("nan"), float("inf"), -float("inf")])
def test_nonfinite_time_does_not_mutate_state(now):
    with pytest.raises(ValueError, match="finite"):
        sm.StateMachine(now=now)
    machine = _machine_in("TELEOP_SYNCING")
    with pytest.raises(ValueError, match="finite"):
        machine.handle("sync_complete", now=now)
    assert machine.state == sm.ControlMode.TELEOP_SYNCING
    assert machine.entered_at == 1.0
    assert machine.last_rejection_reason is None
    assert machine.handle("trajectory_complete", now=2.0).outcome == "accepted"


def test_time_cannot_move_backwards_but_same_timestamp_is_allowed():
    machine = _machine_in("TELEOP_SYNCING")
    machine.handle("switch_sync", now=3.0)
    with pytest.raises(ValueError, match="nondecreasing"):
        machine.handle("sync_complete", now=2.0)
    assert machine.state == sm.ControlMode.TELEOP_SYNCING
    assert machine.entered_at == 1.0
    assert machine.handle("trajectory_complete", now=3.0).outcome == "accepted"
