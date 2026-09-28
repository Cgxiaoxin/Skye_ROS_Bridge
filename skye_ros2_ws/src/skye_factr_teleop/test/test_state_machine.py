# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

"""Task 4.1 behavioral contract; implementation belongs to task 4.2.

The conditional marker only covers the absent StateMachine placeholder. Once
the class exists, all tests run normally, including missing API failures.
"""

import pytest

from skye_factr_teleop import state_machine as sm


pytestmark = pytest.mark.xfail(
    not hasattr(sm, "StateMachine"),
    reason="Task 4.2 pending: StateMachine is not implemented",
    raises=AttributeError,
    strict=True,
)


def _machine_in(state):
    machine = sm.StateMachine(now=0.0)
    assert machine.state == sm.ControlMode.IDLE
    events = {
        "IDLE": (),
        "TELEOP_SYNCING": ("switch_sync",),
        "SYNCED": ("switch_sync", "sync_complete"),
        "TELEOP": ("switch_sync", "sync_complete", "switch_teleop"),
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
        ("TELEOP_SYNCING", "sync_complete", "SYNCED", "accepted"),
        ("TELEOP_SYNCING", "sync_timeout", "IDLE", "accepted"),
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
    ],
)
def test_transition_contract(initial, event, expected, outcome):
    machine = _machine_in(initial)
    result = machine.handle(event, now=4.0)
    assert machine.state == getattr(sm.ControlMode, expected)
    assert result.outcome == outcome
    if outcome == "rejected":
        assert isinstance(result.reason, str)
        assert result.reason.strip()


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
