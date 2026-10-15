# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

from skye_factr_teleop.friction_test_switch import SwitchSession


def test_switch_uses_confirmed_phase_and_blocks_duplicate_request():
    session = SwitchSession()
    session.set_mode("IDLE")
    session.set_phase("idle")
    assert session.next_phase() is None

    session.set_mode("FRICTION_TEST")
    session.set_phase("off")
    assert session.next_phase() == "on"
    session.mark_sent("on", 10)
    assert session.next_phase() is None
    session.set_phase("off")
    assert session.next_phase() is None
    session.set_phase("on")
    assert session.next_phase() == "off"


def test_switch_recovers_from_unconfirmed_request_and_stops_at_idle():
    session = SwitchSession()
    session.set_mode("FRICTION_TEST")
    session.set_phase("off")
    session.mark_sent("on", 10)
    assert not session.expire_pending(10.5)
    assert session.expire_pending(11.1)
    assert session.next_phase() == "on"
    session.set_mode("IDLE")
    session.set_phase("idle")
    assert session.next_phase() is None
