# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

from types import SimpleNamespace

import pytest

from skye_factr_teleop import sync_trial, torque_monitor
from skye_factr_teleop.torque_monitor import TORQUE_TERMS, TorqueDisplay, CurrentDisplay


@pytest.mark.parametrize("term", TORQUE_TERMS)
def test_display_selects_correct_component_and_preserves_axis_order(term):
    display = TorqueDisplay("left", term)
    packet = dict(side="left", mode="TELEOP_SYNCING", sequence=42, torque_enabled_cached=True)
    for i, (field, _) in enumerate(TORQUE_TERMS.values()):
        packet[field] = [i + joint / 100 for joint in range(8)]
    display.update(packet, 10.0)
    text = display.render(10.2)
    assert TORQUE_TERMS[term][1] in text
    assert "mode=TELEOP_SYNCING" in text
    assert "sequence=42" in text
    for i, value in enumerate(packet[display.field]):
        name = f"J{i + 1}" if i < 7 else "Gripper(ID8)"
        assert f"{name:<14} {value:+12.6f}" in text


def test_display_marks_missing_and_stale_data_without_reusing_old_values():
    display = TorqueDisplay("left", "gravity")
    assert "等待诊断消息" in display.render(10.0)
    display.update(dict(side="left", mode="IDLE", sequence=1, torque_enabled_cached=False,
                        gravity_torque_nm=[0.123456] * 8), 10.0)
    assert "+0.123456" in display.render(10.2)
    text = display.render(11.1)
    assert "STALE" in text
    assert "+0.123456" not in text


def test_display_ignores_wrong_side_and_invalid_vectors():
    display = TorqueDisplay("left", "gravity")
    for side, values in [("right", [0.0] * 8), ("left", [0.0] * 7),
                         ("left", [float("nan")] * 8)]:
        display.update(dict(side=side, gravity_torque_nm=values), 10.0)
    assert display.packet is None


@pytest.mark.parametrize("term", ["limit", "current"])
def test_monitor_cli_runs_without_config_and_defaults_to_five_hz(monkeypatch, term):
    calls = []
    monkeypatch.setattr(torque_monitor, "monitor_torque", calls.append)
    monkeypatch.setattr(sync_trial, "_load", lambda args: pytest.fail("monitor must not load config"))
    sync_trial.main(["monitor", "--side", "left", "--term", term])
    assert vars(calls[0]) == vars(SimpleNamespace(command="monitor", side="left", term=term, rate=5.0))


@pytest.mark.parametrize("rate", ["0", "-1", "nan", "inf", "51"])
def test_monitor_cli_rejects_invalid_refresh_rate(rate):
    with pytest.raises(SystemExit) as exc:
        sync_trial.main(["monitor", "--side", "left", "--term", "gravity", "--rate", rate])
    assert exc.value.code == 2


def current_packet():
    return dict(side="left", mode="TELEOP_SYNCING", sequence=42, torque_enabled_cached=True,
                present_current_ma=[52.0, -2.0] + [0.0] * 6,
                current_limit_ma=[50.0] * 8,
                current_before_clip_ma=[40.0, -80.0] + [0.0] * 6,
                current_after_clip_ma=[40.0, -50.0] + [0.0] * 6)


def test_current_display_distinguishes_measured_overlimit_from_command_clipping():
    display = CurrentDisplay("left")
    display.update(current_packet(), 10.0)
    lines = display.render(10.1).splitlines()
    first = next(line for line in lines if line.startswith("ID1/J1"))
    second = next(line for line in lines if line.startswith("ID2/J2"))
    assert "+52.00" in first and "50.00" in first and "104%" in first
    assert first.endswith("--/--")
    assert "-2.00" in second and "-80.00-> -50.00" in second
    assert second.endswith("CLIP/CLIP")
    assert any(line.startswith("ID8/Grip") for line in lines)


def test_current_display_retains_short_clipping_events_until_refresh():
    display = CurrentDisplay("left")
    display.update(current_packet(), 10.0)
    unclipped = current_packet()
    unclipped["current_before_clip_ma"] = unclipped["current_after_clip_ma"]
    display.update(unclipped, 10.1)
    line = next(line for line in display.render(10.2).splitlines() if line.startswith("ID2/J2"))
    assert line.endswith("--/CLIP")
    line = next(line for line in display.render(10.3).splitlines() if line.startswith("ID2/J2"))
    assert line.endswith("--/--")


def test_current_display_missing_limits_and_stale_samples_are_explicit():
    display = CurrentDisplay("left")
    assert "等待诊断消息" in display.render(10.0)
    packet = current_packet()
    packet.pop("current_limit_ma")
    display.update(packet, 10.0)
    text = display.render(10.1)
    assert "限值缺失" in text
    assert "CLIP/CLIP" in text
    text = display.render(11.1)
    assert "STALE" in text
    assert "+52.00" not in text
    assert "CLIP/CLIP" not in text


def test_current_display_rejects_wrong_side_and_invalid_limits():
    display = CurrentDisplay("left")
    packet = current_packet()
    packet["side"] = "right"
    display.update(packet, 10.0)
    packet = current_packet()
    packet["current_limit_ma"] = [0.0] * 8
    display.update(packet, 10.0)
    assert display.packet is None
