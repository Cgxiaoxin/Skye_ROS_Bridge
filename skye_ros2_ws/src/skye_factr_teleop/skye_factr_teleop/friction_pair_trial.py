# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

"""Pure evidence collector for one off/on static-friction test session."""

import math


class _PhaseSamples:
    def __init__(self):
        self.start_position = None
        self.end_position = None
        self.command_samples = 0
        self.present_samples = 0
        self.positive_peak_ma = 0.0
        self.negative_peak_ma = 0.0
        self.other_peak_ma = 0.0
        self.target_present_min_ma = None
        self.target_present_max_ma = None
        self.above_threshold_samples = 0

    def command(self, values, index):
        target = values[index]
        self.command_samples += 1
        self.positive_peak_ma = max(self.positive_peak_ma, target)
        self.negative_peak_ma = min(self.negative_peak_ma, target)
        self.other_peak_ma = max(self.other_peak_ma,
                                 *(abs(v) for i, v in enumerate(values) if i != index))

    def present(self, values, index):
        target = values[index]
        self.present_samples += 1
        self.target_present_min_ma = (target if self.target_present_min_ma is None
                                      else min(self.target_present_min_ma, target))
        self.target_present_max_ma = (target if self.target_present_max_ma is None
                                      else max(self.target_present_max_ma, target))

    def as_dict(self):
        return {
            "start_position_rad": self.start_position,
            "end_position_rad": self.end_position,
            "command_samples": self.command_samples,
            "present_current_samples": self.present_samples,
            "positive_peak_command_ma": self.positive_peak_ma,
            "negative_peak_command_ma": self.negative_peak_ma,
            "other_ids_peak_command_ma": self.other_peak_ma,
            "target_present_current_min_ma": self.target_present_min_ma,
            "target_present_current_max_ma": self.target_present_max_ma,
            "above_threshold_state_samples": self.above_threshold_samples,
        }


class FrictionPairTrial:
    def __init__(self, joint_id, current_limit_ma, enable_speed):
        self.joint_id = joint_id
        self.current_limit_ma = current_limit_ma
        self.enable_speed = enable_speed
        self.phase = "waiting"
        self.active_phase = None
        self.phase_changed_at = None
        self.states = []
        self.phases = []
        self.issues = []
        self.last_position = None
        self.switch_off_position = None
        self.off = _PhaseSamples()
        self.on = _PhaseSamples()
        self.final_zero_observed = False

    def state(self, value, elapsed):
        self.states.append({"time_s": elapsed, "state": value})
        if value == "IDLE" and self.phase == "waiting":
            return
        if value == "FRICTION_TEST" and self.phase == "waiting":
            if not any(item["state"] == "IDLE" for item in self.states[:-1]):
                self.issues.append("FRICTION_TEST observed without initial IDLE")
            self.phase = "testing"
        elif value == "IDLE" and self.phase == "testing":
            self.phase = "finished"
            self.active_phase = None
        elif value not in ("IDLE", "FRICTION_TEST"):
            self.issues.append(f"unexpected state: {value}")

    def phase_event(self, value, elapsed):
        if value not in ("idle", "off", "on"):
            self.issues.append(f"unexpected friction phase: {value}")
            return
        self.phases.append({"time_s": elapsed, "phase": value})
        if value == "idle":
            self.active_phase = None
        elif self.phase != "finished":
            if (value == "on" and self.active_phase == "off"
                    and self.on.start_position is None):
                self.switch_off_position = (list(self.last_position)
                                            if self.last_position is not None else None)
            self.active_phase = value
            self.phase_changed_at = elapsed
            samples = self.off if value == "off" else self.on
            if value == "off" and samples.start_position is None and self.last_position is not None:
                samples.start_position = list(self.last_position)

    def current(self, values, elapsed=None):
        if len(values) != 8 or not all(math.isfinite(v) for v in values):
            self.issues.append("invalid eight-ID commanded current")
            return
        if self.phase == "finished" and all(v == 0 for v in values):
            self.final_zero_observed = True
        if self.phase != "testing" or self.active_phase is None:
            return
        # Current and phase are separate ROS topics; exclude the boundary from
        # phase statistics while retaining every message in the trace.
        if (elapsed is not None and self.phase_changed_at is not None
                and elapsed - self.phase_changed_at < 0.1):
            return
        (self.off if self.active_phase == "off" else self.on).command(
            values, self.joint_id - 1,
        )

    def present_current(self, values, elapsed=None):
        if len(values) != 8 or not all(math.isfinite(v) for v in values):
            self.issues.append("invalid eight-ID present current")
            return
        if self.phase == "testing" and self.active_phase is not None:
            if (elapsed is not None and self.phase_changed_at is not None
                    and elapsed - self.phase_changed_at < 0.1):
                return
            (self.off if self.active_phase == "off" else self.on).present(
                values, self.joint_id - 1,
            )

    def position(self, values, velocities):
        if (len(values) != 7 or len(velocities) != 7
                or not all(math.isfinite(v) for v in (*values, *velocities))):
            self.issues.append("invalid seven-joint state")
            return
        self.last_position = list(values)
        if self.phase == "testing" and self.active_phase is not None:
            samples = self.off if self.active_phase == "off" else self.on
            if samples.start_position is None:
                samples.start_position = list(values)
            samples.end_position = list(values)
            if self.active_phase == "on":
                samples.above_threshold_samples += (
                    abs(velocities[self.joint_id - 1]) >= self.enable_speed
                )

    def summary(self, *, effect, threshold_observed, motion, end_method, note):
        issues = list(self.issues)
        states = [item["state"] for item in self.states]
        phases = [item["phase"] for item in self.phases]
        if self.phase != "finished" or not ("IDLE" in states and "FRICTION_TEST" in states):
            issues.append("IDLE -> FRICTION_TEST -> IDLE not fully observed")
        if "off" not in phases or "on" not in phases or phases.index("off") > phases.index("on"):
            issues.append("off -> on phase sequence not observed")
        if (not self.off.command_samples or not self.on.command_samples
                or not self.off.present_samples or not self.on.present_samples):
            issues.append("off/on command or present-current samples missing")
        if not self.final_zero_observed:
            issues.append("final zero command not observed")
        a, b = self.switch_off_position, self.on.start_position
        switch_pose_max_difference = (max(abs(x - y) for x, y in zip(a, b))
                                      if isinstance(a, list) and isinstance(b, list)
                                      and len(a) == len(b) == 7 else None)
        same_pose = (switch_pose_max_difference is not None
                     and switch_pose_max_difference <= 0.1)
        if not same_pose:
            issues.append("pose changed by more than 0.1 rad at first off/on switch or is missing")
        if self.on.positive_peak_ma <= 0 or self.on.negative_peak_ma >= 0:
            issues.append("both dither signs were not observed")
        if not self.on.above_threshold_samples:
            issues.append("selected joint did not reach speed threshold in on phase")
        limit_violation = (self.off.positive_peak_ma > 0 or self.off.negative_peak_ma < 0
                           or self.off.other_peak_ma > 0 or self.on.other_peak_ma > 0
                           or max(self.on.positive_peak_ma,
                                  -self.on.negative_peak_ma) > self.current_limit_ma + 1e-9)
        if limit_violation:
            issues.append("off output or on current violated target-only cap")
        if limit_violation or motion in ("sudden", "other") or effect == "worse" or threshold_observed == "no":
            assessment = "FAIL"
        elif issues:
            assessment = "INCOMPLETE"
        elif effect != "less" or threshold_observed != "yes" or motion != "none":
            assessment = "INCONCLUSIVE"
        elif end_method not in ("stop", "timeout"):
            assessment = "INCONCLUSIVE"
        else:
            assessment = "PENDING_SHUTDOWN_CHECK"
        return {
            "state_events": self.states,
            "phase_events": self.phases,
            "baseline": self.off.as_dict(),
            "active": self.on.as_dict(),
            "switch_off_position_rad": self.switch_off_position,
            "switch_pose_max_difference_rad": switch_pose_max_difference,
            "baseline_within_pose": same_pose,
            "final_zero_command_observed": self.final_zero_observed,
            "operator_effect_vs_disabled": effect,
            "operator_threshold_observed": threshold_observed,
            "operator_motion": motion,
            "operator_end_method": end_method,
            "operator_note": note,
            "issues": issues,
            "assessment": assessment,
        }
