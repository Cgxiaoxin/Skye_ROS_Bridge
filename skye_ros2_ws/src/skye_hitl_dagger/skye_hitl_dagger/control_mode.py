from enum import Enum, auto
from typing import Optional


class ControlModeState(Enum):
    AUTONOMOUS = auto()  # operator table: IDLE (policy source)
    HANDOVER_SYNC = auto()  # SYNCING / SYNCED waiting for enter_teleop
    HUMAN = auto()  # TELEOP (relative)
    FAULT = auto()


class ControlArbiterLogic:
    def __init__(self) -> None:
        self._mode = ControlModeState.AUTONOMOUS
        self._teleop_requested = False
        self._fault_reason: Optional[str] = None

    def mode(self) -> ControlModeState:
        return self._mode

    def teleop_requested(self) -> bool:
        return self._teleop_requested

    def fault_reason(self) -> Optional[str]:
        return self._fault_reason

    def active_source(self) -> str:
        if self._mode == ControlModeState.AUTONOMOUS:
            return "policy"
        if self._mode == ControlModeState.HANDOVER_SYNC:
            return "hold"
        if self._mode == ControlModeState.FAULT:
            return "fault"
        return "teleop"

    def request_takeover(self) -> bool:
        """Enter sync-only handover; does not request teleop yet."""
        if self._mode != ControlModeState.AUTONOMOUS:
            return False
        self._mode = ControlModeState.HANDOVER_SYNC
        self._teleop_requested = False
        self._fault_reason = None
        return True

    def request_enter_teleop(self) -> bool:
        """Operator confirmed sync; allow switch_teleop → HUMAN."""
        if self._mode != ControlModeState.HANDOVER_SYNC:
            return False
        if self._teleop_requested:
            return False
        self._teleop_requested = True
        return True

    def sync_completed(self) -> bool:
        if self._mode != ControlModeState.HANDOVER_SYNC:
            return False
        if not self._teleop_requested:
            return False
        self._mode = ControlModeState.HUMAN
        self._teleop_requested = False
        return True

    def request_return(self) -> bool:
        if self._mode not in (
                ControlModeState.HUMAN,
                ControlModeState.HANDOVER_SYNC,
                ControlModeState.FAULT):
            return False
        self._mode = ControlModeState.AUTONOMOUS
        self._teleop_requested = False
        self._fault_reason = None
        return True

    def request_stop(self) -> bool:
        """Any state → AUTONOMOUS (IDLE). Idempotent; always accepted."""
        self._mode = ControlModeState.AUTONOMOUS
        self._teleop_requested = False
        self._fault_reason = None
        return True

    def report_sync_fault(self, reason: str) -> bool:
        """Deterministic or exhausted sync failure during handover."""
        if self._mode != ControlModeState.HANDOVER_SYNC:
            return False
        self._mode = ControlModeState.FAULT
        self._teleop_requested = False
        self._fault_reason = reason
        return True
