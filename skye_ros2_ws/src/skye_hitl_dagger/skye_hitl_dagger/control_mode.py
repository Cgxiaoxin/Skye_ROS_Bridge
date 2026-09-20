from enum import Enum, auto


class ControlModeState(Enum):
    AUTONOMOUS = auto()
    HANDOVER_SYNC = auto()
    HUMAN = auto()


class ControlArbiterLogic:
    def __init__(self) -> None:
        self._mode = ControlModeState.AUTONOMOUS
        self._teleop_requested = False

    def mode(self) -> ControlModeState:
        return self._mode

    def teleop_requested(self) -> bool:
        return self._teleop_requested

    def active_source(self) -> str:
        if self._mode == ControlModeState.AUTONOMOUS:
            return "policy"
        if self._mode == ControlModeState.HANDOVER_SYNC:
            return "hold"
        return "teleop"

    def request_takeover(self) -> bool:
        """Enter sync-only handover; does not request teleop yet."""
        if self._mode != ControlModeState.AUTONOMOUS:
            return False
        self._mode = ControlModeState.HANDOVER_SYNC
        self._teleop_requested = False
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
                ControlModeState.HUMAN, ControlModeState.HANDOVER_SYNC):
            return False
        self._mode = ControlModeState.AUTONOMOUS
        self._teleop_requested = False
        return True
