from enum import Enum, auto


class ControlModeState(Enum):
    AUTONOMOUS = auto()
    HANDOVER_SYNC = auto()
    HUMAN = auto()


class ControlArbiterLogic:
    def __init__(self) -> None:
        self._mode = ControlModeState.AUTONOMOUS

    def mode(self) -> ControlModeState:
        return self._mode

    def active_source(self) -> str:
        if self._mode == ControlModeState.AUTONOMOUS:
            return "policy"
        if self._mode == ControlModeState.HANDOVER_SYNC:
            return "hold"
        return "teleop"

    def request_takeover(self) -> bool:
        if self._mode != ControlModeState.AUTONOMOUS:
            return False
        self._mode = ControlModeState.HANDOVER_SYNC
        return True

    def sync_completed(self) -> bool:
        if self._mode != ControlModeState.HANDOVER_SYNC:
            return False
        self._mode = ControlModeState.HUMAN
        return True

    def request_return(self) -> bool:
        if self._mode != ControlModeState.HUMAN:
            return False
        self._mode = ControlModeState.AUTONOMOUS
        return True
