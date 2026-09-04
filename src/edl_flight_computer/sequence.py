"""EDL event timeline. Replace DEFAULT_PHASES with the mission profile."""

from __future__ import annotations

from dataclasses import dataclass, field
from time import monotonic
from typing import Sequence

DEFAULT_PHASES: Sequence[str] = (
    "preentry",
    "entry",
    "peak_heating",
    "parachute",
    "terminal_descent",
    "touchdown",
)


@dataclass
class EDLState:
    phases: tuple[str, ...] = DEFAULT_PHASES
    index: int = 0
    phase_started_at: float = field(default_factory=monotonic)
    started: bool = False

    @property
    def phase(self) -> str:
        return self.phases[self.index]


_state: EDLState | None = None


def start_edl(phases: Sequence[str] | None = None) -> EDLState:
    global _state
    _state = EDLState(
        phases=tuple(phases) if phases else DEFAULT_PHASES,
        index=0,
        phase_started_at=monotonic(),
        started=True,
    )
    return _state


def _require() -> EDLState:
    if _state is None or not _state.started:
        raise RuntimeError("start_edl() first")
    return _state


def advance_phase() -> str:
    state = _require()
    if state.index >= len(state.phases) - 1:
        return state.phase
    state.index += 1
    state.phase_started_at = monotonic()
    return state.phase


def phase_elapsed() -> float:
    state = _require()
    return monotonic() - state.phase_started_at


def run_terminal_descent() -> str:
    state = _require()
    try:
        state.index = state.phases.index("terminal_descent")
    except ValueError:
        state.index = max(0, len(state.phases) - 2)
    state.phase_started_at = monotonic()
    return state.phase
