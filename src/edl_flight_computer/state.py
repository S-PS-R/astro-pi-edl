"""Simulated mission state, kept separate from measured sensor values."""

from dataclasses import dataclass, field
from enum import Enum


class Phase(Enum):
    INITIALIZING = 'initializing'
    ENTRY = 'entry'


@dataclass
class MissionState:
    altitude_m: float
    velocity_m_s: float
    flight_path_angle_rad: float
    phase: Phase = Phase.INITIALIZING
    elapsed_s: float = 0.0
    # Absolute monotonic-clock time, set only after readiness checks succeed.
    mission_started_at: float | None = None
    # Each mission receives its own dictionary of initial sensor readings.
    initial_measurements: dict[str, float] = field(default_factory=dict)
