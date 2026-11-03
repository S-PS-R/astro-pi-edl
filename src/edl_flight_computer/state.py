"""Shared state for initialization and the start of entry."""

from dataclasses import dataclass, field
from enum import Enum


class Phase(Enum):
    INITIALIZING = "initializing"
    ENTRY = "entry"


@dataclass
class MissionState:
    # Simulated entry conditions, separate from physical sensor measurements.
    altitude_m: float
    velocity_m_s: float
    flight_path_angle_rad: float
    phase: Phase = Phase.INITIALIZING
    elapsed_s: float = 0.0
    mission_started_at: float | None = None
    initial_measurements: dict[str, float] = field(default_factory=dict)
