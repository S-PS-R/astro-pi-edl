"""Mission state, kept separate from raw sensor measurements."""

from dataclasses import dataclass
from ..shared.phases import Phase
from time import monotonic
from threading import Lock


class SimulationClock:
    """Accumulate mission time continuously across live speed changes."""

    def __init__(self, time_scale):
        self._lock = Lock()
        self._mission_time_s = 0.0
        self._last_real_time = monotonic()
        self._speed = time_scale

    def _advance(self, now):
        # Monotonic mission time
        if now > self._last_real_time:
            self._mission_time_s += (now - self._last_real_time) * self._speed
            self._last_real_time = now

    def reset(self, now):
        with self._lock:
            self._mission_time_s = 0.0
            self._last_real_time = now

    def snapshot(self, real_time):
        with self._lock:
            self._advance(real_time)
            return self._mission_time_s, self._speed

    def set_scale(self, scale):
        with self._lock:
            self._advance(monotonic())
            self._speed = scale

@dataclass
class MissionState:
    altitude_m: float
    velocity_m_s: float
    flight_path_angle_rad: float
    mass: float
    planet: dict
    phase: Phase = Phase.INITIALIZING
    elapsed_s: float = 0.0
    mission_started_at: float = 0.0


def initialize(altitude_m, velocity_m_s, flight_path_angle_rad, mass_kg, planet, phase=Phase.ENTRY):
    state = MissionState(
        altitude_m=altitude_m,
        velocity_m_s=velocity_m_s,
        flight_path_angle_rad=flight_path_angle_rad,
        mass=mass_kg,
        planet=planet,
        phase=phase,
        mission_started_at = monotonic()
    )
    return state
