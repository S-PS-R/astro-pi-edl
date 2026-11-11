"""Check initial conditions and sensors, then begin Phase.ENTRY."""

from math import isfinite, pi, radians
from time import monotonic
from .sensors import SensorReader
from .state import MissionState, Phase


def initialize(sense, altitude_m, velocity_m_s, flight_path_angle_rad, mass_kg):
    # Initial altitude and speed describe the simulated entry, not sensor readings.

    state = MissionState(
        altitude_m=altitude_m,
        velocity_m_s=velocity_m_s,
        flight_path_angle_rad=flight_path_angle_rad,
        mass=mass_kg,
    )

    # All sensor groups are due at time zero, so this reads each one once.
    reader = SensorReader(sense)
    state.initial_measurements = reader.read(0.0)
    state.mission_started_at = monotonic()
    state.phase = Phase.ENTRY
    return state
