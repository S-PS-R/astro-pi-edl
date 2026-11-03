"""Check initial conditions and sensors, then begin Phase.ENTRY."""

from math import isfinite, pi, radians
from time import monotonic
from .sensors import SensorReader
from .state import MissionState, Phase


def initialize(sense, altitude_m=120000.0, velocity_m_s=7800.0,
               flight_path_angle_rad=radians(-6.5)):
    # Initial altitude and speed describe the simulated entry, not sensor readings.
    if not isfinite(altitude_m) or altitude_m <= 0 or altitude_m > 150000:
        raise ValueError('Entry altitude must be finite and in (0, 150000] metres')
    if not isfinite(velocity_m_s) or velocity_m_s <= 0:
        raise ValueError('Entry velocity must be finite and positive')
    if not isfinite(flight_path_angle_rad) or not -pi / 2 <= flight_path_angle_rad < 0:
        raise ValueError('Entry FPA must be finite and in [-pi/2, 0) radians')

    state = MissionState(
        altitude_m=altitude_m,
        velocity_m_s=velocity_m_s,
        flight_path_angle_rad=flight_path_angle_rad,
    )

    # All sensor groups are due at time zero, so this reads each one once.
    reader = SensorReader(sense)
    readings = reader.read(0.0)
    for name, value in readings.items():
        if not isfinite(value) or value == -1e6:
            raise ValueError(f'Invalid initial {name} reading')
    if readings['pressure'] <= 0:
        raise ValueError('Initial pressure must be positive')
    if readings['humidity'] < 0 or readings['humidity'] > 100:
        raise ValueError('Initial humidity must be between 0 and 100 percent')
    state.initial_measurements = readings

    # Add any other readiness checks here, before starting the mission clock.
    state.mission_started_at = monotonic()
    state.phase = Phase.ENTRY
    return state
