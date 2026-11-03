"""Initialize the selected board and mission state, then enter Phase.ENTRY."""

from math import isfinite, pi, radians
from time import monotonic

from .state import MissionState, Phase


def initialize(sense, altitude_m=120000.0, velocity_m_s=7800.0,
               flight_path_angle_rad=radians(-6.5)):
    """Check initial conditions and sensor availability before starting the clock.

    sense is the real board or explicitly selected emulator supplied by run.py.
    Initial sensor readings are measured values, not the simulated entry state.
    This verifies readable, finite values; it does not calibrate sensors or
    certify their accuracy. No later phase or flight calculations run here.
    """
    conditions = (altitude_m, velocity_m_s, flight_path_angle_rad)
    if not all(isfinite(value) for value in conditions):
        raise ValueError("Entry conditions must be finite")
    if not 0 < altitude_m <= 150000 or velocity_m_s <= 0:
        raise ValueError("Entry altitude must be in (0, 150000] m and velocity must be positive")
    if not -pi / 2 <= flight_path_angle_rad < 0:
        raise ValueError("Entry FPA must be negative downward, between -pi/2 and 0 radians")

    state = MissionState(altitude_m, velocity_m_s, flight_path_angle_rad)
    readings = {
        "temperature_humidity": sense.get_temperature_from_humidity(),
        "temperature_pressure": sense.get_temperature_from_pressure(),
        "pressure": sense.get_pressure(),
        "humidity": sense.get_humidity(),
    }
    for prefix, vector in (("accel", sense.get_accelerometer_raw()),
                           ("gyro", sense.get_gyroscope_raw()),
                           ("mag", sense.get_compass_raw())):
        readings.update({f"{prefix}_{axis}": vector[axis] for axis in "xyz"})
    for name, value in readings.items():
        if not isfinite(value) or value == -1e6:  # Emulator's OVR diagnostic.
            raise ValueError(f"Initialization failed: invalid {name} reading")
    if readings["pressure"] <= 0 or not 0 <= readings["humidity"] <= 100:
        raise ValueError("Initialization failed: invalid pressure or humidity reading")

    state.initial_measurements = readings
    # TODO: Add any additional pre-entry checks you require above this point.
    # Begin mission time only after initialization succeeds.
    state.mission_started_at = monotonic()
    state.phase = Phase.ENTRY
    return state
