"""Independent acceleration, gyroscope and magnetic readings in board axes."""

from math import radians
from random import Random
from .hardware_profile import SENSOR_RATES_HZ

# Fixed RTIMULib default ranges, expressed in the public API units.
ACCEL_LIMIT = 8.0             # g
GYRO_LIMIT = radians(500)    # rad/s
MAG_LIMIT = 400.0            # Microtesla (4 gauss).


class IMUSensor:
    period = 1 / SENSOR_RATES_HZ['accel']
    keys = ('accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y', 'gyro_z')

    def __init__(self, seed):
        self.random = Random(seed)

    def sample(self, environment):
        # Sensor noise and limits
        readings = {}
        groups = (('accel', 0.02, ACCEL_LIMIT), ('gyro', radians(0.2), GYRO_LIMIT))
        for prefix, noise_std, limit in groups:
            for axis in 'xyz':
                name = prefix + '_' + axis
                value = environment[name] + self.random.gauss(0, noise_std)
                readings[name] = min(limit, max(-limit, value))
        return readings


class MagnetometerSensor:
    period = 1 / SENSOR_RATES_HZ['mag']
    keys = ('mag_x', 'mag_y', 'mag_z')

    def __init__(self, seed):
        self.random = Random(seed)

    def sample(self, environment):
        # Magnetic noise and limits
        readings = {}
        for axis in 'xyz':
            name = 'mag_' + axis
            value = environment[name] + self.random.gauss(0, 40.0)
            readings[name] = min(MAG_LIMIT, max(-MAG_LIMIT, value))
        return readings
