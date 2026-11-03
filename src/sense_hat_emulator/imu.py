"""
Independent X/Y/Z IMU noise, in public API units: g, rad/s, and µT.
Author: Samir Rathore
"""

from math import radians
from random import Random
from .humidity import SampleChannel
from .hardware_profile import SENSOR_RATES_HZ

# RTIMULib defaults: +/-8 g, +/-500 deg/s, +/-4 gauss.
ACCEL_LIMIT = 8.0
MAG_LIMIT = 400.0  # 4 gauss, expressed in microtesla
GYRO_LIMIT = radians(500)

class IMUSensor:
    """Independent axes on the fixed 119 Hz acceleration/gyro output clock."""
    period = 1 / SENSOR_RATES_HZ['accel']
    specifications = (
        ('accel', -ACCEL_LIMIT, ACCEL_LIMIT, 1 / 0.000244, 0.1),
        ('gyro', -GYRO_LIMIT, GYRO_LIMIT, 1 / radians(0.0175), radians(1)),
    )

    def __init__(self, seed, targets):
        self.random = Random(seed)
        self.channels = {}
        # Default sensitivities: 0.244 mg, 17.5 mdps, 0.14 mgauss per LSB.
        # Preserve the existing synthetic noise parameters.
        for prefix, low, high, factor, error in self.specifications:
            for axis in 'xyz':
                key = f'{prefix}_{axis}'
                self.channels[key] = (SampleChannel(targets[key], 10, low, high, factor), error)

    def sample(self, targets):
        return {key: channel.sample(targets[key], error, self.random)
                for key, (channel, error) in self.channels.items()}

    @property
    def keys(self):
        return tuple(self.channels)


class MagnetometerSensor(IMUSensor):
    """Independent magnetic output clock at the fixed 20 Hz default."""
    period = 1 / SENSOR_RATES_HZ['mag']
    specifications = (('mag', -MAG_LIMIT, MAG_LIMIT, 1 / 0.014, 200.0),)
