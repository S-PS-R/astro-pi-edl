"""
Independent X/Y/Z IMU noise, in public API units: g, rad/s, and µT.
Author: Samir Rathore
"""

from math import radians
from random import Random
from .humidity import SampleChannel

# LSM9DS1 maximum full-scale settings; datasheet Table 3 sensitivities.
ACCEL_LIMIT = 16.0
MAG_LIMIT = 1600.0  # 16 gauss, expressed in microtesla
GYRO_LIMIT = radians(2000)

class IMUSensor:
    """LSM9DS1 accelerometer and gyro at their maximum normal-mode ODR."""
    period = 1 / 952
    specifications = (
        ('accel', -ACCEL_LIMIT, ACCEL_LIMIT, 1 / 0.000732, 0.1),
        ('gyro', -GYRO_LIMIT, GYRO_LIMIT, 1 / radians(0.070), radians(1)),
    )

    def __init__(self, seed, targets):
        self.random = Random(seed)
        self.channels = {}
        # Maximum-range sensitivities: 0.732 mg, 70 mdps, 0.58 mgauss per LSB.
        # Preserve the existing synthetic noise parameters.
        for prefix, low, high, factor, error in self.specifications:
            for axis in 'xyz':
                key = f'{prefix}_{axis}'
                self.channels[key] = (SampleChannel(targets[key], 10, low, high, factor), error)

    def sample(self, targets):
        return {key: channel.sample(targets[key], error, self.random)
                for key, (channel, error) in self.channels.items()}


class MagnetometerSensor(IMUSensor):
    """Independent magnetic clock; highest standard ODR with FAST_ODR off."""
    period = 1 / 80
    specifications = (('mag', -MAG_LIMIT, MAG_LIMIT, 1 / 0.058, 200.0),)
