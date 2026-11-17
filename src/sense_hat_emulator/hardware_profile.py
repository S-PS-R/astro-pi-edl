"""Fixed nominal output profile for the upstream Sense HAT/RTIMULib defaults.

Sources and register interpretations: docs/sensor_timing.md. There are no
runtime rate settings. These are chip output rates, not Python loop deadlines.
"""

from types import MappingProxyType

# Sensor rates
SENSOR_RATES_HZ = MappingProxyType({
    'humidity': 12.5, 'pressure': 25.0,
    'accel': 119.0, 'gyro': 119.0, 'mag': 20.0,
})

# Driver settings
EXPECTED_IMU_SETTINGS = MappingProxyType({
    'LSM9DS1GyroSampleRate': 2, 'LSM9DS1AccelSampleRate': 3,
    'LSM9DS1CompassSampleRate': 5,
    'LSM9DS1GyroFsr': 1, 'LSM9DS1AccelFsr': 3, 'LSM9DS1CompassFsr': 0,
})
