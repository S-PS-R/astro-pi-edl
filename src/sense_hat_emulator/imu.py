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
        true_accel_x = environment['accel_x']
        true_accel_y = environment['accel_y']
        true_accel_z = environment['accel_z']
        true_gyro_x = environment['gyro_x']
        true_gyro_y = environment['gyro_y']
        true_gyro_z = environment['gyro_z']

        # Synthetic noise standard deviations; these still need calibration.
        acceleration_noise_std = 0.02  # g
        gyro_noise_std = radians(0.2)  # rad/s

        accel_noise_x = self.random.gauss(0, acceleration_noise_std)
        accel_noise_y = self.random.gauss(0, acceleration_noise_std)
        accel_noise_z = self.random.gauss(0, acceleration_noise_std)
        gyro_noise_x = self.random.gauss(0, gyro_noise_std)
        gyro_noise_y = self.random.gauss(0, gyro_noise_std)
        gyro_noise_z = self.random.gauss(0, gyro_noise_std)

        accel_reading_x = true_accel_x + accel_noise_x
        accel_reading_y = true_accel_y + accel_noise_y
        accel_reading_z = true_accel_z + accel_noise_z
        gyro_reading_x = true_gyro_x + gyro_noise_x
        gyro_reading_y = true_gyro_y + gyro_noise_y
        gyro_reading_z = true_gyro_z + gyro_noise_z

        accel_reading_x = min(ACCEL_LIMIT, max(-ACCEL_LIMIT, accel_reading_x))
        accel_reading_y = min(ACCEL_LIMIT, max(-ACCEL_LIMIT, accel_reading_y))
        accel_reading_z = min(ACCEL_LIMIT, max(-ACCEL_LIMIT, accel_reading_z))
        gyro_reading_x = min(GYRO_LIMIT, max(-GYRO_LIMIT, gyro_reading_x))
        gyro_reading_y = min(GYRO_LIMIT, max(-GYRO_LIMIT, gyro_reading_y))
        gyro_reading_z = min(GYRO_LIMIT, max(-GYRO_LIMIT, gyro_reading_z))

        return {
            'accel_x': accel_reading_x,
            'accel_y': accel_reading_y,
            'accel_z': accel_reading_z,
            'gyro_x': gyro_reading_x,
            'gyro_y': gyro_reading_y,
            'gyro_z': gyro_reading_z,
        }


class MagnetometerSensor:
    period = 1 / SENSOR_RATES_HZ['mag']
    keys = ('mag_x', 'mag_y', 'mag_z')

    def __init__(self, seed):
        self.random = Random(seed)

    def sample(self, environment):
        true_mag_x = environment['mag_x']
        true_mag_y = environment['mag_y']
        true_mag_z = environment['mag_z']

        # Retained synthetic per-sample noise; not a measured hardware value.
        magnetic_noise_std = 40.0  # Microtesla; still needs calibration.
        mag_noise_x = self.random.gauss(0, magnetic_noise_std)
        mag_noise_y = self.random.gauss(0, magnetic_noise_std)
        mag_noise_z = self.random.gauss(0, magnetic_noise_std)

        mag_reading_x = true_mag_x + mag_noise_x
        mag_reading_y = true_mag_y + mag_noise_y
        mag_reading_z = true_mag_z + mag_noise_z

        mag_reading_x = min(MAG_LIMIT, max(-MAG_LIMIT, mag_reading_x))
        mag_reading_y = min(MAG_LIMIT, max(-MAG_LIMIT, mag_reading_y))
        mag_reading_z = min(MAG_LIMIT, max(-MAG_LIMIT, mag_reading_z))

        return {
            'mag_x': mag_reading_x,
            'mag_y': mag_reading_y,
            'mag_z': mag_reading_z,
        }
