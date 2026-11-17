"""Read board sensors and poll each group at its fixed nominal rate."""

from math import floor, isfinite
from sense_hat_emulator.hardware_profile import SENSOR_RATES_HZ
from sense_hat_emulator.emulator import RANGES

SENSOR_CHANNELS = ('accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y', 'gyro_z',
                   'mag_x', 'mag_y', 'mag_z', 'pressure', 'temperature_pressure',
                   'temperature_humidity', 'humidity')

class SensorReader:
    """Keep previous readings until each sensor group's next polling time.

    now is seconds since the acquisition loop started. These are software poll
    times; they do not guarantee delivery of every physical hardware sample.
    """

    def __init__(self, sense):
        self.sense = sense
        self.readings = {}
        self.sample_times_s = {}
        self.updated_groups = set()
        self.errors = {}
        self._groups = {
            'imu': (SENSOR_RATES_HZ['accel'], self._read_imu),
            'mag': (SENSOR_RATES_HZ['mag'], self._read_magnetic),
            'pressure': (SENSOR_RATES_HZ['pressure'], self._read_pressure),
            'humidity': (SENSOR_RATES_HZ['humidity'], self._read_humidity),
        }
        self._next_read_s = {group: 0.0 for group in self._groups}

    def read(self, now, *, recover_errors=False):
        """Poll due groups and return cached values for every other group.

        sample_times_s records software poll times, not chip timestamps.
        """
        self.updated_groups.clear()
        for group, (rate, read_group) in self._groups.items():
            # Due sensor groups
            if now + 1e-10 < self._next_read_s[group]:
                continue
            # Next poll time
            self._next_read_s[group] = (floor((now + 1e-10) * rate) + 1) / rate
            # Read and validate
            try:
                group_readings = read_group()
                if recover_errors:
                    for name, value in group_readings.items():
                        lower, upper = RANGES[name]
                        if not isfinite(value) or not lower <= value <= upper:
                            raise ValueError(f'{name} outside sensor range')
            except (OSError, RuntimeError, ValueError) as error:
                if not recover_errors:
                    raise
                self.errors[group] = str(error)
                continue
            # Cache successful readings
            self.readings.update(group_readings)
            self.sample_times_s[group] = now
            self.updated_groups.add(group)
            self.errors.pop(group, None)
        return self.readings.copy()

    def _read_imu(self):
        acceleration = self.sense.get_accelerometer_raw()
        gyroscope = self.sense.get_gyroscope_raw()
        return {
            **{f'accel_{axis}': acceleration[axis] for axis in 'xyz'},
            **{f'gyro_{axis}': gyroscope[axis] for axis in 'xyz'},
        }

    def _read_magnetic(self):
        magnetic_field = self.sense.get_compass_raw()
        return {f'mag_{axis}': magnetic_field[axis] for axis in 'xyz'}

    def _read_pressure(self):
        return {
            'pressure': self.sense.get_pressure(),
            'temperature_pressure': self.sense.get_temperature_from_pressure(),
        }

    def _read_humidity(self):
        return {
            'humidity': self.sense.get_humidity(),
            'temperature_humidity': self.sense.get_temperature_from_humidity(),
        }
