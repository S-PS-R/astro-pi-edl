"""Read board sensors and poll each group at its fixed nominal rate."""

from math import floor
from sense_hat_emulator.hardware_profile import SENSOR_RATES_HZ
from .hardware import get_hat

SENSOR_GROUP_CHANNELS = {
    'imu': ('accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y', 'gyro_z'),
    'mag': ('mag_x', 'mag_y', 'mag_z'),
    'pressure': ('pressure', 'temperature_pressure'),
    'humidity': ('humidity', 'temperature_humidity'),
}

class SensorReader:
    """Keep previous readings until each sensor group's next polling time.

    now is seconds since the acquisition loop started. These are software poll
    times; they do not guarantee delivery of every physical hardware sample.
    """

    def __init__(self, sense, initial_readings=None):
        self.sense = sense
        self.readings = dict(initial_readings) if initial_readings is not None else {}
        self.updated_groups = set()
        self.errors = {}
        self.sample_times_s = {}
        self._groups = {
            'imu': (SENSOR_RATES_HZ['accel'], self._read_imu),
            'mag': (SENSOR_RATES_HZ['mag'], self._read_magnetic),
            'pressure': (SENSOR_RATES_HZ['pressure'], self._read_pressure),
            'humidity': (SENSOR_RATES_HZ['humidity'], self._read_humidity),
        }
        self._next_read_s = {group: 0.0 for group in self._groups}

    def read(self, now, *, recover_errors=False):
        """Poll due groups and return cached values for every other group.

        updated_groups contains successful polls from this call only. errors
        persists per group until a later successful poll. With recover_errors,
        a failed group retains its previous readings while other groups continue.
        sample_times_s records successful software poll times, not chip timestamps.
        """
        self.updated_groups.clear()
        for group, (rate, read_group) in self._groups.items():
            # A tolerance avoids missing a tick due to floating-point rounding.
            if now + 1e-10 < self._next_read_s[group]:
                continue
            # Advance even on failure and retry at the next nominal group tick.
            # Skip missed ticks rather than inventing samples to catch up.
            self._next_read_s[group] = (floor((now + 1e-10) * rate) + 1) / rate
            try:
                group_readings = read_group()
            except (OSError, RuntimeError) as error:
                self.errors[group] = str(error)
                if not recover_errors:
                    raise
            else:
                # Commit the whole group only after all its getters succeed.
                self.readings.update(group_readings)
                self.updated_groups.add(group)
                self.sample_times_s[group] = now
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
