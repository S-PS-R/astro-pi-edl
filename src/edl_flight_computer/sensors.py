"""HAT sensors: HTS221 temperature, LSM9DS1 IMU, LPS25HB barometer."""

from __future__ import annotations

from .hardware import get_hat
from sense_hat_emulator.hardware_profile import SENSOR_RATES_HZ


def read_temperature() -> float:
    return float(get_hat().get_temperature_from_humidity())


def read_acceleration() -> tuple[float, float, float]:
    raw = get_hat().get_accelerometer_raw()
    return (float(raw["x"]), float(raw["y"]), float(raw["z"]))


def read_pressure() -> float:
    return float(get_hat().get_pressure())


def read_humidity() -> float:
    return float(get_hat().get_humidity())


def read_gyroscope() -> tuple[float, float, float]:
    raw = get_hat().get_gyroscope_raw()
    return tuple(float(raw[axis]) for axis in 'xyz')


def read_magnetic_field() -> tuple[float, float, float]:
    raw = get_hat().get_compass_raw()
    return tuple(float(raw[axis]) for axis in 'xyz')


class SensorReader:
    """Poll independent groups at their fixed nominal rates, retaining values."""

    def __init__(self, sense):
        self.sense = sense
        self.readings = {}
        self.next_due = {group: 0.0 for group in ('accel', 'mag', 'pressure', 'humidity')}

    def read(self, now):
        from math import floor
        for group in self.next_due:

            if now + 1e-10 < self.next_due[group]:
                continue
            if group == 'accel':
                for prefix, getter in (('accel', self.sense.get_accelerometer_raw),
                                       ('gyro', self.sense.get_gyroscope_raw)):
                    vector = getter()
                    self.readings.update({f'{prefix}_{axis}': vector[axis] for axis in 'xyz'})

            elif group == 'mag':
                vector = self.sense.get_compass_raw()
                self.readings.update({f'mag_{axis}': vector[axis] for axis in 'xyz'})

            elif group == 'pressure':
                self.readings.update(pressure=self.sense.get_pressure(), temperature_pressure=self.sense.get_temperature_from_pressure())

            else:
                self.readings.update(humidity=self.sense.get_humidity(), temperature_humidity=self.sense.get_temperature_from_humidity())

            rate = SENSOR_RATES_HZ[group]
            self.next_due[group] = (floor((now + 1e-10) * rate) + 1) / rate
            
        return self.readings.copy()
