"""HAT sensors: HTS221 temperature, LSM9DS1 IMU, LPS25HB barometer."""

from __future__ import annotations

from .hardware import get_hat


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
