"""HAT sensors: HTS221 temperature, LSM9DS1 IMU, LPS25HB barometer."""

from __future__ import annotations

from .hat import get_hat


def read_temperature() -> float:
    return float(get_hat().get_temperature_from_humidity())


def read_acceleration() -> tuple[float, float, float]:
    raw = get_hat().get_accelerometer_raw()
    return (float(raw["x"]), float(raw["y"]), float(raw["z"]))


def read_pressure() -> float:
    return float(get_hat().get_pressure())


read_aux = read_pressure
