"""Touchdown flag from IMU magnitude."""

from __future__ import annotations

TOUCHDOWN_G = 2.2
QUIET_G = 1.15


def detect_touchdown(accel: tuple[float, float, float]) -> bool:
    ax, ay, az = accel
    return (ax * ax + ay * ay + az * az) ** 0.5 >= TOUCHDOWN_G


def accel_magnitude(accel: tuple[float, float, float]) -> float:
    ax, ay, az = accel
    return (ax * ax + ay * ay + az * az) ** 0.5


def is_quiet(accel: tuple[float, float, float]) -> bool:
    return accel_magnitude(accel) <= QUIET_G
