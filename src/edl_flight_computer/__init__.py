"""C&DH software for the Pi 5 / Sense HAT V2 EDL stack."""

from .hardware import get_hat, is_simulated
from .sensors import (read_acceleration, read_gyroscope, read_humidity,
                      read_magnetic_field, read_pressure, read_temperature)

__all__ = ['get_hat', 'is_simulated', 'read_acceleration', 'read_gyroscope',
           'read_humidity', 'read_magnetic_field', 'read_pressure', 'read_temperature']
