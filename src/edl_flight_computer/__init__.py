"""Public sensor access functions for the EDL flight computer."""

from .hardware import get_hat, is_simulated
from .sensors import SENSOR_GROUP_CHANNELS, SensorReader

__all__ = [
    "get_hat",
    "is_simulated",
    "SENSOR_GROUP_CHANNELS",
    "SensorReader",
]