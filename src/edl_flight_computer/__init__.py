"""Public sensor access functions for the EDL flight computer."""

from .shared.hardware import get_hat, is_simulated
from .shared.sensors import SensorReader

__all__ = [
    "get_hat",
    "is_simulated",
    "SensorReader",
]
