"""
LPS25H pressure and its independent temperature channel.
Author: Samir Rathore
"""

from random import Random
from .humidity import SampleChannel


class PressureSensor:
    period = 0.04

    def __init__(self, seed, targets):
        self.random = Random(seed)
        self.pressure = SampleChannel(targets['pressure'], 25, 260, 1260, 4096)
        # Consistent encode/decode offset; avoid upstream's restore inconsistency.
        self.temperature = SampleChannel(targets['temperature_pressure'], 25, -30, 105, 480, 37)

    def sample(self, targets):
        p, t = targets['pressure'], targets['temperature_pressure']
        p_error = 0.2 if 800 <= p <= 1100 and 20 <= t <= 60 else 1.0
        t_error = 2.0 if 0 <= t <= 65 else 4.0
        return {
            'pressure': self.pressure.sample(p, p_error, self.random),
            'temperature_pressure': self.temperature.sample(t, t_error, self.random),
        }
