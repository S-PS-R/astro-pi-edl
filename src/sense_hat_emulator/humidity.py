"""HTS221 humidity/temperature model and a shared rolling-sample helper."""

from collections import deque
from random import Random


class SampleChannel:
    """Average N noisy samples, then clamp and quantize like a sensor register."""
    def __init__(self, default, count, low, high, factor, offset=0):
        self.count, self.low, self.high = count, low, high
        self.factor, self.offset = factor, offset
        self.reset(default)

    def reset(self, target):
        self.history = deque([target] * self.count, maxlen=self.count)
        self.value = target

    def sample(self, target, error, rng):
        self.history.append(target + rng.gauss(0, 0.2) * error)
        mean = sum(self.history) / self.count
        limited = min(self.high, max(self.low, mean))
        self.value = int((limited - self.offset) * self.factor) / self.factor + self.offset
        return self.value


class HumiditySensor:
    period = 0.13

    def __init__(self, seed, targets):
        self.random = Random(seed)
        self.humidity = SampleChannel(targets['humidity'], 10, 0, 100, 256)
        self.temperature = SampleChannel(targets['temperature_humidity'], 31, -40, 120, 64)

    def sample(self, targets):
        h, t = targets['humidity'], targets['temperature_humidity']
        h_error = 3.5 if 20 <= h <= 80 else 5.0
        t_error = 0.5 if 15 <= t <= 40 else 1.0 if 0 <= t <= 60 else 2.0
        return {
            'humidity': self.humidity.sample(h, h_error, self.random),
            'temperature_humidity': self.temperature.sample(t, t_error, self.random),
        }
