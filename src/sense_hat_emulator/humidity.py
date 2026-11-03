"""
Simulated humidity and the temperature measured by the humidity sensor.
Author: Samir Rathore
"""

from random import Random
from .hardware_profile import SENSOR_RATES_HZ


class HumiditySensor:

    # The emulator calls sample() on this fixed clock, only when noise is enabled.
    period = 1 / SENSOR_RATES_HZ['humidity']  # HTS221 CTRL_REG1=0x87.
    keys = ('humidity', 'temperature_humidity')

    def __init__(self, seed):
        self.random = Random(seed)

    def sample(self, environment):
        """Return one reading from the current simulated environmental values."""
        true_humidity = environment['humidity']
        true_temperature = environment['temperature_humidity']

        # Random noise (need to quantify)
        if 20 <= true_humidity <= 80:
            humidity_noise_std = 0.7  # Percentage points of relative humidity.
        else:
            humidity_noise_std = 1.0

        if 15 <= true_temperature <= 40:
            temperature_noise_std = 0.1  # deg C
        elif 0 <= true_temperature <= 60:
            temperature_noise_std = 0.2
        else:
            temperature_noise_std = 0.4

        humidity_noise = self.random.gauss(0, humidity_noise_std)
        temperature_noise = self.random.gauss(0, temperature_noise_std)
        humidity_reading = true_humidity + humidity_noise
        temperature_reading = true_temperature + temperature_noise

        # Limit the readings to the modeled sensor ranges.
        humidity_reading = min(100, max(0, humidity_reading))
        temperature_reading = min(120, max(-40, temperature_reading))

        return {
            'humidity': humidity_reading,
            'temperature_humidity': temperature_reading,
        }
