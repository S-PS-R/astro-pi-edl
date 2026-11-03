"""Simulated pressure and the temperature measured by the pressure sensor."""

from random import Random
from .hardware_profile import SENSOR_RATES_HZ


class PressureSensor:
    period = 1 / SENSOR_RATES_HZ['pressure']
    keys = ('pressure', 'temperature_pressure')

    def __init__(self, seed):
        self.random = Random(seed)

    def sample(self, environment):
        true_pressure = environment['pressure']
        true_temperature = environment['temperature_pressure']

        # Synthetic noise standard deviations; these still need calibration.
        if 800 <= true_pressure <= 1100 and 20 <= true_temperature <= 60:
            pressure_noise_std = 0.04  # hPa
        else:
            pressure_noise_std = 0.2

        if 0 <= true_temperature <= 65:
            temperature_noise_std = 0.4  # Degrees Celsius.
        else:
            temperature_noise_std = 0.8

        pressure_noise = self.random.gauss(0, pressure_noise_std)
        temperature_noise = self.random.gauss(0, temperature_noise_std)
        pressure_reading = true_pressure + pressure_noise
        temperature_reading = true_temperature + temperature_noise

        pressure_reading = min(1260, max(260, pressure_reading))
        temperature_reading = min(105, max(-30, temperature_reading))

        return {
            'pressure': pressure_reading,
            'temperature_pressure': temperature_reading,
        }
