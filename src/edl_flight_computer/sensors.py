"""Read board sensors and poll each group at its fixed nominal rate."""

from math import floor
from sense_hat_emulator.hardware_profile import SENSOR_RATES_HZ
from .hardware import get_hat


def read_temperature():
    sense = get_hat()
    return float(sense.get_temperature_from_humidity())


def read_pressure():
    sense = get_hat()
    return float(sense.get_pressure())


def read_humidity():
    sense = get_hat()
    return float(sense.get_humidity())


def read_acceleration():
    sense = get_hat()
    acceleration = sense.get_accelerometer_raw()
    x = float(acceleration['x'])
    y = float(acceleration['y'])
    z = float(acceleration['z'])
    return x, y, z


def read_gyroscope():
    sense = get_hat()
    gyroscope = sense.get_gyroscope_raw()
    x = float(gyroscope['x'])
    y = float(gyroscope['y'])
    z = float(gyroscope['z'])
    return x, y, z


def read_magnetic_field():
    sense = get_hat()
    magnetic_field = sense.get_compass_raw()
    x = float(magnetic_field['x'])
    y = float(magnetic_field['y'])
    z = float(magnetic_field['z'])
    return x, y, z


class SensorReader:
    """Keep previous readings until each sensor group's next polling time.

    now is seconds since the acquisition loop started. These are software poll
    times; they do not guarantee delivery of every physical hardware sample.
    """

    def __init__(self, sense):
        self.sense = sense
        self.readings = {}
        self.next_acceleration_time_s = 0.0
        self.next_magnetic_time_s = 0.0
        self.next_pressure_time_s = 0.0
        self.next_humidity_time_s = 0.0

    def read(self, now):
        # A tiny tolerance avoids missing a tick because of floating-point rounding.
        if now + 1e-10 >= self.next_acceleration_time_s:
            acceleration = self.sense.get_accelerometer_raw()
            self.readings['accel_x'] = acceleration['x']
            self.readings['accel_y'] = acceleration['y']
            self.readings['accel_z'] = acceleration['z']

            gyroscope = self.sense.get_gyroscope_raw()
            self.readings['gyro_x'] = gyroscope['x']
            self.readings['gyro_y'] = gyroscope['y']
            self.readings['gyro_z'] = gyroscope['z']

            rate = SENSOR_RATES_HZ['accel']
            self.next_acceleration_time_s = (floor((now + 1e-10) * rate) + 1) / rate

        if now + 1e-10 >= self.next_magnetic_time_s:
            magnetic_field = self.sense.get_compass_raw()
            self.readings['mag_x'] = magnetic_field['x']
            self.readings['mag_y'] = magnetic_field['y']
            self.readings['mag_z'] = magnetic_field['z']

            rate = SENSOR_RATES_HZ['mag']
            self.next_magnetic_time_s = (floor((now + 1e-10) * rate) + 1) / rate

        if now + 1e-10 >= self.next_pressure_time_s:
            self.readings['pressure'] = self.sense.get_pressure()
            self.readings['temperature_pressure'] = self.sense.get_temperature_from_pressure()

            rate = SENSOR_RATES_HZ['pressure']
            self.next_pressure_time_s = (floor((now + 1e-10) * rate) + 1) / rate

        if now + 1e-10 >= self.next_humidity_time_s:
            self.readings['humidity'] = self.sense.get_humidity()
            self.readings['temperature_humidity'] = self.sense.get_temperature_from_humidity()

            rate = SENSOR_RATES_HZ['humidity']
            self.next_humidity_time_s = (floor((now + 1e-10) * rate) + 1) / rate

        return self.readings.copy()
