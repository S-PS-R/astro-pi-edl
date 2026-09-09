"""SenseEmu and its sensor/LED storage. No GUI is needed to use this class."""

from collections.abc import Mapping

from math import isfinite, atan2, hypot, sin, cos, degrees, tau
from numbers import Real
from threading import RLock

DEFAULTS = {
    "temperature": 20.0, "pressure": 1013.25, "humidity": 0.0,
    "accel_x": 0.0, "accel_y": 0.0, "accel_z": 1.0,
    "mag_x": 25.0, "mag_y": 0.0, "mag_z": 0.0,
    "gyro_x": 0.0, "gyro_y": 0.0, "gyro_z": 0.0,
}


class EmulatorState:
    def __init__(self):
        self.lock = RLock()
        self.readings = DEFAULTS.copy()
        self.pixels = [(0, 0, 0)] * 64

    def snapshot(self):
        """Return a consistent copy, never the mutable internal dictionaries."""
        with self.lock:
            return self.readings.copy(), [list(rgb) for rgb in self.pixels]

    def update(self, changes):
        checked = {}
        for key, value in changes.items():
            if key not in DEFAULTS:
                raise KeyError(f"Unknown sensor: {key}")
            if isinstance(value, bool) or not isinstance(value, Real):
                raise TypeError(f"{key} must be a real number")
            value = float(value)
            if not isfinite(value):
                raise ValueError(f"{key} must be finite")
            if key == "humidity" and not 0 <= value <= 100:
                raise ValueError("Humidity must be between 0 and 100 percent")
            if key == "pressure" and value <= 0:
                raise ValueError("Pressure must be positive")
            if key == "temperature" and value < -273.15:
                raise ValueError("Temperature cannot be below absolute zero")
            checked[key] = value
        # Validate everything first, so a bad vector cannot partially update.
        with self.lock:
            self.readings.update(checked)

    def reset(self):
        with self.lock:
            self.readings = DEFAULTS.copy()


def validate_colour(colour):
    if not isinstance(colour, (tuple, list)) or len(colour) != 3:
        raise ValueError("RGB must be a tuple or list of three integers")
    if any(type(c) is not int or not 0 <= c <= 255 for c in colour):
        raise ValueError("RGB channels must be integers from 0 to 255")
    return (colour[0] & 0xF8, colour[1] & 0xFC, colour[2] & 0xF8)


def pixel_index(x, y):
    if type(x) is not int or type(y) is not int or not (0 <= x < 8 and 0 <= y < 8):
        raise ValueError("Pixel coordinates must be integers from 0 to 7")
    return y * 8 + x


class SenseEmu:
    """A simulated board. Pass this same object to code that should share it."""

    def __init__(self):
        self._state = EmulatorState()

    def get_readings(self):
        """Get a consistent dictionary of all 12 scalar sensor channels."""
        return self._state.snapshot()[0]

    def _get(self, name):
        with self._state.lock:
            return self._state.readings[name]

    def _set(self, name, value):
        self._state.update({name: value})

    def get_temperature(self):
        return self._get("temperature")

    def set_temperature(self, value):
        self._set("temperature", value)

    # This emulator currently has one temperature channel for both sources.
    get_temperature_from_humidity = get_temperature
    get_temperature_from_pressure = get_temperature
    set_temperature_from_humidity = set_temperature
    set_temperature_from_pressure = set_temperature

    def get_pressure(self):
        return self._get("pressure")

    def set_pressure(self, value):
        self._set("pressure", value)

    def get_humidity(self):
        return self._get("humidity")

    def set_humidity(self, value):
        self._set("humidity", value)

    def _get_vector(self, prefix):
        with self._state.lock:
            return {axis: self._state.readings[f"{prefix}_{axis}"] for axis in "xyz"}

    def _set_vector(self, prefix, x, y, z):
        if isinstance(x, Mapping):
            if y is not None or z is not None or set(x) != set("xyz"):
                raise ValueError("Supply a dictionary with exactly x, y, z")
            vector = x
        else:
            if y is None or z is None:
                raise TypeError("Supply x, y, z or a dictionary with those keys")
            vector = dict(x=x, y=y, z=z)
        self._state.update({f"{prefix}_{axis}": vector[axis] for axis in "xyz"})

    def get_accelerometer_raw(self):
        """Return x/y/z acceleration in g."""
        return self._get_vector("accel")

    def set_accelerometer_raw(self, x, y=None, z=None):
        self._set_vector("accel", x, y, z)

    def get_compass_raw(self):
        """Return x/y/z magnetic field in microteslas, not a compass heading."""
        return self._get_vector("mag")

    def set_compass_raw(self, x, y=None, z=None):
        self._set_vector("mag", x, y, z)

    def get_gyroscope_raw(self):
        """Return x/y/z angular velocity in radians per second."""
        return self._get_vector("gyro")

    def set_gyroscope_raw(self, x, y=None, z=None):
        self._set_vector("gyro", x, y, z)

    def get_orientation_radians(self):
        """Static tilt/heading estimate, not gyro fusion or flight dynamics.

        Convention: flat acceleration is +Z and magnetic north is +X.
        Uses roll/pitch to level the magnetic vector (NXP AN4248 approach).
        Raises ValueError when gravity, roll, or heading is undefined.
        """
        readings = self.get_readings()
        ax, ay, az = (readings[f"accel_{axis}"] for axis in "xyz")
        mx, my, mz = (readings[f"mag_{axis}"] for axis in "xyz")
        # Normalize first to avoid overflow from large finite API values.
        a_scale = max(abs(ax), abs(ay), abs(az))
        m_scale = max(abs(mx), abs(my), abs(mz))
        if a_scale == 0 or m_scale == 0:
            raise ValueError("Orientation needs nonzero acceleration and magnetic field")
        ax, ay, az = ax / a_scale, ay / a_scale, az / a_scale
        mx, my, mz = mx / m_scale, my / m_scale, mz / m_scale
        if hypot(ay, az) < 1e-10:
            raise ValueError("Roll/yaw are ambiguous at a vertical pitch")
        roll = atan2(ay, az)
        pitch = atan2(-ax, hypot(ay, az))
        north = mx * cos(pitch) + (my * sin(roll) + mz * cos(roll)) * sin(pitch)
        east = my * cos(roll) - mz * sin(roll)
        if hypot(north, east) < 1e-10:
            raise ValueError("Heading needs a horizontal magnetic component")
        yaw = atan2(-east, north)
        return {"pitch": pitch % tau, "roll": roll % tau, "yaw": yaw % tau}

    def get_orientation_degrees(self):
        return {axis: degrees(angle) % 360 for axis, angle in self.get_orientation_radians().items()}

    get_orientation = get_orientation_degrees
    orientation = property(get_orientation_degrees)
    orientation_radians = property(get_orientation_radians)

    # Descriptive emulator aliases; official raw names above remain available.
    get_acceleration = get_accelerometer_raw
    set_acceleration = set_accelerometer_raw
    get_magnetic_field = get_compass_raw
    set_magnetic_field = set_compass_raw

    temperature = property(get_temperature, set_temperature)
    pressure = property(get_pressure, set_pressure)
    humidity = property(get_humidity, set_humidity)

    def reset_sensors(self):
        """Restore readings, leaving LED output unchanged."""
        self._state.reset()

    def set_pixel(self, x, y, *colour):
        index = pixel_index(x, y)
        rgb = validate_colour(colour[0] if len(colour) == 1 else colour)
        with self._state.lock:
            self._state.pixels[index] = rgb

    def get_pixel(self, x, y):
        index = pixel_index(x, y)
        with self._state.lock:
            return list(self._state.pixels[index])

    def set_pixels(self, pixel_list):
        pixels = list(pixel_list)
        if len(pixels) != 64:
            raise ValueError("Provide exactly 64 RGB pixels")
        pixels = [validate_colour(rgb) for rgb in pixels]
        with self._state.lock:
            self._state.pixels = pixels

    def get_pixels(self):
        return self._state.snapshot()[1]

    def clear(self, *colour):
        rgb = (0, 0, 0) if not colour else colour[0] if len(colour) == 1 else colour
        rgb = validate_colour(rgb)
        self.set_pixels([rgb] * 64)

    def show_gui(self):
        """Open the GUI on the main thread; return when the window closes."""
        from threading import current_thread, main_thread
        if current_thread() is not main_thread():
            raise RuntimeError("Call show_gui() on the main thread")
        from .gui import main
        main(self)
