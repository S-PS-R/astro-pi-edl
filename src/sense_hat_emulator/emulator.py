"""SenseEmu and its sensor/LED storage. No GUI is needed to use this class."""

from collections.abc import Mapping

from math import isfinite, radians, ceil
from random import Random
from secrets import randbits
from time import monotonic
from .humidity import HumiditySensor
from .pressure import PressureSensor
from .imu import IMUSensor, rotate_to_board, orientation_rates
import csv
from pathlib import Path
from numbers import Real
from threading import RLock

ANGLES = ('roll', 'pitch', 'yaw')
ENVIRONMENT = ('temperature_humidity', 'temperature_pressure', 'pressure', 'humidity')
RAW_IMU = tuple(f'{prefix}_{axis}' for prefix in ('accel', 'gyro', 'mag') for axis in 'xyz')
CSV_REQUIRED = ('time_s', *ENVIRONMENT, *RAW_IMU)
DEFAULTS = {
    'temperature': 20.0, 'temperature_humidity': 20.0, 'temperature_pressure': 20.0,
    'pressure': 1013.25, 'humidity': 0.0,
    'accel_x': 0.0, 'accel_y': 0.0, 'accel_z': 1.0,
    'mag_x': 33.0, 'mag_y': 0.0, 'mag_z': 0.0,
    'gyro_x': 0.0, 'gyro_y': 0.0, 'gyro_z': 0.0,
    'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
    'world_accel_x': 0.0, 'world_accel_y': 0.0, 'world_accel_z': 1.0,
    'world_mag_x': 33.0, 'world_mag_y': 0.0, 'world_mag_z': 0.0,
}


def checked_values(changes):
    """Validate the entire change before touching state."""
    checked = {}
    for key, value in changes.items():
        if key not in DEFAULTS:
            raise KeyError(f'Unknown sensor: {key}')
        if isinstance(value, bool) or not isinstance(value, Real):
            raise TypeError(f'{key} must be a real number')
        value = float(value)
        if not isfinite(value):
            raise ValueError(f'{key} must be finite')
        if key == 'humidity' and not 0 <= value <= 100:
            raise ValueError('Humidity must be between 0 and 100 percent')
        if key == 'pressure' and value <= 0:
            raise ValueError('Pressure must be positive')
        if key.startswith('temperature') and value < -273.15:
            raise ValueError('Temperature cannot be below absolute zero')
        checked[key] = (value+180) % 360-180 if key in ANGLES else value
    if 'temperature' in checked:
        checked['temperature_humidity'] = checked['temperature']
        checked['temperature_pressure'] = checked['temperature']
    elif 'temperature_humidity' in checked:
        checked['temperature'] = checked['temperature_humidity']
    return checked


def read_csv_samples(path):
    """Validate a complete timestamped recording before starting playback."""
    samples = []
    with Path(path).open(newline='', encoding='utf-8-sig') as stream:
        reader = csv.DictReader(stream)
        fields = reader.fieldnames or []
        if len(fields) != len(set(fields)):
            raise ValueError('CSV contains duplicate column names')
        missing = set(CSV_REQUIRED)-set(fields)
        unknown = set(fields)-set(CSV_REQUIRED)-set(ANGLES)
        if missing or unknown:
            raise ValueError(f'CSV columns: missing {sorted(missing)}, unknown {sorted(unknown)}')
        has_orientation = all(key in fields for key in ANGLES)
        if any(key in fields for key in ANGLES) and not has_orientation:
            raise ValueError('Include all three roll, pitch, yaw columns or none')
        for row in reader:
            line = reader.line_num
            try:
                if None in row or any(value is None or not value.strip() for value in row.values()):
                    raise ValueError('Missing or extra cell')
                values = {key: float(value) for key, value in row.items()}
                time_s = values.pop('time_s')
                if not isfinite(time_s) or time_s < 0:
                    raise ValueError('time_s must be finite and nonnegative')
                if not samples and time_s != 0:
                    raise ValueError('The first time_s must be 0')
                if samples and time_s <= samples[-1][0]:
                    raise ValueError('time_s must strictly increase')
                samples.append((time_s, checked_values(values)))
            except (ValueError, TypeError) as exc:
                raise ValueError(f'CSV line {line}: {exc}') from exc
    if not samples:
        raise ValueError('CSV must contain at least one sample')
    return samples, has_orientation


class EmulatorState:
    def __init__(self, noise, seed, realtime):
        self.lock = RLock()
        self.noise, self.seed, self.realtime = noise, seed, realtime
        self.mode = 'world'
        self.targets = DEFAULTS.copy()
        self.readings = self.targets.copy()
        self.pixels = [(0, 0, 0)] * 64
        self.elapsed = 0.0
        self.wall_last = monotonic()
        self.replay = []
        self.replay_index = 0
        self.replay_paused = False
        self.replay_finished = False
        self.replay_orientation = True
        self.saved_noise = noise
        self._make_models()

    def _derive_world(self):
        angles = [self.targets[key] for key in ANGLES]
        for prefix in ('accel', 'mag'):
            vector = [self.targets[f'world_{prefix}_{axis}'] for axis in 'xyz']
            self.targets.update(zip((f'{prefix}_{axis}' for axis in 'xyz'),
                                    rotate_to_board(vector, *angles)))

    def _make_models(self):
        if self.mode == 'world':
            self._derive_world()
            self.targets.update({f'gyro_{axis}': 0.0 for axis in 'xyz'})
        random = Random(self.seed)
        self.models = [model(random.getrandbits(64), self.targets)
                       for model in (HumiditySensor, PressureSensor, IMUSensor)]
        self.ticks = [0, 0, 0]
        self.origin = self.elapsed
        self.previous_orientation = tuple(self.targets[key] for key in ANGLES)
        self.readings = self.targets.copy()

    def _sample_to(self, end, before=False):
        for index, model in enumerate(self.models):
            relative = (end-self.origin) / model.period
            due = max(0, ceil(relative-1e-8)-1) if before else int(relative+1e-8)
            if not self.noise and not (index == 2 and self.mode == 'world'):
                self.ticks[index] = due
                continue
            while self.ticks[index] < due:
                if index == 2 and self.mode == 'world':
                    current = tuple(self.targets[key] for key in ANGLES)
                    rates = orientation_rates(self.previous_orientation, current, model.period)
                    self.targets.update(zip((f'gyro_{axis}' for axis in 'xyz'), rates))
                    self.previous_orientation = current
                if self.noise:
                    self.readings.update(model.sample(self.targets))
                else:
                    self.readings.update({key: self.targets[key] for key in RAW_IMU})
                self.ticks[index] += 1
        if not self.noise:
            self.readings.update(self.targets)
        self.readings['temperature'] = self.readings['temperature_humidity']

    def run_to(self, end):
        if self.mode == 'replay':
            if self.replay_paused or self.replay_finished:
                return
            end = min(end, self.replay[-1][0])
            while self.replay_index+1 < len(self.replay) and self.replay[self.replay_index+1][0] <= end+1e-10:
                time_s, row = self.replay[self.replay_index+1]
                # At an exact boundary, apply the new CSV row before its sensor tick.
                self._sample_to(time_s, before=True)
                self.targets.update(row)
                if not self.noise:
                    self.readings.update(row)
                self.replay_index += 1
            self._sample_to(end)
            self.elapsed = end
            self.replay_finished = end >= self.replay[-1][0]
        else:
            self._sample_to(end)
            self.elapsed = end

    def sync(self):
        now = monotonic()
        delta = max(0.0, now-self.wall_last) if self.realtime else 0.0
        self.wall_last = now
        self.run_to(self.elapsed+delta)

    def snapshot(self):
        with self.lock:
            self.sync()
            return self.targets.copy(), [list(rgb) for rgb in self.pixels]

    def update(self, changes):
        checked = checked_values(changes)
        with self.lock:
            self.sync()
            if self.mode == 'replay':
                raise RuntimeError('Stop CSV replay before changing sensor targets')
            has_world = any(key.startswith('world_') for key in checked)
            has_raw = any(key in RAW_IMU for key in checked)
            if has_world and has_raw:
                raise ValueError('Do not mix world and raw IMU inputs')
            new_mode = 'world' if has_world else 'direct' if has_raw else self.mode
            changed_mode = self.mode != new_mode
            self.mode = new_mode
            self.targets.update(checked)
            if self.mode == 'world':
                self._derive_world()
            if changed_mode:
                self._make_models()
            if not self.noise:
                self.readings.update(self.targets)

    def reset(self):
        with self.lock:
            if self.mode == 'replay':
                self.noise = self.saved_noise
            self.mode = 'world'
            self.replay = []
            self.replay_paused = self.replay_finished = False
            self.replay_orientation = True
            self.targets = DEFAULTS.copy()
            self.elapsed = 0.0
            self.wall_last = monotonic()
            self._make_models()


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

    def __init__(self, noise=True, seed=None, realtime=True):
        if type(noise) is not bool or type(realtime) is not bool:
            raise TypeError('noise and realtime must be True or False')
        if seed is not None and type(seed) is not int:
            raise TypeError('seed must be an integer or None')
        self._state = EmulatorState(noise, randbits(64) if seed is None else seed, realtime)
        self.print_settings()

    @property
    def seed(self):
        return self._state.seed

    @property
    def noise(self):
        return self._state.noise

    @noise.setter
    def noise(self, enabled):
        if type(enabled) is not bool:
            raise TypeError('noise must be True or False')
        with self._state.lock:
            if enabled == self._state.noise:
                return
            self._state.sync()
            self._state.noise = enabled
            self._state._make_models()
        self.print_settings()

    def print_settings(self):
        print(f"[SenseEmu] noise={'ON' if self.noise else 'OFF'} | seed={self.seed} | "
              f"time={'realtime' if self._state.realtime else 'manual'} | mode={self.mode}")

    def advance(self, seconds):
        """Advance deterministic simulation time; requires realtime=False."""
        if isinstance(seconds, bool) or not isinstance(seconds, Real):
            raise TypeError('seconds must be a real number')
        seconds = float(seconds)
        if not isfinite(seconds) or seconds < 0:
            raise ValueError('seconds must be finite and nonnegative')
        with self._state.lock:
            if self._state.realtime:
                raise RuntimeError('advance() requires SenseEmu(realtime=False)')
            elapsed = self._state.elapsed + seconds
            if not isfinite(elapsed):
                raise ValueError('Simulation time overflow')
            self._state.run_to(elapsed)

    @property
    def mode(self):
        return self._state.mode

    def set_mode(self, mode):
        """Select world/direct controls. CSV replay starts through load_csv()."""
        if mode not in ('world', 'direct'):
            raise ValueError('mode must be world or direct')
        with self._state.lock:
            self._state.sync()
            if self.mode == 'replay':
                self._state.noise = self._state.saved_noise
            self._state.mode = mode
            self._state.replay_paused = self._state.replay_finished = False
            self._state.replay_orientation = True
            self._state._make_models()
        self.print_settings()

    def set_orientation(self, roll, pitch, yaw):
        """Command angles in degrees. In world mode these generate raw IMU inputs."""
        self._state.update(dict(roll=roll, pitch=pitch, yaw=yaw))

    set_orientation_degrees = set_orientation

    def set_world_acceleration(self, x, y=None, z=None):
        """World accelerometer-input vector in g (default +Z), not vehicle dynamics."""
        self._set_vector('world_accel', x, y, z)

    def set_world_magnetic_field(self, x, y=None, z=None):
        """World magnetic field in µT. Selects world mode."""
        self._set_vector('world_mag', x, y, z)

    def load_csv(self, path, noise=False):
        """Validate and start a recording at time zero; raw sample-hold by default."""
        if type(noise) is not bool:
            raise TypeError('noise must be True or False')
        samples, orientation = read_csv_samples(path)
        with self._state.lock:
            state = self._state
            state.sync()
            if state.mode != 'replay':
                state.saved_noise = state.noise
            state.mode = 'replay'
            state.noise = noise
            state.replay = samples
            state.replay_orientation = orientation
            state.replay_index = 0
            state.replay_paused = False
            state.replay_finished = len(samples) == 1
            state.elapsed = 0.0
            state.wall_last = monotonic()
            state.targets.update(samples[0][1])
            state._make_models()
        self.print_settings()

    def pause_replay(self):
        with self._state.lock:
            if self.mode != 'replay':
                raise RuntimeError('No CSV replay loaded')
            self._state.sync()
            self._state.replay_paused = True

    def resume_replay(self):
        with self._state.lock:
            if self.mode != 'replay':
                raise RuntimeError('No CSV replay loaded')
            self._state.wall_last = monotonic()
            self._state.replay_paused = False

    def restart_replay(self):
        with self._state.lock:
            state = self._state
            if state.mode != 'replay':
                raise RuntimeError('No CSV replay loaded')
            state.replay_index = 0
            state.elapsed = 0.0
            state.wall_last = monotonic()
            state.replay_paused = False
            state.replay_finished = len(state.replay) == 1
            state.targets.update(state.replay[0][1])
            state._make_models()

    def stop_replay(self):
        """Return to world mode and restore the noise setting from before replay."""
        self.set_mode('world')

    def get_replay_status(self):
        with self._state.lock:
            self._state.sync()
            return dict(active=self.mode == 'replay', time_s=self._state.elapsed,
                        sample=self._state.replay_index,
                        samples=len(self._state.replay),
                        paused=self._state.replay_paused, finished=self._state.replay_finished)

    def get_readings(self):
        """Get measured channels, including both temperature sources."""
        with self._state.lock:
            self._state.sync()
            readings = self._state.readings.copy()
            for key in ANGLES:
                readings.pop(key, None)
            if self.mode != 'replay' or self._state.replay_orientation:
                readings.update({key: self._state.targets[key] for key in ANGLES})
            return {key: value for key, value in readings.items() if not key.startswith('world_')}

    def _get(self, name):
        with self._state.lock:
            self._state.sync()
            return self._state.readings[name]

    def _set(self, name, value):
        self._state.update({name: value})

    def get_temperature(self):
        return self._get("temperature")

    def set_temperature(self, value):
        self._set("temperature", value)

    def get_temperature_from_humidity(self):
        return self._get('temperature_humidity')

    def get_temperature_from_pressure(self):
        return self._get('temperature_pressure')

    def set_temperature_from_humidity(self, value):
        self._set('temperature_humidity', value)

    def set_temperature_from_pressure(self, value):
        self._set('temperature_pressure', value)

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
            self._state.sync()
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
        with self._state.lock:
            self._state.sync()
            if self.mode == 'replay' and not self._state.replay_orientation:
                raise ValueError('CSV does not include roll, pitch, yaw')
            return {key: radians(self._state.targets[key]) for key in ANGLES}

    def get_orientation_degrees(self):
        with self._state.lock:
            self._state.sync()
            if self.mode == 'replay' and not self._state.replay_orientation:
                raise ValueError('CSV does not include roll, pitch, yaw')
            return {key: self._state.targets[key] % 360 for key in ANGLES}

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
