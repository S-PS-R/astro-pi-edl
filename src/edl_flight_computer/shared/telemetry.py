"""Record flight calculations and raw board measurements in the same CSV."""

import csv
from pathlib import Path
from time import monotonic

# default header used by mars, earth drop has its own header since most of these are not applicable to the live demo drop
HEADER = (
    'time_s', 'wall_time_s', 'time_scale', 'phase', 'altitude_m', 'velocity_m_s', 'flight_path_angle_rad',
    'temperature_k', 'pressure_pa', 'density_kg_m3', 'gravity_m_s2',
    'dynamic_pressure_pa', 'drag_n', 'lift_n', 'lift_vertical_n', 'bank_angle_deg',
    'vertical_velocity_m_s', 'events',
    'acceleration_m_s2', 'deceleration_m_s2', 'g_load', 'heating_kw_m2',
    'accel_x', 'accel_y', 'accel_z',
    'gyro_x', 'gyro_y', 'gyro_z',
    'mag_x', 'mag_y', 'mag_z',
    'temperature_humidity', 'temperature_pressure',
    'pressure', 'humidity',
    'imu_sample_s', 'mag_sample_s', 'pressure_sample_s', 'humidity_sample_s',
)


class DropLog:
    def __init__(self, path, *, fieldnames=HEADER, flush_interval_s=0.0):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        # New recording
        self.stream = self.path.open('x', newline='', encoding='utf-8')
        self.writer = csv.DictWriter(self.stream, fieldnames=fieldnames)
        self.flush_interval_s = flush_interval_s
        self._last_flush = monotonic()
        self.writer.writeheader()
        self.stream.flush()

    def write(self, row):
        self.writer.writerow(row)
        if self.flush_interval_s <= 0 or monotonic() - self._last_flush >= self.flush_interval_s:
            self.stream.flush()
            self._last_flush = monotonic()

    def close(self):
        self.stream.close()

    # File cleanup
    def __enter__(self):
        return self

    def __exit__(self, *_args):
        self.close()
