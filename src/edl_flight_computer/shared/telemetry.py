"""Record flight calculations and raw board measurements in the same CSV."""

import csv
from datetime import datetime
from pathlib import Path
from time import monotonic
from .sensors import SENSOR_CHANNELS

# Mars CSV: time, phase, calculated values, sensor values
HEADER = (
    'time_s', 'phase',
    'altitude_m', 'velocity_m_s', 'flight_path_angle_rad', 'downrange_m',
    'crossrange_m', 'latitude_deg', 'longitude_deg',
    'ground_speed_m_s', 'wind_east_m_s', 'wind_north_m_s',
    'temperature_k', 'pressure_pa', 'density_kg_m3', 'gravity_m_s2',
    'dynamic_pressure_pa', 'bank_angle_deg',
    'vertical_velocity_m_s',
    'acceleration_m_s2', 'g_load', 'heating_kw_m2',
    'mach', 'equilibrium_speed_m_s', 'vertical_acceleration_m_s2',
    'vehicle_cd', 'parachute_cd', 'effective_cd', 'effective_cl',
    'vehicle_drag_n', 'parachute_drag_n', 'total_drag_n',
    'mass_kg', 'propellant_kg', 'thrust_n', 'propellant_flow_kg_s', 'horizontal_speed_m_s',
    'landing_target_error_m',
    *SENSOR_CHANNELS,
)


def new_run_csv(prefix):
    run_name = prefix + datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    run_dir = Path(__file__).resolve().parents[3] / 'logs' / run_name
    run_dir.mkdir(parents=True, exist_ok=True)
    return run_dir / (run_name + '.csv')


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
