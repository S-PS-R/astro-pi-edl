"""Live plots of downlinked telemetry, drawn by the ground station.

This module only visualizes telemetry packets handed to it; receiving,
logging, and event display live in ground_station.py.
"""

from collections import deque
from .appearance import DARK_PLOT_STYLE, TRACE_COLORS

# Panels per mission: (title, telemetry keys)
PANELS = {
    'earth': (
        ('Acceleration (g)', ('accel_x', 'accel_y', 'accel_z')),
        ('Gyroscope (rad/s)', ('gyro_x', 'gyro_y', 'gyro_z')),
        ('Magnetic field (µT)', ('mag_x', 'mag_y', 'mag_z')),
        ('Pressure (hPa)', ('pressure',)),
        ('Temperature (°C)', ('temperature_pressure', 'temperature_humidity')),
        ('Humidity (%)', ('humidity',)),
    ),
    'mars': (
        ('Altitude (m)', ('altitude_m',)),
        ('Speed (m/s)', ('velocity_m_s',)),
        ('Convective heat flux (kW/m²)', ('heating_kw_m2',)),
        ('Load factor (g)', ('g_load',)),
        ('Board pressure (hPa)', ('pressure',)),
        ('Board temperature (°C)', ('temperature_pressure', 'temperature_humidity')),
    ),
}
TIME_LABELS = {'earth': 'Recording time (s)', 'mars': 'Mission time (s)'}


def sensor_group(name):
    if name.startswith(('accel_', 'gyro_')):
        return 'imu'
    if name.startswith('mag_'):
        return 'mag'
    if name in ('pressure', 'temperature_pressure'):
        return 'pressure'
    return 'humidity'


class SnapshotBuffer:
    """Bounded plot history for one run, independent of the flight CSV."""

    def __init__(self, maxlen=16000):
        self.rows = deque(maxlen=maxlen)
        self.latest = None
        self.typical_dt = None

    def ingest(self, time_s, row):
        if self.latest is not None:
            dt = time_s - self.latest[0]
            if dt <= 0:
                return False
            # Break the line where snapshots went missing
            if self.typical_dt is not None and dt > 3.0 * self.typical_dt:
                self.rows.append((time_s - 1e-3, None))
            else:
                self.typical_dt = dt if self.typical_dt is None else 0.9 * self.typical_dt + 0.1 * dt
        self.latest = (time_s, row)
        self.rows.append(self.latest)
        return True


class TelemetryPlot:
    def __init__(self, port):
        from matplotlib import rc_context
        import matplotlib.pyplot as plt
        with rc_context(DARK_PLOT_STYLE):
            self.figure = plt.figure(figsize=(12, 8), layout='constrained')
            self.figure.suptitle(f'Waiting for telemetry on UDP port {port}')
        self.run = None
        self.mission = None
        self.history = SnapshotBuffer()
        self.axes = []
        self.dirty = False

    def _build(self, mission):
        from matplotlib import rc_context

        # Mission-specific panels
        self.figure.clear()
        self.axes = []
        panels = PANELS.get(mission, ())
        with rc_context(DARK_PLOT_STYLE):
            for index, (title, channels) in enumerate(panels):
                axis = self.figure.add_subplot(3, 2, index + 1)
                axis.set_title(title)
                axis.set_xlabel(TIME_LABELS.get(mission, 'Time (s)'))
                axis.grid(alpha=0.5)
                traces = [(name, axis.plot([], [], label=name, color=color)[0])
                          for name, color in zip(channels, TRACE_COLORS)]
                if len(channels) > 1:
                    axis.legend(loc='upper left', fontsize='small')
                self.axes.append((axis, traces))
        self.mission = mission

    def add(self, packet):
        """Add one telemetry packet; a new run clears the history."""
        try:
            run, mission, time_s, data = packet['run'], packet['mission'], float(packet['time_s']), packet['data']
        except (KeyError, TypeError, ValueError):
            return
        if run != self.run:
            self.run = run
            self.history = SnapshotBuffer()
            if mission != self.mission:
                self._build(mission)
        self.dirty |= self.history.ingest(time_s, data)

    def draw(self):
        if not self.dirty:
            return
        self.dirty = False
        rows = list(self.history.rows)
        times = [time_s for time_s, _ in rows]
        nan = float('nan')
        for axis, traces in self.axes:
            for name, line in traces:
                status_key = sensor_group(name) + '_status'
                values = []
                for _, row in rows:
                    value = None if row is None else row.get(name)
                    # Earth snapshots flag failed sensor groups
                    if value is None or row.get(status_key, 'ok') != 'ok':
                        value = nan
                    values.append(value)
                line.set_data(times, values)
            axis.relim()
            axis.autoscale_view()

        # Title
        time_s, latest = self.history.latest
        phase = latest.get('phase', '?')
        if self.mission == 'earth':
            flight = latest.get('flight_elapsed_s')
            detail = 'waiting for release' if flight is None else f'flight {flight:.2f} s'
        else:
            detail = f'T+{time_s:.1f} s'
        self.figure.suptitle(f'{self.mission.upper()} — {self.run} — {phase} — {detail}',
                            color=DARK_PLOT_STYLE['text.color'])
