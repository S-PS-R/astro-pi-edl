"""Laptop receiver: python -m edl_flight_computer.earth.live_plot --port 8765."""

import argparse
from collections import deque
import json
import socket


class SnapshotBuffer:
    """Bounded plot history, independent of the Pi's complete CSV."""

    def __init__(self):
        self.rows = deque(maxlen=16000)
        self.latest = None

    def ingest(self, packet):
        try:
            row = json.loads(packet)
            time_s = row['time_s']
            sequence = row['sequence']
            # Recording order
            if self.latest is not None:
                if sequence == 0 and time_s < self.latest['time_s']:
                    self.rows.clear()
                elif sequence <= self.latest['sequence'] or time_s < self.latest['time_s']:
                    return False
                elif time_s - self.latest['time_s'] > 0.15:
                    # Missing snapshots
                    self.rows.append(dict(time_s=time_s - 0.001))
            self.rows.append(row)
            self.latest = row
            while self.rows and self.rows[0]['time_s'] < time_s - 600.0:
                self.rows.popleft()
            return True
        except (KeyError, TypeError, ValueError, UnicodeError):
            return False


def main(argv=None):
    # Receiver settings
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8765)
    args = parser.parse_args(argv)

    # Laptop plotting
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    # UDP receiver
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(('0.0.0.0', args.port))
    receiver.setblocking(False)
    history = SnapshotBuffer()
    
    # Sensor panels
    panels = (
        ('Acceleration (g)', ('accel_x', 'accel_y', 'accel_z')),
        ('Gyroscope (rad/s)', ('gyro_x', 'gyro_y', 'gyro_z')),
        ('Magnetic field (µT)', ('mag_x', 'mag_y', 'mag_z')),
        ('Pressure (hPa)', ('pressure',)),
        ('Temperature (°C)', ('temperature_pressure', 'temperature_humidity')),
        ('Humidity (%)', ('humidity',)),
    )

    figure, axes = plt.subplots(3, 2, figsize=(12, 8), layout='constrained')
    lines = []
    for axis, (title, channels) in zip(axes.flat, panels):
        axis.set_title(title)
        axis.set_xlabel('Recording time (s)')
        axis.grid(alpha=0.25)
        lines.append([(name, axis.plot([], [], label=name)[0]) for name in channels])
        axis.legend(loc='upper left', fontsize='small')
    figure.suptitle(f'Waiting for Pi on UDP port {args.port}')

    def refresh(_):

        # Latest snapshots
        for _ in range(512):
            try:
                packet, _address = receiver.recvfrom(65535)
            except OSError:
                break
            history.ingest(packet)
        if history.latest is None:
            return
        rows = list(history.rows)
        times = [row['time_s'] for row in rows]
        
        # Plot refresh
        for axis, traces in zip(axes.flat, lines):
            for name, line in traces:
                values = []
                if name.startswith(('accel_', 'gyro_')):
                    group = 'imu'
                elif name.startswith('mag_'):
                    group = 'mag'
                elif name in ('pressure', 'temperature_pressure'):
                    group = 'pressure'
                else:
                    group = 'humidity'
                for row in rows:
                    value = row.get(name)
                    values.append(value if value is not None and row.get(group+'_status') == 'ok' else float('nan'))
                line.set_data(times, values)
            axis.relim()
            axis.autoscale_view()
        latest = history.latest
        flight = latest.get('flight_elapsed_s')
        flight_label = 'waiting for release' if flight is None else f'flight {flight:.2f} s'
        figure.suptitle(f"Earth — {latest['phase']} — {flight_label}")

    animation = FuncAnimation(figure, refresh, interval=50, cache_frame_data=False)
    print(f'Listening on UDP {args.port}. Allow Python through the laptop firewall on your private Wi-Fi network.')
    try:
        plt.show()
    finally:
        receiver.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
