"""Ground station: python -m edl_flight_computer.shared.ground_station --port 8765

Receives the UDP downlink from Earth drops (Pi) or the Mars simulator, prints
critical events as they arrive, shows a once-per-second telemetry status line,
plots telemetry live (unless --no-plot), and saves packet logs and plot PNGs
on the ground station.
"""

import argparse
from datetime import datetime
import json
from pathlib import Path
import re
import socket
from time import monotonic, sleep

from .downlink import DEFAULT_PORT, PROTOCOL_VERSION

RUN_END_EVENTS = {'landing', 'impact', 'stopped', 'deployment_failed'}

# Status line fields: (telemetry key, label, format)
STATUS_FIELDS = {
    'mars': (('phase', 'phase', '{}'), ('altitude_m', 'alt', '{:.0f} m'),
             ('velocity_m_s', 'V', '{:.1f} m/s'), ('heating_kw_m2', 'q', '{:.1f} kW/m2'),
             ('g_load', 'load', '{:.2f} g'), ('temperature_pressure', 'board T', '{:.1f} C'),
             ('pressure', 'board P', '{:.1f} hPa')),
    'earth': (('phase', 'phase', '{}'), ('flight_elapsed_s', 'flight', '{:.2f} s'),
              ('ekf_displacement_m', 'dz', '{:.2f} m'), ('ekf_vertical_velocity_m_s', 'vz', '{:.2f} m/s'),
              ('accel_magnitude_g', '|a|', '{:.2f} g'), ('pressure', 'P', '{:.1f} hPa')),
}


def default_log_path():
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    return Path(__file__).resolve().parents[3] / 'logs' / f'ground_{stamp}.jsonl'


class RunLink:
    """Per-run link statistics."""

    def __init__(self):
        self.received = 0
        self.first_seq = None
        self.max_seq = -1
        self.event_ids = set()
        self.latest = None
        self.last_heard = None
        self.finished = False

    @property
    def lost(self):
        if self.first_seq is None:
            return 0
        # Repeated event copies also count as received datagrams.
        return max(0, self.max_seq - self.first_seq + 1 - self.received)


class GroundStation:
    def __init__(self, port=DEFAULT_PORT, log_path=None, echo=True):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('0.0.0.0', port))
        self.socket.setblocking(False)
        self.port = port
        self.echo = echo
        self.runs = {}
        self.current_run = None
        self.log = None
        self.log_path = None
        if log_path is not None:
            self.log_path = Path(log_path)
            self.log_path.parent.mkdir(parents=True, exist_ok=True)
            self.log = self.log_path.open('a', encoding='utf-8')

    def poll(self, max_packets=512):
        """Read waiting datagrams; handle events and return new telemetry packets."""
        telemetry = []
        for _ in range(max_packets):
            try:
                payload, address = self.socket.recvfrom(65535)
            except BlockingIOError:
                break
            except OSError:
                # Windows reports ICMP errors from earlier sends here; ignore them.
                continue
            packet = self.ingest(payload, address)
            if packet is not None and packet['type'] == 'telemetry':
                telemetry.append(packet)
        if self.log is not None:
            self.log.flush()
        return telemetry

    def ingest(self, payload, address=None):
        try:
            packet = json.loads(payload)
            if packet.get('v') != PROTOCOL_VERSION or packet['type'] not in ('telemetry', 'event'):
                return None
            run_id, seq = packet['run'], int(packet['seq'])
        except (KeyError, TypeError, ValueError, UnicodeError, AttributeError):
            return None

        # Link statistics
        link = self.runs.get(run_id)
        if link is None:
            link = self.runs[run_id] = RunLink()
            if self.echo:
                where = f' from {address[0]}' if address else ''
                print(f"\n=== New {packet.get('mission', '?').upper()} run {run_id}{where} ===", flush=True)
        self.current_run = run_id
        link.received += 1
        link.last_heard = monotonic()
        link.first_seq = seq if link.first_seq is None else min(link.first_seq, seq)
        link.max_seq = max(link.max_seq, seq)

        if packet['type'] == 'event':
            # Duplicate copies of an event
            if packet.get('event_id') in link.event_ids:
                return None
            link.event_ids.add(packet.get('event_id'))
            if packet.get('event') in RUN_END_EVENTS:
                link.finished = True
            self.show_event(packet)
        else:
            if link.latest is not None and seq <= link.latest['seq']:
                return None  # out of order
            link.latest = packet
        if self.log is not None:
            packet['received_at'] = datetime.now().isoformat(timespec='milliseconds')
            self.log.write(json.dumps(packet, separators=(',', ':')) + '\n')
        return packet

    def show_event(self, packet):
        if not self.echo:
            return
        clock = datetime.now().strftime('%H:%M:%S')
        name = packet.get('event', '?')
        line = f"[{clock}] {packet.get('mission', '?').upper()} T+{packet.get('time_s', 0):.2f} s  {name.upper()}"
        message = packet.get('message', '')
        if name == 'landing':
            bar = '*' * 60
            print(f'\n{bar}\n{line}\n{message}\n{bar}', flush=True)
        elif name == 'fault':
            print(f'\n!!! {line}: {message}', flush=True)
        else:
            print(f'\n{line}: {message}', flush=True)

    def status_line(self):
        link = self.runs.get(self.current_run)
        if link is None:
            return f'Waiting for downlink on UDP port {self.port}...'
        silent = monotonic() - link.last_heard
        if link.latest is None:
            return f'{self.current_run}: no telemetry yet'
        packet = link.latest
        data = packet.get('data', {})
        parts = [f"T+{packet['time_s']:.1f} s"]
        for key, label, fmt in STATUS_FIELDS.get(packet.get('mission'), ()):
            value = data.get(key)
            if value is not None:
                parts.append(f'{label} {fmt.format(value)}')
        if data.get('faults'):
            parts.append('FAULT ' + ', '.join(data['faults']))
        total = link.received + link.lost
        parts.append(f'loss {100.0 * link.lost / total:.1f}%' if total else 'loss -')
        if silent > 2.0:
            parts.append(f'NO SIGNAL {silent:.0f} s')
        return ' | '.join(parts)

    def close(self):
        self.socket.close()
        if self.log is not None:
            self.log.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port', type=int, default=DEFAULT_PORT)
    parser.add_argument('--log', type=Path, default=None, help='JSONL packet log (default logs/ground_*.jsonl)')
    parser.add_argument('--no-log', action='store_true', help='do not record packets')
    parser.add_argument('--no-plot', action='store_true', help='console only, no plot window')
    args = parser.parse_args(argv)

    session_path = args.log or default_log_path()
    log_path = None if args.no_log else session_path
    station = GroundStation(args.port, log_path)
    print(f'Ground station listening on UDP {args.port}. '
          'Allow Python through the laptop firewall on your private Wi-Fi network.', flush=True)
    if log_path is not None:
        print(f'Recording packets: {log_path}', flush=True)
    next_status = 0.0
    plot = None
    plotted_run = 0
    saved_history = None
    saved_snapshot = None

    def save_plot():
        nonlocal plotted_run, saved_history, saved_snapshot
        if plot is None or not plot.axes:
            return
        history = plot.history
        if history.latest is None or history.latest is saved_snapshot:
            return
        if history is not saved_history:
            plotted_run += 1
            saved_history = history
        saved_snapshot = history.latest
        run_name = re.sub(r'[^a-zA-Z0-9_-]', '_', str(plot.run))[:80]
        path = session_path.with_name(f'{session_path.stem}_plot_{plotted_run:02d}_{run_name}.png')
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            plot.draw()
            plot.figure.savefig(path, dpi=150)
        except OSError as error:
            print(f'Could not save downlink plot: {error}', flush=True)
        else:
            print(f'Saved downlink plot: {path}', flush=True)

    def service():
        # Receive, then report status once per second
        nonlocal next_status
        for packet in station.poll():
            if plot is not None:
                plot.add(packet, before_reset=save_plot)
        if plot is not None and plot.run in station.runs and station.runs[plot.run].finished:
            save_plot()
        now = monotonic()
        if now >= next_status:
            print(station.status_line(), flush=True)
            next_status = now + 1.0

    try:
        if args.no_plot:
            while True:
                service()
                sleep(0.02)
        else:
            import matplotlib.pyplot as plt
            from matplotlib.animation import FuncAnimation
            from .live_plot import TelemetryPlot

            plot = TelemetryPlot(args.port)

            def refresh(_frame):
                service()
                plot.draw()

            animation = FuncAnimation(plot.figure, refresh, interval=50, cache_frame_data=False)
            plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        station.close()
        save_plot()
        print('Ground station stopped.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
