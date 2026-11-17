"""Best-effort UDP downlink of telemetry and event packets to a ground station.

Every datagram is one UTF-8 JSON object:

    {"v": 1, "type": "telemetry", "mission": "mars", "run": "run_...", "seq": 41,
     "time_s": 12.3, "data": {...}}
    {"v": 1, "type": "event", "mission": "mars", "run": "run_...", "seq": 42,
     "time_s": 12.3, "event_id": 3, "event": "peak_heating", "message": "...", "data": {...}}

`seq` counts every datagram sent in a run, so the ground station can measure
packet loss. UDP may drop packets, so each event is sent several times with
the same `event_id`; the ground station keeps the first copy it receives.
Network failures never raise into the flight loop. The local CSV remains the
authoritative record.
"""

import json
import math
import socket
from enum import Enum
from time import monotonic, sleep

PROTOCOL_VERSION = 1
DEFAULT_PORT = 8765


def _clean(value):
    # JSON has no NaN/inf; send null instead.
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(key): _clean(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_clean(item) for item in value]
    if isinstance(value, Enum):
        return value.value  # e.g. Phase
    return value


class Downlink:
    def __init__(self, host, port=DEFAULT_PORT, *, mission, run_id, telemetry_hz=10.0,
                 event_repeats=3, repeat_interval_s=0.25):
        if telemetry_hz <= 0:
            raise ValueError('telemetry_hz must be positive')
        self.mission = mission
        self.run_id = run_id
        self.period_s = 1.0 / telemetry_hz
        self.event_repeats = event_repeats
        self.repeat_interval_s = repeat_interval_s
        self.next_telemetry_at = 0.0
        self.sent = 0
        self.failed = 0
        self._seq = 0
        self._event_id = 0
        self._pending = []  # [next_send_at, remaining, packet]
        self.socket = None
        self.address = None

        # Ground station address
        if not host:
            return
        try:
            self.address = (socket.gethostbyname(host), port)
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.setblocking(False)
        except OSError as error:
            print(f'Downlink disabled: cannot resolve ground station {host!r} ({error}).', flush=True)
            self.socket = None

    @property
    def enabled(self):
        return self.socket is not None

    def describe(self):
        if not self.enabled:
            return 'Downlink disabled'
        return f'Downlink: UDP {self.address[0]}:{self.address[1]} (run {self.run_id})'

    def _packet(self, kind, time_s, **fields):
        return dict(v=PROTOCOL_VERSION, type=kind, mission=self.mission, run=self.run_id,
                    time_s=time_s, **fields)

    def _send(self, packet):
        packet['seq'] = self._seq
        self._seq += 1
        try:
            payload = json.dumps(_clean(packet), separators=(',', ':'), allow_nan=False)
            self.socket.sendto(payload.encode('utf-8'), self.address)
            self.sent += 1
        except (OSError, TypeError, ValueError):
            # Unreachable laptop, full buffer, or unserializable value: drop the packet.
            self.failed += 1

    def service(self, now=None):
        """Resend pending event copies whose repeat time has arrived."""
        if not self._pending:
            return
        now = monotonic() if now is None else now
        for item in self._pending:
            if now >= item[0]:
                self._send(dict(item[2]))
                item[0] = now + self.repeat_interval_s
                item[1] -= 1
        self._pending = [item for item in self._pending if item[1] > 0]

    def telemetry(self, time_s, data, now=None, *, force=False):
        """Send a rate-limited telemetry snapshot; `force` bypasses the rate limit."""
        if not self.enabled:
            return False
        now = monotonic() if now is None else now
        self.service(now)
        if not force and now < self.next_telemetry_at:
            return False
        self.next_telemetry_at = now + self.period_s
        self._send(self._packet('telemetry', time_s, data=data))
        return True

    def event(self, name, time_s, message, **data):
        """Send a critical event immediately and schedule repeat copies."""
        print(f'EVENT {name} at T+{time_s:.2f} s: {message}', flush=True)
        if not self.enabled:
            return
        self._event_id += 1
        packet = self._packet('event', time_s, event_id=self._event_id, event=name,
                              message=message, data=data)
        self._send(dict(packet))
        if self.event_repeats > 1:
            self._pending.append([monotonic() + self.repeat_interval_s,
                                  self.event_repeats - 1, packet])

    def close(self):
        """Flush outstanding event repeats (briefly), then close the socket."""
        if not self.enabled:
            return
        while self._pending:
            sleep(0.05)
            for item in self._pending:
                self._send(dict(item[2]))
                item[1] -= 1
            self._pending = [item for item in self._pending if item[1] > 0]
        self.socket.close()
        self.socket = None


def open_downlink(settings, *, mission, run_id, telemetry_hz):
    """Create a Downlink from a GROUND_STATION settings dictionary."""
    settings = settings or {}
    host = settings.get('host') if settings.get('enabled', False) else None
    downlink = Downlink(host, settings.get('port', DEFAULT_PORT), mission=mission,
                        run_id=run_id, telemetry_hz=telemetry_hz)
    print(downlink.describe(), flush=True)
    return downlink


def landing_message(planet_name, mission_time_s, velocity_m_s, peaks):
    lines = [f'{planet_name.upper()} touchdown at T+{mission_time_s:.1f} s, '
             f'{velocity_m_s:.1f} m/s.']
    for name, peak in peaks.items():
        if peak.get('confirmed'):
            lines.append(f"{name}: {peak['value']:.2f} {peak['units']} at {peak['time_s']:.1f} s")
    return '\n'.join(lines)
