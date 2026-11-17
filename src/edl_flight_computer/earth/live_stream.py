"""Optional best-effort UDP plot snapshots; the local CSV is authoritative."""

import json
import socket


class PlotSender:
    def __init__(self, host, port=8765):
        if not host:
            raise ValueError("Set EARTH_SETTINGS['laptop_ip'] when live_plotting is enabled")
        
        # Laptop address
        self.address = (socket.gethostbyname(host), port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setblocking(False)
        self.next_send_at = 0.0

    def publish(self, row, now, *, force=False):
        if not force and now < self.next_send_at:
            return
        self.next_send_at = now + 0.05
        try:
            self.socket.sendto(json.dumps(row, separators=(',', ':'), allow_nan=False).encode('utf-8'), self.address)
        except OSError:
            # Skip unavailable laptop
            pass

    def close(self):
        self.socket.close()
