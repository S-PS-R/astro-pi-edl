"""An 8x8 altitude/time trajectory with persistent peak-event markers."""

from threading import Event, Lock, Thread
from time import monotonic


class TrajectoryLEDGrid:
    # Grid axes: time left to right; altitude top to bottom
    # Shared peak cells: cycling event colors
    # Bottom-right: flashing green countdown; solid green landed
    COLOURS = {
        'peak_acceleration': (255, 0, 0), #red
        'peak_heating': (255, 165, 0), #orange
        'peak_velocity': (128, 0, 128), #purple
    }

    def __init__(self, sense, initial_altitude_m, time_span_s=720.0):
        self.sense = sense
        self.altitude_max_m = max(1.0, initial_altitude_m)
        self.time_span_s = max(1.0, time_span_s)
        self.points = []
        self.events = []
        self.landed = False
        self._countdown_started_at = None
        self._lock = Lock()
        self._stop = Event()
        self._thread = None
        self._previous = None

    def add_sample(self, time_s, altitude_m):
        with self._lock:
            self._expand(time_s, altitude_m)
            self.points.append((time_s, altitude_m))
            if len(self.points) > 5000:
                self.points = self.points[::2] + [self.points[-1]]

    def add_event(self, name, time_s, altitude_m):
        if name not in self.COLOURS or altitude_m is None:
            return
        with self._lock:
            self._expand(time_s, altitude_m)
            self.events.append((name, time_s, altitude_m))

    def _expand(self, time_s, altitude_m):
        # Expand plot range
        while time_s > self.time_span_s:
            self.time_span_s *= 2
        self.altitude_max_m = max(self.altitude_max_m, altitude_m)

    def mark_landed(self):
        with self._lock:
            self.landed = True

    def start_landing_countdown(self):
        """Latch the green warning until ground contact, without blocking."""
        with self._lock:
            if self._countdown_started_at is None:
                self._countdown_started_at = monotonic()

    @staticmethod
    def cell(time_s, altitude_m, time_span_s, altitude_max_m):
        x = max(0, min(7, int(8 * time_s / time_span_s)))
        y = max(0, min(7, int(8 * (1 - altitude_m / altitude_max_m))))
        return x, y

    def render(self, wall_time_s=None):
        with self._lock:
            points = self.points.copy()
            events = self.events.copy()
            span, height = self.time_span_s, self.altitude_max_m
            landed = self.landed
            countdown_started_at = self._countdown_started_at

        # Trajectory trail
        pixels = [(0, 0, 0)] * 64
        for time_s, altitude_m in points:
            x, y = self.cell(time_s, altitude_m, span, height)
            pixels[y*8 + x] = (0, 16, 24)  # Dim trajectory trail.

        if points and not landed:
            current_time_s, current_altitude_m = points[-1]
            x, y = self.cell(current_time_s, current_altitude_m, span, height)
            pixels[y*8 + x] = (180, 180, 180)
            
        # Peak markers
        buckets = {}
        for name, time_s, altitude_m in events:
            x, y = self.cell(time_s, altitude_m, span, height)
            names = buckets.setdefault(y*8 + x, [])
            if name not in names:
                names.append(name)
                
        now = monotonic() if wall_time_s is None else wall_time_s

        tick = int(now / 0.6)
        for index, names in buckets.items():
            colours = [self.COLOURS[name] for name in names]
            pixels[index] = colours[tick % len(colours)]

        if landed:
            # Landed indicator
            pixels[7*8 + 7] = (0, 255, 0)
        elif countdown_started_at is not None:
            # Countdown pulse
            on = (now - countdown_started_at) % 1.0 < 0.5
            pixels[7*8 + 7] = (0, 255, 0) if on else (0, 0, 0)
        if pixels != self._previous:
            self.sense.set_pixels(pixels)
            self._previous = pixels

        return pixels

    def start(self):
        self._thread = Thread(target=self._run, name='EDL LED grid', daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            self.render()
            self._stop.wait(0.1)

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2)
