"""Render and save flight plots in a process with its own Tk event loop."""

from multiprocessing import get_context
from queue import Empty, Full
from pathlib import Path
from math import degrees, isfinite
import numpy as np

from ..shared.appearance import ACCENT, DARK_PLOT_STYLE, TRACE_COLORS, apply_tk_dark_theme


PANELS = (
    ('Downrange', 'Range (km)', .001, (('downrange_m', 'Downrange'),)),
    ('Flight-path angle', 'Angle (deg)', degrees(1),
     (('flight_path_angle_rad', 'Flight-path angle'),)),
    ('Drag force', 'Drag (kN)', .001,
     (('total_drag_n', 'Total drag'), ('parachute_drag_n', 'Canopy drag'),
      ('vehicle_drag_n', 'Vehicle drag'))),
    ('Speed', 'Speed (m/s)', 1,
     (('velocity_m_s', 'Vehicle'), ('equilibrium_speed_m_s', 'Local equilibrium'))),
    ('Dynamic pressure', 'Dynamic pressure (Pa)', 1, (('dynamic_pressure_pa', 'Dynamic pressure'),)),
    ('Convective heating', 'Heat flux (kW/m²)', 1, (('heating_kw_m2', 'Heat flux'),)),
    ('Vertical acceleration', 'Vertical acceleration (m/s²)', 1,
     (('vertical_acceleration_m_s2', 'Vertical acceleration'),)),
    ('Mach number', 'Mach', 1, (('mach', 'Mach'),)),
    ('Aerodynamic coefficients', 'Coefficient', 1,
     (('vehicle_cd', 'Vehicle Cd'), ('parachute_cd', 'Canopy Cd'), ('effective_cl', 'Effective Cl'))),
)


class PlotProcess:
    """Nonblocking sample delivery; plotting never runs on the mission thread."""

    def __init__(self, update_hz=3.0):
        context = get_context('spawn')
        self.update_hz = context.Value('d', update_hz)
        self.deployment_altitude_m = context.Value('d', float('nan'))
        self.samples = context.Queue(maxsize=256)
        self.process = context.Process(
            target=run_plot_window,
            args=(self.samples, self.update_hz, self.deployment_altitude_m),
            name='EDL plots', daemon=True)
        self.last_sample = None
        self.process.start()

    def set_update_hz(self, update_hz):
        self.update_hz.value = update_hz

    def add_sample(self, sample):
        self._queue_samples((sample,))

    def add_deployment(self, before, after):
        self.deployment_altitude_m.value = after['altitude_m']
        self._queue_samples((before, after))

    def _queue_samples(self, samples):
        self.last_sample = samples[-1]
        message = ('samples', samples)

        try:
            self.samples.put_nowait(message)
        except Full:
            # Skip oldest queued point
            try:
                self.samples.get_nowait()
            except Empty:
                pass
            try:
                self.samples.put_nowait(message)
            except Full:
                pass

    def save(self, path):
        """Queue a save after prior samples; retry later if the queue is full."""
        try:
            self.samples.put_nowait(('save', str(path), self.last_sample))
        except Full:
            return False
        return True

    def close(self, path):
        # Final save and shutdown
        try:
            if self.process.is_alive():
                self.samples.put(('close', str(path) if path else None,
                                  self.last_sample), timeout=10)
                self.process.join(timeout=10)
        except Full:
            print('Plot queue full; final PNG could not be saved.', flush=True)
        finally:
            if self.process.is_alive():
                print('Plot process did not close; stopping it.', flush=True)
                self.process.terminate()
                self.process.join(timeout=2)
            self.samples.cancel_join_thread()
            self.samples.close()


class FlightPlotWindow:
    def __init__(self, samples, update_hz, deployment_altitude_m):
        # Plot process GUI
        import tkinter as tk
        from tkinter import ttk
        from matplotlib import rc_context
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

        self.samples = samples
        self.background = None
        self.saving = False
        self.saved_path = None
        self.update_hz = update_hz
        self.deployment_altitude_m = deployment_altitude_m
        self.deployment_marked = False
        self.refresh_ms = round(1000 / update_hz.value)
        self.root = tk.Tk()
        self.root.title('EDL Plots')

        # Shared window lifecycle
        self.root.protocol('WM_DELETE_WINDOW', lambda: None)
        apply_tk_dark_theme(self.root)
        self.full_altitude_km = 1.0
        self.descent_detail = tk.BooleanVar(master=self.root)
        self.detail_control = ttk.Checkbutton(self.root, text='Descent detail', variable=self.descent_detail, command=self.change_altitude_view)
        self.detail_control.state(['disabled'])
        self.detail_control.pack(anchor='e', padx=14, pady=(8, 0))
        with rc_context(DARK_PLOT_STYLE):
            self.figure = Figure(figsize=(12.6, 8.2), dpi=100)
            self.figure.subplots_adjust(left=.07, right=.97, bottom=.08, top=.94, wspace=.32, hspace=.46)
            update_plot(self.figure)
        self.canvas = FigureCanvasTkAgg(self.figure, master=self.root)
        self.canvas.get_tk_widget().pack(fill='both', expand=True)
        self.traces = [(ax, list(ax.lines)) for ax in self.figure.axes]
        for _, lines in self.traces:
            for line in lines:
                line.set_animated(True)
        self.canvas.mpl_connect('draw_event', self.on_draw)
        self.canvas.draw()
        self.root.after(self.refresh_ms, self.refresh)

    def on_draw(self, event):
        if not self.saving:
            self.background = self.canvas.copy_from_bbox(self.figure.bbox)
            self.draw_lines()

    def draw_lines(self):
        self.canvas.restore_region(self.background)
        for ax, lines in self.traces:
            for line in lines:
                ax.draw_artist(line)
            if ax.get_legend() is not None:
                ax.draw_artist(ax.get_legend())
        self.canvas.blit(self.figure.bbox)

    def set_altitude_limits(self, *, detail):
        lower, upper = 0.0, self.full_altitude_km
        if detail:
            deploy_altitude = self.deployment_altitude_m.value
            if isfinite(deploy_altitude):
                upper = min(upper, max(1.0, deploy_altitude * .00125))
            else:
                # Zoom to the lowest part recorded if the run ended before deployment.
                lowest = min(self.traces[0][1][0].get_ydata(), default=0.0)
                span = max(1.0, (upper - lowest) / 4)
                lower = max(0.0, lowest - .05 * span)
                upper = min(upper, lowest + span)
        self.figure.axes[0].set_ylim(lower, upper)

    def change_altitude_view(self):
        self.set_altitude_limits(detail=self.descent_detail.get())
        self.background = None
        self.canvas.draw_idle()

    def save(self, path):
        if path == self.saved_path:
            return
        path = Path(path)
        detail_path = path.with_stem(path.stem + '_descent_detail')
        lines = [line for _, traces in self.traces for line in traces]
        original_limits = self.figure.axes[0].get_ylim()
        self.saving = True
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            for line in lines:
                line.set_animated(False)
            for output, detail in ((path, False), (detail_path, True)):
                self.set_altitude_limits(detail=detail)
                self.figure.savefig(output, dpi=150, facecolor=self.figure.get_facecolor())
                print(f'Plots saved: {output}', flush=True)
            self.saved_path = str(path)
        except OSError as error:
            print(f'Could not save plots: {error}', flush=True)
        finally:
            self.figure.axes[0].set_ylim(*original_limits)
            for line in lines:
                line.set_animated(True)
            self.saving = False
            self.background = None

    def refresh(self):
        self.refresh_ms = round(1000 / self.update_hz.value)
        points = []
        action = path = None
        for _ in range(256):
            try:
                message = self.samples.get_nowait()
            except Empty:
                # Queue drained
                break
            if message[0] == 'samples':
                points.extend(message[1])
            else:
                action, path, last_sample = message
                if last_sample is not None and (not points or points[-1] != last_sample):
                    points.append(last_sample)
                # Final queued sample
                break
        limits = [(ax.get_xlim(), ax.get_ylim()) for ax in self.figure.axes]
        if points:
            update_plot(self.figure, points)
            self.full_altitude_km = max(self.full_altitude_km, self.figure.axes[0].get_ylim()[1])
            if self.descent_detail.get():
                self.set_altitude_limits(detail=True)
        altitude = self.deployment_altitude_m.value
        if not self.deployment_marked and isfinite(altitude):
            mark_deployment(self.figure, altitude)
            self.deployment_marked = True
            self.detail_control.state(['!disabled'])
            self.background = None
            self.canvas.draw_idle()
        if action:
            self.save(path)
            if action == 'close':
                self.root.destroy()
                return
        if points or action:
            new_limits = [(ax.get_xlim(), ax.get_ylim()) for ax in self.figure.axes]
            if self.background is None or limits != new_limits:
                self.background = None
                self.canvas.draw_idle()
            else:
                self.draw_lines()
        self.root.after(self.refresh_ms, self.refresh)


def run_plot_window(samples, update_hz, deployment_altitude_m):
    window = FlightPlotWindow(samples, update_hz, deployment_altitude_m)
    window.root.mainloop()


def update_plot(figure, samples=()):
    """Append flight telemetry to the altitude plots on the GUI thread."""
    max_points = 5000
    if not figure.axes:
        axes = figure.subplots(3, 3, sharey=True).flat
        for index, (ax, (title, xlabel, _, channels)) in enumerate(zip(axes, PANELS)):
            for trace_index, (_, label) in enumerate(channels):
                color = TRACE_COLORS[trace_index]
                linestyle = '--' if label in ('Local equilibrium', 'Total drag') else '-'
                ax.plot([], [], label=label, color=color, linewidth=1.6, linestyle=linestyle)
            ax.set_title(title)
            ax.set_xlabel(xlabel)
            if index % 3 == 0:
                ax.set_ylabel('Altitude (km)')
            ax.set_ylim(0, 1)
            ax.margins(x=0.1)
            ax.grid(True, alpha=0.55)
            if len(channels) > 1:
                location = 'center right' if title == 'Drag force' else 'upper right'
                ax.legend(loc=location, fontsize=8, framealpha=1)

    if not samples:
        return

    new_altitude = np.asarray([row['altitude_m'] for row in samples], dtype=float) / 1000
    for ax, (_, _, factor, channels) in zip(figure.axes, PANELS):
        expand = False
        left, right = ax.get_xlim()
        for line, (key, _) in zip(ax.lines, channels):
            x, y = line.get_data()
            new_x = np.asarray([row.get(key) for row in samples], dtype=float) * factor
            x_data = np.concatenate((x, new_x))
            y_data = np.concatenate((y, new_altitude))
            if len(x_data) > max_points:
                indices = np.linspace(0, len(x_data) - 1, max_points // 2, dtype=int)
                x_data, y_data = x_data[indices], y_data[indices]
            line.set_data(x_data, y_data)
            finite = new_x[np.isfinite(new_x)]
            if len(finite) and (not np.isfinite(x).any() or finite.min() < left or finite.max() > right):
                expand = True
        if expand:
            ax.relim()
            ax.autoscale_view(scalex=True, scaley=False)

    upper = max(1.0, new_altitude.max() * 1.05)
    if upper > figure.axes[0].get_ylim()[1]:
        figure.axes[0].set_ylim(0, upper)


def mark_deployment(figure, altitude_m):
    """Mark the exact deployment altitude on the flight plots."""
    altitude_km = altitude_m / 1000
    for ax in figure.axes:
        ax.axhline(altitude_km, color=ACCENT, linestyle=':', linewidth=1, alpha=.75)
    drag_axis = next(ax for ax in figure.axes if ax.get_title() == 'Drag force')
    drag_axis.annotate(f'Chute deploy · {altitude_km:.2f} km',
                       xy=(.22, altitude_km), xycoords=drag_axis.get_yaxis_transform(),
                       xytext=(0, 5), textcoords='offset points', ha='left',
                       va='bottom', color=ACCENT, fontsize=8)
