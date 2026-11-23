"""Render and save flight plots in a process with its own Tk event loop."""

from multiprocessing import get_context
from queue import Empty, Full
from pathlib import Path
from math import degrees, isfinite
from time import monotonic
import numpy as np

from ..shared.appearance import ACCENT, BORDER, DARK_PLOT_STYLE, PANEL, TEXT, TRACE_COLORS, apply_tk_dark_theme


PANELS = (
    ('Downrange', 'Range (km)', .001, (('downrange_m', 'Downrange'),)),
    ('Flight-path angle', 'Angle (deg)', degrees(1),
     (('flight_path_angle_rad', 'Flight-path angle'),)),
    ('Drag force', 'Drag (kN)', .001, (('total_drag_n', 'Total drag'),)),
    ('Velocity', 'Velocity (m/s)', 1, (('velocity_m_s', 'Velocity magnitude'),)),
    ('Dynamic pressure', 'Dynamic pressure (Pa)', 1, (('dynamic_pressure_pa', 'Dynamic pressure'),)),
    ('Convective heating', 'Heat flux (kW/m²)', 1, (('heating_kw_m2', 'Heat flux'),)),
    ('Acceleration', 'Along-path acceleration (m/s²)', 1,
     (('acceleration_m_s2', 'Along-path acceleration'),)),
    ('Atmospheric density', 'Density (kg/m³)', 1, (('density_kg_m3', 'Density'),)),
    ('Aerodynamic coefficients', 'Coefficient', 1,
     (('effective_cd', 'Total Cd'), ('effective_cl', 'Total Cl'))),
)
TRACK_FIELDS = ('altitude_m', 'downrange_m', 'crossrange_m', 'longitude_deg', 'latitude_deg')


class PlotProcess:
    """Nonblocking sample delivery; plotting never runs on the mission thread."""

    def __init__(self, update_hz=3.0):
        context = get_context('spawn')
        self.update_hz = context.Value('d', update_hz)
        self.deployment = context.Array('d', [float('nan')] * len(TRACK_FIELDS))
        self.samples = context.Queue(maxsize=256)
        self.process = context.Process(
            target=run_plot_window,
            args=(self.samples, self.update_hz, self.deployment),
            name='EDL plots', daemon=True)
        self.last_sample = None
        self.process.start()

    def set_update_hz(self, update_hz):
        self.update_hz.value = update_hz

    def add_sample(self, sample):
        self._queue_samples((sample,))

    def add_deployment(self, before, after):
        self.deployment[:] = [after.get(key, float('nan')) for key in TRACK_FIELDS]
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
    def __init__(self, samples, update_hz, deployment):
        # Plot process GUI
        import tkinter as tk
        from tkinter import ttk
        from matplotlib import rc_context
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

        self.samples = samples
        self.saved_path = None
        self.update_hz = update_hz
        self.deployment = deployment
        self.deployment_marked = False
        self.plots_dirty = False
        self.last_draw = 0.0
        self.refresh_ms = round(1000 / update_hz.value)
        self.root = tk.Tk()
        self.root.title('EDL Plots')

        # Shared window lifecycle
        self.root.protocol('WM_DELETE_WINDOW', lambda: None)
        apply_tk_dark_theme(self.root)
        self.full_altitude_km = 1.0
        self.descent_detail = tk.BooleanVar(master=self.root)
        self.tabs = ttk.Notebook(self.root)
        self.tabs.pack(fill='both', expand=True)
        altitude_tab = ttk.Frame(self.tabs)
        track_tab = ttk.Frame(self.tabs)
        self.tabs.add(altitude_tab, text='Altitude plots')
        self.tabs.add(track_tab, text='Trajectory and ground track')
        self.track_tab = track_tab
        self.detail_control = ttk.Checkbutton(altitude_tab, text='Descent detail', variable=self.descent_detail, command=self.change_altitude_view)
        self.detail_control.state(['disabled'])
        self.detail_control.pack(anchor='e', padx=14, pady=(8, 0))
        with rc_context(DARK_PLOT_STYLE):
            self.figure = Figure(figsize=(12.6, 8.2), dpi=100)
            self.figure.subplots_adjust(left=.07, right=.97, bottom=.08, top=.94, wspace=.32, hspace=.46)
            grid = self.figure.add_gridspec(3, 3)
            self.track = TrajectoryPlot(Figure(figsize=(12.6, 8.2), dpi=100))
            self.altitude_axes = create_altitude_panels(self.figure, grid)
        self.track_canvas = FigureCanvasTkAgg(self.track.figure, master=track_tab)
        self.track_canvas.get_tk_widget().pack(fill='both', expand=True)
        self.tabs.bind('<<NotebookTabChanged>>', self.draw_selected_tab)
        self.canvas = FigureCanvasTkAgg(self.figure, master=altitude_tab)
        self.canvas.get_tk_widget().pack(fill='both', expand=True)
        self.canvas.draw()
        self.root.after(self.refresh_ms, self.refresh)

    def draw_selected_tab(self, _event=None):
        canvas = self.track_canvas if self.tabs.select() == str(self.track_tab) else self.canvas
        canvas.draw_idle()
        self.plots_dirty = False
        self.last_draw = monotonic()

    def set_altitude_limits(self, *, detail):
        lower, upper = 0.0, self.full_altitude_km
        if detail:
            deploy_altitude = self.deployment[0]
            if isfinite(deploy_altitude):
                upper = min(upper, max(1.0, deploy_altitude * .00125))
            else:
                # Zoom to the lowest part recorded if the run ended before deployment.
                lowest = min(self.altitude_axes[0].lines[0].get_ydata(), default=0.0)
                span = max(1.0, (upper - lowest) / 4)
                lower = max(0.0, lowest - .05 * span)
                upper = min(upper, lowest + span)
        self.altitude_axes[0].set_ylim(lower, upper)

    def change_altitude_view(self):
        self.set_altitude_limits(detail=self.descent_detail.get())
        self.draw_selected_tab()

    def save(self, path):
        if path is None or path == self.saved_path:
            return
        path = Path(path)
        detail_path = path.with_stem(path.stem + '_descent_detail')
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            for output, detail in ((path, False), (detail_path, True)):
                self.set_altitude_limits(detail=detail)
                self.figure.savefig(output, dpi=150, facecolor=self.figure.get_facecolor())
                print(f'Plots saved: {output}', flush=True)
            if self.track.has_ground_track:
                track_path = path.with_stem(path.stem + '_trajectory')
                self.track.figure.savefig(track_path, dpi=150)
                print(f'Plots saved: {track_path}', flush=True)
            self.saved_path = str(path)
        except OSError as error:
            print(f'Could not save plots: {error}', flush=True)
        finally:
            self.set_altitude_limits(detail=self.descent_detail.get())
            self.plots_dirty = True

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
        deployment = self.deployment[:]
        altitude = deployment[0]
        new_deployment = not self.deployment_marked and isfinite(altitude)
        if points:
            update_plot(self.altitude_axes, points)
            batch_top_km = max(point['altitude_m'] for point in points) * .00105
            self.full_altitude_km = max(self.full_altitude_km, batch_top_km)
        if points or new_deployment:
            self.track.update(points, deployment)
            self.set_altitude_limits(detail=self.descent_detail.get())
            self.plots_dirty = True
        if new_deployment:
            mark_deployment(self.altitude_axes, altitude)
            self.deployment_marked = True
            self.detail_control.state(['!disabled'])
        if action:
            self.save(path)
            if action == 'close':
                self.root.destroy()
                return
        if self.plots_dirty and monotonic() - self.last_draw >= .1:
            self.draw_selected_tab()
        self.root.after(self.refresh_ms, self.refresh)


def run_plot_window(samples, update_hz, deployment):
    window = FlightPlotWindow(samples, update_hz, deployment)
    window.root.mainloop()


class TrajectoryPlot:
    def __init__(self, figure):
        from matplotlib.ticker import MaxNLocator

        self.figure = figure
        figure.subplots_adjust(left=.035, right=.97, bottom=.10, top=.93, wspace=.25)
        self.space = figure.add_subplot(121, projection='3d', proj_type='ortho')
        self.space.set(title='3D trajectory · altitude at 3× scale', xlabel='Downrange (km)',
                       ylabel='Crossrange (km)', zlabel='Altitude (km)')
        self.space.view_init(elev=24, azim=-65)
        self.space.xaxis.labelpad = 24
        self.space.yaxis.set_major_locator(MaxNLocator(nbins=3))
        for axis in (self.space.xaxis, self.space.yaxis, self.space.zaxis):
            axis.pane.set_facecolor(PANEL)
        self.ground = figure.add_subplot(122)
        self.ground.set(title='Ground track', xlabel='Longitude (°E)  ·  West ← → East',
                        ylabel='Latitude (°N)')
        self.ground.grid(True, alpha=.55)
        self.path_3d, = self.space.plot([], [], [], color=ACCENT)
        self.path_2d, = self.ground.plot([], [], color=ACCENT)
        self.markers = []
        for label, marker, color, size in (('Chute deployment', 'D', TRACE_COLORS[1], 9),
                                           ('Touchdown', 'o', TRACE_COLORS[2], 5)):
            marker_3d, = self.space.plot([], [], [], marker=marker, color=color, linestyle='none')
            marker_2d, = self.ground.plot([], [], marker=marker, color=color, linestyle='none',
                                          markersize=size, label=label)
            self.markers.append((label, marker_3d, marker_2d))
        self.legend = self.ground.legend(loc='upper left', facecolor=PANEL, edgecolor=BORDER,
                                         labelcolor=TEXT, framealpha=1)
        self.data = np.empty((0, len(TRACK_FIELDS)))
        self.touchdown = None

    @property
    def has_ground_track(self):
        return len(self.data) > 0

    def update(self, samples, deployment):
        points = [tuple(row[key] for key in TRACK_FIELDS) for row in samples if 'longitude_deg' in row]
        if points:
            self.data = np.concatenate((self.data, points))
            if len(self.data) > 5000:
                self.data = self.data[np.linspace(0, len(self.data) - 1, 2500, dtype=int)]
            if points[-1][0] == 0:
                self.touchdown = points[-1]
        if not self.has_ground_track:
            return
        altitude, downrange, crossrange, longitude, latitude = self.data.T
        east = np.unwrap(longitude, period=360)
        self.path_3d.set_data_3d(downrange / 1000, crossrange / 1000, altitude / 1000)
        self.path_2d.set_data(east, latitude)
        events = (deployment, self.touchdown)
        for event, (label, marker_3d, marker_2d), text in zip(events, self.markers, self.legend.get_texts()):
            if event is not None and all(map(isfinite, event)):
                h, along, across, lon, lat = event
                text.set_text(f'{label} ({lat:+.3f}°, {lon:+.3f}°)')
                lon += 360 * round((east[np.argmin(abs(downrange - along))] - lon) / 360)
                marker_3d.set_data_3d([along / 1000], [across / 1000], [h / 1000])
                marker_2d.set_data([lon], [lat])
        self.space.auto_scale_xyz(downrange / 1000, crossrange / 1000, altitude / 1000)
        along_span = np.ptp(self.space.get_xlim())
        # Leave room for the crossrange axis labels even when lateral motion is small.
        across_limit = max(.5, .11 * along_span, 1.05 * np.abs(crossrange).max() / 1000)
        height = max(1, altitude.max() / 1000 * 1.05)
        self.space.set_ylim(-across_limit, across_limit)
        self.space.set_zlim(0, height)
        self.space.set_box_aspect((along_span, 2 * across_limit, 3 * height), zoom=.82)
        self.ground.relim()
        self.ground.autoscale_view()
        if np.ptp(latitude) < 1:
            self.ground.set_ylim(min(latitude[0] - .5, latitude.min()),
                                 max(latitude[0] + .5, latitude.max()), auto=True)


def create_altitude_panels(figure, grid):
    axes = []
    for index, (title, xlabel, _, channels) in enumerate(PANELS):
        ax = figure.add_subplot(grid[index // 3, index % 3], sharey=axes[0] if axes else None)
        axes.append(ax)
        for trace_index, (_, label) in enumerate(channels):
            ax.plot([], [], label=label, color=TRACE_COLORS[trace_index], linewidth=1.6)
        ax.set(title=title, xlabel=xlabel, ylim=(0, 1))
        if index % 3 == 0:
            ax.set_ylabel('Altitude (km)')
        else:
            ax.tick_params(labelleft=False)
        ax.margins(x=.1)
        ax.grid(True, alpha=.55)
        if len(channels) > 1:
            ax.legend(loc='upper right', fontsize=8, framealpha=1)
    return axes


def update_plot(axes, samples):
    """Append flight telemetry to the 2D altitude plots."""
    max_points = 5000
    new_altitude = np.asarray([row['altitude_m'] for row in samples], dtype=float) / 1000
    for ax, (_, _, factor, channels) in zip(axes, PANELS):
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


def mark_deployment(axes, altitude_m):
    """Mark the exact deployment altitude on the 2D flight plots."""
    altitude_km = altitude_m / 1000
    for ax in axes:
        ax.axhline(altitude_km, color=ACCENT, linestyle=':', linewidth=1, alpha=.75)
    drag_axis = next(ax for ax in axes if ax.get_title() == 'Drag force')
    drag_axis.annotate(f'Chute deploy · {altitude_km:.2f} km',
                       xy=(.22, altitude_km), xycoords=drag_axis.get_yaxis_transform(),
                       xytext=(0, 5), textcoords='offset points', ha='left',
                       va='bottom', color=ACCENT, fontsize=8)
