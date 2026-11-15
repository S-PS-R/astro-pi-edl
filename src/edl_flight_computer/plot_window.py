"""Render and save flight plots in a process with its own Tk event loop."""

from multiprocessing import get_context
from queue import Empty, Full
from pathlib import Path
import numpy as np


class PlotProcess:
    """Nonblocking sample delivery; plotting never runs on the mission thread."""

    def __init__(self, update_hz=3.0):
        context = get_context('spawn')
        self.update_hz = context.Value('d', update_hz)
        self.samples = context.Queue(maxsize=256)
        self.process = context.Process(target=run_plot_window, args=(self.samples, self.update_hz),
                                       name='EDL plots', daemon=True)
        self.last_sample = None
        self.process.start()

    def set_update_hz(self, update_hz):
        self.update_hz.value = update_hz

    def add_sample(self, sample):
        self.last_sample = sample
        message = ('sample', sample)
        try:
            self.samples.put_nowait(message)
        except Full:
            # Drop an old plotting point rather than hold up flight calculations.
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
        """Wait only during shutdown, giving the plotter time to save its PNG."""
        try:
            if self.process.is_alive():
                self.samples.put(('close', str(path) if path else None,
                                  self.last_sample), timeout=10)
                self.process.join(timeout=10)
            if self.process.is_alive():
                print('Plot process did not shut down in time; PNG saving may be incomplete.',
                      flush=True)
                self.process.terminate()
                self.process.join(timeout=2)
            elif self.process.exitcode:
                print(f'Plot process exited with code {self.process.exitcode}.', flush=True)
        except Full:
            print('Plot process stopped accepting data; PNG could not be saved.', flush=True)
            self.process.terminate()
            self.process.join(timeout=2)
        finally:
            self.samples.cancel_join_thread()
            self.samples.close()


class FlightPlotWindow:
    def __init__(self, samples, update_hz):
        # Import GUI dependencies only in the child process.
        import tkinter as tk
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

        self.samples = samples
        self.background = None
        self.saving = False
        self.saved_path = None
        self.update_hz = update_hz
        self.refresh_ms = round(1000 / update_hz.value)
        self.root = tk.Tk()
        self.root.title('Live EDL Flight Plots')
        # The mission window owns shutdown for all displays.
        self.root.protocol('WM_DELETE_WINDOW', lambda: None)
        self.figure = Figure(figsize=(10, 8), dpi=100)
        self.figure.subplots_adjust(left=0.10, right=0.97, bottom=0.09,
                                    top=0.94, wspace=0.32, hspace=0.40)
        self.canvas = FigureCanvasTkAgg(self.figure, master=self.root)
        self.canvas.get_tk_widget().pack(fill='both', expand=True)
        update_plot(self.figure)
        for ax in self.figure.axes:
            ax.lines[0].set_animated(True)
        self.canvas.mpl_connect('draw_event', self.on_draw)
        self.canvas.draw()
        self.root.after(self.refresh_ms, self.refresh)

    def on_draw(self, event):
        if not self.saving:
            self.background = self.canvas.copy_from_bbox(self.figure.bbox)
            self.draw_lines()

    def draw_lines(self):
        self.canvas.restore_region(self.background)
        for ax in self.figure.axes:
            ax.draw_artist(ax.lines[0])
        self.canvas.blit(self.figure.bbox)

    def save(self, path):
        if path == self.saved_path:
            return
        lines = [ax.lines[0] for ax in self.figure.axes]
        self.saving = True
        try:
            path = Path(path)
            path.parent.mkdir(parents=True, exist_ok=True)
            for line in lines:
                line.set_animated(False)
            self.figure.savefig(path, dpi=150)
            self.saved_path = str(path)
            print(f'Plots saved: {path}', flush=True)
        except OSError as error:
            print(f'Could not save plots: {error}', flush=True)
        finally:
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
                # An empty queue just means the plot has caught up.
                break
            if message[0] == 'sample':
                points.append(message[1])
            else:
                action, path, last_sample = message
                if last_sample is not None and (not points or points[-1] != last_sample):
                    points.append(last_sample)
                # Save only after consuming all earlier samples in FIFO order.
                break
        limits = [(ax.get_xlim(), ax.get_ylim()) for ax in self.figure.axes]
        if points:
            update_plot(self.figure, points)
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


def run_plot_window(samples, update_hz):
    window = FlightPlotWindow(samples, update_hz=update_hz)
    window.root.mainloop()


def update_plot(figure, samples=()):
    """Append (altitude, density, velocity, heating, acceleration) samples.

    Call on the GUI thread. Heating is in kW/m^2; other inputs use SI units.
    """
    max_points = 5000
    if not figure.axes:
        axes = figure.subplots(2, 2, sharey=True).flat
        labels = (
            ('Altitude vs Density', 'Density (kg/m³)'),
            ('Altitude vs Velocity', 'Velocity (m/s)'),
            ('Altitude vs Heating', 'Convective heat flux (kW/m²)'),
            ('Altitude vs Acceleration', 'Acceleration (m/s²)'),
        )
        for ax, (title, xlabel) in zip(axes, labels):
            ax.plot([], [], linewidth=1.5)
            ax.set_title(title)
            ax.set_xlabel(xlabel)
            ax.set_ylabel('Altitude (m)')
            ax.set_ylim(0, 1)
            ax.margins(x=0.1)
            ax.grid(True, alpha=0.3)

    samples = list(samples)
    if not samples:
        return

    for column, ax in enumerate(figure.axes, start=1):
        line = ax.lines[0]
        x, y = line.get_data()
        new_x = np.array([row[column]
                          for row in samples])
        x_data = np.concatenate((x, new_x))
        y_data = np.concatenate((y, [row[0] for row in samples]))
        # Thin older history rather than allowing rendering cost to grow forever.
        if len(x_data) > max_points:
            indices = np.linspace(0, len(x_data) - 1, max_points // 2, dtype=int)
            x_data, y_data = x_data[indices], y_data[indices]
        line.set_data(x_data, y_data)

        # Keep axes steady until a new value exceeds the current view. This
        # allows the GUI to redraw only the curves between axis changes.
        finite = new_x[np.isfinite(new_x)]
        left, right = ax.get_xlim()
        if len(finite) and (not len(x) or finite.min() < left or finite.max() > right):
            ax.relim()
            ax.autoscale_view(scalex=True, scaley=False)

    upper = max(1.0, max(row[0] for row in samples) * 1.05)
    if upper > figure.axes[0].get_ylim()[1]:
        figure.axes[0].set_ylim(0, upper)
