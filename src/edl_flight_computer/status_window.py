"""Read-only status display. Closing it does not stop the flight program."""

from math import isfinite
from multiprocessing import get_context
from queue import Empty, Full
from time import monotonic


def format_reading(value):
    if value is None:
        return '—'
    if not isfinite(value) or value == -1e6:
        return 'OVR / invalid'
    return f'{value:.4f}'


def format_vector(readings, name):
    x = format_reading(readings.get(name + '_x'))
    y = format_reading(readings.get(name + '_y'))
    z = format_reading(readings.get(name + '_z'))
    return f'{x} / {y} / {z}'


class StatusDisplay:
    """The widgets and refresh loop, running only in the window process."""

    def __init__(self, root, updates, stop_requested):
        import tkinter as tk
        from tkinter import ttk

        self.root = root
        self.updates = updates
        self.stop_requested = stop_requested
        self.latest = None

        root.title('EDL — Flight computer status')
        root.minsize(500, 490)
        panel = ttk.Frame(root, padding=16)
        panel.pack(fill='both', expand=True)
        panel.columnconfigure(1, weight=1)
        ttk.Label(panel, text='Flight computer', font=('Segoe UI', 15, 'bold')).grid(
            row=0, column=0, columnspan=2, sticky='w', pady=(0, 12))

        self.connection = tk.StringVar(master=root, value='Waiting for telemetry')
        self.connection_label = ttk.Label(panel, textvariable=self.connection)
        self.connection_label.grid(row=1, column=0, columnspan=2, sticky='w')
        self.source = self.add_row(panel, 2, 'Source')
        self.phase = self.add_row(panel, 3, 'Phase')
        self.elapsed = self.add_row(panel, 4, 'Mission time (s)')
        self.age = self.add_row(panel, 5, 'Telemetry age (s)')
        ttk.Separator(panel).grid(row=6, column=0, columnspan=2, sticky='ew', pady=12)

        self.humidity_temperature = self.add_row(panel, 7, 'Temperature — humidity (°C)')
        self.pressure_temperature = self.add_row(panel, 8, 'Temperature — pressure (°C)')
        self.pressure = self.add_row(panel, 9, 'Pressure (hPa)')
        self.humidity = self.add_row(panel, 10, 'Humidity (%)')
        self.acceleration = self.add_row(panel, 11, 'Acceleration X / Y / Z (g)')
        self.gyroscope = self.add_row(panel, 12, 'Gyroscope X / Y / Z (rad/s)')
        self.magnetic_field = self.add_row(panel, 13, 'Magnetic X / Y / Z (µT)')

        self.error = tk.StringVar(master=root)
        ttk.Label(panel, textvariable=self.error, foreground='#b3261e', wraplength=480).grid(
            row=14, column=0, columnspan=2, sticky='w', pady=(10, 0))
        ttk.Label(panel, text='Closing this window leaves the flight program running.\n'
                             'Stop the flight program with Ctrl+C in its terminal.').grid(
            row=15, column=0, columnspan=2, sticky='w', pady=(12, 0))
        root.protocol('WM_DELETE_WINDOW', root.destroy)

    def add_row(self, parent, row, label):
        from tkinter import StringVar, ttk

        text = StringVar(master=self.root, value='—')
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky='w', padx=(0, 20), pady=3)
        ttk.Label(parent, textvariable=text).grid(row=row, column=1, sticky='w')
        return text

    def update_labels(self, snapshot):
        telemetry_age = max(0, monotonic() - snapshot['updated_at'])
        if telemetry_age > 2:
            self.connection.set('Telemetry stale')
            self.connection_label.configure(foreground='#b3261e')
        elif not snapshot['connected']:
            self.connection.set('Sensor read failed')
            self.connection_label.configure(foreground='#b3261e')
        else:
            self.connection.set('Connected')
            self.connection_label.configure(foreground='#137333')

        self.source.set(snapshot['source'])
        self.phase.set(snapshot['phase'].upper())
        self.elapsed.set(f"{snapshot['elapsed_s']:.1f}")
        self.age.set(f'{telemetry_age:.1f}')
        self.error.set(snapshot.get('error', ''))

        readings = snapshot['readings']
        self.humidity_temperature.set(format_reading(readings.get('temperature_humidity')))
        self.pressure_temperature.set(format_reading(readings.get('temperature_pressure')))
        self.pressure.set(format_reading(readings.get('pressure')))
        self.humidity.set(format_reading(readings.get('humidity')))
        self.acceleration.set(format_vector(readings, 'accel'))
        self.gyroscope.set(format_vector(readings, 'gyro'))
        self.magnetic_field.set(format_vector(readings, 'mag'))

    def refresh(self):
        if self.stop_requested.is_set():
            self.root.destroy()
            return

        # An empty queue just means no new snapshot has arrived yet.
        try:
            self.latest = self.updates.get_nowait()
        except Empty:
            pass

        if self.latest is not None:
            self.update_labels(self.latest)
        self.root.after(100, self.refresh)


def show_window(updates, stop_requested):
    # Import Tk here so the flight process does not need to initialize a GUI.
    try:
        import tkinter as tk
    except ImportError:
        print('Tkinter unavailable; flight program continues in terminal.', flush=True)
        return
    try:
        root = tk.Tk()
    except tk.TclError as error:
        print(f'Cannot open status window: {error}. Flight program continues.', flush=True)
        return

    display = StatusDisplay(root, updates, stop_requested)
    display.refresh()
    try:
        root.mainloop()
    except KeyboardInterrupt:
        root.destroy()


class StatusWindow:
    """Used by run.py to start the window and send snapshots without waiting."""

    def __init__(self):
        context = get_context('spawn')
        self.updates = context.Queue(maxsize=1)
        self.stop_requested = context.Event()
        self.process = context.Process(target=show_window,
                                       args=(self.updates, self.stop_requested), daemon=True)
        self.process.start()

    def publish(self, snapshot):
        if not self.process.is_alive():
            return
        try:
            self.updates.put_nowait(snapshot)
        except Full:
            # One snapshot is already waiting. Skip this display update;
            # never make sensor acquisition wait for the window.
            pass

    def close(self):
        self.stop_requested.set()
        self.process.join(timeout=1)
        if self.process.is_alive():
            self.process.terminate()
            self.process.join(timeout=1)
        self.updates.cancel_join_thread()
        self.updates.close()
