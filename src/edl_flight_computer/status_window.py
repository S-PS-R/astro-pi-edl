"""Read-only mission dashboard in a separate process; closing it never stops flight."""

from math import isfinite
from multiprocessing import get_context
from queue import Empty, Full
from time import monotonic


def _format(value):
    if value is None:
        return '—'
    if not isfinite(value) or value == -1e6:
        return 'OVR / invalid'
    return f'{value:.4f}'


def _show_window(updates):
    # Tk owns the child process's main thread. It never imports/opens hardware.
    try:
        import tkinter as tk
        from tkinter import ttk
    except ImportError:
        print('Tkinter unavailable; flight program continues in terminal.', flush=True)
        return

    try:
        root = tk.Tk()
    except tk.TclError as exc:
        print(f'Status window unavailable ({exc}); flight program continues.', flush=True)
        return
    root.title('EDL — Flight computer status')
    root.minsize(500, 490)
    panel = ttk.Frame(root, padding=16)
    panel.pack(fill='both', expand=True)
    panel.columnconfigure(1, weight=1)
    ttk.Label(panel, text='Flight computer', font=('Segoe UI', 15, 'bold')).grid(
        row=0, column=0, columnspan=2, sticky='w', pady=(0, 12))
    source, phase, elapsed, age = (tk.StringVar(value='—') for _ in range(4))
    connection = tk.StringVar(value='Waiting for telemetry')
    connection_label = ttk.Label(panel, textvariable=connection)
    connection_label.grid(row=1, column=0, columnspan=2, sticky='w')
    for row, (label, variable) in enumerate((('Source', source), ('Phase', phase),
                                             ('Mission time (s)', elapsed), ('Telemetry age (s)', age)), 2):
        ttk.Label(panel, text=label).grid(row=row, column=0, sticky='w', padx=(0, 20))
        ttk.Label(panel, textvariable=variable).grid(row=row, column=1, sticky='w')
    ttk.Separator(panel).grid(row=6, column=0, columnspan=2, sticky='ew', pady=12)
    fields = (
        ('Temperature — humidity (°C)', ('temperature_humidity',)),
        ('Temperature — pressure (°C)', ('temperature_pressure',)),
        ('Pressure (hPa)', ('pressure',)), ('Humidity (%)', ('humidity',)),
        ('Acceleration X / Y / Z (g)', ('accel_x', 'accel_y', 'accel_z')),
        ('Gyroscope X / Y / Z (rad/s)', ('gyro_x', 'gyro_y', 'gyro_z')),
        ('Magnetic X / Y / Z (µT)', ('mag_x', 'mag_y', 'mag_z')),
    )
    values = []
    for row, (label, keys) in enumerate(fields, 7):
        variable = tk.StringVar(value='—')
        values.append((variable, keys))
        ttk.Label(panel, text=label).grid(row=row, column=0, sticky='w', pady=5, padx=(0, 20))
        ttk.Label(panel, textvariable=variable).grid(row=row, column=1, sticky='w')
    error = tk.StringVar()
    ttk.Label(panel, textvariable=error, foreground='#b3261e', wraplength=480).grid(
        row=14, column=0, columnspan=2, sticky='w', pady=(10, 0))
    ttk.Label(panel, text='Closing this window leaves the flight program running.\n'
                         'Stop the flight program with Ctrl+C in its terminal.', wraplength=480).grid(
        row=15, column=0, columnspan=2, sticky='w', pady=(12, 0))
    latest = None

    def refresh():
        nonlocal latest
        while True:
            try:
                message = updates.get_nowait()
            except Empty:
                break
            if message is None:
                root.destroy()
                return
            latest = message
        if latest is not None:
            stale = monotonic() - latest['updated_at'] > 2
            connected = latest['connected'] and not stale
            connection.set('Connected' if connected else 'Telemetry stale' if stale else 'Sensor read failed')
            connection_label.configure(foreground='#137333' if connected else '#b3261e')
            source.set(latest['source'])
            phase.set(latest['phase'].upper())
            elapsed.set(f"{latest['elapsed_s']:.1f}")
            age.set(f"{max(0, monotonic() - latest['updated_at']):.1f}")
            error.set(latest.get('error', ''))
            for variable, keys in values:
                variable.set(' / '.join(_format(latest['readings'].get(key)) for key in keys))
        root.after(100, refresh)

    root.protocol('WM_DELETE_WINDOW', root.destroy)
    root.after(0, refresh)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        root.destroy()


class StatusWindow:
    """Bounded latest-value channel: a slow or closed GUI cannot block the loop."""

    def __init__(self):
        context = get_context('spawn')
        self.updates = context.Queue(maxsize=1)
        self.process = context.Process(target=_show_window, args=(self.updates,), daemon=True)
        self.process.start()

    def publish(self, snapshot):
        if not self.process.is_alive():
            return
        try:
            self.updates.put_nowait(snapshot)
        except Full:
            try:
                self.updates.get_nowait()
            except Empty:
                return
            try:
                self.updates.put_nowait(snapshot)
            except Full:
                pass

    def close(self):
        if self.process.is_alive():
            self.publish(None)
            self.process.join(timeout=1)
            if self.process.is_alive():
                self.process.terminate()
                self.process.join(timeout=1)
        self.updates.cancel_join_thread()
        self.updates.close()
