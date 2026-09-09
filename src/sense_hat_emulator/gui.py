"""One-window sensor controls and an API-driven 8x8 RGB display."""

import tkinter as tk
from tkinter import ttk, filedialog, messagebox

# Preserve direct script launch as well as installed package launch.
if __package__:
    from .emulator import SenseEmu
else:
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from sense_hat_emulator.emulator import SenseEmu


class GuiBinding:
    def __init__(self, sense, values, draw_pixels):
        self.sense = sense
        self.values = values
        self.draw_pixels = draw_pixels
        self.refreshing = False
        self.last_pixels = None
        for key, variable in values.items():
            variable.trace_add("write", self._on_change(key, variable))

    def _on_change(self, key, variable):
        def changed(*_args):
            if not self.refreshing and self.sense.mode != 'replay':
                self.sense._state.update({key: variable.get()})
        return changed

    def refresh(self):
        readings, pixels = self.sense._state.snapshot()
        self.refreshing = True
        try:
            for key, variable in self.values.items():
                value = readings[key]
                # API values may exceed the initial teaching ranges. Expand the
                # slider so the GUI represents the actual value, without clipping.
                slider = getattr(variable, "slider", None)
                if slider is not None:
                    low = min(float(slider.cget("from")), value)
                    high = max(float(slider.cget("to")), value)
                    slider.configure(from_=low, to=high)
                    label = getattr(variable, "range_label", None)
                    if label is not None:
                        label.configure(text=f"{variable.title} ({low:g} to {high:g})")
                if variable.get() != value:
                    variable.set(value)
        finally:
            self.refreshing = False
        if pixels != self.last_pixels:
            self.draw_pixels(pixels)
            self.last_pixels = pixels


def add_sensor(parent, row, title, unit, minimum, maximum, default, decimals=2):
    """One compact horizontal slider, with a label and live value."""
    frame = ttk.Frame(parent, padding=(6, 3))
    frame.grid(row=row, column=0, sticky="ew")
    parent.columnconfigure(0, weight=1)
    frame.columnconfigure(0, weight=1)
    value = tk.DoubleVar(master=parent, value=default)
    reading = tk.StringVar(master=parent)

    def update_reading(*_args):
        reading.set(f"{value.get():.{decimals}f} {unit}")

    value.trace_add("write", update_reading)
    value.title = title
    value.range_label = ttk.Label(frame, text=f"{title} ({minimum:g} to {maximum:g})")
    value.range_label.grid(row=0, column=0, sticky="w")
    ttk.Label(frame, textvariable=reading, width=12, anchor="e").grid(row=0, column=1)
    value.slider = ttk.Scale(frame, from_=minimum, to=maximum, orient="horizontal",
                             length=160, variable=value)
    value.slider.grid(row=1, column=0, columnspan=2, sticky="ew")
    update_reading()
    return value


class LEDMatrix(ttk.LabelFrame):
    """Render a snapshot of LED state; the API owns and validates the pixels."""

    def __init__(self, parent):
        super().__init__(parent, text="8 × 8 RGB LEDs", padding=10)
        self.pixels = [(0, 0, 0)] * 64
        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)
        self.canvas = tk.Canvas(self, width=224, height=224,
                                background="#171c24", highlightthickness=0)
        self.canvas.grid(row=0, column=0, sticky="nsew")
        self.canvas.bind("<Configure>", self.redraw)
        ttk.Label(self, text="(0, 0) top left · X → · Y ↓").grid(row=1, column=0, pady=(4, 0))

    def display_pixels(self, pixels):
        self.pixels = pixels
        self.redraw()

    def geometry_for_grid(self):
        width, height = self.canvas.winfo_width(), self.canvas.winfo_height()
        size = min(width, height)
        return (width - size) / 2, (height - size) / 2, size / 8

    def redraw(self, _event=None):
        left, top, cell = self.geometry_for_grid()
        self.canvas.delete("all")
        for index, rgb in enumerate(self.pixels):
            x, y = index % 8, index // 8
            gap = cell * 0.09
            self.canvas.create_rectangle(
                left + x * cell + gap, top + y * cell + gap,
                left + (x + 1) * cell - gap, top + (y + 1) * cell - gap,
                fill="#%02x%02x%02x" % tuple(rgb), outline="#394351",
            )


def build_gui(root, sense=None):
    """World controls or direct/replay inputs, all connected to one SenseEmu."""
    sense = SenseEmu() if sense is None else sense
    root.title('Sense HAT V2 Emulator')
    root.minsize(900, 550)
    root.columnconfigure(0, weight=1)
    root.rowconfigure(0, weight=1)
    panel = ttk.Frame(root, padding=8)
    panel.grid(row=0, column=0, sticky='nsew')
    panel.columnconfigure(1, weight=1)
    panel.rowconfigure(1, weight=1)

    toolbar = ttk.Frame(panel)
    toolbar.grid(row=0, column=0, columnspan=2, sticky='ew', pady=(0, 8))
    selected_mode = tk.StringVar(master=root, value=sense.mode)
    def select_mode():
        sense.set_mode(selected_mode.get())
    ttk.Radiobutton(toolbar, text='World', variable=selected_mode, value='world',
                    command=select_mode).grid(row=0, column=0)
    ttk.Radiobutton(toolbar, text='Direct inputs', variable=selected_mode, value='direct',
                    command=select_mode).grid(row=0, column=1)

    def load_recording():
        path = filedialog.askopenfilename(parent=root, title='Load sensor recording',
                                         filetypes=[('Sensor CSV', '*.csv'), ('All files', '*.*')])
        if path:
            try:
                sense.load_csv(path)  # Replay is exact by default; opt into noise below.
            except (OSError, ValueError, UnicodeError) as exc:
                messagebox.showerror('Cannot load recording', str(exc), parent=root)

    def pause_resume():
        status = sense.get_replay_status()
        if status['paused']:
            sense.resume_replay()
        else:
            sense.pause_replay()

    ttk.Button(toolbar, text='Load CSV…', command=load_recording).grid(row=0, column=2, padx=6)
    pause_button = ttk.Button(toolbar, text='Pause', command=pause_resume)
    pause_button.grid(row=0, column=3)
    restart_button = ttk.Button(toolbar, text='Restart', command=sense.restart_replay)
    restart_button.grid(row=0, column=4, padx=4)
    stop_button = ttk.Button(toolbar, text='Stop CSV', command=sense.stop_replay)
    stop_button.grid(row=0, column=5)

    left = ttk.Frame(panel)
    left.grid(row=1, column=0, sticky='ns', padx=(0, 8))
    left.columnconfigure(0, weight=1)
    matrix = LEDMatrix(left)
    matrix.grid(row=0, column=0, sticky='ew')
    readouts = ttk.LabelFrame(left, text='Orientation / gyro output', padding=6)
    readouts.grid(row=1, column=0, sticky='ew', pady=(8, 0))
    orientation_text = tk.StringVar(master=root)
    ttk.Label(readouts, textvariable=orientation_text, wraplength=230).grid(row=0, column=0, sticky='w')
    root.controls_area = ttk.Frame(left, height=30)
    root.controls_area.grid(row=2, column=0, sticky='nsew')
    left.rowconfigure(2, weight=1)

    sensors = ttk.Frame(panel)
    sensors.grid(row=1, column=1, sticky='nsew')
    for index in range(2):
        sensors.columnconfigure(index, weight=1, uniform='groups')
        sensors.rowconfigure(index, weight=1)
    values = {}
    world_groups, direct_groups = [], []

    def add_group(row, column, title, settings):
        group = ttk.LabelFrame(sensors, text=title, padding=3)
        group.grid(row=row, column=column, sticky='nsew', padx=3, pady=3)
        for position, sensor in enumerate(settings):
            key, label, unit, low, high, default, decimals = sensor
            values[key] = add_sensor(group, position, label, unit, low, high, default, decimals)
        return group

    add_group(0, 0, 'Environment targets', [
        ('temperature', 'Temperature', '°C', 0, 65, 20.0, 2),
        ('pressure', 'Pressure', 'hPa', 260, 1260, 1013.25, 2),
        ('humidity', 'Humidity', '%', 0, 100, 0.0, 1),
    ])
    for row, column, title, prefix, unit, low, high, initial in [
        (0, 1, 'World magnetic field', 'world_mag', 'µT', -100, 100, (33, 0, 0)),
        (1, 0, 'World acceleration input', 'world_accel', 'g', -8, 8, (0, 0, 1)),
        (0, 1, 'Board magnetic field', 'mag', 'µT', -100, 100, (33, 0, 0)),
        (1, 0, 'Board acceleration', 'accel', 'g', -8, 8, (0, 0, 1)),
        (1, 1, 'Board gyroscope', 'gyro', 'rad/s', -9, 9, (0, 0, 0)),
    ]:
        group = add_group(row, column, title, [
            (f'{prefix}_{axis}', axis.upper(), unit, low, high, default, 2)
            for axis, default in zip('xyz', initial)
        ])
        (world_groups if prefix.startswith('world_') else direct_groups).append(group)
    world_groups.append(add_group(1, 1, 'Commanded orientation', [
        (axis, axis.title(), '°', -180, 180, 0, 1) for axis in ('roll', 'pitch', 'yaw')
    ]))

    binding = GuiBinding(sense, values, matrix.display_pixels)
    noise_enabled = tk.BooleanVar(master=root, value=sense.noise)
    ttk.Checkbutton(panel, text='Sensor noise', variable=noise_enabled,
                    command=lambda: setattr(sense, 'noise', noise_enabled.get())).grid(
        row=2, column=0, sticky='w', pady=6)
    ttk.Button(panel, text='Reset sensors', command=sense.reset_sensors).grid(row=2, column=1, sticky='e')
    status_text = tk.StringVar(master=root)
    ttk.Label(panel, textvariable=status_text, wraplength=850).grid(
        row=3, column=0, columnspan=2, sticky='w')
    root.sense = sense
    root.led_matrix = sense
    timer = None

    def refresh():
        nonlocal timer
        binding.refresh()
        mode = sense.mode
        selected_mode.set(mode)
        noise_enabled.set(sense.noise)
        for group in world_groups:
            group.grid() if mode == 'world' else group.grid_remove()
        for group in direct_groups:
            group.grid_remove() if mode == 'world' else group.grid()
        for variable in values.values():
            variable.slider.state(['disabled'] if mode == 'replay' else ['!disabled'])
        status = sense.get_replay_status()
        for button in (pause_button, restart_button, stop_button):
            button.state(['!disabled'] if mode == 'replay' else ['disabled'])
        pause_button.configure(text='Resume' if status['paused'] else 'Pause')
        pause_button.state(['disabled'] if status['finished'] or mode != 'replay' else ['!disabled'])
        if mode == 'replay':
            phase = 'Finished — holding last sample' if status['finished'] else 'Paused' if status['paused'] else 'Playing'
            status_text.set(f"CSV: {phase} | {status['time_s']:.3f} s | row {status['sample']+1}/{status['samples']} | "
                            f"noise {'ON' if sense.noise else 'OFF'}")
        elif mode == 'world':
            status_text.set('World vectors rotate into board axes. Orientation changes generate gyro rates. Sliders set targets.')
        else:
            status_text.set('Direct board-axis inputs bypass world rotation. Orientation remains explicitly commanded.')
        try:
            angles = sense.get_orientation_degrees()
            text = '\n'.join(f'{axis.title()}: {angles[axis]:.1f}°' for axis in ('pitch', 'roll', 'yaw'))
        except ValueError:
            text = 'Orientation unavailable\nCSV has no angle columns'
        gyro = sense.get_gyroscope_raw()
        text += '\n\nGyro (rad/s)\n' + '\n'.join(f'{axis.upper()}: {gyro[axis]:.4f}' for axis in 'xyz')
        orientation_text.set(text)
        timer = root.after(33, refresh)

    def on_destroy(event):
        if event.widget is root and timer is not None:
            root.after_cancel(timer)
    root.bind('<Destroy>', on_destroy, add='+')
    refresh()
    return values


def main(sense=None):
    root = tk.Tk()
    build_gui(root, sense)
    root.mainloop()


if __name__ == '__main__':
    main()
