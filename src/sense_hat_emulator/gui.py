"""One-window sensor controls and an API-driven 8x8 RGB display."""

import tkinter as tk
from tkinter import ttk

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
            if not self.refreshing:
                # One-axis edits must not overwrite other axes updated by code.
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
    """Matrix and reserved space on the left; four sensor groups on the right."""
    sense = SenseEmu() if sense is None else sense
    root.title("Sense HAT V2 Emulator")
    root.minsize(860, 440)
    root.columnconfigure(0, weight=1)
    root.rowconfigure(0, weight=1)
    panel = ttk.Frame(root, padding=8)
    panel.grid(row=0, column=0, sticky="nsew")
    panel.columnconfigure(1, weight=1)
    panel.rowconfigure(0, weight=1)

    left = ttk.Frame(panel)
    left.grid(row=0, column=0, sticky="ns", padx=(0, 8))
    left.columnconfigure(0, weight=1)
    left.rowconfigure(1, weight=1)
    matrix = LEDMatrix(left)
    matrix.grid(row=0, column=0, sticky="ew")
    orientation_panel = ttk.LabelFrame(left, text="Orientation estimate", padding=6)
    orientation_panel.grid(row=1, column=0, sticky="ew", pady=(8, 0))
    orientation_text = tk.StringVar(master=root)
    ttk.Label(orientation_panel, textvariable=orientation_text).grid(row=0, column=0, sticky="w")
    ttk.Label(orientation_panel, text="Static accel + magnetic estimate.\nGyroscope is not integrated.",
              wraplength=220).grid(row=1, column=0, sticky="w", pady=(4, 0))
    # Keep an empty area below the readouts available for future controls.
    left.rowconfigure(1, weight=0)
    left.rowconfigure(2, weight=1)
    root.controls_area = ttk.Frame(left, height=30)
    root.controls_area.grid(row=2, column=0, sticky="nsew", pady=(8, 0))

    sensors = ttk.Frame(panel)
    sensors.grid(row=0, column=1, sticky="nsew")
    for index in range(2):
        sensors.columnconfigure(index, weight=1, uniform="groups")
        sensors.rowconfigure(index, weight=1)
    values = {}

    def add_group(row, column, title, settings):
        group = ttk.LabelFrame(sensors, text=title, padding=3)
        group.grid(row=row, column=column, sticky="nsew", padx=3, pady=3)
        for position, sensor in enumerate(settings):
            key, label, unit, low, high, default, decimals = sensor
            values[key] = add_sensor(group, position, label, unit, low, high, default, decimals)

    add_group(0, 0, "Environment", [
        ("temperature", "Temperature", "°C", 0, 65, 20.0, 2),
        ("pressure", "Pressure", "hPa", 260, 1260, 1013.25, 2),
        ("humidity", "Humidity", "%", 0, 100, 0.0, 1),
    ])
    groups = [
        (0, 1, "Magnetic field", "mag", "µT", -100, 100, (25, 0, 0)),
        (1, 0, "Acceleration", "accel", "g", -8, 8, (0, 0, 1)),
        (1, 1, "Gyroscope", "gyro", "rad/s", -35, 35, (0, 0, 0)),
    ]
    for row, column, title, prefix, unit, low, high, initial in groups:
        add_group(row, column, title, [
            (f"{prefix}_{axis}", axis.upper(), unit, low, high, default, 2)
            for axis, default in zip("xyz", initial)
        ])

    binding = GuiBinding(sense, values, matrix.display_pixels)
    binding.refresh()

    def reset_sensors():
        sense.reset_sensors()
        binding.refresh()

    ttk.Button(panel, text="Reset sensors", command=reset_sensors).grid(
        row=1, column=1, sticky="e", pady=(6, 0)
    )
    root.sense = sense
    root.led_matrix = sense  # Keep earlier examples using this reference working.
    timer = None

    def refresh():
        nonlocal timer
        binding.refresh()
        try:
            angles = sense.get_orientation_degrees()
            orientation_text.set("\n".join(f"{axis.title()}: {angles[axis]:.1f}°" for axis in ("pitch", "roll", "yaw")))
        except ValueError:
            orientation_text.set("Orientation unavailable\nCheck acceleration / magnetic field")
        timer = root.after(33, refresh)

    def on_destroy(event):
        if event.widget is root and timer is not None:
            root.after_cancel(timer)

    root.bind("<Destroy>", on_destroy, add="+")
    refresh()
    return values


def main(sense=None):
    root = tk.Tk()
    build_gui(root, sense)
    root.mainloop()


if __name__ == "__main__":
    main()
