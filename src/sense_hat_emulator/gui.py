"""Sensor controls, CSV playback and the emulator's 8x8 LED display."""

import tkinter as tk
from tkinter import ttk, filedialog, messagebox

# Allow both python -m sense_hat_emulator and direct execution of this file.
if __package__:
    from .emulator import SenseEmu, OVR
    from .imu import ACCEL_LIMIT, GYRO_LIMIT, MAG_LIMIT
else:
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from sense_hat_emulator.emulator import SenseEmu, OVR
    from sense_hat_emulator.imu import ACCEL_LIMIT, GYRO_LIMIT, MAG_LIMIT


class EmulatorGUI:
    """Keep this window's widgets together; sensor calculations stay in the models."""

    def __init__(self, root, sense):
        self.root = root
        self.sense = sense
        self.controls = {}
        self.timer = None
        self.previous_pixels = None
        self.initial_values, _ = sense._state.snapshot()

        root.title('Sense HAT V2 Emulator')
        root.minsize(900, 550)
        root.columnconfigure(0, weight=1)
        root.rowconfigure(0, weight=1)
        panel = ttk.Frame(root, padding=10)
        panel.grid(row=0, column=0, sticky='nsew')
        panel.columnconfigure(1, weight=1)
        panel.rowconfigure(1, weight=1)

        # Playback buttons.
        toolbar = ttk.Frame(panel)
        toolbar.grid(row=0, column=0, columnspan=2, sticky='w', pady=(0, 8))
        ttk.Button(toolbar, text='Load CSV', command=self.load_csv).grid(row=0, column=0, padx=3)
        self.pause_button = ttk.Button(toolbar, text='Pause', command=self.pause_or_resume)
        self.pause_button.grid(row=0, column=1, padx=3)
        self.restart_button = ttk.Button(toolbar, text='Restart', command=sense.restart_replay)
        self.restart_button.grid(row=0, column=2, padx=3)
        self.stop_button = ttk.Button(toolbar, text='Stop CSV', command=sense.stop_replay)
        self.stop_button.grid(row=0, column=3, padx=3)

        # Create the 64 LED squares once; refresh only changes their colours.
        led_frame = ttk.LabelFrame(panel, text='8 x 8 RGB LEDs', padding=10)
        led_frame.grid(row=1, column=0, sticky='nw', padx=(0, 10))
        self.canvas = tk.Canvas(led_frame, width=224, height=224,
                                background='#171c24', highlightthickness=0)
        self.canvas.pack()
        ttk.Label(led_frame, text='(0, 0) top left; X right, Y down').pack(pady=(6, 0))
        self.led_squares = []
        for y in range(8):
            for x in range(8):
                square = self.canvas.create_rectangle(
                    x * 28 + 3, y * 28 + 3, x * 28 + 25, y * 28 + 25,
                    fill='#000000', outline='#394351')
                self.led_squares.append(square)

        # Four groups of independent sensor inputs.
        sensors = ttk.Frame(panel)
        sensors.grid(row=1, column=1, sticky='nsew')
        sensors.columnconfigure(0, weight=1)
        sensors.columnconfigure(1, weight=1)
        environment = ttk.LabelFrame(sensors, text='Environment targets', padding=6)
        magnetic = ttk.LabelFrame(sensors, text='Board magnetic field', padding=6)
        acceleration = ttk.LabelFrame(sensors, text='Board acceleration', padding=6)
        gyroscope = ttk.LabelFrame(sensors, text='Board gyroscope', padding=6)
        environment.grid(row=0, column=0, sticky='nsew', padx=3, pady=3)
        magnetic.grid(row=0, column=1, sticky='nsew', padx=3, pady=3)
        acceleration.grid(row=1, column=0, sticky='nsew', padx=3, pady=3)
        gyroscope.grid(row=1, column=1, sticky='nsew', padx=3, pady=3)
        self.add_sensor(environment, 0, 'temperature', 'Temperature', '°C', -40, 120, 2)
        self.add_sensor(environment, 1, 'pressure', 'Pressure', 'hPa', 260, 1260, 2)
        self.add_sensor(environment, 2, 'humidity', 'Humidity', '%', 0, 100, 1)
        self.add_vector(magnetic, 'mag', 'µT', MAG_LIMIT, 2)
        self.add_vector(acceleration, 'accel', 'g', ACCEL_LIMIT, 4)
        self.add_vector(gyroscope, 'gyro', 'rad/s', GYRO_LIMIT, 4)

        # Noise and reset controls.
        self.noise_enabled = tk.BooleanVar(master=root, value=sense.noise)
        ttk.Checkbutton(panel, text='Sensor noise', variable=self.noise_enabled,
                        command=self.change_noise).grid(row=2, column=0, sticky='w', pady=8)
        ttk.Button(panel, text='Reset sensors', command=sense.reset_sensors).grid(
            row=2, column=1, sticky='e')
        self.status = tk.StringVar(master=root)
        ttk.Label(panel, textvariable=self.status, wraplength=850).grid(
            row=3, column=0, columnspan=2, sticky='w')

        root.bind('<Destroy>', self.on_destroy, add='+')
        self.refresh()

    def add_sensor(self, parent, row, name, label, unit, minimum, maximum, decimals):
        """One slider changes a target; its numeric label shows the sampled reading."""
        frame = ttk.Frame(parent, padding=(0, 4))
        frame.grid(row=row, column=0, sticky='ew')
        parent.columnconfigure(0, weight=1)
        frame.columnconfigure(0, weight=1)
        target_value = tk.DoubleVar(master=self.root, value=self.initial_values[name])
        reading_text = tk.StringVar(master=self.root)

        def change_value(value):
            if self.sense.mode != 'replay':
                self.sense._state.update({name: float(value)})

        ttk.Label(frame, text=f'{label} ({minimum:g} to {maximum:g})').grid(
            row=0, column=0, sticky='w')
        ttk.Label(frame, textvariable=reading_text, width=14, anchor='e').grid(row=0, column=1)
        slider = ttk.Scale(frame, from_=minimum, to=maximum, orient='horizontal',
                           variable=target_value, command=change_value)
        slider.grid(row=1, column=0, columnspan=2, sticky='ew')
        self.controls[name] = {
            'target': target_value, 'reading': reading_text, 'slider': slider,
            'unit': unit, 'decimals': decimals,
        }

    def add_vector(self, parent, prefix, unit, limit, decimals):
        for row, axis in enumerate('xyz'):
            self.add_sensor(parent, row, f'{prefix}_{axis}', axis.upper(),
                            unit, -limit, limit, decimals)

    def load_csv(self):
        path = filedialog.askopenfilename(parent=self.root, title='Load sensor recording',
                                         filetypes=[('Sensor CSV', '*.csv')])
        if not path:
            return
        # A bad user-selected file should show a useful message, not close the GUI.
        try:
            self.sense.load_csv(path)
        except (OSError, ValueError, UnicodeError) as error:
            messagebox.showerror('Cannot load recording', str(error), parent=self.root)

    def pause_or_resume(self):
        playback = self.sense.get_replay_status()
        if playback['paused']:
            self.sense.resume_replay()
        else:
            self.sense.pause_replay()

    def change_noise(self):
        self.sense.noise = self.noise_enabled.get()

    def refresh(self):
        """Read current values, update widgets, then schedule the next display refresh."""
        targets, pixels = self.sense._state.snapshot()
        readings = self.sense.get_readings()
        playback = self.sense.get_replay_status()
        replay_active = self.sense.mode == 'replay'

        for name, control in self.controls.items():
            # Changing the Tk variable does not invoke the slider's mouse command,
            # so external API updates do not get written back into the sensor model.
            control['target'].set(targets[name])
            value = readings[name]
            if value == OVR:
                control['reading'].set('OVR')
            else:
                control['reading'].set(f"{value:.{control['decimals']}f} {control['unit']}")
            control['slider'].configure(state='disabled' if replay_active else 'normal')

        if pixels != self.previous_pixels:
            for square, colour in zip(self.led_squares, pixels):
                red, green, blue = colour
                colour_code = f'#{red:02x}{green:02x}{blue:02x}'
                self.canvas.itemconfigure(square, fill=colour_code)
            self.previous_pixels = pixels

        self.noise_enabled.set(self.sense.noise)
        button_state = 'normal' if replay_active else 'disabled'
        self.restart_button.configure(state=button_state)
        self.stop_button.configure(state=button_state)
        self.pause_button.configure(text='Resume' if playback['paused'] else 'Pause')
        if replay_active and not playback['finished']:
            self.pause_button.configure(state='normal')
        else:
            self.pause_button.configure(state='disabled')

        if replay_active:
            if playback['finished']:
                description = 'Finished - holding final target'
            elif playback['paused']:
                description = 'Paused'
            else:
                description = 'Playing'
            self.status.set(f"CSV: {description} | {playback['time_s']:.3f} s | "
                            f"row {playback['sample'] + 1}/{playback['samples']}")
        else:
            self.status.set('Sliders set physical targets; labels show sampled readings. Gyro units: rad/s.')

        self.timer = self.root.after(33, self.refresh)

    def on_destroy(self, event):
        if event.widget is self.root and self.timer is not None:
            self.root.after_cancel(self.timer)


def build_gui(root, sense=None):
    """Build controls around an existing board, or create a new emulator."""
    if sense is None:
        sense = SenseEmu()
    window = EmulatorGUI(root, sense)
    # Retain the existing return format for callers using the target variables.
    return {name: control['target'] for name, control in window.controls.items()}


def main(sense=None):
    root = tk.Tk()
    build_gui(root, sense)
    root.mainloop()


if __name__ == '__main__':
    main()
