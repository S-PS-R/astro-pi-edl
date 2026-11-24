"""Read-only mission display, with the shared emulator in a second window."""

from math import degrees
from pathlib import Path
import tkinter as tk
from tkinter import ttk
from ..shared.appearance import apply_tk_dark_theme, save_window_image
from .plot_window import PlotProcess
from .state import SimulationClock


def format_reading(value):
    return '' if value is None else f'{value:.3f}'


class MissionWindow:
    def __init__(self, planet_name, plot_path, updates_per_sim_second, time_scale):
        self.sim_clock = SimulationClock(time_scale)
        self.updates_per_sim_second = updates_per_sim_second
        update_hz = updates_per_sim_second * time_scale
        self.closed = False
        self.finished = False
        self.latest = {}
        self.displayed = {}
        self.fields = {}
        self.emulator = None
        self.refresh_ms = round(1000 / update_hz)
        self.plot_path = Path(plot_path)
        self.plot_save_requested = False
        self.window_images_saved = False
        # Main window
        self.root = tk.Tk()
        apply_tk_dark_theme(self.root)
        self.root.title('EDL Mission - ' + planet_name.upper())
        self.root.geometry('+30+30')
        self.root.minsize(440, 350)
        panel = ttk.Frame(self.root, padding=16)
        panel.pack(fill='both', expand=True)
        panel.columnconfigure(1, weight=1)
        ttk.Label(panel, text='EDL Mission', font=('Segoe UI', 15, 'bold')).grid(
            row=0, column=0, columnspan=3, sticky='w', pady=(0, 8))
        self.status = tk.StringVar(master=self.root, value='Preparing run...')
        ttk.Label(panel, textvariable=self.status).grid(row=1, column=0, columnspan=3, sticky='w')

        # Mission readouts
        labels = {
            'phase': 'Phase',
            'elapsed_s': 'Mission time (s)',
            'altitude_m': 'Altitude / height (m)',
            'velocity_m_s': 'Speed (m/s)',
            'flight_path_angle_deg': 'Flight-path angle (deg)',
            'bank_angle_deg': 'Bank angle (deg)',
            'heating_kw_m2': 'Convective heat flux (kW/m²)',
            'acceleration_m_s2': 'Speed acceleration (m/s²)',
        }
        for row, (name, label) in enumerate(labels.items(), start=2):
            self.fields[name] = tk.StringVar(master=self.root, value='-')
            ttk.Label(panel, text=label).grid(row=row, column=0, sticky='w', padx=(0, 16), pady=5)
            ttk.Label(panel, textvariable=self.fields[name]).grid(row=row, column=1, sticky='w')
            if name == 'elapsed_s':
                controls = ttk.Frame(panel)
                controls.grid(row=row, column=2, sticky='w', padx=(12, 0))
                self.speed_selection = tk.IntVar(master=self.root, value=int(time_scale))
                self.speed_buttons = []
                for scale in (1, 2, 5, 10, 50):
                    button = ttk.Radiobutton(
                        controls, text=f'{scale}×', variable=self.speed_selection,
                        value=scale, command=lambda speed=scale: self.set_speed(speed))
                    button.pack(side='left', padx=3)
                    self.speed_buttons.append(button)

        ttk.Button(panel, text='Stop and close', command=self.close).grid(
            row=len(labels) + 2, column=0, columnspan=3, sticky='w', pady=(12, 0))
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.plotter = PlotProcess(update_hz=update_hz)
        self.timer = self.root.after(self.refresh_ms, self.refresh)

    def set_speed(self, scale):
        if self.closed or self.finished:
            return
        self.sim_clock.set_scale(scale)
        self.speed_selection.set(scale)
        rate = self.updates_per_sim_second * scale
        self.refresh_ms = round(1000 / rate)
        self.plotter.set_update_hz(rate)
        self.root.after_cancel(self.timer)
        self.timer = self.root.after(self.refresh_ms, self.refresh)

    def open_emulator(self, sense):
        from sense_hat_emulator.gui import EmulatorGUI
        emulator_root = tk.Toplevel(self.root)
        emulator_root.configure(background=self.root.cget('background'))
        # Shared window lifecycle
        emulator_root.protocol('WM_DELETE_WINDOW', lambda: None)
        self.root.update_idletasks()
        x = self.root.winfo_x() + self.root.winfo_width() + 20
        emulator_root.geometry(f'+{x}+30')
        self.emulator = EmulatorGUI(emulator_root, sense)

    def plot_deployment(self, time_s, before, after):
        self.plotter.add_deployment(dict(before, time_s=time_s), dict(after, time_s=time_s))

    def publish(self, state, values):
        # Latest display snapshot
        self.plotter.add_sample(dict(values, time_s=state.elapsed_s))
        self.latest = {
            'phase': state.phase.value.upper(),
            'elapsed_s': format_reading(state.elapsed_s),
            'altitude_m': format_reading(state.altitude_m),
            'velocity_m_s': format_reading(state.velocity_m_s),
            'flight_path_angle_deg': format_reading(degrees(state.flight_path_angle_rad)),
            **{name: format_reading(values[name])
               for name in ('acceleration_m_s2', 'bank_angle_deg', 'heating_kw_m2')},
        }

    def update_display(self):
        latest = self.latest
        for name, value in latest.items():
            if self.displayed.get(name) != value:
                self.fields[name].set(value)
        self.displayed = latest
        if self.finished or self.closed:
            for button in self.speed_buttons:
                button.configure(state='disabled')
            if latest.get('phase') in ('LANDED', 'IMPACT'):
                status = 'Run finished - close this window to exit'
            else:
                status = 'Run stopped'
        else:
            status = 'Running'
        if status != self.status.get():
            self.status.set(status)

    def refresh(self):
        if self.closed:
            return
        self.update_display()
        if self.finished:
            self.save_plots()
            self.save_final_windows()
        self.timer = self.root.after(self.refresh_ms, self.refresh)

    def save_final_windows(self):
        if self.window_images_saved:
            return
        self.window_images_saved = True
        save_window_image(self.root, self.plot_path.with_name('final_mission_window.png'))
        if self.emulator is not None:
            self.emulator.refresh(schedule_next=False)
            save_window_image(self.emulator.root, self.plot_path.with_name('final_emulator.png'))

    def close(self):
        if not self.closed:
            self.closed = True
            self.root.after_cancel(self.timer)
            self.root.quit()

    def save_plots(self):
        if not self.closed and not self.plot_save_requested:
            self.plot_save_requested = self.plotter.save(self.plot_path)

    def destroy(self):
        try:
            self.update_display()
            self.save_final_windows()
            self.plotter.close(self.plot_path)
        finally:
            self.root.destroy()
