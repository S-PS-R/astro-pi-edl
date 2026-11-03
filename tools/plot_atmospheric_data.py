"""Select an entry JSON and display one figure with six plots; save using the toolbar."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

PLOT_COLUMNS = ('time_s', 'altitude_m', 'density_kg_m3', 'velocity_m_s',
                'accel_x', 'accel_y', 'accel_z')


def _select_json():
    """Return a JSON filename from the native picker, or None on cancellation."""
    try:
        import tkinter as tk
        from tkinter import filedialog
    except ImportError as exc:
        raise RuntimeError('Tkinter is required for the JSON file picker') from exc
    try:
        root = tk.Tk()
    except tk.TclError as exc:
        raise RuntimeError('A desktop display is required for the JSON file picker') from exc
    try:
        root.withdraw()
        return filedialog.askopenfilename(
            parent=root, title='Select entry JSON',
            initialdir=str(Path('data/atmospheric_entry').resolve()),
            filetypes=[('Entry JSON', '*.json')],
        ) or None
    finally:
        root.destroy()


def _load_data(path):
    metadata = json.loads(Path(path).read_text(encoding='utf-8-sig'))
    if not isinstance(metadata, dict):
        raise ValueError('Expected an entry JSON object')
    rows = metadata.get('trajectory')
    if not isinstance(rows, list) or not rows:
        raise ValueError('JSON has no trajectory samples; regenerate it with data/create_atmospheric_data.py')
    data = {key: [] for key in PLOT_COLUMNS}
    for index, row in enumerate(rows):
        if not isinstance(row, dict):
            raise ValueError(f'Invalid trajectory sample {index}')
        for key in PLOT_COLUMNS:
            value = row.get(key)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f'Trajectory sample {index}: {key} must be a finite number')
            if key in ('time_s', 'altitude_m', 'density_kg_m3', 'velocity_m_s') and value < 0:
                raise ValueError(f'Trajectory sample {index}: {key} must be nonnegative')
            data[key].append(value)
    if any(b <= a for a, b in zip(data['time_s'], data['time_s'][1:])):
        raise ValueError('Trajectory times must strictly increase')
    return data


def plot_atmospheric_data(path=None):
    """Pick a JSON if omitted, then show six panels in one interactive window.

    Returns a list containing the Matplotlib figure, or [] on cancellation.
    Uses only JSON data; never reads a CSV or automatically saves images.
    Acceleration is board-axis specific force in g, matching entry.csv.
    """
    if path is None:
        path = _select_json()
        if path is None:
            return []
    data = _load_data(path)
    try:
        import matplotlib
        # Tk supplies interactive windows even if the local default is Agg.
        # No backend changes or GUI work occur at module import time.
        matplotlib.use('TkAgg')
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError('Install dependencies with python -m pip install -e .; Tkinter is also required') from exc
    return _show_plots(data, Path(path).stem, plt)


def _show_plots(data, name, plt):
    """Build the requested axes and show them together in one blocking GUI loop."""
    altitude = [h / 1000 for h in data['altitude_m']]
    velocity = [v / 1000 for v in data['velocity_m_s']]
    time = data['time_s']
    fig, axes = plt.subplots(2, 3, figsize=(15, 9), layout='constrained')
    fig.canvas.manager.set_window_title(f'{name} — Atmospheric entry')
    fig.suptitle(f'{name} — Atmospheric entry')
    panels = iter(axes.flat)

    def panel(title, xlabel, ylabel):
        axis = next(panels)
        axis.set(title=title, xlabel=xlabel, ylabel=ylabel)
        axis.grid(True, alpha=0.3)
        return axis

    panel('Altitude vs density', 'Density (kg/m³)', 'Altitude (km)').plot(data['density_kg_m3'], altitude)
    panel('Altitude vs velocity', 'Velocity (km/s)', 'Altitude (km)').plot(velocity, altitude)
    axis = panel('Altitude vs acceleration', 'Board-axis specific force (g)', 'Altitude (km)')
    for component in 'xyz':
        axis.plot(data[f'accel_{component}'], altitude, label=component.upper())
    axis.legend()
    panel('Altitude vs time', 'Time (s)', 'Altitude (km)').plot(time, altitude)
    panel('Velocity vs time', 'Time (s)', 'Velocity (km/s)').plot(time, velocity)
    axis = panel('Acceleration vs time', 'Time (s)', 'Board-axis specific force (g)')
    for component in 'xyz':
        axis.plot(time, data[f'accel_{component}'], label=component.upper())
    axis.legend()
    plt.show(block=True)
    return [fig]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('path', nargs='?', help='Entry JSON (omit to open the file picker)')
    args = parser.parse_args(argv)
    try:
        plot_atmospheric_data(args.path)
    except (OSError, ValueError, RuntimeError) as exc:
        parser.exit(1, f'Error: {exc}\n')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
