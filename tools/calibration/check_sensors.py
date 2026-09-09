""" Tool to check if you can get readings from the Sense HAT
    Author: Samir Rathore
"""

from pathlib import Path
import subprocess
import sys
import time
import os
import math
import shutil
import shlex
from itertools import cycle
import argparse

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
from edl_flight_computer.hat import get_hat, is_simulated

# Advance once per sensor sample; restart at red when the process starts.
LED_COLORS = [
    (255, 0, 0),      # Red
    (0, 255, 0),      # Green
    (0, 0, 255),      # Blue
    (255, 255, 0),    # Yellow
    (255, 0, 255),    # Magenta
    (0, 255, 255),    # Cyan
    (255, 255, 255),  # White
]
_led_colors = cycle(LED_COLORS)

def open_monitor_terminal(noise=True, seed=None, csv_path=None, replay_noise=False):
    """Launch a terminal for this OS, preserving the current Python environment.

    --monitor prevents the child from launching another terminal. If launching
    fails or no desktop terminal is available, use the current terminal.
    """
    command = [sys.executable, str(Path(__file__).resolve()), "--monitor"]
    if not noise:
        command.append('--no-noise')
    if seed is not None:
        command.extend(['--seed', str(seed)])
    if csv_path is not None:
        command.extend(['--csv', str(Path(csv_path).resolve())])
    if replay_noise:
        command.append('--replay-noise')
    try:
        if sys.platform == "win32":
            # Windows creates a console for the same .venv Python interpreter.
            subprocess.Popen(command, creationflags=subprocess.CREATE_NEW_CONSOLE)
            return

        if sys.platform == "darwin":
            # Pass the shell command as data, rather than embedding paths in
            # AppleScript. shlex.join quotes spaces and shell metacharacters.
            script = '''on run argv tell application "Terminal" do script (item 1 of argv)
            activate end tell end run'''
            subprocess.run(
                ["osascript", "-e", script, shlex.join(command)],
                check=True, capture_output=True, text=True,
            )
            return

        if sys.platform.startswith("linux") and (
            os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
        ):
            # Different desktop terminals use different command separators.
            for name, separator in [
                ("x-terminal-emulator", "-e"),
                ("gnome-terminal", "--"),
                ("konsole", "-e"),
                ("xfce4-terminal", "--execute"),
                ("xterm", "-e"),
            ]:
                terminal = shutil.which(name)
                if terminal:
                    try:
                        subprocess.Popen([terminal, separator, *command])
                        return
                    except OSError:
                        continue
    except (OSError, subprocess.CalledProcessError) as exc:
        print(f"Could not open a new terminal: {exc}")

    print("Using the current terminal for the sensor monitor.")
    monitor_sensors(noise=noise, seed=seed, csv_path=csv_path, replay_noise=replay_noise)


def get_next_led_color():
    """Return the next colour from an in-memory repeating sequence."""
    return next(_led_colors)


def print_readings(sense):
    """Print one sample and advance the LEDs on each sensor update."""
    led_color = get_next_led_color()
    sense.clear(*led_color)
    print(f"LED color: RGB{led_color}")
    # Environmental sensors
    print("\n===== ENVIRONMENTAL SENSORS =====")

    temperature = sense.get_temperature()
    pressure = sense.get_pressure()
    humidity = sense.get_humidity()
    print(f"Temperature (humidity sensor): {temperature:.2f} C")
    print(f"Temperature (pressure sensor): {sense.get_temperature_from_pressure():.2f} C")
    print(f"Pressure:    {pressure:.2f} hPa")
    print(f"Humidity:    {humidity:.1f} %")

    # Accelerometer
    print("\n===== ACCELEROMETER =====")

    acceleration = sense.get_accelerometer_raw()
    print(
        f"x = {acceleration['x']:.2f} g, "
        f"y = {acceleration['y']:.2f} g, "
        f"z = {acceleration['z']:.2f} g"
    )
    accel_magnitude = math.sqrt(
        acceleration["x"] ** 2
        + acceleration["y"] ** 2
        + acceleration["z"] ** 2
    )
    print(f"Total acceleration: {accel_magnitude:.2f} g")

    # Gyroscope
    print("\n===== GYROSCOPE =====")
    gyro = sense.get_gyroscope_raw()
    print(
        f"x = {gyro['x']:.2f}, "
        f"y = {gyro['y']:.2f}, "
        f"z = {gyro['z']:.2f}"
    )

    # Magnetometer
    print("\n===== MAGNETOMETER =====")
    magnetometer = sense.get_compass_raw()
    print(
        f"x = {magnetometer['x']:.2f}, "
        f"y = {magnetometer['y']:.2f}, "
        f"z = {magnetometer['z']:.2f}"
    )

    # Orientation
    print("\n===== ORIENTATION =====")
    try:
        orientation = sense.get_orientation_degrees()
        print(f"Pitch: {orientation['pitch']:.2f} deg")
        print(f"Roll:  {orientation['roll']:.2f} deg")
        print(f"Yaw:   {orientation['yaw']:.2f} deg")
    except ValueError as exc:
        print(f"Orientation unavailable: {exc}")

    print("\nPress Ctrl+C or close the emulator window to stop\n")


def monitor_sensors(noise=True, seed=None, csv_path=None, replay_noise=False):
    """Show the emulator automatically, or monitor the real HAT in a loop."""
    sense = get_hat(noise=noise, seed=seed, force_emulator=csv_path is not None)
    if csv_path is not None:
        sense.load_csv(csv_path, noise=replay_noise)
    simulated = is_simulated()
    print(f"Sensor source: {'SenseEmu (simulated)' if simulated else 'Sense HAT (hardware)'}")
    if simulated:
        print("Orientation is commanded (or supplied by CSV); world mode generates raw IMU readings.")
    try:
        if simulated:
            # Tk stays on the main thread. Timer callbacks keep monitoring and
            # the GUI responsive, using the very same emulator object.
            import tkinter as tk
            from sense_hat_emulator.gui import build_gui
            root = tk.Tk()
            try:
                build_gui(root, sense)
                root.protocol("WM_DELETE_WINDOW", root.quit)
                errors = []

                def tick():
                    try:
                        print_readings(sense)
                    except BaseException as exc:
                        errors.append(exc)
                        root.quit()
                        return
                    root.after(1000, tick)

                root.after(0, tick)
                root.mainloop()
                if errors:
                    raise errors[0]
            finally:
                root.destroy()
        else:
            while True:
                print_readings(sense)
                time.sleep(1)
    except KeyboardInterrupt:
        print("\nExiting sensor monitor...")
    finally:
        sense.clear()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Monitor Sense HAT or SenseEmu sensors")
    parser.add_argument('--monitor', action='store_true', help='Use this terminal')
    parser.add_argument('--noise', action=argparse.BooleanOptionalAction, default=True,
                        help='Enable/disable emulator sensor noise (default: enabled)')
    parser.add_argument('--seed', type=int, help='Emulator random seed')
    parser.add_argument('--csv', type=Path, help='Replay sensor CSV; always uses the emulator')
    parser.add_argument('--replay-noise', action='store_true',
                        help='Add sensor noise to CSV inputs (default: exact replay)')
    args = parser.parse_args()
    if args.monitor:
        monitor_sensors(noise=args.noise, seed=args.seed, csv_path=args.csv, replay_noise=args.replay_noise)
    else:
        open_monitor_terminal(noise=args.noise, seed=args.seed, csv_path=args.csv, replay_noise=args.replay_noise)
