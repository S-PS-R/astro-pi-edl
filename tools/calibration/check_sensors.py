""" Tool to check if you can get readings from the Sense HAT
    Author: Samir Rathore
"""

from sense_hat import SenseHat
from pathlib import Path
import subprocess
import sys
import time
import os
import math

# LED colors to cycle through each time the monitor is run
LED_COLORS = [
    (255, 0, 0),      # Red
    (0, 255, 0),      # Green
    (0, 0, 255),      # Blue
    (255, 255, 0),    # Yellow
    (255, 0, 255),    # Magenta
    (0, 255, 255),    # Cyan
    (255, 255, 255),  # White
]

def open_monitor_terminal():
    """
    Open this script in a new terminal window for easier visual monitoring of the Sense HAT sensor readings.
    The --monitor argument prevents the new process from opening another terminal recursively.
    """

    script_path = Path(__file__).resolve()

    # sys.executable uses .venv python interpreter
    python_path = sys.executable

    subprocess.Popen([
        "x-terminal-emulator",
        "-e",
        "bash",
        "-c",
        f"'{python_path}' '{script_path}' --monitor; exec bash"
    ])

def get_next_led_color():
    """
    Get the next LED color in the sequence.

    The current color index is stored in a small file so that each time
    the program runs it moves to the next color.
    """

    config_dir = Path.home() / ".config" / "astro-pi-edl"
    config_dir.mkdir(parents=True, exist_ok=True)

    color_file = config_dir / "sensor_monitor_color.txt"

    # Default to the first color
    color_index = 0

    if color_file.exists():
        try:
            previous_index = int(color_file.read_text().strip())
            color_index = (previous_index + 1) % len(LED_COLORS)
        except ValueError:
            color_index = 0

    # Save the current index for the next run
    color_file.write_text(str(color_index))

    return LED_COLORS[color_index]


def monitor_sensors():
    """Continuously display Sense HAT sensor readings."""

    # Initialize Sense HAT
    sense = SenseHat()

    try:
        while True:

            # Choose a different LED color for this run
            led_color = get_next_led_color()

            # Turn the entire 8x8 LED matrix on
            sense.clear(*led_color)

            print(f"LED color for this run: RGB{led_color}")
            
            # Environmental sensors
            print("\n===== ENVIRONMENTAL SENSORS =====")

            temperature = sense.get_temperature()
            pressure = sense.get_pressure()
            humidity = sense.get_humidity()
            print(f"Temperature: {temperature:.2f} C")
            print(f"Pressure:    {pressure:.2f} hPa")
            print(f"Humidity:    {humidity:.2f} %")

            # Accelerometer
            print("\n===== ACCELEROMETER =====")

            acceleration = sense.get_accelerometer_raw()
            print(
                f"x = {acceleration['x']:.4f} g, "
                f"y = {acceleration['y']:.4f} g, "
                f"z = {acceleration['z']:.4f} g"
            )
            accel_magnitude = math.sqrt(
                acceleration["x"] ** 2
                + acceleration["y"] ** 2
                + acceleration["z"] ** 2
            )
            print(f"Total acceleration: {accel_magnitude:.4f} g")

            # Gyroscope
            print("\n===== GYROSCOPE =====")
            gyro = sense.get_gyroscope_raw()
            print(
                f"x = {gyro['x']:.4f}, "
                f"y = {gyro['y']:.4f}, "
                f"z = {gyro['z']:.4f}"
            )

            # Magnetometer
            print("\n===== MAGNETOMETER =====")
            magnetometer = sense.get_compass_raw()
            print(
                f"x = {magnetometer['x']:.4f}, "
                f"y = {magnetometer['y']:.4f}, "
                f"z = {magnetometer['z']:.4f}"
            )

            # Orientation
            print("\n===== ORIENTATION =====")
            orientation = sense.get_orientation_degrees()
            print(f"Pitch: {orientation['pitch']:.2f} deg")
            print(f"Roll:  {orientation['roll']:.2f} deg")
            print(f"Yaw:   {orientation['yaw']:.2f} deg")

            print("\nPress Ctrl+C to stop\n")
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nExiting sensor monitor...")
        sense.clear()


if __name__ == "__main__":

    if "--monitor" in sys.argv:
        monitor_sensors()
    else:
        open_monitor_terminal()