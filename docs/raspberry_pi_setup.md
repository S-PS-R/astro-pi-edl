# Raspberry Pi setup

Install Raspberry Pi OS and enable I2C using raspi-config. Install system hardware
bindings and Tkinter, then create a virtual environment that can see those bindings:

```sh
sudo apt update
sudo apt install sense-hat python3-venv python3-tk octave
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -e .
python tools/calibration/check_sensors.py
python tools/calibration/check_sensor_profile.py
```

Clone the repository before creating the environment; run these commands from
its root.

Calibrate using `python tools/calibration/calibrate_imu.py`

Run the flight program with `python -m edl_flight_computer.run`, using the
settings described in the root README. The emulator controls open as part of
the mission when a real Sense HAT is unavailable. The calibration commands
above are separate preparation tools for the Pi hardware.
