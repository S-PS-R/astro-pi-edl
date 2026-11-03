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

Hardware-only tools fail clearly when the board is unavailable. Desktop
simulation is explicitly launched with `python -m sense_hat_emulator` or the
CSV replay command in the root README.
