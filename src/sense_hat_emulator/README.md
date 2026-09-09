# Sense HAT emulator

Install once from the repository root:

```powershell
python -m pip install -e .
```

Use it in your Python code:

```python
from sense_hat_emulator import SenseEmu

sense = SenseEmu()
sense.set_temperature(25)
sense.set_pressure(950)
sense.set_humidity(40)
sense.set_accelerometer_raw(0, 0, 1)

print(sense.get_temperature())
print(sense.get_accelerometer_raw())
sense.set_pixel(0, 0, 255, 0, 0)
sense.show_gui()
```

Pass the same `sense` object to other functions. Its sensor and LED setters
update the open GUI; sliders update what its getters read. `show_gui()` runs
until the window closes. Each new instance represents an independent board.

Supported sensor getter/setter pairs: `temperature`, `pressure`, `humidity`,
`accelerometer_raw`, `compass_raw`, and `gyroscope_raw`. Vectors use x/y/z.
Temperature-source getters share the same temperature value.
LED methods: `set_pixel`, `get_pixel`, `set_pixels`, `get_pixels`, and `clear`.
This implements the current sensors and LEDs, not the entire SenseHat library;
joystick and text display are not implemented yet.

Files: `emulator.py` stores readings and defines SenseEmu; `gui.py` displays it;
`__init__.py` makes the import above work. The root `pyproject.toml` enables installation.

## Monitor and orientation

`python tools/calibration/check_sensors.py` automatically opens the GUI when
hardware selection falls back to SenseEmu. It shares the monitor's instance;
closing the window stops monitoring. The real-HAT path stays terminal-only.
Direct `SenseEmu()` construction still works without a GUI; use `show_gui()`
when using the class outside the monitor.

The GUI shows derived pitch, roll, and yaw below the matrix. Read them with
`get_orientation_degrees()` (or `get_orientation()`) and
`get_orientation_radians()`. Angles wrap into 0–360° / 0–2π. The flat reference
has acceleration along +Z and magnetic north along +X.

This static estimate uses gravity to calculate tilt and then levels the magnetic
vector to calculate heading. Gyroscope rates are not integrated. Acceleration
from a drop, engine thrust, or impacts can be mistaken for gravity; this is not
a flight-grade orientation filter. Zero vectors, vertical pitch singularities,
and a magnetic field parallel to gravity raise ValueError; the GUI reports
orientation unavailable instead of inventing a heading.

References: [Sense HAT orientation API](https://sense-hat.readthedocs.io/en/latest/api/)
and [NXP tilt-compensated compass explanation](https://www.nxp.com/docs/en/application-note/AN4248.pdf).
