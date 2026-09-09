# Sense HAT emulator

Install from the repository root: `python -m pip install -e .`
^Make sure to do this within virtual environment

```python
from sense_hat_emulator import SenseEmu

sense = SenseEmu(noise=True, seed=42)
sense.set_temperature(25)
sense.show_gui()

# For repeatable timing
sense = SenseEmu(seed=42, realtime=False)
sense.advance(1.0)
```

Each instance prints its settings, for example:

```text
[SenseEmu] noise=ON | seed=42 | time=realtime | mode=world
```

Noise defaults to on. Omit `seed` to generate a seed, which is also printed.
Use `SenseEmu(noise=False)` for exact targets without noise, smoothing, or
quantization. Change `sense.noise = False` / `True` while running, or use the
GUI's Sensor noise checkbox. Toggling noise restarts sample histories and random
streams from the current targets. Reset sensors restores default targets and
restarts those streams; neither operation clears the LEDs.

## Monitor

```powershell
python tools/calibration/check_sensors.py --seed 42
python tools/calibration/check_sensors.py --no-noise
```

The settings are forwarded to the new terminal. Add `--monitor` to use the
current terminal. The GUI opens automatically when the monitor uses SenseEmu.
The real HAT is selected first unless `--csv` is supplied; noise/seed options
only affect the emulator.
The monitor prints both temperature-source readings on each update and cycles
the LED colour. Closing the GUI stops the monitor.

## Targets and readings

Sliders and setters control ideal **targets**. Getters return the latest simulated
**readings**. GUI slider labels show targets; terminal readings include noise and
response delay. Noise does not move the sliders.

`set_temperature(25)` sets both temperature targets. The two sensors produce
independent results through `get_temperature_from_humidity()` and
`get_temperature_from_pressure()`. `get_temperature()` uses the humidity source
like the official sensor does. Source-specific temperature setters can set different targets;
the single GUI slider follows the humidity target and sets both again when moved.

Other getter/setter pairs: pressure, humidity, accelerometer_raw, compass_raw,
and gyroscope_raw. Vector setters accept x/y/z or a dictionary with those keys.
Pass the same SenseEmu instance to functions sharing one simulated board.

## Repeatable simulation

A seed repeats the random sequence. Exact results also require identical target
changes at identical simulated times. Real-time runs can differ because mouse
input and OS scheduling differ. For deterministic runs:

```python
sense = SenseEmu(seed=42, realtime=False)
sense.set_temperature(25)
sense.advance(1.0)
print(sense.get_temperature_from_humidity())
print(sense.get_temperature_from_pressure())
```

Manual time only moves with `advance(seconds)`. Repeated getters at the same time
return the same samples. In real-time mode, sampling catches up to a monotonic
clock before reads/target changes; no background thread is required. An idle
instance catches up its samples on the next access, so a long pause may take
some processing time. Values begin at their defaults until the first sample.

## Models and files

- `__init__.py`: exposes `SenseEmu`.
- `emulator.py`: public methods, shared state, clock, settings, and LEDs.
- `gui.py`: controls, display, and synchronization.
- `humidity.py`: humidity and its temperature; shared rolling-sample helper.
- `pressure.py`: pressure and its temperature.
- `imu.py`: three-axis acceleration, gyro, magnetic field, and orientation-driven world rotation.

## Sensor sampling and update rates

These are the rates implemented by this emulator, not configurable hardware
output-data-rate settings. A tick generates one new sample for each channel in
that model. Frequency in hertz (Hz) is `1 / interval_in_seconds`.

| Model / channels | Tick interval | Sampling rate | Samples averaged per channel | Full history replacement |
| --- | --- | --- | --- | --- |
| Humidity | 130 ms | ≈7.692 Hz | 10 | 1.30 s |
| Humidity-sensor temperature | 130 ms | ≈7.692 Hz | 31 | 4.03 s |
| Pressure | 40 ms | 25 Hz | 25 | 1.00 s |
| Pressure-sensor temperature | 40 ms | 25 Hz | 25 | 1.00 s |
| Accelerometer X/Y/Z | 16 ms | 62.5 Hz | 10 per axis | 0.160 s |
| Gyroscope X/Y/Z | 16 ms | 62.5 Hz | 10 per axis | 0.160 s |
| Magnetometer X/Y/Z | 16 ms | 62.5 Hz | 10 per axis | 0.160 s |

Full history replacement is `sample_count × interval`. It describes how long
it takes to replace every old sample after a target change, approximately;
the exact delay depends on where that change falls between ticks. New readings
are available every tick, not only once a whole history has been replaced.
These histories implement a moving average, not an exponential response model.

The three model clocks advance independently. With noise on, the scheduler uses:

```text
due_ticks = int((elapsed_time - model_start_time) / period + 1e-8)
```

It generates all ticks not yet processed. The tiny tolerance avoids missing a
tick at a floating-point time boundary. Reads and target changes synchronize
pending samples; past samples are processed before applying a new target.
No background sampling thread is used. Reading more often within the same tick
does not generate more noise. Reading less often catches up the missing samples.

The display has a different update rate:

- GUI refresh: scheduled every 33 ms, approximately 30.3 updates/second.
- Terminal monitor and its LED colour cycle: approximately once per second.
- Orientation: getters return the commanded angles immediately; they are not
  calculated from noisy readings. Generated gyro targets update on IMU ticks.

GUI callbacks and terminal printing take time, so those display intervals are
not hard real-time guarantees. They do not change the sensor periods. In manual
mode, a GUI refresh does not move time; only `advance(seconds)` does.
With noise off, smoothing, clipping, and quantization are bypassed: getters
return validated targets immediately. World-mode gyro generation still runs
on 16 ms IMU ticks. Exact CSV replay follows row timestamps instead of sensor ticks.

## How noise is generated

For every channel and sample, the implementation performs these steps:

```text
Gaussian draw → add to target → rolling average → clamp → quantize → reading
```

### 1. Gaussian perturbation

```python
sample = target + rng.gauss(0, 0.2) * error
```

A Gaussian distribution is a bell-shaped distribution. Here it has a mean of
zero and standard deviation 0.2 before multiplying by `error`:

```text
sample_noise_standard_deviation = 0.2 × error
```

`error` is a scale parameter, NOT a hard ± limit. Before averaging and clipping,
about 68% of Gaussian noise falls within ±1 standard deviation and 95% within
±1.96 standard deviations. Larger deviations remain possible.

Every channel gets its own draw. Each model has a separate seeded random-number
generator; channels within a model share that generator but receive different
draws. This is independent synthetic noise, not a model of cross-axis coupling,
long-term bias drift, or correlated physical disturbances.

### 2. Exact error parameters

Conditions below use the current target values. Range endpoints are inclusive.
The narrower temperature condition takes priority where ranges overlap.

| Channel | Target conditions | Error parameter | Sample noise standard deviation, before averaging |
| --- | --- | --- | --- |
| Humidity | 20–80% relative humidity | 3.5 percentage points | 0.7 percentage points |
| Humidity | Outside 20–80% | 5 percentage points | 1 percentage point |
| Humidity temperature | 15–40 °C | 0.5 °C | 0.1 °C |
| Humidity temperature | 0–60 °C, excluding 15–40 °C | 1 °C | 0.2 °C |
| Humidity temperature | Outside 0–60 °C | 2 °C | 0.4 °C |
| Pressure | 800–1100 hPa AND pressure-sensor temperature 20–60 °C | 0.2 hPa | 0.04 hPa |
| Pressure | All other conditions | 1 hPa | 0.2 hPa |
| Pressure temperature | 0–65 °C | 2 °C | 0.4 °C |
| Pressure temperature | Outside 0–65 °C | 4 °C | 0.8 °C |
| Acceleration, each axis | All targets | 0.1 g | 0.02 g |
| Gyroscope, each axis | All targets | π/180 rad/s (1 degree/s) | ≈0.003491 rad/s (0.2 degree/s) |
| Magnetic field, each axis | All targets | 200 µT (2 gauss) | 40 µT (0.4 gauss) |

Humidity noise is measured in percentage points, not as a percentage of the
humidity target. The magnetometer value intentionally follows the upstream
emulator's relatively large noise; it is not a claim about real sensor accuracy.
Gyroscope parameters are converted to radians/second consistently in our code.

### 3. Rolling averages and response delay

For a channel with a history length `N`:

```text
average = (sample_1 + sample_2 + ... + sample_N) / N
```

The newest sample replaces the oldest. At initialization/reset, the entire
history is filled with the target, so the first samples gradually introduce
noise. A later target change leaves old samples in the history, producing a
smooth transition toward the new target.

At a constant target, after the initial history is replaced, the approximate
standard deviation of the average is:

```text
averaged_standard_deviation = sample_standard_deviation / sqrt(N)
```

This approximation is before clipping and quantization. Successive averaged
outputs are correlated because their histories overlap.

Examples under the lower-noise environmental conditions:

| Channel | Sample standard deviation | N | Averaged standard deviation, approximately |
| --- | --- | --- | --- |
| Humidity | 0.7 percentage points | 10 | 0.2214 percentage points |
| Humidity temperature | 0.1 °C | 31 | 0.01796 °C |
| Pressure | 0.04 hPa | 25 | 0.008 hPa |
| Pressure temperature | 0.4 °C | 25 | 0.08 °C |
| Acceleration, per axis | 0.02 g | 10 | 0.006325 g |
| Gyroscope, per axis | 0.003491 rad/s | 10 | 0.001104 rad/s |
| Magnetic field, per axis | 40 µT | 10 | 12.649 µT |

Clipping near a limit can shift the mean: for example, negative humidity readings
are clipped to zero when the target is 0%. Thus the final output is not always
an unbiased Gaussian distribution.

### 4. Clamping and quantization

After averaging, the shared `SampleChannel` helper uses:

```python
limited = min(high, max(low, average))
reading = int((limited - offset) * factor) / factor + offset
```

`int()` truncates toward zero; it does not round to the nearest integer.
Quantization makes the reading change in discrete increments of `1 / factor`.
This resolution is different from accuracy or noise amplitude.

| Channel | Clamped range | Factor | Offset | Approximate resolution |
| --- | --- | --- | --- | --- |
| Humidity | 0–100% | 256 | 0 | 0.00390625 percentage points |
| Humidity temperature | −40 to 120 °C | 64 | 0 | 0.015625 °C |
| Pressure | 260–1260 hPa | 4096 | 0 | 0.00024414 hPa |
| Pressure temperature | −30 to 105 °C | 480 | 37 °C | 0.00208333 °C |
| Acceleration, each axis | −8 to +8 g | 4081.6327 | 0 | 0.000245 g |
| Gyroscope, each axis | ±500 degrees/s (≈±8.72665 rad/s) | 57.142857 × 180/π | 0 | 0.00030543 rad/s |
| Magnetic field, each axis | −400 to +400 µT | 7142.8571 / 100 | 0 | 0.014 µT |

These are model output limits. They are distinct from GUI slider ranges and
API target validation. A valid target outside a model's output limits produces
a saturated reading with noise on. With noise off, the exact target is returned.
Initial/reset readings are also exact targets until sampling starts.

Temperature from humidity and temperature from pressure are separate sensor
channels, not calculations from relative humidity or pressure. For example,
`set_temperature(25)` supplies the same target to both, but their different
noise, histories, and resolutions produce different measurements.

## Orientation and world inputs

The default mode is `world`. The GUI has pitch, roll, and yaw sliders, adjustable
world acceleration and magnetic vectors, and generated gyro readouts. The
8x8 display and the reserved area beneath it remain in the same window.

```python
sense = SenseEmu(noise=False, realtime=False)
sense.set_world_acceleration(0, 0, 1)       # g; default
sense.set_world_magnetic_field(33, 0, 0)   # microtesla; default
sense.set_orientation(roll=0, pitch=0, yaw=90)  # degrees
print(sense.get_accelerometer_raw())      # approximately (0, 0, 1)
print(sense.get_compass_raw())            # approximately (0, -33, 0)
sense.advance(0.016)
print(sense.get_gyroscope_raw())          # z = radians(90) / 0.016
sense.advance(0.016)
print(sense.get_gyroscope_raw())          # zero: orientation is now stationary
```

Vectors are expressed in fixed **world axes**. Rotation changes their components
in **board axes**, which are the values returned by raw getters. Changing a
world vector changes the environment independently of orientation. Vector setters
accept three numbers or a dictionary with exactly `x`, `y`, `z` keys.

The acceleration input is the total accelerometer-input vector, in g, including
any desired gravity/support/flight effects. The emulator rotates exactly what you
supply; it does not add another 1 g or compute forces, trajectory, or free fall.
For example, supply a zero vector to represent zero ideal accelerometer output.

### Rotation matrix

Following the official emulator, roll is around X, pitch around Y, yaw around Z.
Angles are converted to radians for trigonometry. Let `cr/sr`, `cp/sp`, `cy/sy`
mean cosine/sine of roll, pitch, yaw respectively:

```text
R = Rz(yaw) Ry(pitch) Rx(roll)

    [ cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr ]
R = [ sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr ]
    [   -sp,             cp*sr,             cp*cr ]

accel_board = transpose(R) * accel_world
mag_board   = transpose(R) * mag_world
```

Rotation preserves each vector's magnitude. With default vectors, yaw +90 degrees
leaves acceleration along +Z and moves magnetic field from +X to -Y. Pitch +90
degrees moves acceleration from +Z to -X. Noise and averaging are applied to the
resulting board-axis targets using the unchanged models above.

### Generated gyro readings

On each 16 ms IMU tick, the emulator compares the current commanded angles with
the angles at the previous tick:

```text
delta_degrees = (current - previous + 180) % 360 - 180
gyro_target = radians(delta_degrees) / 0.016
```

Roll differences drive gyro X, pitch drive Y, and yaw drive Z. This follows the
official emulator's Euler-angle finite-difference approach, with consistent rad/s
units and shortest-angle wrapping (179 to -179 degrees means +2 degrees).
It is an approximation, not the general transformation from Euler derivatives
to physical body angular velocity during combined rotations. A change of more
than 180 degrees between ticks is ambiguous and takes the shortest path.

A slider jump produces a one-tick rate pulse, then zero if the angle stays fixed.
With noise on, the existing ten-sample average spreads that pulse, and the usual
gyro output limits apply. To simulate sustained rotation, change orientation
progressively over time. Multiple changes within one tick are observed only as
the final angle. For measured gyro data, use direct inputs or CSV replay.

### Orientation getters

`get_orientation()`, `get_orientation_degrees()`, and the `orientation` property
return the **commanded** angles wrapped to `[0, 360)` degrees.
`get_orientation_radians()` / `orientation_radians` return signed angles in
`[-pi, pi)`. `get_readings()` includes signed angle values in degrees.

No accelerometer/magnetometer inference or real-HAT sensor fusion is performed.
Changing a world magnetic vector does not alter commanded yaw. There is no gyro
integration, bias correction, or declination correction. This matches the scope
of an orientation-driven emulator rather than a complete flight physics model.

### Direct inputs

Choose **Direct inputs** in the GUI or call `sense.set_mode('direct')` to control
board-axis acceleration, magnetic field, and gyro independently. Existing
`set_accelerometer_raw`, `set_compass_raw`, and `set_gyroscope_raw` calls also
select direct mode automatically. Their inputs bypass world rotation; noise and
sensor timing still apply when enabled. Orientation remains explicitly commanded
via `set_orientation`; raw inputs do not estimate it.

Choose **World**, `sense.set_mode('world')`, or use a `set_world_*` method to return
to world rotation. Mode changes reset noise histories to current targets, restart
the seeded streams, and establish a new gyro baseline to avoid a transition spike.

## CSV sensor replay

CSV replay supplies board-axis raw IMU values directly, bypassing world rotation
and generated gyro. Each row is a timestamped sample. This lets a testing function
read a recording through the same SenseEmu getters it normally uses.

Required header (column order may vary):

```csv
time_s,temperature_humidity,temperature_pressure,pressure,humidity,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z
0,20,21,1013.25,50,0,0,1,0,0,0,33,0,0
0.1,22,23,1000,48,0.1,0,1.2,0,0,0.5,32,-1,0
0.2,24,25,980,46,0.2,0,1.5,0,0,0.7,31,-2,0
```

Units: seconds from start; temperatures in degrees C; pressure in hPa; humidity
in percent; acceleration in g; gyro in rad/s; magnetic field in microtesla.
Optionally add **all three** `roll,pitch,yaw` columns in degrees. If omitted,
orientation getters raise `ValueError` and the GUI reports orientation unavailable.
No orientation is invented from raw values.

The first timestamp must be zero; later timestamps must strictly increase.
All required cells must be present and finite. Duplicate/unknown headers and
partial orientation columns are rejected. The entire file is validated before
replacing the running simulation. Very large recordings are held in memory.

```python
sense = SenseEmu(seed=42, realtime=False)
sense.load_csv('flight.csv')    # exact replay: extra noise OFF by default
print(sense.get_pressure())     # first row, at time zero
sense.advance(0.1)
print(sense.get_accelerometer_raw())
print(sense.get_replay_status())
sense.restart_replay()         # time zero; same seeded noise sequence
```

Each row takes effect at its timestamp and is held until the next row; there is
no interpolation. Exact replay bypasses noise, smoothing, clipping, quantization,
and sensor tick gating, preserving the supplied samples. After the last row,
playback stops advancing and holds the last outputs. The status contains elapsed
`time_s`, zero-based `sample`, total `samples`, and `paused`/`finished` flags.

For synthetic targets that should pass through sensor models, use
`sense.load_csv('flight.csv', noise=True)`. Existing 130/40/16 ms clocks and noise
parameters then apply. At an exact row/tick boundary the new row is applied before
sampling. A row between ticks may not be sampled by every sensor. At EOF the last
*measured outputs* are frozen, even if averaging has not reached the last targets;
include a final dwell interval in the recording when settling matters.

Real-time playback uses the elapsed monotonic clock; manual playback uses
`advance()`. `pause_replay()` freezes time; `resume_replay()` resumes it without
catching up paused wall time. `restart_replay()` starts over. `stop_replay()` returns
to world mode and restores the noise setting from before loading CSV. Reset sensors
also exits replay. Target setters are blocked during replay; LED setters still work.
The GUI offers Load CSV, Pause/Resume, Restart, Stop CSV, and disables target sliders
while playing. Its noise checkbox can opt into sensor models during replay.

Run the monitor with a recording:

```powershell
python tools/calibration/check_sensors.py --csv flight.csv
python tools/calibration/check_sensors.py --csv flight.csv --replay-noise --seed 42
```

`--csv` forces the emulator even on a Pi with a real HAT attached. The GUI and
monitor share the same instance. To use replay in your own flight-computer tests,
pass that same instance into the functions under test. Separate processes or
separately constructed SenseEmu objects do not share state.

## Remaining scope and references

LED methods remain set_pixel/get_pixel, set_pixels/get_pixels, and clear, with
RGB565 readback. Joystick and text display are not implemented.

The current implementation is the authority for the details documented here:
`humidity.py`, `pressure.py`, `imu.py`, `emulator.py`, and `gui.py` in this folder.
Pressure-temperature conversion uses a consistent offset, and gyro units remain
rad/s throughout, avoiding upstream inconsistencies discussed during development.

Source models: [humidity](https://github.com/astro-pi/python-sense-emu/blob/master/sense_emu/humidity.py),
[pressure](https://github.com/astro-pi/python-sense-emu/blob/master/sense_emu/pressure.py),
[IMU](https://github.com/astro-pi/python-sense-emu/blob/master/sense_emu/imu.py).
