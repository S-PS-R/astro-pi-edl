# Sense HAT emulator
Author: Samir Rathore

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
[SenseEmu] noise=ON | seed=42 | time=realtime | mode=direct
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
the LED colour unless CSV replay supplies RGB values. Closing the GUI stops the monitor.

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

## Hardware ranges used by this emulator

The IMU model uses the maximum ranges shown in the Sense HAT hardware specs.
These replace the narrower settings inherited from the official emulator.

| Sensor | Current range, per axis where applicable | Units returned by the API |
| --- | --- | --- |
| Accelerometer | −16 to +16 g | g |
| Magnetometer | −16 to +16 gauss = −1600 to +1600 µT | µT |
| Gyroscope | −34.90659 to +34.90659 rad/s | rad/s |
| Pressure | 260–1260 hPa | hPa |
| Humidity | 0–100% relative humidity | % |

Conversion: `1 gauss = 100 µT`; `radians/second = degrees/second × pi / 180`.
Gyro values describe rotation **speed**, whereas pitch/roll/yaw describe angles.
The emulator uses one fixed IMU range configuration; it does not select a real
board's hardware range or provide runtime full-scale register configuration.

The IMU quantization steps also match these maximum-range settings; see the
clamping and quantization table below. Noise amplitudes and averaging sample counts are retained; sensor tick rates
use the hardware-supported settings listed below. With **Sensor noise** disabled, exact mode
bypasses clamping as well as noise and quantization. It can therefore return
values outside the hardware ranges, including during default CSV replay.

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
- `imu.py`: independent three-axis acceleration, gyro, and magnetic sensor models.

## Sensor sampling and update rates

These are fixed, hardware-supported output data rates selected for this emulator.
They do not reconfigure a real HAT or claim to match its driver defaults. A tick generates one new sample for each channel in
that model. Frequency in hertz (Hz) is `1 / interval_in_seconds`.

| Model / channels | Tick interval | Sampling rate | Samples averaged per channel | Full history replacement |
| --- | --- | --- | --- | --- |
| Humidity | 80 ms | 12.5 Hz | 10 | 0.80 s |
| Humidity-sensor temperature | 80 ms | 12.5 Hz | 31 | 2.48 s |
| Pressure | 40 ms | 25 Hz | 25 | 1.00 s |
| Pressure-sensor temperature | 40 ms | 25 Hz | 25 | 1.00 s |
| Accelerometer X/Y/Z | ≈1.05042 ms | 952 Hz | 10 per axis | ≈0.01050 s |
| Gyroscope X/Y/Z | ≈1.05042 ms | 952 Hz | 10 per axis | ≈0.01050 s |
| Magnetometer X/Y/Z | 12.5 ms | 80 Hz | 10 per axis | 0.125 s |

Full history replacement is `sample_count × interval`. It describes how long
it takes to replace every old sample after a target change, approximately;
the exact delay depends on where that change falls between ticks. New readings
are available every tick, not only once a whole history has been replaced.
These histories implement a moving average, not an exponential response model.

Rate sources: [HTS221 product specifications](https://www.st.com/en/mems-and-sensors/hts221.html)
list 1–12.5 Hz, and [LPS25HB specifications](https://www.st.com/en/mems-and-sensors/lps25hb.html)
list 1–25 Hz. The [LSM9DS1 datasheet](https://www.st.com/resource/en/datasheet/lsm9ds1.pdf)
lists 952 Hz for gyro/acceleration (Tables 9 and 68) and 80 Hz as the highest
standard magnetometer ODR (Table 111). Magnetometer FAST_ODR modes above 80 Hz
are not modeled. The startup terminal prints the selected rates.

The retained moving-average histories are synthetic emulator smoothing, not a
validated replica of hardware digital filtering or physical response time.
Higher ODR shortens the history duration without changing its sample count.
Pressure remains 25 Hz because it already matched the chip's highest ODR.

The four model clocks advance independently; acceleration/gyro share one clock,
while magnetic samples have a separate clock and random stream. With noise on, the scheduler uses:

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


GUI callbacks and terminal printing take time, so those display intervals are
not hard real-time guarantees. They do not change the sensor periods. In manual
mode, a GUI refresh does not move time; only `advance(seconds)` does.
With noise off, smoothing, clipping, and quantization are bypassed: getters
return validated targets immediately. Exact CSV replay follows row timestamps
instead of sensor ticks.

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
generator (magnetic samples are separate from acceleration/gyro); channels within a model share that generator but receive different
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
| Acceleration, each axis | −16 to +16 g | 1 / 0.000732 | 0 | 0.000732 g |
| Gyroscope, each axis | ±34.90659 rad/s | 1 / radians(0.070) | 0 | 0.00122173 rad/s |
| Magnetic field, each axis | −1600 to +1600 µT | 1 / 0.058 | 0 | 0.058 µT |

The IMU now uses the maximum full-scale settings in the Sense HAT product brief,
replacing the official emulator's narrower ±8 g, ±500 degrees/s, and ±4 gauss
settings. Quantization uses the corresponding typical sensitivities in
[LSM9DS1 datasheet Table 3](https://www.st.com/resource/en/datasheet/lsm9ds1.pdf):
0.732 mg/LSB, 70 millidegrees/s/LSB, and 0.58 milligauss/LSB. A 16-bit output
register does not imply that resolution is simply the range divided by 65536.
IMU slider ranges stay fixed at these limits, including when an API or CSV
input exceeds the modeled hardware range.
Noise amplitudes and averaging sample counts retain the synthetic emulator model;
update rates use the hardware-supported settings above; the product brief does not specify a replacement noise process.

The product brief's 15–40 °C (humidity sensor) and 0–65 °C (pressure sensor)
intervals are **accuracy intervals**, not hard temperature output limits. The
existing error parameters already use ±0.5 °C and ±2 °C respectively within
those intervals. These scale synthetic noise and are not guaranteed error bounds.
Pressure 260–1260 hPa and humidity 0–100% already match the brief. The
emulator also accepts a pressure target of exactly 0 hPa as a useful vacuum
boundary; negative pressure is rejected.

These are model output limits. They are distinct from GUI slider ranges and API
target validation. A numeric target outside a sensor's measurable range is
reported as **OVR** (over range): getters return the sentinel `-1e6`, and the
monitor prints `OVR`. The internal target remains unchanged so it can be brought
back into range later. This applies to temperature, pressure, humidity,
accelerometer, gyroscope, and magnetometer channels, including CSV replay.
Initial/reset readings are exact targets until sampling starts, then the same
range check is applied.

Temperature from humidity and temperature from pressure are separate sensor
channels, not calculations from relative humidity or pressure. For example,
`set_temperature(25)` supplies the same target to both, but their different
noise, histories, and resolutions produce different measurements.

## Board sensor inputs

The emulator supplies board-axis sensor readings for flight-computer tests. It
has one set of controls, with no orientation inputs or World/Direct mode selector. Acceleration,
magnetic field, and gyro are independent inputs: changing one does not change
the others. The GUI also retains environmental controls, the 8x8 LEDs, and the
reserved area beneath the display.

```python
sense = SenseEmu(noise=False, realtime=False)
sense.set_accelerometer_raw(0, 0, 1)  # g, along board X/Y/Z
sense.set_compass_raw(33, 0, 0)       # microtesla, along board X/Y/Z
sense.set_gyroscope_raw(0, 0, 0.5)    # rad/s, around board X/Y/Z
sense.advance(1)
print(sense.get_gyroscope_raw())      # {'x': 0.0, 'y': 0.0, 'z': 0.5}
```

Vector setters accept three numbers or a dictionary with exactly `x`, `y`, `z`
keys. A gyro target is a rotation **rate**; it remains at the supplied target
until changed, with noise and averaging when enabled. It is not calculated from
orientation; integration and sensor fusion belong in the flight-computer code.

IMU slider endpoints stay fixed at ±16 g, ±1600 µT, and ±34.90659 rad/s.
Gyro uses **radians per second everywhere**: GUI sliders, terminal output, raw
getters/setters, sensor models, and CSV `gyro_x/y/z` columns. There is no display
conversion. A slider target of `1.5` means `1.5 rad/s` in the API and CSV as well.
This matches the official Sense HAT `get_gyroscope_raw()` units. Existing rad/s
recordings remain unchanged. Degrees/s appear below only when explaining the
original hardware datasheet specifications.

API/CSV targets can exceed slider endpoints; their numeric values remain visible
while the slider thumbs stay at the endpoints. With noise enabled, output is
averaged, clamped, and quantized. Exact/no-noise mode preserves out-of-range
values for testing. There are no orientation or gyro readouts beneath the LED
matrix; that space is reserved for future controls.

There is no coordinate rotation, gravity calculation, generated gyro, or sensor
fusion. Supply sensor values already expressed in the board's coordinates.
Flight dynamics and interpretation of readings belong in your scenario generator
or flight-computer code. The initial acceleration `(0, 0, 1)` g and magnetic field
`(33, 0, 0)` µT are editable starting values, not an assumed planetary environment.

### Orientation is calculated downstream

This emulator supplies raw sensor readings only. It has no pitch/roll/yaw
controls, orientation getters/setters/properties, gyro integration, or sensor
fusion. The real [Sense HAT API](https://sense-hat.readthedocs.io/en/latest/api/)
provides orientation functions, but these are outside this emulator's supported
subset. Feed raw readings into your flight code to calculate orientation there.
GUI refreshes do not integrate gyro rates or change sensor targets.

### Updating older scripts

World controls and `set_world_acceleration`, `set_world_magnetic_field`, and
`set_mode` have been removed. Supplied-orientation methods (`set_orientation`,
`set_orientation_degrees`, `get_orientation`, `get_orientation_degrees`,
`get_orientation_radians`) and orientation properties have also been removed. Use the raw setters with board-axis values instead;
world-coordinate recordings must be converted outside this emulator first.
The read-only `mode` property reports `direct` for slider/API input and `replay`
for CSV playback. Use `load_csv()` and `stop_replay()` to start/stop playback.

## CSV sensor replay

CSV replay supplies board-axis raw IMU values directly. Each row is a timestamped sample. This lets a testing function
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
The sensor columns above are required. Optional RGB columns are described below. Remove `roll`, `pitch`, and
`yaw` from older CSV recordings; orientation columns are now rejected as unknown
headers. Keep reference orientation data separately for downstream tests.

The first timestamp must be zero; later timestamps must strictly increase.
All required cells must be present and finite. Duplicate/unknown headers and
partial RGB column groups are rejected. The entire file is validated before
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
`sense.load_csv('flight.csv', noise=True)`. The 12.5 Hz humidity, 25 Hz pressure, 952 Hz acceleration/gyro, and 80 Hz magnetic
clocks and existing noise parameters then apply. At an exact row/tick boundary the new row is applied before
sampling. A row between ticks may not be sampled by every sensor. At EOF the last
*measured outputs* are frozen, even if averaging has not reached the last targets;
include a final dwell interval in the recording when settling matters.

Real-time playback uses the elapsed monotonic clock; manual playback uses
`advance()`. `pause_replay()` freezes time; `resume_replay()` resumes it without
catching up paused wall time. `restart_replay()` starts over. `stop_replay()` returns
to slider/API inputs and restores the noise setting from before loading CSV.
It keeps the last raw targets. Reset sensors
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

### Optional LED colour columns

Add all three `led_r,led_g,led_b` columns to set the entire 8x8 matrix at each
row timestamp. Each cell must contain an integer from 0 through 255. Partial RGB
headers, fractional values, and out-of-range channels are rejected before the
recording starts. Files without RGB remain supported and do not change LEDs.

```text
led_r,led_g,led_b
255,0,0       -> red
255,255,0     -> yellow
0,255,0       -> green
```

These columns are appended to the sensor header, not loaded as a separate file.
The first colour applies immediately on load, and restart restores it. Pause and
EOF hold the current colour. `get_replay_status()['controls_leds']` indicates
whether playback owns the matrix, so the monitor does not overwrite CSV colours
with its normal colour cycle. API LED writes remain possible, but the next CSV
row with RGB will replace them. RGB565 readback still applies: 255 red/blue
becomes 248, and 255 green becomes 252.

## Example: Earth exosphere to ocean replay

Load `data/earth_exosphere_drop_to_surface.csv` from the repository root using
the GUI's **Load CSV** button, or run:

```powershell
python tools/calibration/check_sensors.py --csv data/earth_exosphere_drop_to_surface.csv
```

The file contains 3,601 samples at 20 Hz (0.05 s spacing), covering three minutes.
RGB columns set red before 70 s (space), yellow from 70 s to just before 165 s
(re-entry/descent), and green from 165 s onward (touchdown). The 120 km altitude
at 70 s is this scenario's chosen re-entry marker, not a universal boundary.
The sensor values are unchanged by adding RGB.
It is a synthetic environmental test profile with a compressed altitude schedule,
not a solved orbital re-entry or a prediction that an unprotected box survives.
The prescribed altitude schedule and acceleration do not form a dynamically
integrated trajectory. No parachute, ablation, heat transfer, mass, or ballistic
coefficient is modeled. Black surface colour therefore has no modeled thermal
effect. Altitudes below are geometric heights; they are documented here because
the GUI's strict CSV schema has no altitude or layer columns.

| Replay time (s) | Altitude (km) | Scenario milestone |
| --- | --- | --- |
| 0 | 700 | Exosphere start; approximately zero accelerometer output |
| 20 | 500 | Upper-atmosphere descent; layer boundaries vary with conditions |
| 50 | 200 | Thermosphere |
| 70 | 120 | Lower thermosphere |
| 85 | 86 | Mesopause region |
| 105 | 50 | Stratopause region |
| 125 | 20 | Lower stratosphere |
| 135 | 11 | Tropopause region |
| 145 | 5 | Troposphere |
| 153 | 2 | Moist marine lower atmosphere |
| 160 | 0.5 | Ocean approach |
| 165 | 0 | Ocean surface; splashdown pulse peaks at 165.15 s |
| 180 | 0 | Floating/settled end state, approximately +1 g along board Z |

Altitude is linearly interpolated between these milestones. Environmental values
are defined as follows:

- **Temperature and pressure:** based on the U.S. Standard Atmosphere 1976.
  Below 86 km, a piecewise hydrostatic lapse-rate model uses geopotential height.
  Above 86 km, selected standard-atmosphere table points are interpolated linearly
  in temperature and logarithmically in pressure. These are an idealized reference
  atmosphere, not actual weather for a particular ocean location or date.
- **Temperature channel meaning:** both channels deliberately contain the same
  ambient gas temperature, from approximately −86.28 to +726.82 °C, ending at
  15 °C. Upper-atmosphere gas kinetic temperature is not the box temperature.
  These values are environmental proxies for software tests, not plausible
  readings from a real HAT exposed to the entire descent. Sensor thermal lag,
  radiative heating/cooling, and aerodynamic heating are not modeled.
- **Humidity:** an assumed weather profile, not part of the standard atmosphere.
  It uses a dry placeholder of 0% at/above 30 km, then 0.1% at 20 km, 1% at 15 km,
  10% at 11 km, 25% at 8 km, 45% at 5 km, 65% at 3 km, 75% at 2 km, 82% at
  0.5 km, and 85% at the surface. It rises to 99% over three seconds after
  splashdown to represent wet air around a floating, surviving enclosure.
  Relative humidity in near-vacuum is not a meaningful sensor measurement;
  the numeric placeholder is required by the CSV schema. No underwater humidity
  reading or hydrostatic submersion pressure is implied.
- **Acceleration:** near-zero specific force in the initial free-fall portion,
  a prescribed aerodynamic load peaking at 4.5 g around 110 s, roughly 1 g late
  in descent, and a brief approximately 11 g splashdown pulse. Board Z dominates
  during substantial loads because the prescribed tumble damps toward broad-face
  descent. X/Y carry smaller lateral components. Z dominance is a scenario choice,
  not a general property of arbitrary tumbling objects.
- **Gyro and magnetometer:** estimated all-axis tumbling damps toward the surface.
  Body-axis gyro rates are derived from the same external attitude history used
  to rotate the magnetic vector; gyro units are rad/s. A reference surface field
  `(25, 0, 40)` µT scales with `(6371 / (6371 + altitude_km))³`, producing a total
  field of approximately 34.5–47.2 µT. This is an illustrative dipole scaling,
  not a location-specific geomagnetic model. No random noise is baked into the file.

**Use exact replay (Sensor noise OFF, the CSV default).** Most pressures aloft are
below the HAT's 260 hPa lower limit, and many temperatures exceed its ranges.
Enabling sensor noise also enables clipping, averaging, and quantization, so the
hardware model will no longer reproduce the environmental profile. The short
splashdown pulse may be missed by the once-per-second terminal display; the
20 Hz CSV provides several samples across it for faster testing code.

Acceleration is intentionally zero for the first 60 s in near-vacuum, then rises
as aerodynamic loads develop. Early values can be tiny: at 70 s the prescribed
load is only 0.0001 g. Around 105 s, board Z is about 2.82 g, and around 165.15 s
it exceeds 10 g. GUI and terminal acceleration values show four decimal places
so small changes are less easily hidden by rounding. Raising model ODR does not
invent extra motion samples: exact replay remains 20 Hz, the GUI paints at about
30 Hz, and the terminal prints about once per second. A higher-resolution source
CSV is needed to test motion changing faster than the existing recording.

Sources: [U.S. Standard Atmosphere 1976 (NASA/NOAA/USAF)](https://ntrs.nasa.gov/citations/19770009539),
[PDAS standard-atmosphere tables](https://www.pdas.com/bigtables.html),
[NASA atmospheric layers](https://science.nasa.gov/reference/the-heliopedia/),
and [NOAA magnetic field ranges](https://www.ncei.noaa.gov/products/geomagnetism-frequently-asked-questions).

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
