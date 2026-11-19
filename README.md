# Astro Pi EDL

Mars entry simulation and Sense HAT drop recording. The main program is
`src/edl_flight_computer/run.py`.

## Install and run

From the repository root in PowerShell:

```powershell
python -m venv .venv
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
.\.venv\Scripts\Activate.ps1
python -m pip install -e .
python -m edl_flight_computer.run earth
python -m edl_flight_computer.run mars
```

Skip creating `.venv` if it already exists. For Raspberry Pi installation and
calibration, see [Pi setup](docs/raspberry_pi_setup.md).

## Code layout

`run.py` selects a mission and imports only its entry point. With no argument it
starts Earth recording. It holds the settings for both missions and passes them
to their separate execution modules:

```sh
python -m edl_flight_computer.run earth
python -m edl_flight_computer.run mars
```

You can also launch either mission directly with
`python -m edl_flight_computer.earth.drop` or
`python -m edl_flight_computer.mars.sim`. Earth runs headless on real hardware
and opens sensor controls when using the emulator. Mars opens its
mission, plot, and optional emulator windows. Use **Stop and close** in the Mars
mission window to save and close the group.

Mission-specific code lives in `earth/` and `mars/`. Shared components live
in `shared/`; the package root contains only `run.py` and `__init__.py`.

```text
src/edl_flight_computer/
    run.py
    __init__.py
    earth/
        drop.py, ekf.py
    mars/
        sim.py, model.py, state.py, led_grid.py, status_window.py, plot_window.py
    shared/
        hardware.py, sensors.py, telemetry.py, phases.py,
        downlink.py, ground_station.py, live_plot.py
```

The old flat module paths have been replaced by these package paths. The
`edl_flight_computer.run earth` and `edl_flight_computer.run mars` launch commands
remain unchanged.

| File | Responsibility |
| --- | --- |
| `run.py` | Earth/Mars configuration and launcher; defaults to Earth |
| `earth/drop.py` | Earth recording loop, release and landing detection |
| `earth/ekf.py` | Earth's vertical displacement/velocity estimator |
| `mars/sim.py` | Mars flight loop and desktop mission lifecycle |
| `mars/model.py` | Mars simulation's atmosphere, flight equations, RK4, and peak tracking |
| `mars/state.py` | Mars mission state and playback clock |
| `shared/phases.py` | Common mission phase names |
| `mars/status_window.py` | Mars mission readouts, speed buttons, and display updates |
| `mars/plot_window.py` | Mars plots, their separate process, and PNG saving |
| `mars/led_grid.py` | Altitude/time LED trail and event colors |
| `shared/sensors.py` / `shared/hardware.py` | Sensor polling and real-board/emulator connection |
| `shared/telemetry.py` | CSV output |
| `shared/downlink.py` | UDP telemetry and event packets to the ground station |
| `shared/ground_station.py` | Laptop ground station: events, link status, packet log, live plots |
| `shared/live_plot.py` | Ground station's telemetry plots (visualization only) |

## Ground station downlink

Both missions send periodic **telemetry** snapshots and critical **events** to a
ground station laptop over UDP. Mars events are entry interface, confirmed peak
velocity, heating, and deceleration, `parachute_deploy`, `terminal_descent`,
`landing_warning`, landing (with a summary of the confirmed peaks), and stop. Earth events
are recording start, calibration complete, release, confirmed landing (with
flight time), and stop. Each event is also printed in the flight terminal.

Configure the shared `GROUND_STATION` dictionary in `run.py`:

| Setting | Meaning |
| --- | --- |
| `enabled` | `False` creates no socket |
| `host` | Laptop's Wi-Fi IPv4 address for Pi runs; `127.0.0.1` (the default) for a ground station on the same computer, e.g. the Mars simulator or emulator runs |
| `port` | UDP port, default 8765; must match the receiver |

Telemetry rates are `MARS_SETTINGS['telemetry_hz']` (10 Hz real time, independent
of playback speed) and `EARTH_SETTINGS['telemetry_hz']` (20 Hz). Events are sent
immediately and repeated three times, 0.25 s apart, with the same `event_id`; the
ground station shows the first copy. Every packet is one JSON object:

```json
{"v": 1, "type": "event", "mission": "mars", "run": "run_20261117_...", "seq": 42,
 "time_s": 512.3, "event_id": 4, "event": "landing", "message": "...", "data": {...}}
```

Telemetry packets have `"type": "telemetry"` and carry the full CSV row (Mars) or
plot snapshot (Earth) in `data`. `run` is the flight CSV's file name, and `seq`
counts every datagram so the receiver can report packet loss. Network errors never
stop the flight loop. The Pi's CSV remains the authoritative record.

Start a ground station on the laptop before the run:

```sh
python -m edl_flight_computer.shared.ground_station --port 8765
```

The terminal prints events as they arrive, with a landing banner, and a
once-per-second status line showing key telemetry, packet loss, and `NO SIGNAL`
after two silent seconds. A plot window (`shared/live_plot.py`) shows the
telemetry: Earth sensor panels for drops, and altitude, speed, heat flux, load
factor, and board pressure/temperature for Mars. A new run clears the plots.
Received packets are appended to `logs/ground_<timestamp>.jsonl`. Options:
`--no-plot` for console only, `--log PATH`, `--no-log`.

## Fault detection

Each mission range-checks raw sensor readings against `fault_limits` in `run.py`,
which maps a channel to its nominal `(low, high)` range (`None` = no limit). A
reading outside its range for 0.5 s sends a `fault` event; back in range for
0.5 s sends `fault_cleared`. Active faults are listed in every telemetry
packet's `faults` field and on the ground station status line. Both missions
check temperature (overtemp), humidity, and pressure (`ENVIRONMENT_LIMITS`).
Mars also checks gyro rotation, because the board should sit still during the
simulation; Earth does not, since the drop is supposed to move. Checks run on
real time at every sensor poll. With the emulator, trigger a fault by moving a
sensor slider out of range. Tune the temperature limit to the packaged board's
idle reading: Pi CPU heat raises it.

`peak_tracker` remembers the highest heating, strongest braking acceleration, and speed seen so
far. It reports a peak once the signal stays 1% below that maximum for one
simulated second. It also keeps the time and altitude where the maximum occurred.

## Configure the run

Edit the two settings dictionaries in `run.py`. Direct module launches also use
these same defaults:

| Run | Settings |
| --- | --- |
| Mars simulation | `MARS_SETTINGS`: entry conditions, vehicle and parachute parameters, update rate, playback speed |
| Live Earth drop | `EARTH_SETTINGS`: detector thresholds, pressure corrections, hardware and laptop streaming |

Earth uses the connected Sense HAT or automatically falls back to the emulator.
Real hardware opens no windows. Emulator fallback opens its sensor controls and
LED display using the same board instance as the recorder. A vertical EKF estimates displacement and
velocity from sensors; the configured Earth flight trajectory is not run. See
the recording and laptop plotting instructions below.

Mars uses the connected Sense HAT or falls back to the emulator. It uses the flight
equations and records both model values and board readings; the EKF is not run.
The mission window opens automatically; when the
emulator is selected, its controls and live LED display open beside it. Closing
the mission window stops the run and closes the windows. A separate process
renders the live altitude plots so rendering does not block the mission
display or emulator. The Mars windows and saved plots always use dark mode.

Each Mars run saves its files in `logs/run_YYYYMMDD_HHMMSS_microseconds/`.
The images are saved when the mission finishes or you choose **Stop and close**.

- `run_<timestamp>.csv` with the flight and sensor data
- `run_<timestamp>.png` with the full-flight plots
- `run_<timestamp>_descent_detail.png` with the plots zoomed into descent
- `final_mission_window.png` with the final mission readouts
- `final_emulator.png` with the final emulator display when the emulator is open

`MARS_SETTINGS['updates_per_sim_second'] = 3.0` in `run.py` sets the flight-state, display,
and plot resolution per simulated second. The real update rate is this value
multiplied by `time_scale`: 3 Hz at 1x and 30 Hz at 10x. Every published state
is plotted and logged. Sensors are still
polled independently at up to 200 Hz, and RK4 uses internal steps no larger
than 0.02 simulated seconds between state publications. `time_scale` defaults
to 10 for Mars, so the default flight reaches ground contact after about 76 real
seconds (761 simulated seconds). Set it to 1 for real time. The Earth recorder
uses real elapsed time and is independent of these playback settings.
Select **1x**, **2x**, **5x**, **10x**, or **50x** beside mission time during a run.
The default remains **10x**. State, mission-display, and plot refresh rates target
3, 6, 15, 30, or 150 Hz
respectively. Mission time accumulates
continuously across speed changes instead of being rescaled retroactively.
Mars CSV columns are ordered as `time_s`, phase, calculated values, then raw
sensor values. `time_s` is simulated mission time. Playback speed and sensor
poll timestamps stay internal. Speed is along the flight path, and altitude
changes at `V * sin(FPA)`. The CSV and UDP telemetry include `downrange_m`,
the distance along Mars's surface from entry interface. The model integrates
its rate as `V * cos(FPA) * radius / (radius + altitude)`.

`acceleration_m_s2` is the original speed derivative, `dV/dt`: negative when
slowing and positive when speeding up. It is along the flight path, rather than
vertical. `g_load` is that signed acceleration divided by local gravity.
Downward motion is represented by negative `vertical_velocity_m_s`; accelerating
downward in free fall increases speed, so `dV/dt` is positive in that case.
`vertical_acceleration_m_s2` is the derivative of vertical velocity, positive
upward. The simulation also publishes `mach` and
`equilibrium_speed_m_s`, which is populated during descending flight under canopy.
The Mars CSV and downlink also report vehicle Cd, canopy Cd, effective Cl, and
vehicle, canopy, and total drag. Canopy Cd is blank while stowed.

The altitude plots are arranged in three columns. The first shows downrange,
speed with local equilibrium speed, and vertical acceleration. The second
shows flight-path angle, dynamic pressure, and Mach. The third shows drag,
convective heating, and aerodynamic coefficients. A dotted line marks the exact deployment altitude.
The drag plot preserves both sides of the instantaneous drag change.
Use **Descent detail** in the plot window to zoom in on the canopy portion of
the flight.

Peak heating, entry peak deceleration, and peak velocity are detected during
the run. A peak is confirmed after the signal stays at least 1% below its
observed maximum for one simulated second. The first confirmed entry deceleration
peak emits one `peak_acceleration` event. The `parachute_deploy` event carries the
opening load and its along-path acceleration.
The 8x8 LED display maps
mission time left to right and altitude top to bottom. A dim cyan trail shows
the trajectory. White marks the current position. Persistent event markers are red
for entry peak deceleration, orange for peak heating, purple for peak velocity,
blue for parachute deployment, and yellow for terminal descent. Markers use the
event or peak's simulated time and altitude. Shared cells
cycle through event colors every 0.6 real seconds instead of overwriting them.
The purple velocity marker appears only after confirmation, at the maximum's
recorded time and altitude. There is no unconfirmed purple/white preview.
The time axis starts at 0–720 simulated seconds and doubles if needed; the
altitude range starts at entry altitude and expands if exceeded. The mission
window keeps the mission readouts; grid-axis details are comments in `mars/led_grid.py`. The grid continues cycling after landing
until the window is closed. The bottom-right cell `(7, 7)` turns solid green on
ground contact, overriding any trajectory/event color in that status cell.
The white position marker turns off at ground contact.
Stopping or cancelling before ground contact does not turn it green.
LEDs clear when the application closes or a new
run starts. The
terminal reports the maximum's time and the later confirmation time. The CSV
records the calculated values and phase; peak events appear in the terminal, the
LED display, and the ground station downlink. A later, higher heating or velocity
maximum rearms detection. Peaks awaiting a sustained decrease remain unconfirmed.

Mars starts at the configured entry altitude, speed, and flight-path angle with
fixed lift-up entry at zero bank. `MARS_SETTINGS['parachute']` configures the
21.35 m disk-gap-band canopy and deployment and terminal
descent thresholds. The deployment event fires while descending at Mach 1.8 or
below, at or below 750 Pa dynamic pressure, and at or above 1,000 m altitude.
The terminal prints `EVENT parachute_deploy` with altitude, speed, Mach, dynamic
pressure, the change in total drag, and the along-path acceleration at opening.
The UDP event includes
the separate vehicle and canopy drag forces. The CSV captures the deployment
state even when the crossing falls between regular samples.

The model computes Mach at each RK4 stage from `V / sqrt(gamma * R * T)`.
It uses the pure CO₂ gas constant of 188.9 J/(kg·K) for density and sound
speed, and derives `gamma = cp / (cp - R)` from a temperature-dependent CO₂
heat capacity. The current atmosphere is isothermal at 214 K, which gives
a sound speed of about 232.3 m/s. [NASA/TM–2019-220019, Eq. 7](https://ntrs.nasa.gov/api/citations/20190001449/downloads/20190001449.pdf#page=12)

The canopy adds its full drag immediately at deployment, using a
[Mach-dependent drag curve from NASA's MSL model](https://ntrs.nasa.gov/api/citations/20130012762/downloads/20130012762.pdf#page=25).
The flight model sums the vehicle and canopy drag areas and assumes zero
effective lift under canopy. Terminal descent begins after the speed stays
within 5% of local equilibrium speed and the magnitude of vertical acceleration
stays at or below 0.10 m/s² for one simulated second. The terminal event reports
both speeds, altitude, flight-path angle, and vertical acceleration.

The separate landing warning fires when the altitude divided by the current
descent rate reaches 10 seconds. It starts the pulsing green LED, which turns
solid green at ground contact. This model ends at surface contact under canopy.
Inflation loads, vehicle separation, and powered soft landing are outside the
simulation. Heating uses the planet's Sutton-Graves coefficient.

## Headless Earth recording

Configure `EARTH_SETTINGS` in `run.py` and run:

```sh
python -m edl_flight_computer.run
```

Keep the package stationary for two seconds of accelerometer/gyro calibration.
Recording starts immediately, including calibration and waiting, and continues
until release followed by five continuous seconds of settling. The flight timer
starts on confirmed release. Renewed movement resets the settling timer; being
stationary before release never ends the run. Ctrl+C closes the CSV without
reporting landing.

When the emulator is selected, its window opens automatically. Recording runs
in a worker thread so sliders, CSV replay, and the LED display stay responsive.
Closing the window stops recording and closes the CSV. After a confirmed landing,
the window stays open to show the final green LED until you close it. Mars also
opens the emulator controls whenever it uses simulated hardware.

Earth uses the same target rates on the Pi and desktop emulator. Configure them
in `EARTH_SETTINGS` in `run.py`:

| Setting | Target rate | Purpose |
| --- | --- | --- |
| `poll_hz` | 250 Hz | Check sensors and update detection/EKF |
| `csv_hz` | 120 Hz | Write CSV snapshots |
| `telemetry_hz` | 20 Hz | Send ground station telemetry snapshots |

The loop sleeps between monotonic deadlines. Faster hardware does not produce
unlimited duplicate CSV rows. Missed deadlines are skipped; no catch-up samples
are invented. Rates are targets, subject to Pi processing, storage, and Linux
scheduling. The final landing/stop row is written immediately outside the regular
CSV schedule when necessary.

Sensor groups retain their configured nominal rates (IMU 119 Hz, pressure 25 Hz,
magnetometer 20 Hz, humidity 12.5 Hz). Slower readings repeat between updates.
EKF updates use actual elapsed time between fresh IMU polls. Poll timestamps,
freshness/status flags, timing diagnostics, and the release timer remain internal
and in laptop snapshots; they are omitted from the normal CSV.

Each Earth run creates `logs/drop_<timestamp>/drop_<timestamp>.csv`. The desktop
emulator saves `final_emulator.png` in the same folder. CSV columns are ordered
as recording time (`time_s`), phase, calculated values, then raw sensor readings.
Calculated values are EKF displacement, vertical velocity, acceleration bias,
displacement/velocity uncertainty, acceleration magnitude, and corrected gyro
magnitude. Raw values include all accelerometer, gyro, magnetometer, pressure,
temperature, and humidity channels. CSV row intervals can be calculated from
consecutive `time_s` values. No simulated trajectory or heating is presented as an Earth
measurement. Writes are buffered and flushed
every 0.25 seconds and when the file closes. Sampling throughput depends on the Pi
and storage; row frequency does not increase the hardware sample rate.

`EARTH_SETTINGS['detection']` in `run.py` contains the initial tuning values: release below
70% of calibrated acceleration magnitude for 0.025 seconds; settling within
0.10 g of the calibrated magnitude, per-axis acceleration standard deviation
below 0.03 g over 0.25 seconds, and bias-corrected gyro magnitude below 0.10 rad/s.
Settling must hold for five seconds; failed reads or IMU gaps above 0.1 seconds
reset that evidence. The internal flight timer includes the confirmation interval;
phase changes in the CSV mark release and confirmed landing.

Validate these thresholds with the packaged Pi. Five seconds of sufficiently
smooth parachute descent can resemble rest to the IMU. Pressure is recorded but
does not control landing detection. The bottom-right green LED flashes on release
as a descent warning, then stays solid after confirmed landing and recording
ends. This Earth warning is not a measured ten-second touchdown prediction.
A `landing` event is sent to the ground station at confirmation.

### Earth EKF estimates

`EarthLandingEKF` in `earth/ekf.py` estimates vertical displacement from release,
upward vertical velocity (negative during descent), and vertical accelerometer
bias. Position and speed start at zero; no drop height is configured. A negative
displacement means the Pi has moved below its starting point, rather than its
height above the floor. Stationary calibration initializes board vertical direction,
gyro bias, and acceleration bias. Gyro-propagated orientation projects all
accelerometer axes onto vertical before removing gravity.

Fresh IMU polls drive prediction with their actual elapsed time. Fresh pressure
polls provide nonlinear barometric corrections relative to pre-release pressure.
Set `EARTH_SETTINGS['ekf_use_pressure'] = False` for a sealed box: trapped pressure is not an
ambient altitude measurement. With pressure disabled, in-flight estimates drift
as IMU errors accumulate. Sensor gaps coast with increased uncertainty rather
than integrating stale accelerations.

`ekf_displacement_m`, `ekf_vertical_velocity_m_s`, `ekf_accel_bias_m_s2`, displacement and
velocity standard deviations are included in every CSV row and laptop snapshot.
Update/gap flags remain in laptop snapshots. Estimates remain blank until calibration completes
and hold between new sensor observations. The laptop's six panels still display
raw sensors. EKF estimates never stop recording or confirm landing; the
five-second IMU detector does that independently and then supplies a zero-speed
constraint. The estimated displacement remains relative to release. Mars uses
its flight equations directly and has no EKF. Physical accuracy needs validation
against the packaged Pi.

## Real-time plots on a laptop

Install this package on the laptop, connect both devices to the same Wi-Fi, and
start the ground station on the laptop:

```sh
python -m edl_flight_computer.shared.ground_station --port 8765
```

Allow Python's UDP traffic on the laptop's private-network firewall. In the Pi's
`run.py`, set `GROUND_STATION['host']` to the laptop's local Wi-Fi IPv4 address
and keep `port = 8765` (or match the receiver's port). See
[Ground station downlink](#ground-station-downlink).

The laptop plots acceleration, gyro, magnetometer, pressure, both temperatures,
and humidity, plus phase and flight time, and prints downlinked events in its
terminal. Plot snapshots are sent at 20 Hz; the
Pi's full-rate CSV remains independent. The plots retain the most recent 16,000
snapshots (over 13 minutes at 20 Hz) and break the line where snapshots are missing. Packet loss or closing the laptop
does not stop Pi recording. All rendering happens on the laptop; the Pi creates
no mission plots or PNGs. Real-hardware Earth runs remain headless; emulator
fallback opens only the sensor-control window locally.
