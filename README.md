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
`python -m edl_flight_computer.mars.sim`. Earth runs headless; Mars opens its
mission, plot, and optional emulator windows. Use **Stop and close** in the Mars
mission window to save and close the group.

Mission-specific code lives in `earth/` and `mars/`. Shared components live
in `shared/`; the package root contains only `run.py` and `__init__.py`.

```text
src/edl_flight_computer/
    run.py
    __init__.py
    earth/
        drop.py, ekf.py, live_stream.py, live_plot.py
    mars/
        sim.py, model.py, state.py, led_grid.py, status_window.py, plot_window.py
    shared/
        hardware.py, sensors.py, telemetry.py, notify.py, phases.py
```

The old flat module paths have been replaced by these package paths. The
`edl_flight_computer.run earth` and `edl_flight_computer.run mars` launch commands
remain unchanged.

| File | Responsibility |
| --- | --- |
| `run.py` | Earth/Mars configuration and launcher; defaults to Earth |
| `earth/drop.py` | Earth recording loop, release and landing detection |
| `earth/ekf.py` | Earth's vertical displacement/velocity estimator |
| `earth/live_stream.py` / `earth/live_plot.py` | Earth UDP snapshots and laptop visualization |
| `mars/sim.py` | Mars flight loop and desktop mission lifecycle |
| `mars/model.py` | Mars simulation's atmosphere, flight equations, RK4, and peak tracking |
| `mars/state.py` | Mars mission state and playback clock |
| `shared/phases.py` | Common mission phase names |
| `mars/status_window.py` | Mars mission readouts, speed buttons, and display updates |
| `mars/plot_window.py` | Mars plots, their separate process, and PNG saving |
| `mars/led_grid.py` | Altitude/time LED trail and event colors |
| `shared/sensors.py` / `shared/hardware.py` | Sensor polling and real-board/emulator connection |
| `shared/telemetry.py` | CSV output |
| `shared/notify.py` | Landing email over Gmail SMTP |

## Landing email

At ground contact the program emails the touchdown time, speed, and confirmed
peaks. It sends in the background and only prints a message if the network is down.
Enable it with a Gmail account that has 2-Step Verification and an
[app password](https://myaccount.google.com/apppasswords):

```powershell
$env:EDL_SMTP_USER = 'you@gmail.com'
$env:EDL_SMTP_PASSWORD = 'abcd efgh ijkl mnop'   # app password, not your login
$env:EDL_EMAIL_TO = 'you@gmail.com'              # optional, comma separated
```

On the Pi use `export NAME=value` instead. Without these variables, the run
works normally and skips the email.

`peak_tracker` remembers the highest heating, deceleration, and speed seen so
far. It reports a peak once the signal stays 1% below that maximum for one
simulated second. It also keeps the time and altitude where the maximum occurred.

## Configure the run

Edit the two settings dictionaries in `run.py`. Direct module launches also use
these same defaults:

| Run | Settings |
| --- | --- |
| Mars simulation | `MARS_SETTINGS`: entry conditions, vehicle parameters, update rate, playback speed |
| Live Earth drop | `EARTH_SETTINGS`: detector thresholds, pressure corrections, hardware and laptop streaming |

Earth requires a real Sense HAT unless `EARTH_SETTINGS['use_emulator'] = True` is explicitly
selected for testing. It opens no windows. A vertical EKF estimates height and
velocity from sensors; the configured Earth flight trajectory is not run. See
the recording and laptop plotting instructions below.

Mars uses the connected Sense HAT or falls back to the emulator. It uses the flight
equations and records both model values and board readings; the EKF is not run.
The mission window opens automatically; when the
emulator is selected, its controls and live LED display open beside it. Closing
the mission window stops the run and closes the windows. A separate process
renders the four live altitude plots so rendering does not block the mission
display or emulator. Plots are saved beside the CSV on completion, Stop and
close.

`MARS_SETTINGS['updates_per_sim_second'] = 3.0` in `run.py` sets the flight-state, display,
and plot resolution per simulated second. The real update rate is this value
multiplied by `time_scale`: 3 Hz at 1x and 30 Hz at 10x. Every published state
is plotted and logged. Sensors are still
polled independently at up to 200 Hz, and RK4 uses internal steps no larger
than 0.02 simulated seconds between state publications. `time_scale` defaults
to 10 for Mars: ten simulated seconds pass per real second, so the current
623-second entry takes about 63 real seconds. Set it to 1 for real time; the
Earth recorder uses real elapsed time and is independent of these playback settings.
Select **1x**, **2x**, **5x**, or **10x** beside mission time during a run.
State, mission-display, and plot refresh rates change to 3, 6, 15, or 30 Hz
respectively. Mission time accumulates
continuously across speed changes instead of being rescaled retroactively.
CSV `time_s` is simulated mission time, while `wall_time_s` and sensor sample
timestamps use real elapsed seconds. Speed is along the flight path; altitude
changes at `V * sin(FPA)`.

Peak heating, speed deceleration, and velocity are detected during the run; there is no
analytical glide forecast. A peak is confirmed after the signal stays at least
1% below its observed maximum for one simulated second for all three channels.
The 8x8 LED display maps
mission time left to right and altitude top to bottom. A dim cyan trail shows
the trajectory; white marks the current position. Persistent peak markers are
red for acceleration (deceleration magnitude), orange for heating, and purple
for velocity, located at the maximum's actual time and altitude. Shared cells
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
terminal reports the maximum's time and the later confirmation time; the CSV
records the event. A later, higher maximum rearms detection. A maximum at the
end of a run remains unconfirmed if no sustained decrease has been observed.

Mars keeps the configured entry-interface altitude, speed, and FPA. The
`target_fpa_deg` vehicle setting (default -5 degrees) enables planar bank-angle
guidance: it rotates the lift vector to track a descending FPA, subject to the
available lift. This is not an equilibrium-glide assumption. The model tracks
vertical lift only; crossrange is not simulated. Remove `target_fpa_deg` to
restore unguided lift-up flight. RK4 integration uses steps no larger than
0.02 seconds, independent of GUI delays. Reaching zero altitude means surface
contact; parachute and powered soft-landing dynamics are not implemented.
Heating uses the planet's Sutton-Graves coefficient unless explicitly overridden
in `vehicle_params`.

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

The loop has no artificial rate cap or fixed sleep. Sensor polling retains the
configured nominal rates (IMU 119 Hz, pressure 25 Hz, magnetometer 20 Hz, humidity
12.5 Hz). Every processing iteration becomes a CSV row, including iterations
with repeated readings. `*_sample_s`, `*_fresh`, and `*_status` identify successful
polls, cached values, and failures; poll timestamps are not chip timestamps.

Files are created as `logs/drop_YYYYMMDD_HHMMSS_microseconds.csv`. Columns include
all raw sensors, row sequence, recording and flight times, loop duration, phase,
events, sensor-group timestamps/status, acceleration and corrected gyro
magnitudes, settling/confirmation state, and separately labeled EKF estimates
and uncertainty. No simulated trajectory or heating is presented as an Earth
measurement. Writes are buffered and flushed
every 0.25 seconds and when the file closes. Sampling throughput depends on the Pi
and storage; row frequency does not increase the hardware sample rate.

`EARTH_SETTINGS['detection']` in `run.py` contains the initial tuning values: release below
70% of calibrated acceleration magnitude for 0.025 seconds; settling within
0.10 g of the calibrated magnitude, per-axis acceleration standard deviation
below 0.03 g over 0.25 seconds, and bias-corrected gyro magnitude below 0.10 rad/s.
Settling must hold for five seconds; failed reads or IMU gaps above 0.1 seconds
reset that evidence. The flight timer includes the confirmation interval, whose
start and end are also logged separately.

Validate these thresholds with the packaged Pi. Five seconds of sufficiently
smooth parachute descent can resemble rest to the IMU. Pressure is recorded but
does not control landing detection. The bottom-right green LED flashes on release
as a descent warning, then stays solid after confirmed landing and recording
ends. This Earth warning is not a measured ten-second touchdown prediction.
The existing optional landing email is sent once after confirmation.

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
velocity standard deviations, and update/gap flags are included in every CSV
row and laptop snapshot. Estimates remain blank until calibration completes
and hold between new sensor observations. The laptop's six panels still display
raw sensors. EKF estimates never stop recording or confirm landing; the
five-second IMU detector does that independently and then supplies a zero-speed
constraint. The estimated displacement remains relative to release. Mars uses
its flight equations directly and has no EKF. Physical accuracy needs validation
against the packaged Pi.

## Real-time plots on a laptop

Install this package on the laptop, connect both devices to the same Wi-Fi, and
start the receiver on the laptop:

```sh
python -m edl_flight_computer.earth.live_plot --port 8765
```

Allow Python's UDP traffic on the laptop's private-network firewall. In the Pi's
`run.py`, set `EARTH_SETTINGS['live_plotting'] = True`, set `laptop_ip` in that
dictionary to the laptop's local Wi-Fi IPv4 address, and keep `stream_port = 8765`
(or match the receiver's port). With `live_plotting = False`, no stream is created.

The laptop plots acceleration, gyro, magnetometer, pressure, both temperatures,
and humidity, plus phase and flight time. Plot snapshots are sent at 20 Hz; the
Pi's full-rate CSV remains independent. The receiver retains ten minutes of
history and marks gaps after a disconnection. Packet loss or closing the laptop
does not stop Pi recording. All rendering happens on the laptop; the Pi creates
no windows, PNGs, or plotting processes.
