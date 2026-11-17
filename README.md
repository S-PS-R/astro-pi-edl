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
python -m edl_flight_computer.run
```

Skip creating `.venv` if it already exists. For Raspberry Pi installation and
calibration, see [Pi setup](docs/raspberry_pi_setup.md).

## Code layout

Start with `run.py`: `main()` configures and opens the mission, and
`run_mission()` reads sensors, advances the flight, checks events, logs the
results, and publishes the display. The mission window is always present.
Launch with `python -m edl_flight_computer.run`; no command-line options are
needed. The plot and emulator windows stay open for the mission. Use **Stop
and close** in the mission window to save and close the group.

| File | Responsibility |
| --- | --- |
| `model.py` | Atmosphere, flight equations, RK4 integration, and `PeakTracker` |
| `state.py` | Mission state and the clock used for playback speed changes |
| `status_window.py` | Mission readouts, speed buttons, and display updates |
| `plot_window.py` | Four plots, their separate process, and PNG saving |
| `led_grid.py` | Altitude/time LED trail and event colors |
| `sensors.py` / `hardware.py` | Sensor polling and real-board/emulator connection |
| `telemetry.py` | CSV output |
| `notify.py` | Landing email over Gmail SMTP |

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

Edit the settings and initial conditions inside `run.main()`:

| Run | Settings |
| --- | --- |
| Mars simulation | `planet_name = 'mars'` |
| Live Earth drop | `planet_name = 'earth'`; set the release height in the Earth initial conditions |

Both runs use the connected Sense HAT, or automatically fall back to the emulator.
The EKF is not run. Mars uses the flight equations and records both model values
and board readings. Earth uses the configured drop model and records raw board
readings; measured drop velocity estimation remains unfinished. The mission window opens automatically; when the
emulator is selected, its controls and live LED display open beside it. Closing
the mission window stops the run and closes the windows. A separate process
renders the four live altitude plots so rendering does not block the mission
display or emulator. Plots are saved beside the CSV on completion, Stop and
close.

`updates_per_sim_second = 3.0` in `run.main()` sets the flight-state, display,
and plot resolution per simulated second. The real update rate is this value
multiplied by `time_scale`: 3 Hz at 1x and 30 Hz at 10x. Every published state
is plotted and logged. Sensors are still
polled independently at up to 200 Hz, and RK4 uses internal steps no larger
than 0.02 simulated seconds between state publications. `time_scale` defaults
to 10 for Mars: ten simulated seconds pass per real second, so the current
623-second entry takes about 63 real seconds. Set it to 1 for real time; the
Earth drop defaults to 1.
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
window shows the current axis ranges. The grid continues cycling after landing
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
