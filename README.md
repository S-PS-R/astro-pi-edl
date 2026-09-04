# EDL flight computer

Pi 5 + Sense HAT V2 for the Group 4 C&DH demo.

```bash
PYTHONPATH=src python3 -m edl_flight_computer.demo
PYTHONPATH=src python3 -m edl_flight_computer.record
PYTHONPATH=src python3 -m edl_flight_computer.plot
```

`demo` simulates EDL timeline. `record` logs at 50 Hz to `logs/edl_log.csv` (and a PNG if matplotlib is installed). `plot` graphs an existing CSV.

UDP off the Pi is optional. Set `EDL_LOG_UDP=0` to keep the file local. Set `EDL_COMMS_HOST` for unicast instead of broadcast.

On the Pi install `sense-hat` and `python3-matplotlib` and turn on I2C. On a laptop with no HAT, the code uses the built-in simulator.

Phase names and order live in `src/edl_flight_computer/sequence.py` (`DEFAULT_PHASES`). Change that list when the mission profile is defined.

## What each file does

Code is in `src/edl_flight_computer/`. It runs on the Pi. The HAT sits on the 40-pin header.

**hat.py** — Opens the Sense HAT. If I2C is missing, it pretends to be the HAT so the rest of the stack still runs.

**sensors.py** — Board temperature (HTS221), accel in g (LSM9DS1), pressure in mbar (LPS25HB).

**gpio_io.py** — Treats LED-matrix columns as event lines (parachute, descent engine, heatshield, safe). Joystick is the discrete input.

**sequence.py** — Phase clock: `start_edl()`, `advance_phase()`. No hardware.

**display.py** — One status line on HDMI and a phase color on the 8×8 matrix.

**landing.py** — Touchdown if IMU |a| crosses `TOUCHDOWN_G`.

**comms.py** — Sends the touchdown string over Wi-Fi UDP (port 5770).

**log.py** — Appends rows to `logs/edl_log.csv`. Can also stream those rows over UDP.

**record.py** — High-rate capture for a toss / drop. Writes the CSV and tries to make the PNG.

**plot.py** — Draws accel, temperature, and pressure from a CSV.

**faults.py** — Safe mode: drop chute and engine columns, light the safe column.

**demo.py** — Runs the sequence end to end.

**__init__.py** — Re-exports the functions the other modules call.
