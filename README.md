# EDL flight computer

Raspberry Pi 5 + Sense HAT V2 as the C&DH box for the Group 4 EDL demo.

```bash
PYTHONPATH=src python3 -m edl_flight_computer.demo      # sequenced EDL run
PYTHONPATH=src python3 -m edl_flight_computer.record    # 50 Hz telemetry to CSV (+ PNG)
PYTHONPATH=src python3 -m edl_flight_computer.plot      # graph an existing CSV
```

`record` writes `logs/edl_log.csv` on the Pi. UDP to a laptop is optional (`EDL_LOG_UDP=0` to disable). Set `EDL_COMMS_HOST` if you need unicast.

On the Pi: `sudo apt install sense-hat python3-matplotlib` and enable I2C. Without the HAT the stack uses the built-in simulator.

Mission-specific timeline goes in `src/edl_flight_computer/sequence.py` (`DEFAULT_PHASES`).

## Files

All under `src/edl_flight_computer/`. Python runs on the Pi 5. The Sense HAT is only a peripheral on the 40-pin header.

| File | Function | Hardware |
|---|---|
| `hat.py` | One shared HAT handle; simulator if I2C is missing | Sense HAT I2C / LED driver |
| `sensors.py` | `read_temperature()`, `read_acceleration()`, `read_pressure()` | HTS221, LSM9DS1, LPS25HB |
| `gpio_io.py` | Event discretes and joystick read | LED matrix columns 0-7; HAT joystick |
| `sequence.py` | EDL phase timeline (`start_edl`, `advance_phase`) | None (software clock) |
| `display.py` | Live status line + phase color on the matrix | Pi HDMI + 8x8 LEDs |
| `landing.py` | Touchdown flag from IMU |a| | LSM9DS1 accel |
| `comms.py` | Touchdown packet (`send_landing_message`) | Pi 5 Wi-Fi (UDP :5770) |
| `log.py` | Append CSV rows; optional UDP stream | microSD (`logs/edl_log.csv`) |
| `record.py` | 50 Hz free-flight capture + PNG | Same sensors as `sensors.py` |
| `plot.py` | Graph a CSV to PNG | None (writes `logs/edl_log.png`) |
| `faults.py` | Safe mode: drop chute/engine, assert safe | LED columns |
| `demo.py` | Walk the EDL sequence end to end | All of the above |
| `__init__.py` | Public imports | None |
