"""Free-flight telemetry at 50 Hz. Writes logs/edl_log.csv and logs/edl_log.png."""

from __future__ import annotations

import os
from time import monotonic, sleep

from .gpio_io import PIN_JOY_CENTER, PIN_SAFE, read_input, set_output
from .hat import is_simulated
from .landing import accel_magnitude, detect_touchdown
from .log import DEFAULT_LOG_PATH, TelemetryLog
from .sensors import read_acceleration, read_pressure, read_temperature

RATE_HZ = float(os.environ.get("EDL_LOG_RATE_HZ", "50"))
UDP = os.environ.get("EDL_LOG_UDP", "1") not in {"0", "false", "False"}
PRINT_EVERY_S = 0.5


def main() -> None:
    dt = 1.0 / max(RATE_HZ, 1.0)
    print(
        f"telemetry {RATE_HZ:.0f} Hz -> {DEFAULT_LOG_PATH}  "
        f"udp={'on' if UDP else 'off'}  hat={'sim' if is_simulated() else 'real'}"
    )
    t0 = monotonic()
    last_print = t0
    n = 0
    saw_touchdown = False
    with TelemetryLog(udp=UDP) as log:
        try:
            while True:
                now = monotonic()
                accel = read_acceleration()
                mag = accel_magnitude(accel)
                landed = detect_touchdown(accel)
                if landed:
                    saw_touchdown = True
                log.write(
                    t_s=now - t0,
                    phase="free_flight",
                    temperature_c=read_temperature(),
                    accel=accel,
                    pressure_mbar=read_pressure(),
                    mag_g=mag,
                    touchdown=landed,
                )
                n += 1
                if now - last_print >= PRINT_EVERY_S:
                    print(
                        f"t={now - t0:6.2f}s  |a|={mag:5.2f}g  "
                        f"T={read_temperature():5.1f}C  P={read_pressure():7.1f}  "
                        f"td={int(saw_touchdown)}  n={n}"
                    )
                    last_print = now
                if _joy_stop():
                    print("joystick center; stop")
                    break
                sleep(dt)
        except KeyboardInterrupt:
            print("\nstopped")
        finally:
            set_output(PIN_SAFE, True)
    print(f"{n} samples -> {DEFAULT_LOG_PATH}")
    _try_plot()


def _try_plot() -> None:
    try:
        from .plot import plot_log

        print(f"wrote {plot_log(DEFAULT_LOG_PATH)}")
    except ImportError:
        print("install matplotlib to write the PNG")
    except Exception as exc:
        print(f"plot skipped ({exc!r})")


def _joy_stop() -> bool:
    try:
        return read_input(PIN_JOY_CENTER)
    except ValueError:
        return False


if __name__ == "__main__":
    main()
