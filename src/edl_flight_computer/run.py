"""Initialize entry, continuously read sensors and optionally display status."""

import argparse
from math import floor, radians
from time import monotonic, sleep

from .hardware import get_hat
from .mission import initialize
from .sensors import SensorReader
from sense_hat_emulator.hardware_profile import SENSOR_RATES_HZ


def monitor(sense, state, source, window=None):
    """Keep acquisition independent of the optional display. No descent physics yet."""
    reader = SensorReader(sense)
    origin = monotonic()
    deadline = 0.0
    next_display = next_print = 0.0
    readings = state.initial_measurements.copy()
    previous_connection = None
    while True:
        now = monotonic()
        relative = now - origin
        if relative < deadline:
            sleep(deadline - relative)
            continue
        state.elapsed_s = now - state.mission_started_at
        error = ''
        try:
            readings = reader.read(relative)
            connected = True
        except (OSError, RuntimeError, ValueError, TypeError, KeyError) as exc:
            connected, error = False, str(exc)
        if connected != previous_connection or relative >= next_print:
            print(f"t={state.elapsed_s:.1f}s | phase={state.phase.value}", flush=True)
            previous_connection = connected
            next_print = relative + 1
        if window is not None and relative >= next_display:
            window.publish(dict(source=source, connected=connected, error=error,phase=state.phase.value, elapsed_s=state.elapsed_s, updated_at=now, readings=readings))
            next_display = relative + 0.1

        # Add ongoing entry calculations here when ready.

        after = monotonic() - origin
        deadline = (floor(after * SENSOR_RATES_HZ['accel']) + 1) / SENSOR_RATES_HZ['accel']
        sleep(max(0, deadline - after))


def main(argv=None):

    # Entry conditions
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--emulator", action="store_true", help="Explicitly use simulated sensors")
    parser.add_argument("--no-window", action="store_true", help="Run sensor monitoring in the terminal only")
    parser.add_argument("--entry-altitude-m", type=float, default=120000.0)
    parser.add_argument("--entry-velocity-m-s", type=float, default=7800.0)
    parser.add_argument("--entry-fpa-deg", type=float, default=-6.5)
    args = parser.parse_args(argv)

    try:
        print("INITIALIZING | source=" + ("EMULATOR" if args.emulator else "REAL SENSE HAT"))
        sense = get_hat(force_emulator=args.emulator, require_hardware=not args.emulator)
        state = initialize(sense, args.entry_altitude_m, args.entry_velocity_m_s, radians(args.entry_fpa_deg))
    except (OSError, RuntimeError, ValueError, KeyError, TypeError) as exc:
        parser.exit(1, f"Initialization failed: {exc}\n")
    print(f"ATMOSPHERIC_ENTRY | phase={state.phase.value} | t={state.elapsed_s:.1f}s | "
          f"altitude={state.altitude_m:.1f}m | velocity={state.velocity_m_s:.1f}m/s | "
          f"FPA={args.entry_fpa_deg:.2f}deg")
    window = None
    try:
        if not args.no_window:
            from .status_window import StatusWindow
            try:
                window = StatusWindow()
            except (OSError, RuntimeError) as exc:
                print(f'Status window unavailable ({exc}); continuing in terminal.', flush=True)
        monitor(sense, state, 'EMULATOR' if args.emulator else 'REAL SENSE HAT', window)
    except KeyboardInterrupt:
        print('\nFlight program stopped.')
    finally:
        if window is not None:
            window.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
