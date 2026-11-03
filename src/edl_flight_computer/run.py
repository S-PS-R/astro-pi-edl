"""Initialize entry, read sensors continuously and optionally show live status."""
# python -m edl_flight_computer.run --emulator

import argparse
from math import floor, radians
from time import monotonic, sleep
from sense_hat_emulator.hardware_profile import SENSOR_RATES_HZ
from .hardware import get_hat
from .mission import initialize
from .sensors import SensorReader


def monitor(sense, state, source, window=None):
    reader = SensorReader(sense)
    monitor_started_at = monotonic()
    sample_rate = SENSOR_RATES_HZ['accel']
    next_read_time_s = 0.0
    next_window_time_s = 0.0
    next_print_time_s = 0.0
    previous_connection = None
    readings = state.initial_measurements.copy()

    while True:
        now = monotonic()
        monitor_time_s = now - monitor_started_at
        if monitor_time_s < next_read_time_s:
            sleep(next_read_time_s - monitor_time_s)
            continue

        state.elapsed_s = now - state.mission_started_at
        error_message = ''
        try:
            readings = reader.read(monitor_time_s)
            sensor_connected = True
        except (OSError, RuntimeError) as error:
            # A hardware read failure is expected to be recoverable. Keep the
            # previous readings and report it; programming errors remain visible.
            sensor_connected = False
            error_message = str(error)

        if sensor_connected != previous_connection or monitor_time_s >= next_print_time_s:
            print(f't={state.elapsed_s:.1f}s | phase={state.phase.value}', flush=True)
            previous_connection = sensor_connected
            next_print_time_s = monitor_time_s + 1.0

        if window is not None and monitor_time_s >= next_window_time_s:
            snapshot = {
                'source': source,
                'connected': sensor_connected,
                'error': error_message,
                'phase': state.phase.value,
                'elapsed_s': state.elapsed_s,
                'updated_at': now,
                'readings': readings,
            }
            window.publish(snapshot)
            next_window_time_s = monitor_time_s + 0.1

        # Add your ongoing entry calculations here when ready.

        # Wait for the next nominal acquisition slot. Skip missed slots rather
        # than inventing readings to catch up after an operating-system delay.
        completed_time_s = monotonic() - monitor_started_at
        completed_slot = floor(completed_time_s * sample_rate)
        next_read_time_s = (completed_slot + 1) / sample_rate
        wait_time_s = next_read_time_s - completed_time_s
        if wait_time_s > 0:
            sleep(wait_time_s)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--emulator', action='store_true', help='Explicitly use simulated sensors')
    parser.add_argument('--no-window', action='store_true', help='Show status in the terminal only')
    parser.add_argument('--entry-altitude-m', type=float, default=120000.0)
    parser.add_argument('--entry-velocity-m-s', type=float, default=7800.0)
    parser.add_argument('--entry-fpa-deg', type=float, default=-6.5)
    args = parser.parse_args(argv)

    if args.emulator:
        source = 'EMULATOR'
    else:
        source = 'REAL SENSE HAT'
    flight_path_angle_rad = radians(args.entry_fpa_deg)

    print(f'INITIALIZING | source={source}')
    try:
        sense = get_hat(force_emulator=args.emulator, require_hardware=not args.emulator)
        state = initialize(sense, args.entry_altitude_m, args.entry_velocity_m_s, flight_path_angle_rad)
    except (OSError, RuntimeError, ValueError) as error:
        parser.exit(1, f'Initialization failed: {error}\n')

    print(f'ATMOSPHERIC_ENTRY | phase={state.phase.value} | t={state.elapsed_s:.1f}s | '
          f'altitude={state.altitude_m:.1f}m | velocity={state.velocity_m_s:.1f}m/s | '
          f'FPA={args.entry_fpa_deg:.2f}deg')

    window = None
    if not args.no_window:
        from .status_window import StatusWindow
        try:
            window = StatusWindow()
        except (OSError, RuntimeError) as error:
            print(f'Cannot open status window: {error}. Continuing in terminal.', flush=True)

    try:
        monitor(sense, state, source, window)
    except KeyboardInterrupt:
        print('\nFlight program stopped.')
    finally:
        if window is not None:
            window.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
