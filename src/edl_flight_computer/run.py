"""Initialize entry, read sensors continuously and optionally show live status."""
# python -m edl_flight_computer.run --emulator

from math import isfinite, radians
from time import monotonic
from sense_hat_emulator.emulator import RANGES
from .hardware import get_hat
from .mission import initialize
from .sensors import SENSOR_GROUP_CHANNELS, SensorReader
from .EKF import PLANETS, LiftingBodyEKF

def monitor(sense, state, source, estimator, window=None):
    reader = SensorReader(sense, initial_readings=state.initial_measurements)
    previous_at = state.mission_started_at
    previous_errors = {}

    while True:
        # One time reference per iteration. dt advances the estimator; elapsed mission time lets SensorReader decide which groups are due.
        now = monotonic()
        dt = now - previous_at
        previous_at = now
        state.elapsed_s = now - state.mission_started_at

        # Due groups are polled; all other groups retain their previous values.
        readings = reader.read(state.elapsed_s, recover_errors=True)
        sensor_connected = not reader.errors
        error_message = '; '.join(f'{group}: {message}' for group, message in reader.errors.items())

        # Check every axis and environmental channel against the shared hardware
        # ranges. A failed group is unavailable even if its cached values look valid.
        measurement_usable = {}
        for group, channels in SENSOR_GROUP_CHANNELS.items():
            for name in channels:
                value = readings[name]
                lower, upper = RANGES[name]
                measurement_usable[name] = (
                    group not in reader.errors
                    and isfinite(value)
                    and lower <= value <= upper
                )

        # Only successfully polled, usable channels are new observations. Cached
        # values can be held as prediction inputs without being corrected again.
        # Values retain sensor units here; convert them before their EKF updates.
        new_measurements = {
            name: readings[name]
            for group in reader.updated_groups
            for name in SENSOR_GROUP_CHANNELS[group]
            if measurement_usable[name]
        }

        # Current Mass
        current_mass = state.mass

        # Add EKF state/covariance prediction here using dt, even after read errors.
        estimator.predict(dt, mass = current_mass)


        # Add measurement corrections here using new_measurements. For a vector
        # update, require all its axes to be usable. Held IMU inputs also need
        # appropriate bias/frame handling and noise modeling during prediction.
        if 'pressure' in new_measurements:
            pressure_pa = new_measurements['pressure'] * 100.0
            # estimator.update_pressure(pressure_pa)

        # Add derived quantities, mission triggers and LED updates here.

        if reader.errors != previous_errors:
            if reader.errors:
                print(
                    f"Sensor Read Failure | t={state.elapsed_s:.3f}s | "
                    f"{error_message}",
                    flush=True,
                )
            else:
                print(
                    f"Sensor Reads Recovered | t={state.elapsed_s:.3f}s",
                    flush=True,
                )

            previous_errors = reader.errors.copy()

        if window is not None:
            snapshot = {
                'source': source,
                'connected': sensor_connected,
                'error': error_message,
                'phase': state.phase.value,
                'elapsed_s': state.elapsed_s,
                'updated_at': now,
                'readings': readings,
                'measurement_usable': measurement_usable,
                'updated_groups': tuple(reader.updated_groups),
                'sample_times_s': reader.sample_times_s.copy(),
                'sensor_errors': reader.errors.copy(),
            }
            window.publish(snapshot)


def main(argv=None):

    #-----------Intial Condtions defined here-----------
    use_emulator = True
    show_window = True
    planet_name = 'mars'
    entry_conditions = {
        "mars": {
            "altitude_m": 120000.0,
            "velocity_m_s": 5500.0,
            "fpa_deg": -14.0,
            "vehicle_params": {
                "CD": 1.5,
                "CL": 0.5,
                "mass": 1000.0,
                "S": 15.0
            },
        },
    }

    planet = PLANETS[planet_name]
    entry = entry_conditions[planet_name]


    source = "EMULATOR" if use_emulator else "REAL SENSE HAT"

    try:
        sense = get_hat(
            force_emulator=use_emulator,
            require_hardware=not use_emulator,
        )
        state = initialize(
            sense,
            altitude_m=entry["altitude_m"],
            velocity_m_s=entry["velocity_m_s"],
            flight_path_angle_rad=radians(entry["fpa_deg"]),
            mass=entry["vehicle_params"]["mass"],
        )
        estimator = LiftingBodyEKF(
                state = state
                dt = 0.01,
                planet = planet,
                vehicle_params = entry["vehicle_params"]
            )
    except (OSError, RuntimeError, ValueError) as error:
        print(f"Initialization failed: {error}")
        return 1


    print(
        f"{source} | planet = {planet_name.upper()} | "
        f"PHASE = {state.phase.value.upper()} | "
        f"Entry Interface = {state.altitude_m/1e3:.1f} km | "
        f"Entry Velcocity = {state.velocity_m_s/1e3:.1f} km/s | "
        f"Entry FPA = {entry['fpa_deg']:.1f} deg",
        flush=True,
        )
    print(
        f"Mass = {entry['vehicle_params']['mass']:.1f} kg | "
        f"CD = {entry['vehicle_params']['CD']:.2f} | "
        f"CL = {entry['vehicle_params']['CL']:.2f} | "
        f"Reference Area = {entry['vehicle_params']['S']:.1f} m²",
        flush=True,
        )

    window = None
    if show_window:
        from .status_window import StatusWindow

        try:
            window = StatusWindow()
        except (OSError, RuntimeError) as error:
            print(f"Cannot open status window: {error}")

    try:
        monitor(sense, state, source, window)
    except KeyboardInterrupt:
        print("\nFlight program stopped.")
    finally:
        if window is not None:
            window.close()

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
