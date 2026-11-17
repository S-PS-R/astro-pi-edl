"""Configure a mission, run the flight loop, then save and close the displays."""
# Start with: EDL_SMTP_USER='adhit.sirip@gmail.com' EDL_SMTP_PASSWORD='qgdh gaoo lkjf gstv' EDL_EMAIL_TO='<recipient email, just one or multiple comma separated>' python -m edl_flight_computer.run

from datetime import datetime
from math import radians
from pathlib import Path
from threading import Thread
from time import monotonic, sleep
from .hardware import get_hat, is_simulated
from .led_grid import TrajectoryLEDGrid
from .model import PLANETS, PeakTracker, advance_state, calculate_state
from .notify import LandingNotifier, landing_message
from .sensors import SensorReader
from .state import Phase, initialize
from .status_window import MissionWindow
from .telemetry import DropLog


def run_mission(sense, state, vehicle_params, window, led_grid, output_path, notifier):
    """Read sensors, advance the flight, record events, and publish each state."""
    sensor_reader = SensorReader(sense)
    peak_tracker = PeakTracker()
    clock = window.sim_clock
    state.mission_started_at = monotonic()
    clock.reset(state.mission_started_at)
    _, playback_speed = clock.snapshot(state.mission_started_at)
    update_period_s = 1 / (window.updates_per_sim_second * playback_speed)
    next_update_time = state.mission_started_at + update_period_s
    previous_mission_time = 0.0

    # Start both displays at the configured entry interface or drop height.
    flight_vector = [state.altitude_m, state.velocity_m_s, state.flight_path_angle_rad]
    _, flight_values = calculate_state(flight_vector, state.planet, vehicle_params)
    peak_tracker.update_peaks(0.0, flight_values)
    led_grid.add_sample(0.0, state.altitude_m)
    window.publish(state, flight_values, peak_tracker.get_peaks())
    print(f"{state.planet['name'].upper()} | {state.phase.value.upper()} | "
          f"Altitude = {state.altitude_m:.1f} m | Speed = {state.velocity_m_s:.1f} m/s")
    print(f'Recording: {output_path}', flush=True)

    with DropLog(output_path) as log:
        while not window.closed:
            real_time = monotonic()
            real_elapsed_s = real_time - state.mission_started_at
            mission_time_s, selected_speed = clock.snapshot(real_time)

            # A speed change changes the schedule, without resetting mission time.
            if selected_speed != playback_speed:
                playback_speed = selected_speed
                update_period_s = 1 / (window.updates_per_sim_second * playback_speed)
                next_update_time = real_time + update_period_s

            # Sensors keep their real-time polling rates between flight updates.
            sensor_readings = sensor_reader.read(real_elapsed_s)
            if real_time < next_update_time:
                sleep(0.005)
                continue

            simulation_step_s = mission_time_s - previous_mission_time
            previous_mission_time = mission_time_s
            next_update_time += update_period_s
            if next_update_time <= real_time:
                next_update_time = real_time + update_period_s

            # RK4 divides this interval into small integration steps.
            flight_vector = advance_state(flight_vector, simulation_step_s,
                                          state.planet, vehicle_params)
            state.altitude_m, state.velocity_m_s, state.flight_path_angle_rad = flight_vector
            state.elapsed_s = mission_time_s
            _, flight_values = calculate_state(flight_vector, state.planet, vehicle_params)
            landed = state.altitude_m <= 0

            # Confirm peaks from the measurements we have seen so far.
            new_peaks = peak_tracker.update_peaks(mission_time_s, flight_values)
            event_names = []
            led_grid.add_sample(mission_time_s, state.altitude_m)
            for name, peak in new_peaks:
                led_grid.add_event(name, peak['time_s'], peak['altitude_m'])
                event_names.append(name)
                print(f"{name}: {peak['value']:.4f} {peak['units']} at "
                      f"{peak['time_s']:.2f} s (confirmed at {mission_time_s:.2f} s)", flush=True)
            if landed:
                state.phase = Phase.LANDING
                led_grid.mark_landed()
                event_names.append('landing')
                print(f'Ground contact at {mission_time_s:.2f} s', flush=True)
                notifier.send(f"EDL landing: {state.planet['name'].upper()}",
                              landing_message(state.planet['name'], mission_time_s,
                                              state.velocity_m_s, peak_tracker.get_peaks()))

            # Each row pairs the flight state with the latest real sensor readings.
            log.write({
                'time_s': mission_time_s,
                'wall_time_s': real_elapsed_s,
                'time_scale': playback_speed,
                'phase': state.phase.value,
                'flight_path_angle_rad': state.flight_path_angle_rad,
                **flight_values,
                **sensor_readings,
                **{group + '_sample_s': sample_time
                   for group, sample_time in sensor_reader.sample_times_s.items()},
                'events': ';'.join(event_names),
            })
            window.publish(state, flight_values, peak_tracker.get_peaks())
            if landed:
                break
            sleep(0.005)

    # The GUI stays open after landing so the results can be reviewed.
    window.finished = True


def main():
    # These are the two configured scenarios. Select the one to run here.
    planet_name = 'mars'

    # Three state updates per simulated second, at the selected playback speed.
    updates_per_sim_second = 3.0
    time_scale = 10.0 if planet_name == 'mars' else 1.0

    planet_initial_conditions = {
        'mars': {
            'phase': Phase.ENTRY,
            'altitude_m': 120000.0,
            'velocity_m_s': 3000.0,
            'fpa_deg': -1.0,
            'vehicle_params': {'CD': 1.5, 'CL': 0.5, 'mass': 8000.0, 'S': 15.0,
                               'Rn': 1, 'target_fpa_deg': -5.0},
        },
        'earth': {
            'phase': Phase.LANDING,
            'altitude_m': 4.5,
            'velocity_m_s': 0.0,
            'fpa_deg': -90.0,
            'vehicle_params': {'CD': 0.8, 'CL': 0.5, 'mass': 0.1, 'S': 0.1, 'Rn': 1},
        },
    }
    initial_conditions = planet_initial_conditions[planet_name]
    planet = PLANETS[planet_name]

    state = initialize(
        altitude_m=initial_conditions['altitude_m'],
        velocity_m_s=initial_conditions['velocity_m_s'],
        flight_path_angle_rad=radians(initial_conditions['fpa_deg']),
        mass_kg=initial_conditions['vehicle_params']['mass'],
        planet=planet,
        phase=initial_conditions['phase'],
    )
    vehicle_params = initial_conditions['vehicle_params']

    # Set up the shared hardware and the always-present mission window.
    sense = get_hat()
    sense.clear()
    output_path = Path('logs') / ('run_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '.csv')
    led_grid = TrajectoryLEDGrid(sense, state.altitude_m)
    window = MissionWindow(planet_name, led_grid, output_path.with_suffix('.png'),
                           updates_per_sim_second, time_scale)
    if is_simulated():
        window.open_emulator(sense)

    notifier = LandingNotifier()
    if not notifier.configured:
        print('Landing email disabled: set EDL_SMTP_USER and EDL_SMTP_PASSWORD to enable it.')
    mission_thread = Thread(target=run_mission,
                            args=(sense, state, vehicle_params, window, led_grid,
                                  output_path, notifier),
                            name='EDL mission')
    led_grid.start()
    mission_thread.start()
    try:
        window.root.mainloop()
    finally:
        # Stop calculations before asking the plot process to save its last points.
        window.close()
        mission_thread.join()
        try:
            window.save_plots()
            window.destroy()
        finally:
            led_grid.stop()
            sense.clear()
            notifier.wait()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
