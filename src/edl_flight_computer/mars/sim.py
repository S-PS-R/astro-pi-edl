"""Mars entry simulation, configuration, and desktop displays."""

from datetime import datetime
from math import degrees, radians
from pathlib import Path
from threading import Thread
from time import monotonic, sleep
from ..shared.downlink import landing_message, open_downlink
from ..shared.hardware import get_hat, is_simulated
from .led_grid import TrajectoryLEDGrid
from .model import MARS, PeakTracker, advance_state, calculate_state
from ..shared.sensors import SensorReader
from .state import Phase, initialize
from ..shared.telemetry import DropLog

PEAK_LABELS = {
    'peak_acceleration': 'Peak deceleration',
    'peak_heating': 'Peak heating',
    'peak_velocity': 'Peak velocity',
}


def run_mission(sense, state, vehicle_params, window, led_grid, output_path, downlink):
    """Read sensors, advance the flight, log data, and publish each state."""
    sensor_reader = SensorReader(sense)
    peak_tracker = PeakTracker()
    clock = window.sim_clock
    state.mission_started_at = monotonic()
    clock.reset(state.mission_started_at)
    _, playback_speed = clock.snapshot(state.mission_started_at)
    update_period_s = 1 / (window.updates_per_sim_second * playback_speed)
    next_update_time = state.mission_started_at + update_period_s
    previous_mission_time = 0.0

    # Initial display
    flight_vector = [state.altitude_m, state.velocity_m_s, state.flight_path_angle_rad]
    _, flight_values = calculate_state(flight_vector, state.planet, vehicle_params)
    peak_tracker.update_peaks(0.0, flight_values)
    led_grid.add_sample(0.0, state.altitude_m)
    window.publish(state, flight_values, peak_tracker.get_peaks())
    print(f"{state.planet['name'].upper()} | {state.phase.value.upper()} | "
          f"Altitude = {state.altitude_m:.1f} m | Speed = {state.velocity_m_s:.1f} m/s")
    print(f'Recording: {output_path}', flush=True)
    downlink.event('entry_interface', 0.0,
                   f'{state.planet["name"].upper()} entry interface: altitude {state.altitude_m:.0f} m, '
                   f'speed {state.velocity_m_s:.0f} m/s, FPA {degrees(state.flight_path_angle_rad):.2f} deg',
                   altitude_m=state.altitude_m, velocity_m_s=state.velocity_m_s,
                   flight_path_angle_rad=state.flight_path_angle_rad)
    landed = False

    with DropLog(output_path) as log:
        while not window.closed:
            real_time = monotonic()
            real_elapsed_s = real_time - state.mission_started_at
            mission_time_s, selected_speed = clock.snapshot(real_time)

            # Playback timing
            if selected_speed != playback_speed:
                playback_speed = selected_speed
                update_period_s = 1 / (window.updates_per_sim_second * playback_speed)
                next_update_time = real_time + update_period_s

            # Sensor polling
            sensor_readings = sensor_reader.read(real_elapsed_s)
            if real_time < next_update_time:
                sleep(0.005)
                continue

            simulation_step_s = mission_time_s - previous_mission_time
            previous_mission_time = mission_time_s
            next_update_time += update_period_s
            if next_update_time <= real_time:
                next_update_time = real_time + update_period_s

            # Flight calculation
            flight_vector = advance_state(flight_vector, simulation_step_s,
                                          state.planet, vehicle_params)
            state.altitude_m, state.velocity_m_s, state.flight_path_angle_rad = flight_vector
            state.elapsed_s = mission_time_s
            _, flight_values = calculate_state(flight_vector, state.planet, vehicle_params)
            landed = state.altitude_m <= 0

            # Ten-second landing warning
            downward_speed = -flight_values['vertical_velocity_m_s']
            if not landed and state.altitude_m <= 10.0 * downward_speed:
                if state.phase != Phase.LANDING:
                    downlink.event('terminal_descent', mission_time_s,
                                   f'Ten seconds to landing: altitude {state.altitude_m:.0f} m, '
                                   f'descent rate {downward_speed:.1f} m/s',
                                   altitude_m=state.altitude_m, descent_rate_m_s=downward_speed)
                state.phase = Phase.LANDING
                led_grid.start_landing_countdown()

            # Peak events
            new_peaks = peak_tracker.update_peaks(mission_time_s, flight_values)
            led_grid.add_sample(mission_time_s, state.altitude_m)
            for name, peak in new_peaks:
                led_grid.add_event(name, peak['time_s'], peak['altitude_m'])
                print(f"{name}: {peak['value']:.4f} {peak['units']} at "
                      f"{peak['time_s']:.2f} s (confirmed at {mission_time_s:.2f} s)", flush=True)
                downlink.event(name, mission_time_s,
                               f"{PEAK_LABELS.get(name, name)}: {peak['value']:.2f} {peak['units']} "
                               f"at T+{peak['time_s']:.1f} s, altitude {peak['altitude_m']:.0f} m",
                               peak=peak)
            if landed:
                state.phase = Phase.LANDED
                led_grid.mark_landed()
                print(f'Ground contact at {mission_time_s:.2f} s', flush=True)

            # CSV, ground station, and display update
            row = {
                'time_s': mission_time_s,
                'phase': state.phase.value,
                'flight_path_angle_rad': state.flight_path_angle_rad,
                **flight_values,
                **sensor_readings,
            }
            log.write(row)
            downlink.telemetry(mission_time_s, row, force=landed)
            if landed:
                downlink.event('landing', mission_time_s,
                               landing_message(state.planet['name'], mission_time_s,
                                               state.velocity_m_s, peak_tracker.get_peaks()),
                               velocity_m_s=state.velocity_m_s, peaks=peak_tracker.get_peaks(),
                               csv=output_path.name)
            window.publish(state, flight_values, peak_tracker.get_peaks())
            if landed:
                break
            sleep(0.005)

    # Stopped from the mission window before ground contact
    if not landed:
        downlink.event('stopped', state.elapsed_s,
                       f'Run stopped before landing at altitude {state.altitude_m:.0f} m')
    downlink.close()

    # Finished display
    window.finished = True


def main(settings=None):
    from .status_window import MissionWindow

    # Intial settings
    if settings is None:
        from ..run import MARS_SETTINGS
        settings = MARS_SETTINGS
    planet_name = 'mars'

    # Run settings
    updates_per_sim_second = settings['updates_per_sim_second']
    time_scale = settings['time_scale']
    initial_conditions = settings['initial_conditions']
    planet = MARS

    state = initialize(
        altitude_m=initial_conditions['altitude_m'],
        velocity_m_s=initial_conditions['velocity_m_s'],
        flight_path_angle_rad=radians(initial_conditions['fpa_deg']),
        mass_kg=initial_conditions['vehicle_params']['mass'],
        planet=planet,
        phase=initial_conditions['phase'],
    )
    vehicle_params = initial_conditions['vehicle_params']

    # Hardware and windows
    sense = get_hat()
    sense.clear()
    output_path = Path(__file__).resolve().parents[3] / 'logs' / ('run_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '.csv')
    led_grid = TrajectoryLEDGrid(sense, state.altitude_m)
    window = MissionWindow(planet_name, output_path.with_suffix('.png'),
                           updates_per_sim_second, time_scale)
    if is_simulated():
        window.open_emulator(sense)

    downlink = open_downlink(settings.get('ground_station'), mission=planet_name,
                             run_id=output_path.stem,
                             telemetry_hz=settings.get('telemetry_hz', 10.0))
    mission_thread = Thread(target=run_mission,
                            args=(sense, state, vehicle_params, window, led_grid,
                                  output_path, downlink),
                            name='EDL mission')
    led_grid.start()
    mission_thread.start()
    try:
        window.root.mainloop()
    finally:
        # Mission cleanup
        window.close()
        mission_thread.join()
        try:
            window.save_plots()
            window.destroy()
        finally:
            led_grid.stop()
            sense.clear()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
