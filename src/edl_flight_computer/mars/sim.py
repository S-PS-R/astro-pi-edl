"""Mars entry simulation, configuration, and desktop displays."""

from math import degrees, radians
from threading import Thread
from time import monotonic, sleep
from ..shared.downlink import landing_message, open_downlink
from ..shared.faults import FaultMonitor, report as report_faults
from ..shared.hardware import get_hat, is_simulated
from .led_grid import TrajectoryLEDGrid
from .model import MARS, MarsTrajectory, PeakTracker
from ..shared.sensors import SensorReader
from .state import Phase, initialize
from ..shared.telemetry import DropLog, new_run_csv

PEAK_LABELS = {
    'peak_acceleration': 'Entry peak deceleration',
    'peak_heating': 'Peak heating',
    'peak_velocity': 'Peak velocity',
}

# Function for pushing events to LED Grid and to downlink
def publish_flight_event(event, led_grid, downlink):
    values = event.values
    altitude = values['altitude_m']
    speed = values['velocity_m_s']
    if event.name == 'parachute_deploy':
        mach = values['mach']
        pressure = values['dynamic_pressure_pa']
        before_drag = event.before_values['total_drag_n']
        total_drag = values['total_drag_n']
        opening_acceleration = values['acceleration_m_s2']
        downlink.event(event.name, event.time_s,
                       f'Parachute deployed at {altitude:.0f} m, Mach {mach:.2f}, '
                       f'{speed:.1f} m/s and {pressure:.0f} Pa dynamic pressure. '
                       f'Total drag rises from {before_drag / 1000:.1f} to {total_drag / 1000:.1f} kN. '
                       f'Along-path acceleration at opening is {opening_acceleration:.1f} m/s^2.',
                       altitude_m=altitude, velocity_m_s=speed, mach=mach,
                       dynamic_pressure_pa=pressure, vehicle_drag_n=values['vehicle_drag_n'],
                       parachute_drag_n=values['parachute_drag_n'], total_drag_n=total_drag,
                       acceleration_m_s2=opening_acceleration)
        led_grid.add_event(event.name, event.time_s, altitude)
    elif event.name == 'terminal_descent':
        equilibrium = values['equilibrium_speed_m_s']
        led_grid.add_event(event.name, event.time_s, altitude)
        downlink.event(event.name, event.time_s,
                       f'Terminal descent at {altitude:.0f} m, '
                       f'{speed:.1f} m/s near the {equilibrium:.1f} m/s equilibrium speed',
                       altitude_m=altitude, velocity_m_s=speed, equilibrium_speed_m_s=equilibrium,
                       flight_path_angle_rad=values['flight_path_angle_rad'],
                       vertical_acceleration_m_s2=values['vertical_acceleration_m_s2'])
    elif event.name == 'landing_warning':
        downward_speed = -values['vertical_velocity_m_s']
        led_grid.start_landing_countdown()
        downlink.event(event.name, event.time_s,
                       f'Estimated ten seconds to landing at {altitude:.0f} m '
                       f'and {downward_speed:.1f} m/s descent rate',
                       altitude_m=altitude, descent_rate_m_s=downward_speed,
                       estimated_time_to_contact_s=altitude / downward_speed)

# Function to check for peaks and publish an downlink event
def publish_peaks(peak_tracker, time_s, values, led_grid, downlink, *, track_entry_deceleration=True):
    for name, peak in peak_tracker.update_peaks(
            time_s, values, track_entry_deceleration=track_entry_deceleration):
        led_grid.add_event(name, peak['time_s'], peak['altitude_m'])
        downlink.event(name, time_s,
                       f"{PEAK_LABELS.get(name, name)}: {peak['value']:.2f} {peak['units']} "
                       f"at T+{peak['time_s']:.1f} s, altitude {peak['altitude_m']:.0f} m",
                       peak=peak)

# Main run
def run_mission(sense, state, vehicle_params, window, led_grid, output_path, downlink,
                fault_monitor=None, *, parachute=None):
    """Read sensors, advance the flight, log data, and publish each state."""
    sensor_reader = SensorReader(sense)
    fault_monitor = fault_monitor or FaultMonitor({})
    peak_tracker = PeakTracker()
    clock = window.sim_clock
    state.mission_started_at = monotonic()
    clock.reset(state.mission_started_at)
    _, playback_speed = clock.snapshot(state.mission_started_at)
    update_period_s = 1 / (window.updates_per_sim_second * playback_speed)
    next_update_time = state.mission_started_at + update_period_s

    # Initial display
    trajectory = MarsTrajectory([state.altitude_m, state.velocity_m_s, state.flight_path_angle_rad], state.planet, vehicle_params, parachute, state.phase)
    flight_values = trajectory.values
    peak_tracker.update_peaks(0.0, flight_values, track_entry_deceleration=not trajectory.canopy_deployed)
    led_grid.add_sample(0.0, state.altitude_m)
    window.publish(state, flight_values)
    print(f"{state.planet['name'].upper()} | {state.phase.value.upper()} | "f"Altitude = {state.altitude_m:.1f} m | Speed = {state.velocity_m_s:.1f} m/s")
    print(f'Recording: {output_path}', flush=True)

    # Initial downlink event
    downlink.event('entry_interface', 0.0,
                   f'{state.planet["name"].upper()} entry interface: altitude {state.altitude_m:.0f} m, '
                   f'speed {state.velocity_m_s:.0f} m/s, FPA {degrees(state.flight_path_angle_rad):.2f} deg',
                   altitude_m=state.altitude_m, velocity_m_s=state.velocity_m_s,
                   flight_path_angle_rad=state.flight_path_angle_rad)
    landed = False

    # Main loop: read sensors, advance flight, log, and downlink
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

            # Sensor faults: checked in real time at every poll
            report_faults(fault_monitor.update(real_elapsed_s, sensor_readings), downlink, mission_time_s)
            if real_time < next_update_time:
                sleep(0.005)
                continue

            next_update_time += update_period_s
            if next_update_time <= real_time:
                next_update_time = real_time + update_period_s

            # Flight calculation
            events = trajectory.advance(mission_time_s - trajectory.time_s)
            state.altitude_m, state.velocity_m_s, state.flight_path_angle_rad = trajectory.vector[:3]
            state.elapsed_s = mission_time_s = trajectory.time_s
            state.phase = trajectory.phase
            flight_values = trajectory.values
            landed = state.phase is Phase.LANDED

            for event in events:
                if event.name == 'parachute_deploy':
                    publish_peaks(peak_tracker, event.time_s, event.before_values, led_grid, downlink)
                    window.plot_deployment(event.before_values, event.values)
                    # Preserve the instantaneous jump even between regular CSV samples.
                    if event.time_s < mission_time_s:
                        log.write({'time_s': event.time_s, 'phase': Phase.PARACHUTE_DESCENT.value, **event.values, **sensor_readings})
                publish_flight_event(event, led_grid, downlink)
            led_grid.add_sample(mission_time_s, state.altitude_m)
            publish_peaks(peak_tracker, mission_time_s, flight_values, led_grid, downlink,
                          track_entry_deceleration=not trajectory.canopy_deployed)
            if landed:
                led_grid.mark_landed()
                led_grid.render()
                print(f'Ground contact at {mission_time_s:.2f} s', flush=True)

            # CSV, ground station, and display update
            row = {
                'time_s': mission_time_s,
                'phase': state.phase.value,
                **flight_values,
                **sensor_readings,
            }
            log.write(row)
            downlink.telemetry(mission_time_s, {**row, 'faults': sorted(fault_monitor.active)}, force=landed)
            if landed:
                downlink.event('landing', mission_time_s,
                               landing_message(state.planet['name'], mission_time_s,
                                               state.velocity_m_s, peak_tracker.get_peaks()),
                                               velocity_m_s=state.velocity_m_s, peaks=peak_tracker.get_peaks(),
                                               csv=output_path.name)
            window.publish(state, flight_values)
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
    output_path = new_run_csv('run_')
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
                                  output_path, downlink,
                                  FaultMonitor(settings.get('fault_limits'))),
                            kwargs={'parachute': settings.get('parachute')},
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
            window.destroy()
        finally:
            led_grid.stop()
            sense.clear()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
