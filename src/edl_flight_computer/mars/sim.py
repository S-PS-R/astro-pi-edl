"""Mars entry simulation, configuration, and desktop displays."""

from math import degrees, radians
from threading import Thread
from time import monotonic, sleep
from ..shared.downlink import open_downlink
from ..shared.faults import FaultMonitor
from ..shared.hardware import get_hat, is_simulated
from .led_grid import TrajectoryLEDGrid
from .model import DeploymentWindowMissed, MARS, MarsTrajectory, PeakTracker
from ..shared.sensors import SensorReader
from .state import Phase, initialize
from ..shared.telemetry import DropLog, new_run_csv

PEAK_LABELS = {
    'peak_acceleration': 'Entry peak deceleration',
    'peak_heating': 'Peak heating',
}
SENSOR_UNITS = {
    'accel': 'g', 'gyro': 'rad/s', 'mag': 'µT', 'pressure': 'hPa',
    'temperature': '°C', 'humidity': '%RH',
}


def publish_event(downlink, name, time_s, altitude_m, reading, message, *, observed_time_s=None, **data):
    display_time_s = time_s if observed_time_s is None else observed_time_s
    line = f'[{display_time_s:.2f} s] [{altitude_m:.0f} m] EVENT {name}: {reading}'
    downlink.event(name, time_s, message, console_line=line, altitude_m=altitude_m, **data)

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
        opening_load = values['opening_load_n']
        opening_factor = values['opening_factor']
        reading = f'Mach {mach:.2f}, estimated opening load {opening_load / 1000:.1f} kN'
        publish_event(downlink, event.name, event.time_s, altitude, reading,
                       f'Parachute deployed at {altitude:.0f} m, Mach {mach:.2f}, '
                       f'{speed:.1f} m/s and {pressure:.0f} Pa dynamic pressure. '
                       f'Estimated canopy opening load {opening_load / 1000:.1f} kN '
                       f'(opening factor {opening_factor:g}). '
                       f'Total drag rises from {before_drag / 1000:.1f} to {total_drag / 1000:.1f} kN. '
                       f'Modeled along-path acceleration at deployment is {opening_acceleration:.1f} m/s^2.',
                       velocity_m_s=speed, mach=mach,
                       dynamic_pressure_pa=pressure, vehicle_drag_n=values['vehicle_drag_n'],
                       parachute_drag_n=values['parachute_drag_n'], total_drag_n=total_drag,
                       opening_factor=opening_factor, opening_load_n=opening_load,
                       acceleration_m_s2=opening_acceleration)
        led_grid.add_event(event.name, event.time_s, altitude)
    elif event.name == 'near_terminal_descent':
        equilibrium = values['equilibrium_speed_m_s']
        led_grid.add_event(event.name, event.time_s, altitude)
        publish_event(downlink, event.name, event.time_s, altitude, f'{speed:.2f} m/s',
                       f'Near-terminal descent at {altitude:.0f} m, '
                       f'{speed:.1f} m/s near the {equilibrium:.1f} m/s equilibrium speed',
                       velocity_m_s=speed, equilibrium_speed_m_s=equilibrium,
                       flight_path_angle_rad=values['flight_path_angle_rad'],
                       vertical_acceleration_m_s2=values['vertical_acceleration_m_s2'])
    elif event.name == 'landing_warning':
        downward_speed = -values['vertical_velocity_m_s']
        led_grid.start_landing_countdown()
        estimate = values['estimated_time_to_contact_s']
        publish_event(downlink, event.name, event.time_s, altitude, f'{estimate:.2f} s',
                       f'Estimated {estimate:.2f} seconds to touchdown under braking. '
                       f'Descent rate {downward_speed:.1f} m/s at {altitude:.0f} m altitude.',
                       descent_rate_m_s=downward_speed, estimated_time_to_contact_s=estimate)
    elif event.phase is not None:
        led_grid.add_event(event.name, event.time_s, altitude)
        if event.phase is Phase.POWERED_DESCENT:
            reading = f'{values["mass_kg"]:.1f} kg, {speed:.1f} m/s airspeed'
        else:
            reading = f'{event.before_values["mass_kg"] - values["mass_kg"]:.1f} kg'
        publish_event(downlink, event.name, event.time_s, altitude, reading,
                       f'{event.phase.value.capitalize()} at {altitude:.0f} m and {speed:.1f} m/s. '
                       f'Mass {values["mass_kg"]:.1f} kg, propellant {values["propellant_kg"]:.1f} kg.',
                       velocity_m_s=speed, mass_kg=values['mass_kg'],
                       propellant_kg=values['propellant_kg'], thrust_n=values['thrust_n'],
                       horizontal_speed_m_s=values['horizontal_speed_m_s'])

# Function to check for peaks and publish an downlink event
def publish_peaks(peak_tracker, time_s, values, led_grid, downlink, *, track_entry_deceleration=True):
    for name, peak in peak_tracker.update_peaks(
            time_s, values, track_entry_deceleration=track_entry_deceleration):
        led_grid.add_event(name, peak['time_s'], peak['altitude_m'])
        velocity = peak['velocity_m_s']
        reading = f"{peak['value']:.2f} {peak['units']} at {velocity:.2f} m/s"
        publish_event(downlink, name, time_s, peak['altitude_m'], reading,
                      f"{PEAK_LABELS.get(name, name)}: {reading} "
                      f"at T+{peak['time_s']:.1f} s, altitude {peak['altitude_m']:.0f} m",
                      observed_time_s=peak['time_s'], velocity_m_s=velocity, peak=peak)

# Main run
def run_mission(sense, state, vehicle_params, window, led_grid, output_path, downlink,
                fault_monitor=None, *, parachute=None, entry_site=None, powered_landing=None,
                entry_guidance=None, wind=None):
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
    trajectory = MarsTrajectory([state.altitude_m, state.velocity_m_s, state.flight_path_angle_rad],
                                state.planet, vehicle_params, parachute, state.phase,
                                entry_site=entry_site, powered_landing=powered_landing,
                                entry_guidance=entry_guidance, wind=wind)
    flight_values = trajectory.values
    state.velocity_m_s = flight_values['velocity_m_s']
    state.flight_path_angle_rad = flight_values['flight_path_angle_rad']
    peak_tracker.update_peaks(0.0, flight_values, track_entry_deceleration=trajectory.phase is Phase.ENTRY)
    led_grid.add_sample(0.0, state.altitude_m)
    led_grid.add_event('entry_interface', 0.0, state.altitude_m)
    window.publish(state, flight_values)
    print(f'Recording: {output_path}', flush=True)
    if trajectory.wind is not None:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.with_name('wind_seed.txt').write_text(f'{trajectory.wind.seed}\n', encoding='utf-8')
        print(f'Mars wind seed: {trajectory.wind.seed}', flush=True)

    # Initial downlink event
    publish_event(downlink, 'entry_interface', 0.0, state.altitude_m, f'{state.velocity_m_s:.2f} m/s',
                   f'{state.planet["name"].upper()} entry interface: altitude {state.altitude_m:.0f} m, '
                   f'speed {state.velocity_m_s:.0f} m/s, FPA {degrees(state.flight_path_angle_rad):.2f} deg',
                   velocity_m_s=state.velocity_m_s,
                   flight_path_angle_rad=state.flight_path_angle_rad)
    contact = False

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
            for fault in fault_monitor.update(real_elapsed_s, sensor_readings):
                channel = fault['channel']
                unit = SENSOR_UNITS[channel.split('_', 1)[0]]
                publish_event(downlink, fault['event'], mission_time_s, state.altitude_m,
                              f'{fault["value"]:.2f} {unit} ({channel})', fault['message'],
                              **{key: fault[key] for key in ('channel', 'value', 'low', 'high')})
            if real_time < next_update_time:
                sleep(0.005)
                continue

            next_update_time += update_period_s
            if next_update_time <= real_time:
                next_update_time = real_time + update_period_s

            # Flight calculation
            try:
                events = trajectory.advance(mission_time_s - trajectory.time_s)
            except DeploymentWindowMissed as error:
                state.elapsed_s = trajectory.time_s
                state.altitude_m = trajectory.values['altitude_m']
                publish_event(downlink, 'deployment_failed', state.elapsed_s, state.altitude_m,
                              'window missed', str(error), mach=trajectory.values['mach'],
                              dynamic_pressure_pa=trajectory.values['dynamic_pressure_pa'])
                log.write({'time_s': state.elapsed_s, 'phase': 'deployment failed',
                           **trajectory.values, **sensor_readings})
                led_grid.mark_impact()
                led_grid.render()
                break
            flight_values = trajectory.values
            state.altitude_m = flight_values['altitude_m']
            state.velocity_m_s = flight_values['velocity_m_s']
            state.flight_path_angle_rad = flight_values['flight_path_angle_rad']
            state.elapsed_s = mission_time_s = trajectory.time_s
            # Both powered flight modes share one phase in the mission readouts.
            state.phase = (Phase.POWERED_DESCENT if trajectory.phase is Phase.VERTICAL_DESCENT
                           else trajectory.phase)
            landed = state.phase is Phase.LANDED
            contact = landed or state.phase is Phase.IMPACT

            for event in events:
                if event.name == 'parachute_deploy':
                    publish_peaks(peak_tracker, event.time_s, event.before_values, led_grid, downlink)
                    window.plot_deployment(event.time_s, event.before_values, event.values)
                    if event.time_s == mission_time_s:
                        flight_values = dict(flight_values, opening_factor=event.values['opening_factor'],
                                             opening_load_n=event.values['opening_load_n'])
                if event.phase is not None and event.time_s < mission_time_s:
                    log.write({'time_s': event.time_s, 'phase': event.phase.value, **event.values, **sensor_readings})
                publish_flight_event(event, led_grid, downlink)
            led_grid.add_sample(mission_time_s, state.altitude_m)
            publish_peaks(peak_tracker, mission_time_s, flight_values, led_grid, downlink,
                          track_entry_deceleration=state.phase is Phase.ENTRY)
            if landed:
                led_grid.mark_landed()
            elif contact:
                led_grid.mark_impact()
            if contact:
                led_grid.render()

            # CSV, ground station, and display update
            row = {
                'time_s': mission_time_s,
                'phase': state.phase.value,
                **flight_values,
                **sensor_readings,
            }
            log.write(row)
            downlink.telemetry(mission_time_s, {**row, 'faults': sorted(fault_monitor.active)}, force=contact)
            if contact:
                contact_speed = flight_values.get('ground_speed_m_s', state.velocity_m_s)
                descent = {key: flight_values[key] for key in ('vertical_velocity_m_s', 'horizontal_speed_m_s',
                                                                'propellant_kg', 'mass_kg') if key in flight_values}
                if trajectory.landing is not None and trajectory.landing.target is not None:
                    error = flight_values['landing_target_error_m']
                    target = trajectory.landing.target
                    within_zone = error <= target['tolerance_m']
                    descent.update(landing_target_error_m=error, landing_target_met=landed and within_zone)
                    publish_event(downlink, 'landing_target_hit' if landed and within_zone else 'landing_target_miss',
                                  mission_time_s, state.altitude_m, f'{error:.2f} m',
                                  f'Contact {error:.2f} m from the landing target',
                                  latitude_deg=flight_values['latitude_deg'],
                                  longitude_deg=flight_values['longitude_deg'], tolerance_m=target['tolerance_m'])
                message = (f'{state.planet["name"].upper()} touchdown at T+{mission_time_s:.1f} s, '
                           f'{contact_speed:.1f} m/s.' if landed
                           else f'Impact at {contact_speed:.1f} m/s outside soft landing limits')
                publish_event(downlink, 'landing' if landed else 'impact', mission_time_s,
                              state.altitude_m, f'{contact_speed:.2f} m/s', message,
                              velocity_m_s=contact_speed, peaks=peak_tracker.get_peaks(),
                              csv=output_path.name, **descent)
            window.publish(state, flight_values)
            if contact:
                break
            sleep(0.005)

    # Stopped from the mission window before ground contact
    if not contact:
        publish_event(downlink, 'stopped', state.elapsed_s, state.altitude_m, f'{state.velocity_m_s:.2f} m/s',
                       f'Run stopped before landing at altitude {state.altitude_m:.0f} m')
    downlink.close()

    # Finished display
    window.finished = True


def configured_entry_guidance(settings):
    """Resolve the entry waypoint from the touchdown target and its offset."""
    guidance = dict(settings.get('entry_guidance') or {})
    if not guidance.pop('enabled', False):
        return None
    offset = guidance.pop('waypoint_east_offset_deg', None)
    if offset is not None:
        target = (settings.get('retro_prop') or {}).get('landing_target')
        if target is None:
            raise ValueError('An entry waypoint offset requires retro_prop.landing_target')
        guidance.update(latitude_deg=target['latitude_deg'],
                        longitude_deg=(target['longitude_deg'] + offset) % 360)
    return guidance


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
    guidance = configured_entry_guidance(settings)
    planet = MARS

    state = initialize(altitude_m=initial_conditions['altitude_m'],
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
    powered_landing = settings.get('retro_prop') or {}
    window = MissionWindow(planet_name, output_path.with_suffix('.png'),
                           updates_per_sim_second, time_scale,
                           landing_target=powered_landing.get('landing_target'))
    if is_simulated():
        window.open_emulator(sense)

    downlink = open_downlink(settings.get('ground_station'), mission=planet_name,
                            run_id=output_path.stem,
                            telemetry_hz=settings.get('telemetry_hz', 10.0))

    wind = dict(settings.get('wind') or {})
    wind = wind if wind.pop('enabled', False) else None

    mission_thread = Thread(target=run_mission,
                            args=(sense, state, vehicle_params, window, led_grid,
                                  output_path, downlink,
                                  FaultMonitor(settings.get('fault_limits'))),
                            kwargs={'parachute': settings.get('parachute'),
                                    'entry_site': initial_conditions.get('entry_site'),
                                    'powered_landing': settings.get('retro_prop'),
                                    'entry_guidance': guidance, 'wind': wind},
                            name='EDL mission')
    led_grid.start()
    mission_thread.start()
    try:
        window.root.mainloop()
    finally:
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
