import csv
import io
from contextlib import redirect_stdout
from math import cos, pi, radians, sin
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest import TestCase
from unittest.mock import patch

import numpy as np

from edl_flight_computer.earth import drop
from edl_flight_computer.mars import sim
from edl_flight_computer.mars.entry_guidance import WindProfile
from edl_flight_computer.mars.led_grid import TrajectoryLEDGrid
from edl_flight_computer.mars.model import (
    DeploymentWindowMissed, MARS, MarsTrajectory, advance_state, calculate_state, parachute_drag_coefficient,
    sphere_cone_areas,
)
from edl_flight_computer.mars.state import SimulationClock, initialize
from edl_flight_computer.mars.status_window import MissionWindow
from edl_flight_computer.run import MARS_AEROSHELL, MARS_SETTINGS, NASA_DRAG_CURVE_DGB
from edl_flight_computer.shared import downlink as downlink_module
from edl_flight_computer.shared.downlink import Downlink
from edl_flight_computer.shared.ground_station import GroundStation
from edl_flight_computer.shared.phases import Phase
from sense_hat_emulator.emulator import SenseEmu


INITIAL = MARS_SETTINGS['initial_conditions']
VEHICLE = INITIAL['vehicle_params']
PARACHUTE = MARS_SETTINGS['parachute']
ENTRY_VECTOR = [INITIAL['altitude_m'], INITIAL['velocity_m_s'], radians(INITIAL['fpa_deg'])]
EQUATORIAL_ENTRY = dict(latitude_deg=0.0, longitude_deg=0.0, heading_deg=90.0)
# Fixed canopy-only benchmark for the known equilibrium and playback checks.
REFERENCE_CASE = (dict(MARS, mu=4.2828e13), dict(VEHICLE, mass=3308.98, S=15.0, Rn=4.5,
                                            CD=1.5, CL=0.5),
                  dict(PARACHUTE, diameter_m=21.35, drag_curve=NASA_DRAG_CURVE_DGB,
                       deploy_altitude_m=7092.0, terminal_speed_tolerance=.05,
                       terminal_vertical_acceleration_tolerance_m_s2=.10))
REFERENCE_ENTRY = (120000.0, 3000.0, radians(-1))


def untargeted_landing():
    return {key: value for key, value in MARS_SETTINGS['retro_prop'].items() if key != 'landing_target'}


class FlightModelTests(TestCase):
    def test_deployment_window_requires_pressure_crossing_and_altitude(self):
        envelope = {key: value for key, value in PARACHUTE.items()
                    if key not in ('deploy_altitude_m', 'deploy_mach')}
        envelope.update(max_deploy_dynamic_pressure_pa=600., min_deploy_altitude_m=6000.)
        flight = MarsTrajectory([7000., 300., radians(-90)], MARS, VEHICLE, envelope)
        self.assertEqual(flight.advance(.02), [])
        flight.deployment_armed = True
        event, = flight.advance(.02)
        self.assertEqual(event.name, 'parachute_deploy')
        self.assertLessEqual(event.values['dynamic_pressure_pa'], 600.)
        self.assertGreaterEqual(event.values['altitude_m'], 6000.)
        missed = MarsTrajectory([5900., 500., radians(-90)], MARS, VEHICLE, envelope)
        with self.assertRaises(DeploymentWindowMissed):
            missed.advance(.02)

    def test_sphere_cone_geometry_separates_aero_and_tps_areas(self):
        reference, wetted = sphere_cone_areas(**MARS_AEROSHELL)
        self.assertAlmostEqual(reference, pi * 2.25**2)
        self.assertAlmostEqual(wetted, 17.83345, places=4)
        self.assertEqual(VEHICLE['S'], reference)
        self.assertEqual(VEHICLE['Rn'], 1.125)
        shield = MARS_SETTINGS['retro_prop']['heatshield']
        self.assertAlmostEqual(shield['mass_kg'] - 274.90256, wetted * 256 * .0312)

    def test_wind_seed_replays_regardless_of_sample_order(self):
        settings = {key: value for key, value in MARS_SETTINGS['wind'].items() if key != 'enabled'}
        first = WindProfile(**dict(settings, seed=42))
        replay = WindProfile(**dict(settings, seed=42))
        later = first.components(90.0, 2000.0)
        self.assertEqual(first.components(25.0, 9000.0), replay.components(25.0, 9000.0))
        self.assertEqual(later, replay.components(90.0, 2000.0))

    def test_live_bank_steering_reduces_chute_deployment_cross_track_error(self):
        guidance = sim.configured_entry_guidance(MARS_SETTINGS)
        guidance.pop('range_bank_schedule')
        guidance.update(latitude_deg=0.1, longitude_deg=24.0, max_bank_deg=10.0)
        wind = {key: value for key, value in MARS_SETTINGS['wind'].items() if key != 'enabled'}
        wind['seed'] = 11
        flights = [MarsTrajectory(REFERENCE_ENTRY, *REFERENCE_CASE, entry_site=EQUATORIAL_ENTRY,
                                  entry_guidance=controller, wind=wind)
                   for controller in (None, guidance)]
        normal = np.cross(flights[1].spatial.up, flights[1].guidance.target)
        normal /= np.linalg.norm(normal)
        errors = []
        for flight in flights:
            events = flight.advance(900)
            deploy, = [event for event in events if event.name == 'parachute_deploy']
            lat, lon = map(radians, (deploy.values['latitude_deg'], deploy.values['longitude_deg']))
            up = (cos(lat) * cos(lon), cos(lat) * sin(lon), sin(lat))
            errors.append(abs(MARS['radius'] * np.arcsin(np.dot(up, normal))))
            self.assertIsNone(deploy.values['bank_angle_deg'])
            self.assertIs(flight.phase, Phase.LANDED)
        self.assertLess(errors[1], .65 * errors[0])

    def test_propellant_exhaustion_stops_thrust_and_reports_impact(self):
        settings = untargeted_landing()
        settings = dict(settings, heatshield=dict(settings['heatshield'], delay_after_chute_s=0),
                        propulsion=dict(settings['propulsion'], propellant_kg=.05))
        flight = MarsTrajectory([2000, 265, radians(-19)], MARS, VEHICLE, PARACHUTE,
                                Phase.PARACHUTE_DESCENT, entry_site=EQUATORIAL_ENTRY,
                                powered_landing=settings)
        flight.advance(60)
        self.assertEqual(flight.phase, Phase.IMPACT)
        self.assertEqual(flight.values['propellant_kg'], 0)
        self.assertEqual(flight.values['thrust_n'], 0)
        dropped_mass = settings['heatshield']['mass_kg'] + settings['backshell']['mass_kg']
        self.assertAlmostEqual(flight.values['mass_kg'], VEHICLE['mass'] - dropped_mass - .05)

    def test_spatial_rotation_and_banked_crossrange(self):
        def trajectory(heading, bank):
            return MarsTrajectory([30000, 2000, radians(-5)], MARS, VEHICLE, None,
                                  entry_site=dict(latitude_deg=0, longitude_deg=0,
                                                  heading_deg=heading, bank_deg=bank))

        east, west, banked = trajectory(90, 0), trajectory(270, 0), trajectory(90, 20)
        self.assertLess(east.values['total_drag_n'], west.values['total_drag_n'])
        for flight in (east, west, banked):
            flight.advance(10)
        self.assertGreater(east.values['longitude_deg'], 0)
        self.assertLess(west.values['longitude_deg'], 0)
        self.assertAlmostEqual(east.values['crossrange_m'], 0)
        self.assertLess(banked.values['crossrange_m'], -10)

    def test_downrange_is_the_surface_projection_of_horizontal_travel(self):
        height, speed, angle = 120000.0, 3000.0, radians(-5)
        state = [height, speed, angle, 0.0]
        derivatives, values = calculate_state(state, MARS, VEHICLE)
        expected = speed * cos(angle) * MARS['radius'] / (MARS['radius'] + height)
        self.assertEqual(values['downrange_m'], 0)
        self.assertAlmostEqual(derivatives[3], expected)
        self.assertAlmostEqual(advance_state(state, .001, MARS, VEHICLE)[3] / .001,
                               expected, delta=.001)
        vertical, _ = calculate_state([height, speed, -pi / 2, 0.0], MARS, VEHICLE)
        self.assertAlmostEqual(vertical[3], 0)

    def test_mach_uses_atmospheric_temperature(self):
        speed = 400.0
        for temperature, sound_speed in ((214.0, 232.29114), (250.0, 248.99685)):
            with self.subTest(temperature=temperature):
                planet = dict(MARS, T0=temperature)
                _, values = calculate_state([6000.0, speed, radians(-25)], planet, VEHICLE)
                self.assertAlmostEqual(speed / values['mach'], sound_speed, places=4)

    def test_drag_curve_and_accelerations(self):
        for mach, expected in ((-0.1, .615), (.925, .537), (.95, .489), (3, .467)):
            self.assertAlmostEqual(parachute_drag_coefficient(mach, NASA_DRAG_CURVE_DGB), expected)

        vector = np.array([5500.0, 418.124055, radians(-25)])
        entry, entry_values = calculate_state(vector, MARS, VEHICLE, parachute=PARACHUTE)
        canopy, canopy_values = calculate_state(vector, MARS, VEHICLE, parachute=PARACHUTE,
                                                canopy_deployed=True)
        pressure = canopy_values['dynamic_pressure_pa']
        expected_cd = parachute_drag_coefficient(canopy_values['mach'], NASA_DRAG_CURVE_DGB)
        self.assertEqual(canopy_values['parachute_cd'], expected_cd)
        canopy_drag_area = expected_cd * pi * PARACHUTE['diameter_m']**2 / 4
        gravity = canopy_values['gravity_m_s2']
        mass = VEHICLE['mass']
        self.assertEqual(entry_values['bank_angle_deg'], 0.0)
        self.assertIsNone(canopy_values['bank_angle_deg'])
        self.assertAlmostEqual(entry[1] - canopy[1], pressure * canopy_drag_area / mass)
        self.assertAlmostEqual(entry[2] - canopy[2], pressure * VEHICLE['S'] * VEHICLE['CL']
                               / (mass * vector[1]))
        equilibrium_speed = canopy_values['equilibrium_speed_m_s']
        drag_area = VEHICLE['CD'] * VEHICLE['S'] + canopy_drag_area
        self.assertAlmostEqual(entry_values['effective_cd'], VEHICLE['CD'])
        self.assertAlmostEqual(canopy_values['effective_cd'], drag_area / VEHICLE['S'])
        self.assertAlmostEqual(pressure * VEHICLE['S'] * canopy_values['effective_cd'],
                               canopy_values['total_drag_n'])
        self.assertAlmostEqual(.5 * canopy_values['density_kg_m3'] * equilibrium_speed**2 * drag_area,
                               -mass * gravity * sin(vector[2]), delta=1e-5)

        dt = .0001
        next_vector = advance_state(vector, dt, MARS, VEHICLE, parachute=PARACHUTE,
                                    canopy_deployed=True)
        vertical_acceleration = (
            next_vector[1] * sin(next_vector[2]) - vector[1] * sin(vector[2])) / dt
        self.assertAlmostEqual(canopy_values['vertical_acceleration_m_s2'], vertical_acceleration,
                               delta=.002)
        self.assertAlmostEqual(canopy[2], (vector[1] / (MARS['radius'] + vector[0])
                                          - gravity / vector[1]) * cos(vector[2]))

    def test_deployment_splits_a_step_at_the_altitude_crossing(self):
        initial = [5610.0, 418.134, radians(-25)]
        parachute = dict(PARACHUTE, deploy_altitude_m=5608.0)
        flight = MarsTrajectory(initial, MARS, VEHICLE, parachute)
        events = flight.advance(.02)
        self.assertEqual([event.name for event in events], ['parachute_deploy'])
        event = events[0]
        self.assertGreater(event.time_s, 0)
        self.assertLess(event.time_s, .02)
        self.assertAlmostEqual(event.values['altitude_m'], 5608.0, places=3)
        self.assertLess(flight.vector[0], 5608.0)
        at_switch = advance_state([*initial, 0.0], event.time_s, MARS, VEHICLE)
        np.testing.assert_allclose(
            at_switch, [event.values[key] for key in (
                'altitude_m', 'velocity_m_s', 'flight_path_angle_rad', 'downrange_m')],
            rtol=0, atol=1e-9)
        expected = advance_state(at_switch, .02 - event.time_s, MARS, VEHICLE,
                                 parachute=parachute, canopy_deployed=True)
        np.testing.assert_allclose(flight.vector, expected, rtol=0, atol=1e-9)
        self.assertEqual(event.before_values['parachute_drag_n'], 0.0)
        self.assertIsNone(event.before_values['parachute_cd'])
        self.assertEqual(event.before_values['vehicle_drag_n'], event.values['vehicle_drag_n'])
        self.assertEqual(event.values['effective_cl'], 0.0)
        self.assertAlmostEqual(event.before_values['lift_n'],
                               event.before_values['dynamic_pressure_pa'] * VEHICLE['S'] * VEHICLE['CL'])
        self.assertEqual(event.values['lift_n'], 0.0)
        self.assertAlmostEqual(event.values['total_drag_n'],
                               event.values['vehicle_drag_n'] + event.values['parachute_drag_n'])

    def test_deployment_prints_even_with_udp_disabled(self):
        flight = MarsTrajectory([6000.0, 400.0, radians(-25)], MARS, VEHICLE,
                                dict(PARACHUTE, deploy_altitude_m=6000.0))
        deploy, = flight.advance(.02)
        wire = Downlink(None, mission='mars', run_id='disabled')
        grid = TrajectoryLEDGrid(SenseEmu(noise=False, realtime=False), 6000)
        with redirect_stdout(io.StringIO()) as console:
            sim.publish_flight_event(deploy, grid, wire)
        text = console.getvalue()
        mach = deploy.values['mach']
        opening_load = deploy.values['opening_factor'] * deploy.values['parachute_drag_n'] / 1000
        self.assertEqual(text, f'[0.00 s] [6000 m] EVENT parachute_deploy: Mach {mach:.2f}, '
                               f'estimated opening load {opening_load:.1f} kN\n')

    def test_no_landing_warning_without_powered_descent(self):
        flight = MarsTrajectory([5600.0, 399.6, radians(-25)], MARS, VEHICLE, None)
        events = flight.advance(120)
        self.assertEqual(events, [])
        self.assertEqual(flight.phase, Phase.LANDED)
        self.assertFalse(flight.canopy_deployed)

    def test_no_late_warning_when_powered_descent_starts_near_contact(self):
        settings = untargeted_landing()
        settings['heatshield'] = dict(settings['heatshield'], delay_after_chute_s=0)
        flight = MarsTrajectory([100.0, 75.0, radians(-90)], MARS, VEHICLE, PARACHUTE,
                                Phase.PARACHUTE_DESCENT, entry_site=EQUATORIAL_ENTRY,
                                powered_landing=settings)
        events = flight.advance(20.0)
        names = [event.name for event in events]
        self.assertIn('powered_descent_start', names)
        self.assertNotIn('landing_warning', names)
        self.assertIs(flight.phase, Phase.IMPACT)

    def test_transient_terminal_match_resets_dwell(self):
        flight = MarsTrajectory([1154.21, 86.91259, -1.3436864], *REFERENCE_CASE,
                                Phase.PARACHUTE_DESCENT)
        flight.advance(.5)
        self.assertEqual(flight.terminal_since_s, 0.0)
        events = flight.advance(.7)
        self.assertIsNone(flight.terminal_since_s)
        self.assertEqual(flight.phase, Phase.PARACHUTE_DESCENT)
        self.assertNotIn('near_terminal_descent', [event.name for event in events])

    def test_homework_4_vertical_reference(self):
        # A large radius approximates HW4's constant gravity over the descent.
        radius, gravity, density_0, decay = 1e12, 3.74, .057, 1.28e-4
        gas_constant = gravity / (decay * MARS['T0'])
        planet = dict(MARS, radius=radius, mu=gravity * radius**2, gas_constant=gas_constant,
                      P0=density_0 * gas_constant * MARS['T0'])
        vehicle = dict(VEHICLE, mass=1200, CD=1.65, S=pi * 4.5**2 / 4)
        parachute = dict(PARACHUTE, diameter_m=21.5, drag_curve=((0, .58),))
        vector = np.array([15000.0, 900.0, -pi / 2])
        while vector[0] > 5000:
            previous = vector
            vector = advance_state(vector, .1, planet, vehicle, parachute=parachute,
                                   canopy_deployed=True)
        fraction = (previous[0] - 5000) / (previous[0] - vector[0])
        vector = previous + fraction * (vector - previous)
        _, values = calculate_state(vector, planet, vehicle, parachute=parachute,
                                    canopy_deployed=True)
        self.assertAlmostEqual(vector[1], 35.910816, places=4)
        self.assertAlmostEqual(values['equilibrium_speed_m_s'], 35.512155, places=4)
        self.assertAlmostEqual((vector[1] / values['equilibrium_speed_m_s'])**2, 1.022578, places=5)


class VirtualWallTime:
    now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class LEDGridTests(TestCase):
    def test_position_blinks_over_event_markers_until_touchdown(self):
        grid = TrajectoryLEDGrid(SenseEmu(noise=False, realtime=False), 8000, time_span_s=80)
        grid.add_event('entry_interface', 0, 8000)
        grid.add_sample(0, 8000)
        grid.add_event('parachute_deploy', 40, 4000)
        grid.add_sample(40, 4000)
        chute_cell = 4 * 8 + 4

        lit = grid.render(.25)
        unlit = grid.render(.75)
        self.assertEqual(lit[0], (0, 170, 110))
        self.assertEqual(unlit[0], lit[0])
        self.assertEqual(lit[chute_cell], (180, 180, 180))
        self.assertEqual(unlit[chute_cell], grid.COLOURS['parachute_deploy'])

        grid.mark_landed()
        landed = grid.render(1.25)
        self.assertEqual(landed[chute_cell], unlit[chute_cell])
        self.assertEqual(landed[-1], (0, 255, 0))


class MissionTests(TestCase):
    def run_mars(self, playback, powered_landing=None, *, entry_site=EQUATORIAL_ENTRY,
                 entry_guidance=None, wind=None, flight_inputs=(MARS, VEHICLE, PARACHUTE),
                 entry_vector=ENTRY_VECTOR):
        planet, vehicle, parachute = flight_inputs
        wall = VirtualWallTime()
        sense = SenseEmu(noise=False, realtime=False)
        grid = TrajectoryLEDGrid(sense, entry_vector[0])
        window = MissionWindow.__new__(MissionWindow)
        window.closed = False
        window.updates_per_sim_second = 3.0
        window.sim_clock = SimulationClock(playback)
        deployment_samples = []
        window.plotter = SimpleNamespace(
            add_sample=lambda sample: None,
            add_deployment=lambda before, after: deployment_samples.append((before, after)))
        state = initialize(*entry_vector, vehicle['mass'], planet)
        warning_pixels = []
        warning_display_times = []
        start_countdown = grid.start_landing_countdown

        def check_countdown():
            start_countdown()
            warning_display_times.append(state.elapsed_s)
            started_at = grid._countdown_started_at
            warning_pixels.extend(grid.render(started_at + offset)[-1] for offset in (.25, .75))

        with patch('edl_flight_computer.shared.ground_station.socket.socket'):
            receiver = GroundStation(echo=False)
        wire = Downlink(None, mission='mars', run_id=f'test-{playback}', telemetry_hz=10)
        received = []

        def send(payload, address):
            packet = receiver.ingest(payload)
            if packet is not None:
                received.append(packet)

        wire.socket = SimpleNamespace(sendto=send, close=lambda: None)
        with TemporaryDirectory() as folder:
            path = Path(folder) / 'flight.csv'
            with (redirect_stdout(io.StringIO()) as console, patch.object(sim, 'monotonic', wall.monotonic),
                  patch.object(sim, 'sleep', wall.sleep),
                  patch.object(grid, 'start_landing_countdown', check_countdown),
                  patch.object(downlink_module, 'monotonic', wall.monotonic),
                  patch.object(downlink_module, 'sleep', wall.sleep)):
                sim.run_mission(sense, state, vehicle, window, grid, path, wire, parachute=parachute,
                                entry_site=entry_site, powered_landing=powered_landing,
                                entry_guidance=entry_guidance, wind=wind)
            with path.open(newline='', encoding='utf-8') as stream:
                rows = list(csv.DictReader(stream))
        receiver.close()
        self.assertEqual(wire.failed, 0)
        self.assertEqual(console.getvalue().count('EVENT parachute_deploy'), 1)
        text = console.getvalue()
        self.assertEqual(text.count('EVENT peak_acceleration'), 1)
        self.assertNotIn('EVENT peak_velocity', text)
        self.assertNotIn('EVENT vertical_descent_start', text)
        self.assertRegex(text, r'EVENT peak_acceleration: -\d+\.\d+ m/s\^2 at \d+\.\d+ m/s')
        self.assertRegex(text, r'EVENT peak_heating: \d+\.\d+ kW/m\^2 at \d+\.\d+ m/s')
        self.assertRegex(text, r'EVENT parachute_deploy: Mach \d+\.\d+, estimated opening load \d+\.\d+ kN')
        self.assertNotIn('Entry peak deceleration:', text)
        self.assertRegex(text, r'\[\d+\.\d{2} s\] \[0 m\] EVENT landing: \d+\.\d{2} m/s')
        self.assertGreater(receiver.runs[wire.run_id].received, len(received))
        self.assertEqual(window.latest['bank_angle_deg'], '')
        self.assertTrue(grid.landed)
        self.assertEqual(warning_pixels, [(0, 255, 0), (0, 0, 0)] if powered_landing else [])
        self.assertNotIn('vertical_descent_start', [packet.get('event') for packet in received])
        self.assertNotIn('vertical descent', {row['phase'] for row in rows})
        self.assertNotIn('vertical descent', {packet['data'].get('phase') for packet in received})
        self.assertEqual(grid.render()[-1], (0, 255, 0))
        self.assertIn('parachute_deploy', [name for name, _, _ in grid.events])
        if powered_landing:
            self.assertRegex(text, r'EVENT powered_descent_start: \d+\.\d+ kg, \d+\.\d+ m/s airspeed')
            displayed_at, = warning_display_times
            self.assertGreaterEqual(state.elapsed_s - displayed_at, 10.0)
            markers = {name for name, _, _ in grid.events}
            self.assertTrue({'entry_interface', 'heatshield_jettison', 'backshell_separation',
                             'powered_descent_start'} <= markers)
        if any(packet.get('event') == 'near_terminal_descent' for packet in received):
            self.assertIn('EVENT near_terminal_descent', text)
            self.assertIn('near_terminal_descent', [name for name, _, _ in grid.events])
        self.assertEqual(sum(name == 'peak_acceleration' for name, _, _ in grid.events), 1)
        before, after = deployment_samples[0]
        self.assertEqual(len(deployment_samples), 1)
        deployment, = [packet for packet in received if packet.get('event') == 'parachute_deploy']
        self.assertEqual(before['time_s'], deployment['time_s'])
        self.assertEqual(after['time_s'], deployment['time_s'])
        self.assertEqual(before['altitude_m'], after['altitude_m'])
        self.assertEqual(before['downrange_m'], after['downrange_m'])
        self.assertGreater(after['total_drag_n'], before['total_drag_n'])
        self.assertGreater(before['lift_n'], 0.0)
        self.assertEqual(after['lift_n'], 0.0)
        return received, rows

    def test_mars_demo_stages_at_configured_triggers_and_hits_target(self):
        guidance = sim.configured_entry_guidance(MARS_SETTINGS)
        settings = MARS_SETTINGS['retro_prop']
        wind = {key: value for key, value in MARS_SETTINGS['wind'].items() if key != 'enabled'}
        received, rows = self.run_mars(10.0, settings, entry_site=INITIAL['entry_site'],
                                       entry_guidance=guidance, wind=wind)
        names = ('parachute_deploy', 'heatshield_jettison', 'near_terminal_descent', 'backshell_separation',
                 'powered_descent_start', 'landing_warning', 'landing')
        events = [packet for packet in received if packet.get('event') in names]
        self.assertEqual([event['event'] for event in events], list(names))
        chute, shield, terminal, shell, ignition, warning, contact = events
        near = terminal['data']
        self.assertLessEqual(abs(near['velocity_m_s'] / near['equilibrium_speed_m_s'] - 1), .07)
        self.assertLessEqual(abs(near['vertical_acceleration_m_s2']), .25)
        self.assertAlmostEqual(chute['data']['mach'], 1.8, places=5)
        self.assertAlmostEqual(chute['data']['opening_factor'], MARS_SETTINGS['parachute']['opening_factor'])
        self.assertAlmostEqual(chute['data']['opening_load_n'],
                               chute['data']['opening_factor'] * chute['data']['parachute_drag_n'])
        deployment_row = next(row for row in rows if row['phase'] == 'parachute descent')
        self.assertEqual(float(deployment_row['opening_load_n']), chute['data']['opening_load_n'])
        self.assertAlmostEqual(shield['time_s'] - chute['time_s'], 20.0)
        self.assertAlmostEqual(shell['data']['altitude_m'], 2100.0, places=3)
        self.assertEqual(shell['time_s'], ignition['time_s'])
        warning_row = next(row for row in rows if float(row['time_s']) >= warning['time_s'])
        self.assertEqual(warning_row['phase'], 'powered descent')
        self.assertGreater(float(warning_row['horizontal_speed_m_s']),
                           settings['propulsion']['horizontal_capture_m_s'])
        self.assertTrue(contact['data']['landing_target_met'])
        self.assertLess(contact['data']['landing_target_error_m'], settings['landing_target']['tolerance_m'])
        self.assertTrue(all(float(row['flight_path_angle_rad']) < 0
                            for row in rows if row['phase'] == 'entry'))
        self.assertGreater(float(rows[-1]['propellant_kg']), 0.0)

    def test_powered_landing_stages_fuel_and_touchdown(self):
        settings = untargeted_landing()
        settings = dict(settings,
                        heatshield=dict(settings['heatshield'], mass_kg=264.72, delay_after_chute_s=15.0),
                        backshell=dict(settings['backshell'], mass_kg=134.8195533055,
                                       separation_altitude_m=1000.0),
                        propulsion=dict(settings['propulsion'], propellant_kg=150.736246428,
                                        max_thrust_n=13145.478588753, vertical_braking_m_s2=3.33683208083))
        vehicle = dict(VEHICLE, mass=2264.72, S=15.0, Rn=4.5, CD=1.5, CL=0.5)
        chute_settings = dict(PARACHUTE, diameter_m=17.7336033676, deploy_altitude_m=11053.0,
                              drag_curve=((0., .60),))
        received, rows = self.run_mars(30.0, settings, entry_vector=REFERENCE_ENTRY,
                                     flight_inputs=(MARS, vehicle, chute_settings))
        names = ('parachute_deploy', 'heatshield_jettison', 'backshell_separation',
                 'powered_descent_start', 'landing_warning', 'landing')
        events = [packet for packet in received if packet.get('event') in names]
        self.assertEqual([event['event'] for event in events], list(names))
        chute, shield, shell, ignition, warning, touchdown = events
        self.assertGreaterEqual(warning['data']['estimated_time_to_contact_s'], 10.0)
        self.assertLessEqual(warning['data']['estimated_time_to_contact_s'], 10.5)
        self.assertAlmostEqual(warning['data']['estimated_time_to_contact_s'],
                               2 * warning['data']['altitude_m'] /
                               (warning['data']['descent_rate_m_s'] +
                                settings['propulsion']['target_descent_speed_m_s']))
        self.assertAlmostEqual(shield['time_s'] - chute['time_s'], settings['heatshield']['delay_after_chute_s'])
        self.assertAlmostEqual(ignition['time_s'] - shell['time_s'], settings['propulsion']['ignition_delay_s'])
        self.assertAlmostEqual(shell['data']['altitude_m'], settings['backshell']['separation_altitude_m'], places=3)
        self.assertAlmostEqual(shield['data']['mass_kg'], vehicle['mass'] - settings['heatshield']['mass_kg'])
        self.assertAlmostEqual(shell['data']['mass_kg'], shield['data']['mass_kg'] - settings['backshell']['mass_kg'])
        self.assertEqual(rows[-2]['phase'], 'powered descent')
        phases = {row['phase'] for row in rows}
        self.assertTrue({'heatshield jettisoned', 'backshell separated', 'powered descent'} <= phases)
        fuel = [float(row['propellant_kg']) for row in rows]
        flow = [float(row['propellant_flow_kg_s']) for row in rows]
        self.assertTrue(np.all(np.diff(fuel) <= 0))
        self.assertTrue(any(rate > 0 for rate in flow))
        self.assertGreater(fuel[-1], 0)
        self.assertAlmostEqual(shell['data']['mass_kg'] - touchdown['data']['mass_kg'], fuel[0] - fuel[-1])
        self.assertAlmostEqual(touchdown['data']['vertical_velocity_m_s'], -2.0, delta=.1)
        self.assertLessEqual(touchdown['data']['horizontal_speed_m_s'], .2)

    def test_event_times_and_payloads_across_playback_rates(self):
        times = []
        key_events = ['parachute_deploy', 'near_terminal_descent', 'landing']
        for speed in (10.0, 50.0):
            received, rows = self.run_mars(speed, flight_inputs=REFERENCE_CASE, entry_vector=REFERENCE_ENTRY)
            events = [packet for packet in received if packet.get('event') in key_events]
            self.assertEqual([packet['event'] for packet in events], key_events)
            deploy, terminal, landing = events
            entry_peak, = [packet for packet in received if packet.get('event') == 'peak_acceleration']
            peak = entry_peak['data']['peak']
            self.assertEqual(peak['phase'], 'entry')
            self.assertLess(entry_peak['time_s'], deploy['time_s'])
            self.assertLess(deploy['data']['acceleration_m_s2'], peak['value'])
            self.assertEqual(landing['data']['peaks']['peak_acceleration'], peak)
            times.append([event['time_s'] for event in events])
            self.assertAlmostEqual(deploy['data']['altitude_m'], 7092, places=3)
            self.assertAlmostEqual(deploy['data']['total_drag_n'],
                                   deploy['data']['parachute_drag_n'] + deploy['data']['vehicle_drag_n'])
            deployment_row = next(row for row in rows if row['phase'] == 'parachute descent')
            self.assertEqual(float(deployment_row['time_s']), deploy['time_s'])
            self.assertEqual(float(deployment_row['total_drag_n']), deploy['data']['total_drag_n'])
            data = terminal['data']
            self.assertLessEqual(abs(data['vertical_acceleration_m_s2']), .10)
            self.assertLessEqual(abs(data['velocity_m_s'] / data['equilibrium_speed_m_s'] - 1), .05)
            self.assertEqual(rows[-1]['phase'], 'landed')
            self.assertEqual(float(rows[-1]['time_s']), landing['time_s'])
            self.assertEqual(float(rows[-1]['altitude_m']), 0.0)
            self.assertEqual(set(row['phase'] for row in rows),
                             {'entry', 'parachute descent', 'terminal descent', 'landed'})
            self.assertEqual(float(rows[0]['bank_angle_deg']), 0)
            self.assertEqual(rows[0]['equilibrium_speed_m_s'], '')
            self.assertTrue(np.all(np.diff([float(row['downrange_m']) for row in rows]) > 0))
            self.assertAlmostEqual(float(rows[-1]['crossrange_m']), 0.0)
            for key in ('mach', 'vertical_acceleration_m_s2', 'equilibrium_speed_m_s',
                        'vehicle_cd', 'parachute_cd', 'effective_cd', 'effective_cl', 'total_drag_n', 'lift_n',
                        'downrange_m', 'crossrange_m', 'latitude_deg', 'longitude_deg'):
                self.assertAlmostEqual(float(rows[-1][key]), received[-2]['data'][key])
        # Playback changes Euler substep boundaries slightly.
        np.testing.assert_allclose(times[0], times[1], rtol=0, atol=.01)

    def test_earth_drop_still_uses_its_own_phases_and_csv(self):
        wall = VirtualWallTime()
        sense = SenseEmu(noise=False, realtime=False)

        def sleep(seconds):
            wall.sleep(seconds)
            sense.set_accelerometer_raw(0, 0, 0 if 6.0 <= wall.now < 6.5 else 1)
            sense.advance(seconds)

        with TemporaryDirectory() as folder:
            path = Path(folder) / 'earth.csv'
            with (redirect_stdout(io.StringIO()), patch.object(drop, 'monotonic', wall.monotonic),
                  patch.object(drop, 'sleep', sleep)):
                detector = drop.run_live_drop(sense, path, stop_requested=lambda: wall.now > 16)
            with path.open(newline='', encoding='utf-8') as stream:
                rows = list(csv.DictReader(stream))
        self.assertEqual(detector.phase, Phase.LANDED)
        self.assertIn('landing', [row['phase'] for row in rows])
        self.assertNotIn('mach', rows[-1])
