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
from edl_flight_computer.mars.led_grid import TrajectoryLEDGrid
from edl_flight_computer.mars.model import (
    MARS, MarsTrajectory, advance_state, calculate_state, parachute_drag_coefficient,
)
from edl_flight_computer.mars.state import SimulationClock, initialize
from edl_flight_computer.mars.status_window import MissionWindow
from edl_flight_computer.run import MARS_SETTINGS, NASA_DRAG_CURVE_DGB
from edl_flight_computer.shared import downlink as downlink_module
from edl_flight_computer.shared.downlink import Downlink
from edl_flight_computer.shared.ground_station import GroundStation
from edl_flight_computer.shared.phases import Phase
from sense_hat_emulator.emulator import SenseEmu


INITIAL = MARS_SETTINGS['initial_conditions']
VEHICLE = INITIAL['vehicle_params']
PARACHUTE = MARS_SETTINGS['parachute']
ENTRY_VECTOR = [INITIAL['altitude_m'], INITIAL['velocity_m_s'], radians(INITIAL['fpa_deg'])]


class FlightModelTests(TestCase):
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
        self.assertIs(PARACHUTE['drag_curve'], NASA_DRAG_CURVE_DGB)
        for mach, expected in ((-0.1, .615), (.925, .537), (.95, .489), (3, .467)):
            self.assertAlmostEqual(parachute_drag_coefficient(mach, NASA_DRAG_CURVE_DGB), expected)

        vector = np.array([5500.0, 418.124055, radians(-25)])
        entry, entry_values = calculate_state(vector, MARS, VEHICLE, parachute=PARACHUTE)
        canopy, canopy_values = calculate_state(vector, MARS, VEHICLE, parachute=PARACHUTE,
                                                canopy_deployed=True)
        pressure = canopy_values['dynamic_pressure_pa']
        canopy_drag_area = .686 * pi * PARACHUTE['diameter_m']**2 / 4
        gravity = canopy_values['gravity_m_s2']
        mass = VEHICLE['mass']
        self.assertEqual(entry_values['bank_angle_deg'], 0.0)
        self.assertIsNone(canopy_values['bank_angle_deg'])
        self.assertAlmostEqual(entry[1] - canopy[1], pressure * canopy_drag_area / mass)
        self.assertAlmostEqual(entry[2] - canopy[2], pressure * VEHICLE['S'] * VEHICLE['CL']
                               / (mass * vector[1]))
        equilibrium_speed = canopy_values['equilibrium_speed_m_s']
        drag_area = VEHICLE['CD'] * VEHICLE['S'] + canopy_drag_area
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

    def test_deployment_splits_a_step_inside_the_envelope(self):
        initial = [5610.0, 418.134, radians(-25)]
        parachute = dict(PARACHUTE, min_deploy_altitude_m=5608.0,
                         max_deploy_dynamic_pressure_pa=1000.0)
        flight = MarsTrajectory(initial, MARS, VEHICLE, parachute)
        events = flight.advance(.02)
        self.assertEqual([event.name for event in events], ['parachute_deploy'])
        event = events[0]
        self.assertGreater(event.time_s, 0)
        self.assertLess(event.time_s, .02)
        self.assertGreater(event.values['altitude_m'], 5608.0)
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
        self.assertAlmostEqual(event.values['total_drag_n'],
                               event.values['vehicle_drag_n'] + event.values['parachute_drag_n'])

    def test_deployment_prints_even_with_udp_disabled(self):
        flight = MarsTrajectory([6000.0, 400.0, radians(-25)], MARS, VEHICLE, PARACHUTE)
        deploy, = flight.advance(.02)
        wire = Downlink(None, mission='mars', run_id='disabled')
        grid = TrajectoryLEDGrid(SenseEmu(noise=False, realtime=False), 6000)
        with redirect_stdout(io.StringIO()) as console:
            sim.publish_flight_event(deploy, grid, wire)
        text = console.getvalue()
        self.assertEqual(text.count('EVENT parachute_deploy'), 1)
        self.assertIn('Total drag rises', text)

    def test_rejected_pressure_envelope_still_allows_landing_warning(self):
        flight = MarsTrajectory([5600.0, 399.6, radians(-25)], MARS, VEHICLE,
                                dict(PARACHUTE, max_deploy_dynamic_pressure_pa=100))
        events = flight.advance(120)
        self.assertEqual([event.name for event in events], ['landing_warning'])
        self.assertEqual(flight.phase, Phase.LANDED)
        self.assertFalse(flight.canopy_deployed)

    def test_transient_terminal_match_resets_dwell(self):
        flight = MarsTrajectory([1154.21, 86.91259, -1.3436864], MARS, VEHICLE,
                                PARACHUTE, Phase.PARACHUTE_DESCENT)
        flight.advance(.5)
        self.assertEqual(flight.terminal_since_s, 0.0)
        events = flight.advance(.7)
        self.assertIsNone(flight.terminal_since_s)
        self.assertEqual(flight.phase, Phase.PARACHUTE_DESCENT)
        self.assertNotIn('terminal_descent', [event.name for event in events])

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


class MissionTests(TestCase):
    def run_mars(self, playback):
        wall = VirtualWallTime()
        sense = SenseEmu(noise=False, realtime=False)
        grid = TrajectoryLEDGrid(sense, ENTRY_VECTOR[0])
        window = MissionWindow.__new__(MissionWindow)
        window.closed = False
        window.updates_per_sim_second = 3.0
        window.sim_clock = SimulationClock(playback)
        deployment_samples = []
        window.plotter = SimpleNamespace(
            add_sample=lambda sample: None,
            add_deployment=lambda before, after: deployment_samples.append((before, after)))
        state = initialize(*ENTRY_VECTOR, VEHICLE['mass'], MARS)
        warning_pixels = []
        start_countdown = grid.start_landing_countdown

        def check_countdown():
            start_countdown()
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
                sim.run_mission(sense, state, VEHICLE, window, grid, path, wire, parachute=PARACHUTE)
            with path.open(newline='', encoding='utf-8') as stream:
                rows = list(csv.DictReader(stream))
        receiver.close()
        self.assertEqual(wire.failed, 0)
        self.assertEqual(console.getvalue().count('EVENT parachute_deploy'), 1)
        self.assertEqual(console.getvalue().count('EVENT peak_acceleration'), 1)
        self.assertGreater(receiver.runs[wire.run_id].received, len(received))
        self.assertEqual(window.latest['bank_angle_deg'], '')
        self.assertTrue(grid.landed)
        self.assertEqual(warning_pixels, [(0, 255, 0), (0, 0, 0)])
        self.assertEqual(grid.render()[-1], (0, 255, 0))
        self.assertIn('parachute_deploy', [name for name, _, _ in grid.events])
        self.assertIn('terminal_descent', [name for name, _, _ in grid.events])
        self.assertEqual(sum(name == 'peak_acceleration' for name, _, _ in grid.events), 1)
        before, after = deployment_samples[0]
        self.assertEqual(len(deployment_samples), 1)
        self.assertEqual(before['altitude_m'], after['altitude_m'])
        self.assertEqual(before['downrange_m'], after['downrange_m'])
        self.assertGreater(after['total_drag_n'], 10 * before['total_drag_n'])
        return received, rows

    def test_event_times_and_payloads_across_playback_rates(self):
        times = []
        key_events = ['parachute_deploy', 'terminal_descent', 'landing_warning', 'landing']
        for speed in (10.0, 50.0):
            received, rows = self.run_mars(speed)
            events = [packet for packet in received if packet.get('event') in key_events]
            self.assertEqual([packet['event'] for packet in events], key_events)
            deploy, terminal, warning, landing = events
            entry_peak, = [packet for packet in received if packet.get('event') == 'peak_acceleration']
            peak = entry_peak['data']['peak']
            self.assertEqual(peak['phase'], 'entry')
            self.assertLess(entry_peak['time_s'], deploy['time_s'])
            self.assertLess(deploy['data']['acceleration_m_s2'], peak['value'])
            self.assertEqual(landing['data']['peaks']['peak_acceleration'], peak)
            times.append([event['time_s'] for event in events])
            self.assertAlmostEqual(deploy['data']['mach'], 1.75257, places=5)
            self.assertAlmostEqual(deploy['data']['altitude_m'], 5996, delta=1)
            self.assertAlmostEqual(deploy['data']['dynamic_pressure_pa'], 750, places=4)
            self.assertAlmostEqual(deploy['data']['velocity_m_s'], 407.1065, places=4)
            self.assertAlmostEqual(deploy['data']['total_drag_n'],
                                   deploy['data']['parachute_drag_n'] + deploy['data']['vehicle_drag_n'])
            deployment_row = next(row for row in rows if row['phase'] == 'parachute descent')
            self.assertEqual(float(deployment_row['time_s']), deploy['time_s'])
            self.assertEqual(float(deployment_row['total_drag_n']), deploy['data']['total_drag_n'])
            data = terminal['data']
            self.assertLessEqual(abs(data['vertical_acceleration_m_s2']), .10)
            self.assertLessEqual(abs(data['velocity_m_s'] / data['equilibrium_speed_m_s'] - 1), .05)
            self.assertAlmostEqual(data['flight_path_angle_rad'], -1.263, places=3)
            self.assertAlmostEqual(warning['data']['estimated_time_to_contact_s'], 10.0, places=5)
            self.assertEqual(rows[-1]['phase'], 'landed')
            self.assertEqual(float(rows[-1]['time_s']), landing['time_s'])
            self.assertEqual(float(rows[-1]['altitude_m']), 0.0)
            self.assertEqual(set(row['phase'] for row in rows),
                             {'entry', 'parachute descent', 'terminal descent', 'landed'})
            self.assertEqual(float(rows[0]['bank_angle_deg']), 0)
            self.assertEqual(rows[0]['equilibrium_speed_m_s'], '')
            self.assertTrue(np.all(np.diff([float(row['downrange_m']) for row in rows]) > 0))
            for key in ('mach', 'vertical_acceleration_m_s2', 'equilibrium_speed_m_s',
                        'vehicle_cd', 'parachute_cd', 'effective_cl', 'total_drag_n', 'downrange_m'):
                self.assertAlmostEqual(float(rows[-1][key]), received[-2]['data'][key])
        np.testing.assert_allclose(times[0], [693.121, 739.516, 750.439, 760.541], atol=.01)
        # Playback changes RK4 step boundaries, so allow 0.1 ms of numerical variation.
        np.testing.assert_allclose(times[0], times[1], rtol=0, atol=1e-4)

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
