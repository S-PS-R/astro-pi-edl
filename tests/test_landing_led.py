"""Check the countdown on the existing LED grid and mission loop."""

from math import pi
import unittest
from unittest.mock import Mock, patch

from edl_flight_computer.led_grid import TrajectoryLEDGrid
from edl_flight_computer.model import PLANETS
from edl_flight_computer.run import run_mission
from edl_flight_computer.state import MissionState, Phase
from sense_hat_emulator import SenseEmu


class LandingLEDTests(unittest.TestCase):
    def setUp(self):
        self.sense = SenseEmu(noise=False, realtime=False, seed=42)
        self.grid = TrajectoryLEDGrid(self.sense, 100.0)
        self.grid.add_sample(0.0, 100.0)
        self.grid.add_event('peak_decel', 2.0, 50.0)

    def start_countdown(self):
        with patch('edl_flight_computer.led_grid.monotonic', return_value=100.0):
            self.grid.start_landing_countdown()

    def test_countdown_flashes_only_the_reserved_pixel(self):
        before = self.grid.render(100.0)
        self.start_countdown()
        for time, colour in ((100.0, (0, 255, 0)), (100.49, (0, 255, 0)),
                             (100.5, (0, 0, 0)), (101.0, (0, 255, 0))):
            pixels = self.grid.render(time)
            self.assertEqual(pixels[63], colour)
            self.assertEqual(pixels[:63], before[:63])

    def test_repeated_start_does_not_restart_pulses(self):
        self.start_countdown()
        with patch('edl_flight_computer.led_grid.monotonic', return_value=100.4):
            self.grid.start_landing_countdown()
        self.assertEqual(self.grid.render(100.5)[63], (0, 0, 0))
        self.assertEqual(self.grid.render(112.0)[63], (0, 255, 0))

    def test_ground_contact_keeps_green_solid(self):
        self.start_countdown()
        self.grid.mark_landed()
        for time in (100.5, 101.0, 112.5):
            self.assertEqual(self.grid.render(time)[63], (0, 255, 0))

    def test_landing_without_countdown_also_sets_green(self):
        self.grid.mark_landed()
        self.assertEqual(self.grid.render(100.5)[63], (0, 255, 0))

    def test_unchanged_pixels_do_not_repeat_board_io(self):
        self.start_countdown()
        with patch.object(self.sense, 'set_pixels', wraps=self.sense.set_pixels) as write:
            self.grid.render(100.0)
            self.grid.render(100.1)
            self.assertEqual(write.call_count, 1)

    def test_mars_and_earth_loops_trigger_at_ten_seconds_and_latch_until_ground(self):
        for planet, phase in (('mars', Phase.ENTRY), ('earth', Phase.LANDING)):
            with self.subTest(planet=planet):
                grid = TrajectoryLEDGrid(self.sense, 100.0)
                state = MissionState(100.0, 5.0, -pi / 2, 1.0, PLANETS[planet], phase)
                params = dict(CD=1.0, CL=0.0, mass=1.0, S=1.0, Rn=1.0)
                window = Mock(closed=False, updates_per_sim_second=1.0)
                window.sim_clock.snapshot.side_effect = [(t, 1.0) for t in range(5)]
                phases = []
                window.publish.side_effect = lambda state, *args: phases.append(state.phase)
                notifier = Mock()
                # First outside the window; then exactly ten seconds; then a
                # revised, longer prediction; finally a step below ground.
                samples = [[51.0, 5.0, -pi / 2], [50.0, 5.0, -pi / 2],
                           [60.0, 1.0, -pi / 2], [-0.1, 1.0, -pi / 2]]
                with patch('edl_flight_computer.run.monotonic', side_effect=range(5)), \
                     patch('edl_flight_computer.run.sleep'), \
                     patch('edl_flight_computer.run.SensorReader') as reader, \
                     patch('edl_flight_computer.run.PeakTracker') as peaks, \
                     patch('edl_flight_computer.run.DropLog') as log, \
                     patch('edl_flight_computer.run.advance_state', side_effect=samples), \
                     patch('builtins.print'), \
                     patch.object(grid, 'start_landing_countdown',
                                  wraps=grid.start_landing_countdown) as start:
                    reader.return_value.read.return_value = {}
                    reader.return_value.sample_times_s = {}
                    peaks.return_value.update_peaks.return_value = []
                    peaks.return_value.get_peaks.return_value = {}
                    run_mission(self.sense, state, params, window, grid, 'unused.csv', notifier)
                    self.assertEqual(start.call_count, 1)
                    self.assertEqual(phases[1:], [phase, Phase.LANDING,
                                                  Phase.LANDING, Phase.LANDED])
                    self.assertTrue(grid.landed)
                    self.assertEqual(grid.render()[63], (0, 255, 0))
                    self.assertEqual(log.return_value.__enter__.return_value.write.call_count, 4)
                    notifier.send.assert_called_once()


if __name__ == '__main__':
    unittest.main()
