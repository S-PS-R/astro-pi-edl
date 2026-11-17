"""Launch Earth recording (default) or the Mars simulation."""
# python -m edl_flight_computer.run mars
# python -m edl_flight_computer.run earth
# python -m edl_flight_computer.earth.live_plot --port 8765

import argparse
from .shared.phases import Phase

# Mars initial conditions
MARS_SETTINGS = {
    'updates_per_sim_second': 3.0,
    'time_scale': 10.0,
    'initial_conditions': {
        'phase': Phase.ENTRY,
        'altitude_m': 120000.0,
        'velocity_m_s': 3000.0,
        'fpa_deg': -1.0,
        'vehicle_params': {'CD': 1.5, 'CL': 0.5, 'mass': 8000.0, 'S': 15.0, 'Rn': 1, 'target_fpa_deg': -5.0},
    },
}

# Earth recording settings
EARTH_SETTINGS = {
    'live_plotting': False,
    'laptop_ip': '',
    'stream_port': 8765,
    'use_emulator': False,
    'ekf_use_pressure': True,  # Vented package only
    'detection': {
        'calibration_s': 2.0,
        'release_fraction': 0.7,
        'release_duration_s': 0.025,
        'accel_tolerance_g': 0.10,
        'accel_std_g': 0.03,
        'gyro_tolerance_rad_s': 0.10,
        'settling_window_s': 0.25,
        'settling_duration_s': 5.0,
        'max_imu_gap_s': 0.10,
    },
}


def main(argv=None):
    # Mission selection
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mission', nargs='?', choices=('earth', 'mars'), default='earth')
    args = parser.parse_args(argv)
    if args.mission == 'earth':
        from .earth.drop import main as run
        return run(EARTH_SETTINGS)
    else:
        from .mars.sim import main as run
        return run(MARS_SETTINGS)


if __name__ == '__main__':
    raise SystemExit(main())
