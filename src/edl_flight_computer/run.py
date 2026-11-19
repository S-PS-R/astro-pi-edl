"""Launch Earth recording (default) or the Mars simulation."""
# Ground station (laptop): events, status, live plots (--no-plot for console only)
# python -m edl_flight_computer.shared.ground_station --port 8765
# python -m edl_flight_computer.run mars
# python -m edl_flight_computer.run earth

import argparse
from .shared.phases import Phase

# UDP telemetry/event downlink, shared by both missions
GROUND_STATION = {
    'enabled': True,
    # Laptop's Wi-Fi IPv4 address when running on the Pi.
    # 127.0.0.1 sends to a ground station on this same computer (desktop simulation).
    'host': '127.0.0.1',
    'port': 8765,
}

# Fault detection: nominal (low, high) range per sensor channel; None = no limit.
# Outside for 0.5 s raises a 'fault' event; back inside for 0.5 s sends 'fault_cleared'.
# Tune temperatures against the packaged board's idle reading (Pi CPU heat raises them).
ENVIRONMENT_LIMITS = {
    'temperature_pressure': (None, 40.0),  # °C, overtemp (heat source / warm water)
    'temperature_humidity': (None, 40.0),  # °C
    'humidity': (None, 70.0),              # %RH (breath, water)
    'pressure': (900.0, 1100.0),           # hPa (sensor sanity / sealed-box leak)
}

# Approximate nominal disk-gap-band drag coefficient vs. Mach (NASA 20130012762, Fig. 9).
NASA_DRAG_CURVE_DGB = (
    (0.00, 0.615), (0.80, 0.615), (0.85, 0.611), (0.90, 0.585),
    (0.95, 0.489), (1.00, 0.594), (1.05, 0.659), (1.10, 0.680),
    (1.20, 0.705), (1.30, 0.725), (1.50, 0.750), (1.60, 0.747),
    (1.80, 0.686), (2.00, 0.577), (2.20, 0.520), (2.40, 0.495),
    (2.60, 0.467),
)

# Mars initial conditions and descent settings
MARS_SETTINGS = {
    'updates_per_sim_second': 3.0,
    'time_scale': 10.0,
    'telemetry_hz': 10.0,  # real-time downlink rate; events are sent immediately
    'ground_station': GROUND_STATION,
    'fault_limits': {
        **ENVIRONMENT_LIMITS,
        # 'gyro_x': (-1.0, 1.0),  # rad/s
        # 'gyro_y': (-1.0, 1.0),
        # 'gyro_z': (-1.0, 1.0),
    },
    'initial_conditions': {
        'phase': Phase.ENTRY,
        'altitude_m': 120000.0,
        'velocity_m_s': 3000.0,
        'fpa_deg': -1.0,
        'vehicle_params': {'CD': 1.5, 'CL': 0.5, 'mass': 3308.98, 'S': 15.0, 'Rn': 1},
    },
    'parachute': {
        'number_chutes' : 1,
        'diameter_m': 21.35,
        'drag_curve': NASA_DRAG_CURVE_DGB,
        'max_deploy_mach': 1.8,
        'max_deploy_dynamic_pressure_pa': 800,
        'min_deploy_altitude_m': 6000,
        'terminal_speed_tolerance': 0.05,
        'terminal_vertical_acceleration_tolerance_m_s2': 0.10,
        'terminal_dwell_s': 1.0,
    },
}

# Earth recording settings
EARTH_SETTINGS = {
    # Target rates: polling / CSV / ground station telemetry
    'poll_hz': 250.0,
    'csv_hz': 120.0,
    'telemetry_hz': 20.0,
    'ground_station': GROUND_STATION,
    'fault_limits': ENVIRONMENT_LIMITS,
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
