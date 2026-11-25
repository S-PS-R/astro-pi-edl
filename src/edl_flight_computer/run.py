"""Launch Earth recording (default) or the Mars simulation."""
# Ground station (laptop): events, status, live plots (--no-plot for console only)
# python -m edl_flight_computer.shared.ground_station --port 8765
# python -m edl_flight_computer.run mars
# python -m edl_flight_computer.run earth

import argparse
from .mars.model import sphere_cone_areas
from .shared.phases import Phase

# UDP telemetry/event downlink, shared by both missions
GROUND_STATION = {
    'enabled': True,
    # Laptop's Wi-Fi IPv4 address when running on the Pi.
    # 127.0.0.1 sends to a ground station on this same computer (desktop simulation).
    # 10.0.0.88 sends to a ground station on the laptop (Pi flight) 
    'host': '10.0.0.88',
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

# Mars 2020 geometry drawing, NASA Planetary Mission Entry Vehicles Guide, Mars 2020 page:
# https://www.nasa.gov/wp-content/uploads/2023/08/final-planetary-mission-entry-vehicles-quick-reference-guide-v4.1-.pdf
MARS_AEROSHELL = {
    'diameter_m': 4.50,
    'half_angle_deg': 70.0,
    'nose_radius_m': 1.125,
    'shoulder_radius_m': 0.126,
}
MARS_ENTRY_AREA_M2, _ = sphere_cone_areas(**MARS_AEROSHELL)

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
        'velocity_m_s': 5330,
        'fpa_deg': -15.0,
        'entry_site': {'latitude_deg': 48.19735, 'longitude_deg': 173.30,
                       'heading_deg': 76.0, 'bank_deg': 0.0},
        # Representative constants inferred from the NASA guide's trim L/D and ballistic coefficient.
        'vehicle_params': {'CD': 1.46, 'CL': 0.36, 'mass': 3300,
                           'S': MARS_ENTRY_AREA_M2, 'Rn': MARS_AEROSHELL['nose_radius_m']},
    },
    'entry_guidance': {
        'enabled': True,
        'waypoint_east_offset_deg': 0.90,
        # Mars 2020 inspired bank magnitudes. The sim uses a fixed bank direction during range control.
        'range_bank_schedule': ((2200.0, 45.0), (4400.0, 60.0)),  # ground speed in m/s, bank in degrees
        'max_bank_deg': 30.0, 'heading_gain': 5.0,
    },
    'parachute': {
        'diameter_m': 17,
        'drag_curve': NASA_DRAG_CURVE_DGB,
        'deploy_mach': 1.8,
        'opening_factor': 1.4,
        'terminal_speed_tolerance': 0.07,
        'terminal_vertical_acceleration_tolerance_m_s2': 0.25,
        'terminal_dwell_s': 1.0,
    },
    'retro_prop': {
        'landing_target': {'latitude_deg': 49.02, 'longitude_deg': 188.15, 'tolerance_m': 1000.0,
                           'response_time_s': 4.0, 'max_divert_speed_m_s': 18.0,
                           'stop_divert_altitude_m': 180.0},
        # The shield mass includes 274.90256 kg of structure and 142.43935 kg of TPS.
        'heatshield': {'mass_kg': 417.34191032, 'delay_after_chute_s': 20.0,
                       'cd': 1.1, 'area_m2': 15.0},
        # Backshell separation also releases the 13.6 kg parachute.
        'backshell': {'mass_kg': 206.4734, 'separation_altitude_m': 2100.0,
                      'lander_cd': 1.0, 'lander_area_m2': 8.0},

        'propulsion': {'propellant_kg': 366.6, 'max_thrust_n': 22075, 'specific_impulse_s': 225.0,
                       'ignition_delay_s': 0.0, 'horizontal_capture_m_s': 0.1,
                       'horizontal_braking_m_s2': 3.0, 'vertical_braking_m_s2': 3.12,
                       'response_time_s': 1.0, 'target_descent_speed_m_s': 2.0,
                       'max_touchdown_vertical_m_s': 2.5, 'max_touchdown_horizontal_m_s': 0.2},
    },
    'wind': {
        'enabled': True, 'seed': 40,
        'min_surface_speed_m_s': 2.0, 'max_surface_speed_m_s': 10.0,
        'gust_m_s': 3.0, 'gust_interval_s': 20.0, 'scale_height_m': 20000.0,
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
