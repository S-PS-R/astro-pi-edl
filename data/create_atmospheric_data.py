"""Generate an idealized Earth-entry trajectory and Sense HAT replay.

Use create_atmospheric_data() from Python or run this file from the repo root.
No hardware, GUI, simulation, or scientific imports run at import time.
See the root README.md for units, assumptions, sources, and examples.
"""

from __future__ import annotations

import argparse
from bisect import bisect_right
import csv
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import sys

G0 = 9.80665
R_AIR = 287.05287
EARTH_RADIUS = 6_356_766.0  # US76 geopotential reference sphere, m
MU = G0 * EARTH_RADIUS**2
CEILING_M = 150_000.0
SURFACE_T_K = 288.15
SURFACE_P_PA = 101_325.0
SURFACE_RHO = SURFACE_P_PA / (R_AIR * SURFACE_T_K)
LECTURE_H_M = 1 / 1.38e-4
DEFAULT_HUMIDITY = (
    (0, 85), (500, 82), (2000, 75), (3000, 65), (5000, 45),
    (8000, 25), (11000, 10), (15000, 1), (20000, 0.1), (30000, 0),
)


@dataclass(frozen=True)
class EntryBody:
    mass_kg: float
    area_m2: float
    cd: float
    lift_to_drag: float
    nose_radius_m: float

    @property
    def ballistic_coefficient_kg_m2(self):
        return self.mass_kg / (self.cd * self.area_m2)


BODY_PRESETS = {
    'sphere': EntryBody(10, math.pi * 0.30**2 / 4, 0.47, 0, 0.15),
    'blunt_capsule': EntryBody(1000, math.pi, 1.5, 0, 0.50),
    'lifting_capsule': EntryBody(1000, math.pi, 1.5, 0.30, 0.50),
    'box': EntryBody(5, 0.20 * 0.20, 1.05, 0, 0.02),
}

# US76 calculated tables, PDAS BigTables Table 1 (geometric altitude).
# Log interpolation for p/rho, linear interpolation for kinetic T. The first
# node is calculated from the lower layers, avoiding a discontinuity at 86 km.
# Upper p and rho are independent: molecular weight changes above 86 km.
UPPER_ATMOSPHERE = (
    (90000, 186.867, 1.8435e-1, 3.4400e-6),
    (95000, 188.418, 7.5775e-2, 1.3873e-6),
    (100000, 195.081, 3.2012e-2, 5.6044e-7),
    (105000, 208.835, 1.4423e-2, 2.3325e-7),
    (110000, 240.000, 7.1493e-3, 9.6734e-8),
    (115000, 300.000, 4.0037e-3, 4.2794e-8),
    (120000, 360.000, 2.5366e-3, 2.2199e-8),
    (125000, 417.231, 1.7360e-3, 1.2918e-8),
    (130000, 469.268, 1.2503e-3, 8.1494e-9),
    (135000, 516.589, 9.3569e-4, 5.4647e-9),
    (140000, 559.627, 7.2029e-4, 3.8313e-9),
    (145000, 598.776, 5.6690e-4, 2.7805e-9),
    (150000, 634.392, 4.5422e-4, 2.0752e-9),
)

SOURCES = [
    {'title': 'Lecture 3 AFM Part 1 2026', 'pages': '16-18: lift, drag, coefficients'},
    {'title': 'Lecture 3 AFM Part 2 2026',
     'pages': '12-15: scale height; 65-75: FPA, beta, Allen-Eggers'},
    {'title': 'Lecture 4 Aerothermodynamics 2026',
     'pages': '5-10: lifting entry; 74-77: Sutton-Graves',
     'earth_sutton_graves_si': 1.7415e-4},
    {'title': 'US Standard Atmosphere 1976, NASA-TM-X-74335, Parts 1 and 4',
     'url': 'https://ntrs.nasa.gov/citations/19770009539'},
    {'title': 'PDAS calculated US76 tables, Table 1; upper-atmosphere interpolation nodes',
     'url': 'https://www.pdas.com/bigtables.html'},
]


def _finite(name, value, *, positive=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{name} must be a finite number')
    if not math.isfinite(value) or (positive and value <= 0):
        raise ValueError(f'{name} must be finite' + (' and positive' if positive else ''))
    return float(value)


def resolve_body(body='blunt_capsule', **overrides):
    """Resolve illustrative defaults; all five properties are required for custom."""
    if body not in (*BODY_PRESETS, 'custom'):
        raise ValueError(f'Unknown body {body!r}; choose {", ".join((*BODY_PRESETS, "custom"))}')
    values = asdict(BODY_PRESETS[body]) if body != 'custom' else {}
    for key, value in overrides.items():
        if key not in EntryBody.__dataclass_fields__:
            raise ValueError(f'Unknown body property {key!r}')
        if value is not None:
            values[key] = value
    missing = set(EntryBody.__dataclass_fields__) - values.keys()
    if missing:
        raise ValueError(f'Custom body requires {", ".join(sorted(missing))}')
    for key, value in values.items():
        values[key] = _finite(key, value, positive=key != 'lift_to_drag')
    if values['lift_to_drag'] < 0:
        raise ValueError('lift_to_drag must be nonnegative')
    return EntryBody(**values)


def _lower_atmosphere(altitude_m):
    """US76 hydrostatic layers, geometric to geopotential altitude conversion."""
    height = EARTH_RADIUS * altitude_m / (EARTH_RADIUS + altitude_m)
    boundaries = (0, 11000, 20000, 32000, 47000, 51000, 71000, 84852.05)
    lapse_rates = (-0.0065, 0, 0.001, 0.0028, 0, -0.0028, -0.002)
    temperature, pressure = SURFACE_T_K, SURFACE_P_PA
    for base, top, lapse in zip(boundaries, boundaries[1:], lapse_rates):
        delta = min(height, top) - base
        next_t = temperature + lapse * delta
        if lapse == 0:
            pressure *= math.exp(-G0 * delta / (R_AIR * temperature))
        else:
            pressure *= (temperature / next_t)**(G0 / (R_AIR * lapse))
        temperature = next_t
        if height <= top:
            break
    return temperature, pressure, pressure / (R_AIR * temperature)


class Atmosphere:
    """An isothermal exponential model or a tabulated/layered US76 approximation."""

    def __init__(self, model='exponential', scale_height_m=None):
        if model not in ('exponential', 'lecture', 'layered'):
            raise ValueError('atmosphere must be exponential, lecture, or layered')
        if model == 'layered' and scale_height_m is not None:
            raise ValueError('scale_height_m cannot override the layered atmosphere')
        self.model = model
        default_h = LECTURE_H_M if model == 'lecture' else R_AIR * SURFACE_T_K / G0
        self.scale_height_m = _finite('scale_height_m',
                                     default_h if scale_height_m is None else scale_height_m,
                                     positive=True)
        self.temperature_k = G0 * self.scale_height_m / R_AIR
        self.surface_pressure_pa = SURFACE_RHO * G0 * self.scale_height_m
        t, p, rho = _lower_atmosphere(86000)
        self.upper = ((86000, t, p, rho), *UPPER_ATMOSPHERE)

    def sample(self, altitude_m):
        """Return (temperature K, pressure Pa, density kg/m3, pressure scale height m)."""
        altitude_m = _finite('altitude_m', altitude_m)
        if not 0 <= altitude_m <= CEILING_M:
            raise ValueError('Atmosphere supports geometric altitudes from 0 to 150000 m')
        if self.model != 'layered':
            decay = math.exp(-altitude_m / self.scale_height_m)
            return (self.temperature_k, self.surface_pressure_pa * decay,
                    SURFACE_RHO * decay, self.scale_height_m)
        if altitude_m <= 86000:
            t, p, rho = _lower_atmosphere(altitude_m)
        else:
            i = min(bisect_right([row[0] for row in self.upper], altitude_m) - 1,
                    len(self.upper) - 2)
            lo, hi = self.upper[i], self.upper[i + 1]
            weight = (altitude_m - lo[0]) / (hi[0] - lo[0])
            t = lo[1] + weight * (hi[1] - lo[1])
            p, rho = (math.exp(math.log(lo[j]) + weight * math.log(hi[j] / lo[j]))
                      for j in (2, 3))
        gravity = MU / (EARTH_RADIUS + altitude_m)**2
        return t, p, rho, p / (rho * gravity)

    def configuration(self):
        result = {'model': self.model, 'ceiling_m': CEILING_M}
        if self.model != 'layered':
            result.update(scale_height_m=self.scale_height_m,
                          surface_temperature_k=self.temperature_k,
                          surface_pressure_pa=self.surface_pressure_pa,
                          surface_density_kg_m3=SURFACE_RHO)
        return result


def ballistic_velocity(altitude_m, entry_altitude_m, entry_velocity_m_s,
                       entry_fpa_deg, beta_kg_m2, scale_height_m, rho0=SURFACE_RHO):
    """Finite-interface Allen-Eggers reference: constant FPA/Cd, no gravity/lift."""
    sine = math.sin(math.radians(entry_fpa_deg))
    if sine >= 0 or beta_kg_m2 <= 0 or scale_height_m <= 0:
        raise ValueError('Reference requires downward FPA and positive beta/scale height')
    exponent = rho0 * scale_height_m / (2 * beta_kg_m2 * sine) * (
        math.exp(-altitude_m / scale_height_m)
        - math.exp(-entry_altitude_m / scale_height_m))
    return entry_velocity_m_s * math.exp(exponent)


def equilibrium_glide_velocity(altitude_m, density, beta_kg_m2, lift_to_drag):
    """Lecture equilibrium-glide reference, valid only for shallow lifting flight."""
    if lift_to_drag <= 0 or beta_kg_m2 <= 0 or density < 0:
        raise ValueError('Glide reference requires positive L/D and beta, nonnegative density')
    radius = EARTH_RADIUS + altitude_m
    return math.sqrt((MU / radius) / (1 + density * lift_to_drag * radius / (2 * beta_kg_m2)))


def _motion(state, body, atmosphere):
    x, y, vx, vy = state[:4]
    radius, speed = math.hypot(x, y), math.hypot(vx, vy)
    if speed < 1e-8:
        raise RuntimeError('Velocity is too small to define the prescribed stable attitude')
    altitude = min(CEILING_M, max(0.0, radius - EARTH_RADIUS))
    temperature, pressure, density, height = atmosphere.sample(altitude)
    q = 0.5 * density * speed**2
    drag = q * body.cd * body.area_m2
    lift = drag * body.lift_to_drag
    tx, ty = vx / speed, vy / speed
    # Lift direction is the outward normal for this prograde vertical-plane entry.
    fx = (-drag * tx + lift * ty) / body.mass_kg
    fy = (-drag * ty - lift * tx) / body.mass_kg
    ax, ay = fx - MU * x / radius**3, fy - MU * y / radius**3
    heat_flux = 1.7415e-4 * math.sqrt(density / body.nose_radius_m) * speed**3
    return (vx, vy, ax, ay, heat_flux), (temperature, pressure, density, height, q, drag, lift, fx, fy)


def _mounting_matrix(angles):
    """Board-to-default rotation Rz(yaw) Ry(pitch) Rx(roll), angles in degrees."""
    import numpy as np
    r, p, y = map(math.radians, angles)
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array(((cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr),
                     (sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr),
                     (-sp, cp*sr, cp*cr)))


def _validate_profile(profile):
    try:
        rows = [(float(h), float(rh)) for h, rh in profile]
    except (TypeError, ValueError) as exc:
        raise ValueError('humidity_profile must contain (altitude_m, RH_percent) pairs') from exc
    if len(rows) < 2 or rows[0][0] != 0:
        raise ValueError('Humidity profile requires at least two nodes and must start at altitude 0')
    if any(not math.isfinite(h) or not math.isfinite(rh) or h < 0 or not 0 <= rh <= 100
           for h, rh in rows) or any(b[0] <= a[0] for a, b in zip(rows, rows[1:])):
        raise ValueError('Humidity altitudes must strictly increase; RH must be finite and in [0, 100]')
    return rows


def create_atmospheric_data(
    body='blunt_capsule', entry_altitude_m=120000.0, entry_velocity_m_s=7800.0,
    entry_fpa_deg=-6.5, *, mass_kg=None, area_m2=None, cd=None,
    lift_to_drag=None, nose_radius_m=None, atmosphere='exponential',
    scale_height_m=None, humidity_profile=None, magnetic_field_ned_ut=(25, 0, 40),
    mounting_deg=(0, 0, 0), output_path=None, output_dir=None,
    sample_rate_hz=20.0, max_time_s=7200.0, rtol=1e-8, max_step_s=1.0,
):
    """Solve descent and write entry.csv and a self-contained entry.json.

    Angles are degrees, FPA is negative downward, and altitudes are geometric
    metres above the reference sphere. mounting_deg is board-to-default roll,
    pitch, yaw. Return a dictionary of absolute paths, configuration and summary.
    output_dir defaults to data/atmospheric_entry. Legacy output_path selects
    an exact replay filename; it cannot be combined with output_dir.
    NumPy/SciPy are required only when this function is called.
    """
    vehicle = resolve_body(body, mass_kg=mass_kg, area_m2=area_m2, cd=cd,
                           lift_to_drag=lift_to_drag, nose_radius_m=nose_radius_m)
    air = Atmosphere(atmosphere, scale_height_m)
    entry_altitude_m = _finite('entry_altitude_m', entry_altitude_m, positive=True)
    entry_velocity_m_s = _finite('entry_velocity_m_s', entry_velocity_m_s, positive=True)
    entry_fpa_deg = _finite('entry_fpa_deg', entry_fpa_deg)
    if entry_altitude_m > CEILING_M or not -90 <= entry_fpa_deg < 0:
        raise ValueError('Entry altitude must be <=150000 m and FPA must be in [-90, 0) degrees')
    for name, value in (('sample_rate_hz', sample_rate_hz), ('max_time_s', max_time_s),
                        ('rtol', rtol), ('max_step_s', max_step_s)):
        _finite(name, value, positive=True)
    if rtol >= 1:
        raise ValueError('rtol must be less than 1')
    profile = _validate_profile(DEFAULT_HUMIDITY if humidity_profile is None else humidity_profile)
    try:
        field = tuple(_finite('magnetic_field_ned_ut', v) for v in magnetic_field_ned_ut)
        mounting = tuple(_finite('mounting_deg', v) for v in mounting_deg)
    except TypeError as exc:
        raise ValueError('Magnetic field and mounting require three numbers each') from exc
    if len(field) != 3 or len(mounting) != 3:
        raise ValueError('Magnetic field and mounting require three numbers each')
    if output_path is not None and output_dir is not None:
        raise ValueError('Use output_dir or output_path, not both')
    if output_path is None:
        output_path = Path(output_dir if output_dir is not None else 'data/atmospheric_entry') / 'entry.csv'
    replay_path = Path(output_path).expanduser().resolve()
    if replay_path.suffix.lower() != '.csv':
        raise ValueError('output_path must end in .csv')
    try:
        import numpy as np
        from scipy.integrate import solve_ivp
    except ImportError as exc:
        raise RuntimeError('Install dependencies: python -m pip install -e .') from exc
    # Read the schema and hardware ranges from the implementation, never duplicate them.
    src = str(Path(__file__).resolve().parents[1] / 'src')
    if src not in sys.path:
        sys.path.insert(0, src)
    from sense_hat_emulator.emulator import CSV_REQUIRED, RANGES
    from sense_hat_emulator.hardware_profile import SENSOR_RATES_HZ

    gamma = math.radians(entry_fpa_deg)
    initial = [EARTH_RADIUS + entry_altitude_m, 0,
               entry_velocity_m_s * math.sin(gamma), entry_velocity_m_s * math.cos(gamma), 0]

    def rhs(t, state):
        return _motion(state, vehicle, air)[0]

    def surface(t, state):
        return math.hypot(*state[:2]) - EARTH_RADIUS

    def ceiling(t, state):
        return math.hypot(*state[:2]) - EARTH_RADIUS - CEILING_M

    surface.terminal, surface.direction = True, -1
    ceiling.terminal, ceiling.direction = True, 1
    sol = solve_ivp(rhs, (0, max_time_s), initial, method='DOP853',
                    dense_output=True, events=(surface, ceiling), rtol=rtol,
                    atol=(1e-4, 1e-4, 1e-6, 1e-6, 1e-3), max_step=max_step_s)
    if not sol.success:
        raise RuntimeError(f'Entry integration failed: {sol.message}')
    reason = 'surface_impact' if len(sol.t_events[0]) else (
        'atmosphere_exit' if len(sol.t_events[1]) else 'time_limit')
    end = float(sol.t[-1])
    if end * sample_rate_hz > 5_000_000:
        raise ValueError('Output exceeds five million samples; reduce sample rate or maximum time')
    times = np.arange(math.ceil(end * sample_rate_hz), dtype=float) / sample_rate_hz
    times = np.append(times[times < end], end)
    states = sol.sol(times).T
    downranges = EARTH_RADIUS * np.unwrap(np.arctan2(states[:, 1], states[:, 0]))
    mount = _mounting_matrix(mounting)
    replay, trajectory = [], []
    violations = {}
    for time_s, state, downrange in zip(times, states, downranges):
        x, y, vx, vy, heat_load = map(float, state)
        radius, speed = math.hypot(x, y), math.hypot(vx, vy)
        altitude = min(CEILING_M, max(0.0, radius - EARTH_RADIUS))
        derivative, env = _motion(state, vehicle, air)
        temperature, pressure, density, height, q, drag, lift, fx, fy = env
        tx, ty, rx, ry = vx / speed, vy / speed, x / radius, y / radius
        # Columns are board default X (outward lift), Y (plane normal), Z (aft).
        frame = np.array(((ty, 0, -tx), (-tx, 0, -ty), (0, 1, 0))) @ mount
        accel = frame.T @ np.array((fx, fy, 0)) / G0
        turn_rate = (vx * derivative[3] - vy * derivative[2]) / speed**2
        gyro = mount.T @ np.array((0, turn_rate, 0))
        north, east, down = field
        magnetic_inertial = np.array((-north * ry - down * rx,
                                      north * rx - down * ry, east))
        magnetic = frame.T @ magnetic_inertial * (EARTH_RADIUS / radius)**3
        rh = float(np.interp(altitude, *zip(*profile)))
        row = dict(time_s=float(time_s), temperature_humidity=temperature - 273.15,
                   temperature_pressure=temperature - 273.15, pressure=pressure / 100, humidity=rh)
        for prefix, vector in (('accel', accel), ('gyro', gyro), ('mag', magnetic)):
            row.update({f'{prefix}_{axis}': float(value) for axis, value in zip('xyz', vector)})
        for key, value in row.items():
            if key in RANGES and not RANGES[key][0] <= value <= RANGES[key][1]:
                record = violations.setdefault(key, {'count': 0, 'first_time_s': float(time_s),
                                                     'range': list(RANGES[key])})
                record['count'] += 1
        replay.append(row)
        effective_r = pressure / (density * temperature)
        trajectory.append(dict(
            time_s=float(time_s), altitude_m=altitude,
            downrange_m=float(downrange), velocity_m_s=speed,
            fpa_deg=math.degrees(math.atan2(vx * rx + vy * ry, vx * -ry + vy * rx)),
            density_kg_m3=density, pressure_pa=pressure, temperature_k=temperature,
            scale_height_m=height, gravity_m_s2=MU / radius**2,
            mach=speed / math.sqrt(1.4 * effective_r * temperature),
            dynamic_pressure_pa=q, drag_n=drag, lift_n=lift,
            specific_force_g=math.hypot(fx, fy) / G0,
            heat_flux_w_m2=derivative[4], heat_load_j_m2=heat_load,
            accel_x=row['accel_x'], accel_y=row['accel_y'], accel_z=row['accel_z'],
        ))

    def peak(key):
        row = max(trajectory, key=lambda row: row[key])
        return {'value': row[key], 'time_s': row['time_s'], 'altitude_m': row['altitude_m']}

    configuration = dict(
        body_preset=body, body=asdict(vehicle),
        ballistic_coefficient_kg_m2=vehicle.ballistic_coefficient_kg_m2,
        entry_type='ballistic' if vehicle.lift_to_drag == 0 else 'lifting',
        entry_altitude_m=entry_altitude_m, entry_velocity_m_s=entry_velocity_m_s,
        entry_fpa_deg=entry_fpa_deg, atmosphere=air.configuration(),
        humidity_profile=profile, magnetic_field_ned_ut=field, mounting_deg=mounting,
        sample_rate_hz=sample_rate_hz, max_time_s=max_time_s,
        solver={'method': 'DOP853', 'rtol': rtol, 'max_step_s': max_step_s,
                'atol': [1e-4, 1e-4, 1e-6, 1e-6, 1e-3]},
        earth_radius_m=EARTH_RADIUS, gravitational_parameter_m3_s2=MU,
    )
    summary = dict(
        termination_reason=reason, duration_s=end, samples=len(replay),
        final_altitude_m=trajectory[-1]['altitude_m'], final_velocity_m_s=trajectory[-1]['velocity_m_s'],
        heat_load_j_m2=trajectory[-1]['heat_load_j_m2'],
        sampled_peaks={key: peak(key) for key in
                       ('specific_force_g', 'dynamic_pressure_pa', 'heat_flux_w_m2')},
    )
    paths = {'replay_csv': str(replay_path),
             'metadata_json': str(replay_path.with_suffix('.json'))}
    metadata = dict(configuration=configuration, summary=summary, paths=paths,
                    sensor_profile={'nominal_rates_hz': dict(SENSOR_RATES_HZ),
                                    'ranges': RANGES,
                                    'timing_reference': 'docs/sensor_timing.md'},
                    trajectory=trajectory,
                    hardware_range_violations=violations, sources=SOURCES,
                    units={'replay': 'time s; temperature degC; pressure hPa; humidity %RH; '
                                     'acceleration g; gyro rad/s; magnetic field microtesla',
                           'trajectory': 'SI units in column names; FPA degrees; specific force and accel_x/y/z in g'},
                    assumptions=[
                        'Illustrative generic presets, constant mass/Cd/L/D; box nose radius is approximate.',
                        'Spherical nonrotating Earth; vertical-plane prograde entry, zero bank, no winds.',
                        'Exponential atmosphere uses constant reference gravity for hydrostatics; motion uses inverse-square gravity.',
                        'Layered option approximates US76; 86-150 km uses tabulated interpolation.',
                        'Mach uses gamma=1.4 and effective gas constant p/(rho*T); high-altitude estimate only.',
                        'Ambient temperature/pressure proxies, not enclosure, shock-layer, or sensor temperatures.',
                        'RH is an assumed weather profile; last node holds above its altitude; near-vacuum RH is a placeholder.',
                        'Board follows velocity with fixed mounting; no solved rotational dynamics or tumble.',
                        'Illustrative local NED magnetic field, inverse-cube scaling; north is downrange.',
                        'Sutton-Graves cold-wall stagnation convective heating estimate, not a thermal/TPS survival model.',
                        'No radiative shock heating, ablation, parachutes, propulsion, impact pulse, or post-impact samples.',
                        'CSV stores physical targets without noise or clipping; emulator may display OVR outside hardware ranges.',
                        'Peak statistics are sampled; cumulative heat load is integrated with the ODE.',
                    ])
    # All validation and integration finish before any output files are opened.
    replay_path.parent.mkdir(parents=True, exist_ok=True)
    with replay_path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_REQUIRED)
        writer.writeheader()
        writer.writerows(replay)
    Path(paths['metadata_json']).write_text(json.dumps(metadata, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    return dict(paths=paths, configuration=configuration,
                termination_reason=reason, summary=summary,
                hardware_range_violations=violations)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--list-bodies', action='store_true', help='Print illustrative preset properties and exit')
    parser.add_argument('--body', choices=(*BODY_PRESETS, 'custom'), default='blunt_capsule')
    parser.add_argument('--entry-altitude-m', type=float, default=120000)
    parser.add_argument('--entry-velocity-m-s', type=float, default=7800)
    parser.add_argument('--entry-fpa-deg', type=float, default=-6.5)
    for option in ('mass-kg', 'area-m2', 'cd', 'lift-to-drag', 'nose-radius-m', 'scale-height-m'):
        parser.add_argument('--' + option, type=float)
    parser.add_argument('--atmosphere', choices=('exponential', 'lecture', 'layered'), default='exponential')
    parser.add_argument('--humidity-profile', type=Path, help='JSON array of [altitude_m, RH_percent] nodes')
    parser.add_argument('--magnetic-field-ned-ut', nargs=3, type=float, default=(25, 0, 40), metavar=('N', 'E', 'D'))
    parser.add_argument('--mounting-deg', nargs=3, type=float, default=(0, 0, 0), metavar=('ROLL', 'PITCH', 'YAW'))
    output = parser.add_mutually_exclusive_group()
    output.add_argument('--output-dir', help='Run folder (default: data/atmospheric_entry)')
    output.add_argument('--output-path', help='Legacy option: exact replay CSV filename')
    parser.add_argument('--sample-rate-hz', type=float, default=20)
    parser.add_argument('--max-time-s', type=float, default=7200)
    args = vars(parser.parse_args(argv))
    if args.pop('list_bodies'):
        print('Illustrative assumptions; not validated mission vehicle data.')
        for name, vehicle in BODY_PRESETS.items():
            print(name, json.dumps({**asdict(vehicle),
                                   'ballistic_coefficient_kg_m2': vehicle.ballistic_coefficient_kg_m2}))
        return 0
    try:
        if args['humidity_profile'] is not None:
            args['humidity_profile'] = json.loads(args['humidity_profile'].read_text(encoding='utf-8'))
        result = create_atmospheric_data(**args)
    except (ValueError, RuntimeError, OSError) as exc:
        parser.exit(1, f'Error: {exc}\n')
    print(json.dumps(result, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
