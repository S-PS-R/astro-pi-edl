"""Atmosphere, flight equations, integration, and observed peak tracking."""

from bisect import bisect_left
from dataclasses import dataclass
from math import asin, atan2, ceil, cos, degrees, exp, hypot, pi, radians, sin, sqrt

import numpy as np

from ..shared.phases import Phase
from .entry_guidance import EntryGuidance, WindProfile
from .powered_descent import PoweredDescent

MAX_STEP_S = 0.02

# Mars constants
MARS = {
    "name": "mars",
    "radius": 3389500.0, #m
    "mu": 4.2828e13, #m³/s²
    "rotation_rad_s": 2 * pi / (24.6229 * 3600),
    "T0": 214.0, #K
    "P0": 636.0, #Pa
    "gas_constant": 188.9,  # Pure CO₂, J/(kg·K)
    "C_SG": 1.9027e-4,
}

def ambient_atmosphere(h, planet):
    """Return temperature [K], pressure [Pa], density [kg/m³], g_ref, H [m]."""
    Rp = planet["radius"]
    g_ref = planet["mu"] / Rp**2
    T = planet["T0"]
    H = planet["gas_constant"] * T / g_ref
    pressure = planet["P0"] * exp(-h / H)
    density = pressure / (planet["gas_constant"] * T)
    return T, pressure, density, g_ref, H


def parachute_drag_coefficient(mach, curve):
    """Interpolate the tabulated drag coefficient and clamp outside its range."""
    index = bisect_left(curve, mach, key=lambda point: point[0]) # nice function which finds the leftmost insertion index for a target value in a sorted list
    if index == 0:
        return curve[0][1]
    if index == len(curve):
        return curve[-1][1]
    lower_mach, lower_cd = curve[index - 1]
    upper_mach, upper_cd = curve[index]
    fraction = (mach - lower_mach) / (upper_mach - lower_mach)
    return lower_cd + fraction * (upper_cd - lower_cd)


def calculate_state(X, planet, vehicle_params, *, parachute=None, canopy_deployed=False):
    """Calculate state derivatives, including downrange when supplied in X."""
    h, V, gamma = X[:3]
    T, pressure, rho, _, _ = ambient_atmosphere(h, planet)
    Rp = planet["radius"]
    g = planet["mu"] / (Rp + h)**2
    CD, CL = vehicle_params["CD"], vehicle_params["CL"]
    m, S = vehicle_params["mass"], vehicle_params["S"]

    sin_gamma, cos_gamma = sin(gamma), cos(gamma)
    dynamic_pressure = 0.5 * rho * V**2
    gas_constant = planet['gas_constant']
    # CO₂ specific heat in J/(kg·K), NASA/TM-2019-220019, Eq. 7.
    cp = 517.7 + 1.1 * T
    heat_capacity_ratio = cp / (cp - gas_constant)
    speed_of_sound = sqrt(heat_capacity_ratio * gas_constant * T)
    mach = V / speed_of_sound
    vehicle_drag_area = S * CD
    parachute_drag_area = 0.0
    parachute_cd = None
    effective_cl = CL  # Fixed lift-up entry
    equilibrium_speed = None
    if canopy_deployed:
        # ASTE 577 Lecture 6, PDF p. 47: sum the vehicle and canopy drag areas.
        canopy_area = pi * parachute['diameter_m']**2 / 4
        parachute_cd = parachute_drag_coefficient(mach, parachute['drag_curve'])
        parachute_drag_area = canopy_area * parachute_cd
        effective_cl = 0.0
        total_drag_area = vehicle_drag_area + parachute_drag_area
        if sin_gamma < 0 and rho * total_drag_area > 0:
            equilibrium_speed = sqrt(-2 * m * g * sin_gamma / (rho * total_drag_area))

    vehicle_drag = dynamic_pressure * vehicle_drag_area
    parachute_drag = dynamic_pressure * parachute_drag_area
    total_drag = vehicle_drag + parachute_drag
    L = dynamic_pressure * S * effective_cl

    # Flight equations
    h_dot = V * sin_gamma
    V_dot = -total_drag / m - g * sin_gamma
    gamma_dot = 0.0
    if V > 0.0:
        gamma_dot = L / (m * V) + (V / (Rp + h) - g / V) * cos_gamma
    vertical_acceleration = V_dot * sin_gamma + V * cos_gamma * gamma_dot

    # Convective heating
    Rn = vehicle_params['Rn']
    C_SG = planet['C_SG']
    heating = C_SG * sqrt(rho / Rn) * V**3 / 1e3

    values = {
        'altitude_m': h,
        'velocity_m_s': V,
        'flight_path_angle_rad': gamma,
        'temperature_k': T,
        'pressure_pa': pressure,
        'density_kg_m3': rho,
        'gravity_m_s2': g,
        'dynamic_pressure_pa': dynamic_pressure,
        'bank_angle_deg': None if canopy_deployed else 0.0,
        'vertical_velocity_m_s': h_dot,
        'acceleration_m_s2': V_dot,
        'g_load': V_dot / g,
        'heating_kw_m2': heating,
        'mach': mach,
        'equilibrium_speed_m_s': equilibrium_speed,
        'vertical_acceleration_m_s2': vertical_acceleration,
        'vehicle_cd': CD,
        'parachute_cd': parachute_cd,
        # Combine drag areas on the vehicle reference area for a common Cd.
        'effective_cd': (vehicle_drag_area + parachute_drag_area) / S,
        'effective_cl': effective_cl,
        'vehicle_drag_n': vehicle_drag,
        'parachute_drag_n': parachute_drag,
        'total_drag_n': total_drag,
    }
    derivatives = [h_dot, V_dot, gamma_dot]
    if len(X) == 4:
        # Project the angular rate of planar flight onto the planet's surface.
        values['downrange_m'] = X[3]
        derivatives.append(Rp * V * cos_gamma / (Rp + h))
    return np.array(derivatives), values


class SpatialFlight:
    """Point-mass dynamics in Mars-fixed Cartesian coordinates."""

    def __init__(self, planet, latitude_deg, longitude_deg, heading_deg, bank_deg=0.0):
        lat, lon, heading = map(radians, (latitude_deg, longitude_deg, heading_deg))
        self.up = np.array((cos(lat) * cos(lon), cos(lat) * sin(lon), sin(lat)))
        east = np.array((-sin(lon), cos(lon), 0.0))
        north = np.array((-sin(lat) * cos(lon), -sin(lat) * sin(lon), cos(lat)))
        self.forward = cos(heading) * north + sin(heading) * east
        self.left = sin(heading) * north - cos(heading) * east
        self.omega = planet.get('rotation_rad_s', 0.0)
        self.bank_deg = bank_deg
        self.radius = planet['radius']

    def initial_vector(self, altitude, inertial_speed, gamma):
        position = (self.radius + altitude) * self.up
        vx, vy, vz = inertial_speed * (cos(gamma) * self.forward + sin(gamma) * self.up)
        x, y, _ = position
        return np.array((*position, vx + self.omega * y, vy - self.omega * x, vz))

    def calculate(self, vector, planet, vehicle, parachute, canopy_deployed, *,
                  landing=None, phase=None, guidance=None, wind=None, time_s=0.0):
        x, y, z, vx, vy, vz = vector[:6]
        radius = hypot(x, y, z)
        radial = (x / radius, y / radius, z / radius)
        flat = landing is not None and landing.up is not None
        ux, uy, uz = landing.up if flat else radial
        altitude = landing.altitude(vector[:3]) if flat else radius - self.radius
        speed = hypot(vx, vy, vz)
        vertical_speed = vx * ux + vy * uy + vz * uz
        horizontal_speed = hypot(uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx)
        avx, avy, avz = vx, vy, vz
        wind_east = wind_north = 0.0
        if wind is not None:
            wind_east, wind_north = wind.components(time_s, altitude)
            longitude = atan2(y, x)
            east_x, east_y = -sin(longitude), cos(longitude)
            rx, ry, rz = radial
            # North is up × east, using the planet-fixed geographic frame.
            avx -= wind_east * east_x - wind_north * rz * east_y
            avy -= wind_east * east_y + wind_north * rz * east_x
            avz -= wind_north * (rx * east_y - ry * east_x)
        air_speed = hypot(avx, avy, avz)
        lx, ly, lz = uy * avz - uz * avy, uz * avx - ux * avz, ux * avy - uy * avx
        air_horizontal = hypot(lx, ly, lz)
        gamma = atan2(avx * ux + avy * uy + avz * uz, air_horizontal)
        _, values = calculate_state((altitude, air_speed, gamma), planet, vehicle,
                                    parachute=parachute, canopy_deployed=canopy_deployed)

        gravity = values['gravity_m_s2']
        omega = self.omega
        # Gravity, Coriolis, and centrifugal acceleration in the rotating frame.
        ax = -gravity * ux + 2 * omega * vy + omega**2 * x
        ay = -gravity * uy - 2 * omega * vx + omega**2 * y
        az = -gravity * uz
        if flat:
            ax, ay, az = -gravity * landing.up
        mass = vehicle['mass']
        bank_deg = self.bank_deg
        if guidance is not None and phase is Phase.ENTRY and horizontal_speed > 0:
            bank_deg = guidance.command(np.asarray(radial), vector[3:6])
        if air_speed > 0:
            along_path = (ax * avx + ay * avy + az * avz) / air_speed
            if values['equilibrium_speed_m_s'] is not None:
                drag_area = values['effective_cd'] * vehicle['S']
                values['equilibrium_speed_m_s'] = (
                    sqrt(2 * mass * along_path / (values['density_kg_m3'] * drag_area))
                    if along_path > 0 else None)
            drag = values['total_drag_n'] / (mass * air_speed)
            ax, ay, az = ax - drag * avx, ay - drag * avy, az - drag * avz
            if air_horizontal > air_speed * 1e-12 and not canopy_deployed and not flat:
                # v × (up × v) is lift-up. Positive bank tilts lift to the right.
                lift = values['dynamic_pressure_pa'] * vehicle['S'] * values['effective_cl'] / mass
                bank = radians(bank_deg)
                up_scale = lift * cos(bank) / (air_speed * air_horizontal)
                right_scale = -lift * sin(bank) / air_horizontal
                ax += up_scale * (avy * lz - avz * ly) + right_scale * lx
                ay += up_scale * (avz * lx - avx * lz) + right_scale * ly
                az += up_scale * (avx * ly - avy * lx) + right_scale * lz

        fuel_flow = thrust_n = 0.0
        if flat:
            force, fuel_flow = landing.thrust(phase, altitude, vector[:3], vector[3:6], (ax, ay, az), mass, vector[6],
                                              hold_horizontal=wind is not None)
            thrust_n = np.linalg.norm(force)
            ax, ay, az = np.array((ax, ay, az)) + force / mass
        speed_dot = (ax * vx + ay * vy + az * vz) / speed if speed > 0 else 0.0
        values.update(
            latitude_deg=degrees(atan2(z, hypot(x, y))), longitude_deg=degrees(atan2(y, x)),
            downrange_m=self.radius * atan2(np.dot(radial, self.forward), np.dot(radial, self.up)),
            crossrange_m=self.radius * asin(max(-1.0, min(1.0, np.dot(radial, self.left)))),
            bank_angle_deg=None if (canopy_deployed or flat) else bank_deg,
            ground_speed_m_s=speed, wind_east_m_s=wind_east, wind_north_m_s=wind_north,
            vertical_velocity_m_s=vertical_speed,
            vertical_acceleration_m_s2=ax * ux + ay * uy + az * uz + (0 if flat else horizontal_speed**2 / radius),
            acceleration_m_s2=speed_dot, g_load=speed_dot / gravity,
        )
        derivatives = (vx, vy, vz, ax, ay, az)
        if landing is not None:
            values.update(mass_kg=mass, propellant_kg=vector[6], thrust_n=thrust_n,
                          propellant_flow_kg_s=fuel_flow, horizontal_speed_m_s=horizontal_speed)
            if landing.target is not None:
                values['landing_target_error_m'] = landing.target_error(radial)
            derivatives += (-fuel_flow,)
        return np.array(derivatives), values


def advance_state(X, dt, planet, vehicle_params, max_step=MAX_STEP_S, *,
                  parachute=None, canopy_deployed=False):
    """Integrate elapsed time with bounded Euler steps; stop at surface contact."""
    X = np.array(X, dtype=float, copy=True)
    if dt <= 0 or X[0] <= 0:
        return X
    steps = max(1, ceil(dt / max_step))
    step = dt / steps

    def derivatives(vector):
        return calculate_state(vector, planet, vehicle_params, parachute=parachute,
                               canopy_deployed=canopy_deployed)[0]

    for _ in range(steps):
        updated = X + step * derivatives(X)
        if updated[0] <= 0:
            # Ground crossing
            fraction = X[0] / (X[0] - updated[0])
            X += fraction * (updated - X)
            X[0] = 0.0
            return X
        X = updated
    return X


@dataclass(frozen=True)
class FlightEvent:
    name: str
    time_s: float
    values: dict
    before_values: dict | None = None
    phase: Phase | None = None


class MarsTrajectory:
    """Advance the Mars physics and latch flight events in simulation time."""

    def __init__(self, vector, planet, vehicle_params, parachute, phase=Phase.ENTRY, *,
                 entry_site=None, powered_landing=None, entry_guidance=None, wind=None):
        self.spatial = SpatialFlight(planet, **entry_site) if entry_site is not None else None
        if self.spatial is None and (entry_guidance is not None or wind is not None):
            raise ValueError('Entry guidance and wind require an entry site')
        self.guidance = EntryGuidance(**entry_guidance) if entry_guidance is not None else None
        self.wind = WindProfile(**wind) if wind is not None else None
        if self.spatial is not None:
            self.vector = self.spatial.initial_vector(*vector)
        else:
            self.vector = np.array((*vector, 0.0) if len(vector) == 3 else vector, dtype=float)
        self.planet = planet
        self.vehicle_params = vehicle_params
        self.landing = None
        if powered_landing is not None:
            if self.spatial is None or parachute is None:
                raise ValueError('Powered landing requires an entry site and a parachute')
            if phase not in (Phase.ENTRY, Phase.PARACHUTE_DESCENT, Phase.TERMINAL_DESCENT):
                raise ValueError('Powered landing must start at entry or attached to the parachute')
            propellant = powered_landing['propulsion']['propellant_kg']
            dry_mass = (vehicle_params['mass'] - powered_landing['heatshield']['mass_kg']
                        - powered_landing['backshell']['mass_kg'] - propellant)
            if propellant < 0 or dry_mass <= 0:
                raise ValueError('Entry mass must cover the propellant, jettisoned hardware and dry lander')
            self.landing = PoweredDescent(powered_landing, self.spatial.radius)
            self.vector = np.append(self.vector, propellant)
            self.dry_attached_mass = vehicle_params['mass'] - propellant
        self.parachute = parachute
        self.phase = phase
        self.canopy_deployed = phase in (Phase.PARACHUTE_DESCENT, Phase.TERMINAL_DESCENT,
                                         Phase.HEATSHIELD_JETTISONED)
        if self.landing is not None and self.canopy_deployed:
            self.landing.chute_time_s = 0.0
        self.time_s = 0.0
        self.terminal_since_s = None
        self.terminal_reported = phase is Phase.TERMINAL_DESCENT
        self.landing_warned = False
        self.values = self._calculate(self.vector)[1]
        if self.values['altitude_m'] <= 0:
            self.phase = Phase.LANDED

    def _calculate(self, vector, time_s=None):
        if self.spatial is not None:
            vehicle = self.vehicle_params
            if self.landing is not None and self.landing.up is not None:
                vehicle = dict(vehicle, mass=self.dry_attached_mass + vector[6])
            return self.spatial.calculate(vector, self.planet, vehicle, self.parachute,
                                          self.canopy_deployed, landing=self.landing, phase=self.phase,
                                          guidance=self.guidance, wind=self.wind,
                                          time_s=self.time_s if time_s is None else time_s)
        return calculate_state(vector, self.planet, self.vehicle_params,
                               parachute=self.parachute, canopy_deployed=self.canopy_deployed)

    def _derivatives(self, vector):
        return self._calculate(vector)[0]

    def _sample(self, duration):
        derivatives = self._derivatives(self.vector)
        vector = self.vector + duration * derivatives
        if self.landing is not None and vector[6] < 0:
            # Stop thrust exactly when the remaining propellant is used up.
            burn_s = -self.vector[6] / derivatives[6]
            vector = self.vector + burn_s * derivatives
            vector[6] = 0.0
            vector += (duration - burn_s) * self._calculate(vector, self.time_s + burn_s)[0]
        return vector, self._calculate(vector, self.time_s + duration)[1]

    def _first_crossing(self, duration, endpoint, condition):
        """Locate a false-to-true crossing bracketed by the current state and endpoint."""
        low = 0.0
        while duration - low > 1e-6:
            midpoint = (low + duration) / 2
            sample = self._sample(midpoint)
            if condition(sample[1]):
                duration, endpoint = midpoint, sample
            else:
                low = midpoint
        return duration, endpoint

    def _deploy_conditions(self, values):
        return (
            values['mach'] <= self.parachute['max_deploy_mach'],
            values['dynamic_pressure_pa'] <= self.parachute['max_deploy_dynamic_pressure_pa'],
            values['altitude_m'] >= self.parachute['min_deploy_altitude_m'],
            values['vertical_velocity_m_s'] < 0,
        )

    def _find_deployment(self, duration, endpoint):
        before = self._deploy_conditions(self.values)
        if all(before):
            return 0.0, (self.vector, self.values)
        after = self._deploy_conditions(endpoint[1])
        candidates = []
        for index, (was_eligible, is_eligible) in enumerate(zip(before, after)):
            if not was_eligible and is_eligible:
                # Locate each opening boundary so a closing boundary in the same
                # substep cannot hide a brief overlap of the deployment conditions.
                time, sample = self._first_crossing(
                    duration, endpoint, lambda values: self._deploy_conditions(values)[index])
                if sample[1]['altitude_m'] > 0 and all(self._deploy_conditions(sample[1])):
                    candidates.append((time, sample))
        return min(candidates, key=lambda crossing: crossing[0], default=None)

    def _terminal_eligible(self, values):
        equilibrium_speed = values['equilibrium_speed_m_s']
        return (
            equilibrium_speed is not None
            and abs(values['velocity_m_s'] - equilibrium_speed)
            <= self.parachute['terminal_speed_tolerance'] * equilibrium_speed
            and abs(values['vertical_acceleration_m_s2'])
            <= self.parachute['terminal_vertical_acceleration_tolerance_m_s2']
        )

    # terminal descent dwelling detection
    def _record_terminal(self, duration, endpoint, events):
        eligible_at_start = self._terminal_eligible(self.values)
        eligible_at_end = self._terminal_eligible(endpoint[1])
        if not eligible_at_start:
            self.terminal_since_s = None
            if not eligible_at_end:
                return
        if self.terminal_since_s is None:
            start = 0.0
            if not eligible_at_start:
                start, _ = self._first_crossing(duration, endpoint, self._terminal_eligible)
            self.terminal_since_s = self.time_s + start

        deadline = self.terminal_since_s + self.parachute['terminal_dwell_s']
        if deadline <= self.time_s + duration:
            offset = max(0.0, deadline - self.time_s)
            _, values = self._sample(offset)
            if values['altitude_m'] > 0 and self._terminal_eligible(values):
                self.terminal_reported = True
                if self.landing is None:
                    self.phase = Phase.TERMINAL_DESCENT
                events.append(FlightEvent('terminal_descent', deadline, values))
        if not eligible_at_end:
            self.terminal_since_s = None

    @staticmethod
    def _landing_warning_due(values):
        downward_speed = -values['vertical_velocity_m_s']
        return downward_speed > 0 and values['altitude_m'] <= 10.0 * downward_speed

    def _record_landing_warning(self, duration, endpoint, events):
        if self.phase is not Phase.VERTICAL_DESCENT or self.landing_warned:
            return
        if self._landing_warning_due(self.values):
            time, values = 0.0, self.values
        elif self._landing_warning_due(endpoint[1]):
            time, (_, values) = self._first_crossing(duration, endpoint, self._landing_warning_due)
        else:
            return
        if values['altitude_m'] > 0:
            self.landing_warned = True
            events.append(FlightEvent('landing_warning', self.time_s + time, values))

    def _staging_transition(self, duration, endpoint):
        """Return the first applicable staging event and its integration boundary."""
        if self.landing is None:
            return None
        settings = self.landing.settings
        delay = None
        if self.phase in (Phase.PARACHUTE_DESCENT, Phase.TERMINAL_DESCENT):
            name, phase = 'heatshield_jettison', Phase.HEATSHIELD_JETTISONED
            delay = self.landing.chute_time_s + settings['heatshield']['delay_after_chute_s'] - self.time_s
        elif self.phase is Phase.HEATSHIELD_JETTISONED:
            name, phase = 'backshell_separation', Phase.BACKSHELL_SEPARATED
            s = settings['backshell']

            def due(values):
                return (values['altitude_m'] <= s['separation_altitude_m']
                        and values['ground_speed_m_s'] <= s['max_separation_speed_m_s']
                        and values['vertical_velocity_m_s'] < 0)
        elif self.phase is Phase.BACKSHELL_SEPARATED:
            name, phase = 'powered_descent_start', Phase.POWERED_DESCENT
            delay = self.landing.separation_time_s + settings['propulsion']['ignition_delay_s'] - self.time_s
        elif self.phase is Phase.POWERED_DESCENT:
            name, phase = 'vertical_descent_start', Phase.VERTICAL_DESCENT

            def due(values):
                target = self.landing.target
                aligned = (target is None or values['landing_target_error_m'] <= target['tolerance_m']
                           or values['altitude_m'] <= target['stop_divert_altitude_m'])
                return aligned and values['horizontal_speed_m_s'] <= settings['propulsion']['horizontal_capture_m_s']
        else:
            return None

        if delay is not None:
            if delay > duration:
                return None
            time = max(0.0, delay)
            sample = self._sample(time)
        elif due(self.values):
            time, sample = 0.0, (self.vector, self.values)
        elif due(endpoint[1]):
            time, sample = self._first_crossing(duration, endpoint, due)
        else:
            return None
        if sample[1]['altitude_m'] > 0:
            return name, phase, time, sample

    def _apply_transition(self, phase):
        self.phase = phase
        if phase is Phase.PARACHUTE_DESCENT:
            self.canopy_deployed = True
            if self.landing is not None:
                self.landing.chute_time_s = self.time_s
        elif phase is Phase.HEATSHIELD_JETTISONED:
            shield = self.landing.settings['heatshield']
            self.dry_attached_mass -= shield['mass_kg']
            self.vehicle_params = dict(self.vehicle_params, mass=self.vehicle_params['mass'] - shield['mass_kg'],
                                       CD=shield['cd'], S=shield['area_m2'])
            self.terminal_since_s = None
        elif phase is Phase.BACKSHELL_SEPARATED:
            shell = self.landing.settings['backshell']
            self.landing.separate(self.vector[:3], self.time_s)
            self.dry_attached_mass -= shell['mass_kg']
            self.vehicle_params = dict(self.vehicle_params, CD=shell['lander_cd'], S=shell['lander_area_m2'], CL=0.0)
            self.canopy_deployed = False
        self.values = self._calculate(self.vector)[1]

    def _contact_phase(self):
        if self.landing is None:
            return Phase.LANDED
        s = self.landing.settings['propulsion']
        soft = (self.phase is Phase.VERTICAL_DESCENT
                and abs(self.values['vertical_velocity_m_s']) <= s['max_touchdown_vertical_m_s']
                and self.values['horizontal_speed_m_s'] <= s['max_touchdown_horizontal_m_s'])
        return Phase.LANDED if soft else Phase.IMPACT

    def advance(self, dt):
        """Advance at most dt seconds and return events at their crossing states."""
        events = []
        end_time = self.time_s + dt
        while self.time_s < end_time and self.phase not in (Phase.LANDED, Phase.IMPACT):
            duration = min(MAX_STEP_S, end_time - self.time_s)
            endpoint = self._sample(duration)
            transition = None
            if self.phase is Phase.ENTRY and self.parachute is not None:
                deployment = self._find_deployment(duration, endpoint)
                if deployment is not None:
                    transition = ('parachute_deploy', Phase.PARACHUTE_DESCENT, *deployment)
            else:
                transition = self._staging_transition(duration, endpoint)
            if transition is not None:
                name, phase, duration, endpoint = transition
            elif endpoint[1]['altitude_m'] <= 0:
                duration, endpoint = self._first_crossing(
                    duration, endpoint, lambda values: values['altitude_m'] <= 0)
                contact_vector, _ = endpoint
                if self.landing is not None and self.landing.up is not None:
                    contact_vector[:3] -= self.landing.altitude(contact_vector[:3]) * self.landing.up
                elif self.spatial is not None:
                    contact_vector[:3] *= self.spatial.radius / hypot(*contact_vector[:3])
                else:
                    contact_vector[0] = 0.0
                contact_values = self._calculate(contact_vector, self.time_s + duration)[1]
                contact_values['altitude_m'] = 0.0
                endpoint = contact_vector, contact_values

            if self.canopy_deployed and not self.terminal_reported:
                self._record_terminal(duration, endpoint, events)
            self._record_landing_warning(duration, endpoint, events)
            self.time_s += duration
            self.vector, self.values = endpoint

            if transition is not None:
                before_values = self.values
                self._apply_transition(phase)
                events.append(FlightEvent(name, self.time_s, self.values, before_values, phase))
            elif self.values['altitude_m'] <= 0:
                self.phase = self._contact_phase()

        return sorted(events, key=lambda event: event.time_s)


class PeakTracker:
    """Confirm maxima after a sustained 1% decrease, with one entry braking event."""
    CHANNELS = {
        'peak_acceleration': ('acceleration_m_s2', 'm/s^2'),
        'peak_heating': ('heating_kw_m2', 'kW/m^2'),
        'peak_velocity': ('velocity_m_s', 'm/s'),
    }

    def __init__(self):
        self.drop_fraction = 0.01
        self.confirmation_s = 1.0
        self.peaks = {}
        self.decline_started_at = {}

    def update_peaks(self, mission_time_s, flight_values, *, track_entry_deceleration=True):
        events = []
        for name, (key, units) in self.CHANNELS.items():
            peak = self.peaks.get(name)
            braking = name == 'peak_acceleration'
            if braking and (not track_entry_deceleration or (peak and peak['confirmed'])):
                continue
            value = flight_values[key]
            # Negative acceleration is braking
            strength = -value if braking else value
            # Initial sample
            if strength <= 0 and name not in self.peaks:
                continue
            peak_strength = 0.0 if peak is None else (
                -peak['value'] if braking else peak['value'])
            if peak is None or strength > peak_strength:
                # New peak
                self.peaks[name] = dict(value=value, units=units, time_s=mission_time_s,
                                        altitude_m=flight_values['altitude_m'],
                                        confirmed=False)
                if braking:
                    self.peaks[name]['phase'] = Phase.ENTRY.value
                self.decline_started_at.pop(name, None)
            elif not peak['confirmed']:
                if strength <= peak_strength * (1 - self.drop_fraction):
                    decline_started_s = self.decline_started_at.setdefault(name, mission_time_s)
                    if mission_time_s - decline_started_s >= self.confirmation_s:
                        peak['confirmed'] = True
                        events.append((name, peak.copy()))
                else:
                    # Reset confirmation
                    self.decline_started_at.pop(name, None)
        return events

    def get_peaks(self):
        # Display snapshot
        return {name: peak.copy() for name, peak in self.peaks.items()}
