"""Atmosphere, flight equations, integration, and observed peak tracking."""

from bisect import bisect_left
from dataclasses import dataclass
from math import ceil, cos, exp, pi, sin, sqrt

import numpy as np

from ..shared.phases import Phase

MAX_STEP_S = 0.02

# Mars constants
MARS = {
    "name": "mars",
    "radius": 3389500.0, #m
    "gravity": 3.73, #m/s^2
    "mu": 4.2828e13, #m³/s²
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


def _rk4_step(X, step, derivatives):
    k1 = derivatives(X)
    k2 = derivatives(X + step * k1 / 2)
    k3 = derivatives(X + step * k2 / 2)
    k4 = derivatives(X + step * k3)
    return X + step * (k1 + 2 * k2 + 2 * k3 + k4) / 6


def advance_state(X, dt, planet, vehicle_params, max_step=MAX_STEP_S, *,
                  parachute=None, canopy_deployed=False):
    """Integrate elapsed time with bounded RK4 steps; stop at surface contact."""
    X = np.array(X, dtype=float, copy=True)
    if dt <= 0 or X[0] <= 0:
        return X
    steps = max(1, ceil(dt / max_step))
    step = dt / steps

    def derivatives(vector):
        return calculate_state(vector, planet, vehicle_params, parachute=parachute,
                               canopy_deployed=canopy_deployed)[0]

    for _ in range(steps):
        updated = _rk4_step(X, step, derivatives)
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


class MarsTrajectory:
    """Advance the Mars physics and latch flight events in simulation time."""

    def __init__(self, vector, planet, vehicle_params, parachute, phase=Phase.ENTRY):
        self.vector = np.array((*vector, 0.0) if len(vector) == 3 else vector, dtype=float)
        self.planet = planet
        self.vehicle_params = vehicle_params
        self.parachute = parachute
        self.phase = phase
        self.canopy_deployed = phase in (Phase.PARACHUTE_DESCENT, Phase.TERMINAL_DESCENT)
        self.time_s = 0.0
        self.terminal_since_s = None
        self.landing_warned = False
        self.values = self._calculate(self.vector)[1]
        if self.vector[0] <= 0:
            self.phase = Phase.LANDED

    def _calculate(self, vector):
        return calculate_state(vector, self.planet, self.vehicle_params,
                               parachute=self.parachute, canopy_deployed=self.canopy_deployed)

    def _derivatives(self, vector):
        return self._calculate(vector)[0]

    def _sample(self, duration):
        vector = _rk4_step(self.vector, duration, self._derivatives)
        return vector, self._calculate(vector)[1]

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
                self.phase = Phase.TERMINAL_DESCENT
                events.append(FlightEvent('terminal_descent', deadline, values))
        if not eligible_at_end:
            self.terminal_since_s = None

    @staticmethod
    def _landing_warning_due(values):
        downward_speed = -values['vertical_velocity_m_s']
        return downward_speed > 0 and values['altitude_m'] <= 10.0 * downward_speed

    def _record_landing_warning(self, duration, endpoint, events):
        if self.landing_warned:
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

    def advance(self, dt):
        """Advance at most dt seconds and return events at their crossing states."""
        events = []
        end_time = self.time_s + dt
        while self.time_s < end_time and self.phase is not Phase.LANDED:
            duration = min(MAX_STEP_S, end_time - self.time_s)
            endpoint = self._sample(duration)
            deployment = None
            if self.phase is Phase.ENTRY and self.parachute is not None:
                deployment = self._find_deployment(duration, endpoint)
            if deployment is not None:
                duration, endpoint = deployment
            elif endpoint[0][0] <= 0:
                duration, endpoint = self._first_crossing(
                    duration, endpoint, lambda values: values['altitude_m'] <= 0)
                contact_vector, _ = endpoint
                contact_vector[0] = 0.0
                endpoint = contact_vector, self._calculate(contact_vector)[1]

            if self.phase is Phase.PARACHUTE_DESCENT:
                self._record_terminal(duration, endpoint, events)
            self._record_landing_warning(duration, endpoint, events)
            self.time_s += duration
            self.vector, self.values = endpoint

            if deployment is not None:
                before_values = self.values
                self.canopy_deployed = True
                self.phase = Phase.PARACHUTE_DESCENT
                self.values = self._calculate(self.vector)[1]
                events.append(FlightEvent('parachute_deploy', self.time_s, self.values,
                                          before_values=before_values))
            elif self.vector[0] <= 0:
                self.phase = Phase.LANDED

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
