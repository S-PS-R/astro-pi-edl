"""Atmosphere, flight equations, integration, and observed peak tracking."""

from math import exp, sin, cos, sqrt, radians, degrees, acos, ceil
import numpy as np

# "https://nssdc.gsfc.nasa.gov/planetary/factsheet/marsfact.html"
PLANETS = {
    "earth": {
        "name": "earth",
        "radius": 6371000.0, #m
        "gravity": 9.80665, #m/s^2
        "mu": 3.9860e14,
        "T0": 288.15,
        "P0": 101325.0,
        "gas_constant": 287.05,
        "C_SG": 1.7415e-4,
    },
    "mars": {
        "name": "mars",
        "radius": 3389500.0, #m
        "gravity": 3.73, #m/s^2
        "mu": 4.2828e13, #m³/s²
        "T0": 214.0, #K
        "P0": 636.0, #Pa
        "gas_constant": 191.18, #J/(kg·K)
        "C_SG": 1.9027e-4,
    }
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


def calculate_state(X, planet, vehicle_params):
    """Calculate state derivatives and instantaneous flight quantities."""
    h, V, gamma = X
    T, pressure, rho, g_ref, H = ambient_atmosphere(h, planet)
    Rp = planet["radius"]
    g = planet["mu"] / (Rp + h)**2
    CD, CL = vehicle_params["CD"], vehicle_params["CL"]
    m, S = vehicle_params["mass"], vehicle_params["S"]

    # Drag and lift.
    dynamic_pressure = 0.5 * rho * V**2
    D = dynamic_pressure * S * CD
    L = dynamic_pressure * S * CL

    # Planar bank guidance: rotate lift to track a descending FPA instead of
    # keeping the entire lift force upward. L remains the total lift force.
    bank_cos = 1.0
    if 'target_fpa_deg' in vehicle_params and L > 0 and V > 0:
        target = radians(vehicle_params['target_fpa_deg'])
        gamma_rate = vehicle_params.get('fpa_gain', 0.05) * (target - gamma)
        vertical_lift = m * (V * gamma_rate + (g - V**2/(Rp + h)) * cos(gamma))
        bank_cos = max(-1.0, min(1.0, vertical_lift / L))

    # Positive speed; flight-path angle is positive above the local horizontal.
    h_dot = V * sin(gamma)
    V_dot = -D/m - g * sin(gamma)
    gamma_dot = 0.0
    if V > 0.0:
        gamma_dot = L*bank_cos/(m*V) + (V/(Rp + h) - g/V) * cos(gamma)

    Rn = vehicle_params['Rn']
    C_SG = planet['C_SG']
    heating = C_SG * sqrt(rho / Rn) * V**3 / 1e3

    values = {
        'altitude_m': h,
        'velocity_m_s': V,
        'temperature_k': T,
        'pressure_pa': pressure,
        'density_kg_m3': rho,
        'gravity_m_s2': g,
        'dynamic_pressure_pa': dynamic_pressure,
        'drag_n': D,
        'lift_n': L,
        'lift_vertical_n': L * bank_cos,
        'bank_angle_deg': degrees(acos(bank_cos)),
        'vertical_velocity_m_s': h_dot,
        'acceleration_m_s2': V_dot,
        'deceleration_m_s2': max(0.0, -V_dot),
        'g_load': sqrt(D**2 + L**2) / (m * planet['gravity']),
        'heating_kw_m2': heating,
    }
    return np.array([h_dot, V_dot, gamma_dot]), values


def advance_state(X, dt, planet, vehicle_params, max_step=0.02):
    """Integrate elapsed time with bounded RK4 steps; stop at surface contact."""
    X = np.array(X, dtype=float, copy=True)
    if dt <= 0 or X[0] <= 0:
        return X
    steps = max(1, ceil(dt / max_step))
    step = dt / steps
    for _ in range(steps):
        k1 = calculate_state(X, planet, vehicle_params)[0]
        k2 = calculate_state(X + step*k1/2, planet, vehicle_params)[0]
        k3 = calculate_state(X + step*k2/2, planet, vehicle_params)[0]
        k4 = calculate_state(X + step*k3, planet, vehicle_params)[0]
        updated = X + step*(k1 + 2*k2 + 2*k3 + k4)/6
        if updated[0] <= 0:
            fraction = X[0] / (X[0] - updated[0])
            X += fraction * (updated - X)
            X[0] = 0.0
            return X
        X = updated
    return X

class PeakTracker:
    """Remember maxima, then confirm them after a sustained 1% decrease."""
    CHANNELS = {
        'peak_decel': ('deceleration_m_s2', 'm/s^2'),
        'peak_heating': ('heating_kw_m2', 'kW/m^2'),
        'peak_velocity': ('velocity_m_s', 'm/s'),
    }

    def __init__(self):
        self.drop_fraction = 0.01
        self.confirmation_s = 1.0
        self.peaks = {}
        self.decline_started_at = {}

    def update_peaks(self, mission_time_s, flight_values):
        events = []
        for name, (key, units) in self.CHANNELS.items():
            value = flight_values[key]
            # Ignore an initial zero, but allow a fall to zero to confirm a peak.
            if value == 0 and name not in self.peaks:
                continue
            peak = self.peaks.get(name)
            if peak is None or value > peak['value']:
                # Keep the maximum's time and altitude, not the later detection time.
                self.peaks[name] = dict(value=value, units=units, time_s=mission_time_s,
                                        altitude_m=flight_values['altitude_m'],
                                        confirmed=False)
                self.decline_started_at.pop(name, None)
            elif not peak['confirmed']:
                if value <= peak['value'] * (1 - self.drop_fraction):
                    decline_started_s = self.decline_started_at.setdefault(name, mission_time_s)
                    if mission_time_s - decline_started_s >= self.confirmation_s:
                        peak['confirmed'] = True
                        events.append((name, peak.copy()))
                else:
                    # A brief dip is not enough; start over if the value recovers.
                    self.decline_started_at.pop(name, None)
        return events

    def get_peaks(self):
        # Give the GUI its own copy while tracking continues in the mission thread.
        return {name: peak.copy() for name, peak in self.peaks.items()}
