from math import exp, sin, cos
import numpy as np

PLANETS = {
    "earth": {
        "radius": 6371000.0, #m
        "mu": 3.9860e14,
        "T0": 288.15,
        "P0": 101325.0,
        "gas_constant": 287.05,
        "fact_sheet": "https://nssdc.gsfc.nasa.gov/planetary/factsheet/earthfact.html",
    },
    "mars": {
        "radius": 3389500.0, #m
        "mu": 4.2828e13, #m³/s²
        "T0": 214.0, #K
        "P0": 636.0, #Pa
        "gas_constant": 191.18, #J/(kg·K)
        "fact_sheet": "https://nssdc.gsfc.nasa.gov/planetary/factsheet/marsfact.html",
    },
}

class LiftingBodyEKF:
    def __init__(self, state, dt, planet, vehicle_params):
        self.dt = dt
        self.Planet = planet
        self.vehicle_params = vehicle_params
        # States: Altitude [m], speed [m/s], FPA [rad].
        self.X = np.array([state.altitude_m, state.velocity_m_s, state.flight_path_angle_rad])
        self.P = np.diag([100.0**2, 10.0**2, np.radians(1.0)**2])  # Initial covariance matrix
        self.Q = np.diag([1.0**2, 0.5**2, np.radians(0.05)**2])  # Process noise covariance matrix
        self.R = np.diag([10.0**2, 1.5**2, np.radians(0.1)**2])  # Measurement noise covariance matrix


    def ambient_atmosphere(self, h, planet):
        """Hydrostatic / Barometric model for the ambient atmosphere."""
        """Return temperature [K], pressure [Pa], density [kg/m³]."""
        
        Rp       = planet["radius"]
        mu      = planet["mu"]
        T0      = planet["T0"]
        p0      = planet["P0"]
        R_gas   = planet["gas_constant"]

        # Constant reference gravity for the exponential atmosphere.
        g_ref = mu / Rp**2
        H = R_gas * T0 / g_ref

        pressure = p0 * exp(-h / H)
        density = pressure / (R_gas * T0)

        return T0, pressure, density, g_ref, H

    def process_model(self, X):
        """Continuous time equations f(X) derived from Lifting Body FBD"""
        h, V, gamma = X[0], X[1], X[2]
        T_amb, P_amb, rho_amb, g_ref, H = self.ambient_atmosphere(h, self.planet)

        # Drag and Lift
        CD, CL, m, S = self.vehicle_params.CD, self.vehicle_params.CL, self.vehicle_params.mass, self.vehicle_params.S
        D = 0.5 * rho_amb * V**2 * S * CD
        L = 0.5 * rho_amb * V**2 * S * CL

        # FPA Rate of Change
        gamma_dot = L/(m*V) - g_ref/V *cos(gamma)

        # Velocity Rate of Change
        V_dot = -D/m + g_ref*sin(gamma)

        # Altitude Rate of Change
        h_dot = V*sin(gamma)

        return np.array([h_dot, V_dot, gamma_dot])

    def jaccobian(self, X):
        """Evaluates df/dX numerically"""
        h, V, gamma = X[0], X[1], X[2]
        Rp = self.Planet["radius"]
        mu = self.Planet["mu"]
        
        _, _, _, g, _ = self.ambient_atmosphere(h)
        dg_dh = -2.0 * mu / (Rp + h)**3

        A = np.zeros((3, 3))
        
        # Row 1: d(h_dot) / dX
        A[0, 0] = 0.0
        A[0, 1] = sin(gamma)
        A[0, 2] = V * cos(gamma)

        # Row 2: d(V_dot) / dX
        A[1, 0] = -dg_dh * (cos(gamma) / self.L_D + sin(gamma))
        A[1, 1] = 0.0
        A[1, 2] = -g * (-sin(gamma) / self.L_D + cos(gamma))

        # Row 3: d(gamma_dot) / dX
        A[2, 0] = -V * cos(gamma) / (Rp + h)**2
        A[2, 1] = cos(gamma) / (Rp + h)
        A[2, 2] = -(V / (Rp + h)) * sin(gamma)

        return A

    def predict(self):
        """EKF Time Update step"""
        A = self.jaccobian(self.X)
        self.X = self.X + self.process_model(self.X) + self.dt
        self.P = A @ self.P @ A.T + self.Q

    def update(self, sensor):
        """EKF Measurement Correction Step"""
        h, V, gamma = self.X[0], self.X[1], self.X[2]
        T_amb, P_amb, rho_amb, g_ref = self.ambient_atmosphere(h, self.planet)

        # if sensor is within out of range skip [return]
        # Set predicted measurements
        # Compute measurement jaccobian
        # Innovation residual
        # Innovation covariance
        # Kalman Gain
        # Update state and covariance

        



        
