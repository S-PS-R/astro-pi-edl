import numpy as np
from .model import PLANETS, ambient_atmosphere, calculate_state
from math import sin, cos

class LandingEKF:
    def __init__(self, state, dt, planet, vehicle_params):
        self.dt = dt
        self.planet = planet
        self.vehicle_params = vehicle_params
        # States: Altitude [m], speed [m/s], FPA [rad].
        self.X = np.array([state.altitude_m, state.velocity_m_s, state.flight_path_angle_rad])
        self.P = np.diag([100.0**2, 10.0**2, np.radians(1.0)**2])  # Initial covariance matrix
        self.Q = np.diag([1.0**2, 0.5**2, np.radians(0.05)**2])  # Process noise covariance matrix
        self.R = np.diag([10.0**2, 1.5**2, np.radians(0.1)**2])  # Measurement noise covariance matrix


    def ambient_atmosphere(self, h, planet):
        return ambient_atmosphere(h, planet)

    def process_model(self, X):
        derivative, _ = calculate_state(X, self.planet, self.vehicle_params)
        return derivative

    def jaccobian(self, X):
        """Original Jacobian scaffold; still needs to match process_model."""
        h, V, gamma = X[0], X[1], X[2]

        Rp = self.planet["radius"]
        mu = self.planet["mu"]
        CD = self.vehicle_params['CD']
        CL = self.vehicle_params['CL']
        L_D = CL/CD
        
        _, _, _, g, _ = self.ambient_atmosphere(h, self.planet)
        dg_dh = -2.0 * mu / (Rp + h)**3

        A = np.zeros((3, 3))
        
        # Row 1: d(h_dot) / dX
        A[0, 0] = 0.0
        A[0, 1] = sin(gamma)
        A[0, 2] = V * cos(gamma)

        # Row 2: d(V_dot) / dX
        A[1, 0] = -dg_dh * (cos(gamma) / L_D + sin(gamma))
        A[1, 1] = 0.0
        A[1, 2] = -g * (-sin(gamma) / L_D + cos(gamma))

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
        T_amb, P_amb, rho_amb, g_ref, H = self.ambient_atmosphere(h, self.planet)

        # if sensor is within out of range skip [return]
        # Set predicted measurements
        # Compute measurement jaccobian
        # Innovation residual
        # Innovation covariance
        # Kalman Gain
        # Update state and covariance