"""Vertical EKF used by the live Earth recorder."""

from math import sin, cos
import numpy as np


class EKF:
    """Vertical displacement [m], upward velocity [m/s], acceleration bias [m/s²]."""

    G = 9.80665
    SCALE_HEIGHT_M = 8434.5

    def __init__(self, up_body, gyro_bias, baseline_g=1.0, use_pressure=True):
        # Initial state at release
        self.X = np.array([0.0, 0.0, (baseline_g - 1.0) * self.G])
        self.P = np.diag([0.5**2, 0.1**2, 0.1**2])
        self.up_body = np.asarray(up_body, dtype=float).copy()
        self.up_body /= np.linalg.norm(self.up_body)
        self.gyro_bias = np.asarray(gyro_bias, dtype=float)
        self.use_pressure = use_pressure
        self.reference_pressure_pa = None
        self.pressure_updated = False

    def predict(self, dt, accel_g, gyro_rad_s, *, imu_gap=False):
        """Advance once per fresh IMU poll, using its actual elapsed time."""
        if dt <= 0:
            return
        if imu_gap:
            # Coast across missed samples
            acceleration = 0.0
            F = np.array([[1., dt, 0.], [0., 1., 0.], [0., 0., 1.]])
        else:
            # Board tilt
            gyro = np.asarray(gyro_rad_s) - self.gyro_bias
            angle = np.linalg.norm(gyro) * dt
            if angle > 1e-12:
                axis = -gyro / np.linalg.norm(gyro)
                x, y, z = axis
                K = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
                self.up_body = (np.eye(3) + sin(angle)*K + (1-cos(angle))*(K@K)) @ self.up_body
                self.up_body /= np.linalg.norm(self.up_body)
            # Vertical acceleration
            acceleration = self.G * (np.dot(accel_g, self.up_body) - 1.0) - self.X[2]
            F = np.array([[1., dt, -0.5*dt*dt], [0., 1., -dt], [0., 0., 1.]])
        # State and covariance prediction
        self.X[0] += self.X[1]*dt + 0.5*acceleration*dt*dt
        self.X[1] += acceleration*dt
        noise = np.array([0.5*dt*dt, dt, 0.0])
        Q = (5.0 if imu_gap else 0.5)**2 * np.outer(noise, noise)
        Q[2, 2] += 0.02**2 * dt
        self.P = F @ self.P @ F.T + Q

    def _correct(self, observed, predicted, H, variance):
        innovation_variance = float(H @ self.P @ H + variance)
        K = self.P @ H / innovation_variance
        self.X += K * (observed - predicted)
        residual = np.eye(3) - np.outer(K, H)
        # Covariance correction
        self.P = residual @ self.P @ residual.T + variance * np.outer(K, K)
        self.P = (self.P + self.P.T) * 0.5

    def constrain_waiting(self, accel_g):
        """Stationary reference before release."""
        accel = np.asarray(accel_g)
        if abs(np.linalg.norm(accel) - 1.0) < 0.1:
            self.up_body = accel / np.linalg.norm(accel)
        self._correct(0.0, self.X[0], np.array([1., 0., 0.]), 0.5**2)
        self._correct(0.0, self.X[1], np.array([0., 1., 0.]), 0.03**2)

    def update_pressure(self, pressure_pa, *, waiting=False):
        self.pressure_updated = False
        if not self.use_pressure or pressure_pa <= 0:
            return
        if waiting:
            if self.reference_pressure_pa is None:
                self.reference_pressure_pa = pressure_pa
            else:
                self.reference_pressure_pa += 0.1 * (pressure_pa - self.reference_pressure_pa)
            return
        if self.reference_pressure_pa is None:
            return
        # Barometric measurement model
        predicted = self.reference_pressure_pa * np.exp(-self.X[0] / self.SCALE_HEIGHT_M)
        H = np.array([-predicted / self.SCALE_HEIGHT_M, 0., 0.])
        self._correct(pressure_pa, predicted, H, 20.0**2)
        self.pressure_updated = True

    def confirm_landed(self):
        """Zero speed after independent landing confirmation."""
        self._correct(0.0, self.X[1], np.array([0., 1., 0.]), 0.03**2)

    def snapshot(self):
        return dict(ekf_displacement_m=float(self.X[0]),
                    ekf_vertical_velocity_m_s=float(self.X[1]),
                    ekf_accel_bias_m_s2=float(self.X[2]),
                    ekf_displacement_std_m=float(np.sqrt(self.P[0, 0])),
                    ekf_velocity_std_m_s=float(np.sqrt(self.P[1, 1])))
