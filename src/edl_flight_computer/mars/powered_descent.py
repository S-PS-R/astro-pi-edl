"""Local flat-ground guidance for a point-mass lander."""

from math import asin, cos, radians, sin, sqrt

import numpy as np
from numpy.typing import NDArray

from ..shared.phases import Phase

STANDARD_GRAVITY_M_S2 = 9.80665


class PoweredDescent:
    def __init__(self, settings, radius):
        self.settings = settings
        self.radius = radius
        self.up: NDArray[np.float64] | None = None
        self.chute_time_s = None
        self.separation_time_s = None
        self.target = settings.get('landing_target')
        self.target_up = None
        if self.target is not None:
            lat, lon = map(radians, (self.target['latitude_deg'], self.target['longitude_deg']))
            self.target_up = np.array((cos(lat) * cos(lon), cos(lat) * sin(lon), sin(lat)))

    def separate(self, position, time_s):
        # The flat ground is tangent to the reference sphere below separation.
        self.up = position / np.linalg.norm(position)
        self.separation_time_s = time_s

    def altitude(self, position):
        up = self.up
        if up is None:
            raise RuntimeError('Landing plane is unavailable before backshell separation')
        return np.dot(position, up) - self.radius

    def target_error(self, radial):
        target = self.target_up
        if target is None:
            return None
        # Convert the chord between unit vectors into a surface distance.
        half_chord = np.linalg.norm(radial - target) / 2
        return 2 * self.radius * asin(min(1.0, half_chord))

    def thrust(self, phase, altitude, position, velocity, acceleration, mass, propellant, *, hold_horizontal=False):
        """Brake and steer horizontally while tracking the vertical descent profile."""
        if phase not in (Phase.POWERED_DESCENT, Phase.VERTICAL_DESCENT) or propellant <= 0:
            return np.zeros(3), 0.0

        s = self.settings['propulsion']
        up = self.up
        if up is None:
            raise RuntimeError('Landing plane is unavailable before backshell separation')
        vertical = np.dot(velocity, up)
        horizontal = velocity - vertical * up
        vertical_accel = np.dot(acceleration, up)
        response = s['response_time_s']
        brake = s['vertical_braking_m_s2']
        target_speed = -sqrt(s['target_descent_speed_m_s']**2 + 2 * brake * max(0, altitude))
        vertical_command = -brake * vertical / abs(target_speed) + (target_speed - vertical) / response
        if phase is Phase.POWERED_DESCENT:
            vertical_command = max(0.0, vertical_command)

        horizontal_command = np.zeros(3)
        if phase is Phase.POWERED_DESCENT or hold_horizontal or self.target is not None:
            desired = np.zeros(3)
            if self.target_up is not None and altitude > self.target['stop_divert_altitude_m']:
                offset = self.radius * self.target_up - position
                offset -= np.dot(offset, up) * up
                desired = offset / self.target['response_time_s']
                speed = np.linalg.norm(desired)
                limit = self.target['max_divert_speed_m_s']
                if speed > limit:
                    desired *= limit / speed
            horizontal_command = (desired - horizontal) / response
            demand = np.linalg.norm(horizontal_command)
            if demand > s['horizontal_braking_m_s2']:
                horizontal_command *= s['horizontal_braking_m_s2'] / demand

        # Reserve the vertical thrust first so a divert cannot spend the braking budget.
        vertical_force = min(s['max_thrust_n'], mass * max(0.0, vertical_command - vertical_accel))
        horizontal_force = mass * horizontal_command
        available = sqrt(max(0.0, s['max_thrust_n']**2 - vertical_force**2))
        demand = np.linalg.norm(horizontal_force)
        if demand > available:
            horizontal_force *= available / demand
        force = vertical_force * up + horizontal_force
        thrust = np.linalg.norm(force)
        return force, thrust / (s['specific_impulse_s'] * STANDARD_GRAVITY_M_S2)
