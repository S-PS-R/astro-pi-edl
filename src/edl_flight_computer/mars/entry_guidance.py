"""Optional bank steering and repeatable horizontal wind for Mars simulations."""

from math import atan2, cos, degrees, exp, floor, pi, radians, sin
from random import Random
from secrets import randbits

import numpy as np


class EntryGuidance:
    def __init__(self, latitude_deg, longitude_deg, max_bank_deg, heading_gain):
        lat, lon = radians(latitude_deg), radians(longitude_deg)
        self.target = np.array((cos(lat) * cos(lon), cos(lat) * sin(lon), sin(lat)))
        self.max_bank_deg = max_bank_deg
        self.heading_gain = heading_gain

    def command(self, up, ground_velocity):
        # Compare the ground track to the great-circle bearing, in the tangent plane.
        desired = self.target - np.dot(self.target, up) * up
        track = ground_velocity - np.dot(ground_velocity, up) * up
        right = np.dot(up, np.cross(desired, track))
        error_deg = degrees(atan2(right, np.dot(desired, track)))
        return max(-self.max_bank_deg, min(self.max_bank_deg, self.heading_gain * error_deg))


class WindProfile:
    def __init__(self, *, seed=None, min_surface_speed_m_s, max_surface_speed_m_s,
                 gust_m_s, gust_interval_s, scale_height_m):
        self.seed = randbits(64) if seed is None else seed
        random = Random(self.seed)
        heading = random.uniform(0, 2 * pi)
        speed = random.uniform(min_surface_speed_m_s, max_surface_speed_m_s)
        self.east, self.north = speed * sin(heading), speed * cos(heading)
        self.interval_s = gust_interval_s
        self.scale_height_m = scale_height_m
        self.gust_m_s = gust_m_s
        self.gusts = []
        self.random = random

    def components(self, time_s, altitude_m):
        index = floor(time_s / self.interval_s)
        while len(self.gusts) <= index + 1:
            heading = self.random.uniform(0, 2 * pi)
            speed = self.random.uniform(0, self.gust_m_s)
            self.gusts.append((speed * sin(heading), speed * cos(heading)))
        fraction = time_s / self.interval_s - index
        weight = fraction**2 * (3 - 2 * fraction)
        east0, north0 = self.gusts[index]
        east1, north1 = self.gusts[index + 1]
        scale = exp(-max(0.0, altitude_m) / self.scale_height_m)
        return (scale * (self.east + east0 + weight * (east1 - east0)),
                scale * (self.north + north0 + weight * (north1 - north0)))
