"""Independent X/Y/Z IMU noise, in public API units: g, rad/s, and µT."""

from math import radians, sin, cos
from random import Random
from .humidity import SampleChannel


class IMUSensor:
    period = 0.016

    def __init__(self, seed, targets):
        self.random = Random(seed)
        self.channels = {}
        # Convert upstream degree/s and gauss parameters to rad/s and µT.
        for prefix, low, high, factor, error in [
            ('accel', -8, 8, 4081.6327, 0.1),
            ('gyro', -radians(500), radians(500), 57.142857 * 180 / 3.141592653589793, radians(1)),
            ('mag', -400, 400, 7142.8571 / 100, 200.0),
        ]:
            for axis in 'xyz':
                key = f'{prefix}_{axis}'
                self.channels[key] = (SampleChannel(targets[key], 10, low, high, factor), error)

    def sample(self, targets):
        return {key: channel.sample(targets[key], error, self.random)
                for key, (channel, error) in self.channels.items()}



def rotate_to_board(vector, roll, pitch, yaw):
    """R.T @ vector, using the official emulator's Rz(yaw) Ry(pitch) Rx(roll)."""
    x, y, z = map(radians, (roll, pitch, yaw))
    c1, c2, c3 = cos(z), cos(y), cos(x)
    s1, s2, s3 = sin(z), sin(y), sin(x)
    rotation = (
        (c1*c2, c1*s2*s3-c3*s1, s1*s3+c1*c3*s2),
        (c2*s1, c1*c3+s1*s2*s3, c3*s1*s2-c1*s3),
        (-s2, c2*s3, c2*c3),
    )
    return tuple(sum(rotation[row][column]*vector[row] for row in range(3))
                 for column in range(3))


def orientation_rates(previous, current, seconds):
    """Official-style Euler differences, converted to rad/s.

    Wrap differences to the shortest signed angle so 179 -> -179 is +2 degrees.
    This approximates gyro rates; it is not a general body-rate transformation.
    """
    return tuple(radians((new-old+180) % 360-180) / seconds
                 for old, new in zip(previous, current))
