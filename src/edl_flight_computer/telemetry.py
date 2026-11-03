"""Local drop telemetry, flushed after every row; no network side effects."""

import csv
from pathlib import Path

HEADER = ('time_s', 'accel_x', 'accel_y', 'accel_z',
          'gyro_x', 'gyro_y', 'gyro_z', 'mag_x', 'mag_y', 'mag_z',
          'temperature_humidity', 'temperature_pressure', 'pressure', 'humidity',
          'imu_sample_s', 'mag_sample_s', 'pressure_sample_s', 'humidity_sample_s')


class DropLog:
    def __init__(self, path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = self.path.open('x', newline='', encoding='utf-8')
        self.writer = csv.DictWriter(self.stream, fieldnames=HEADER)
        self.writer.writeheader()
        self.stream.flush()

    def write(self, row):
        self.writer.writerow(row)
        self.stream.flush()

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        self.stream.close()
