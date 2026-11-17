"""Headless Live drop recording and landing detection."""

from collections import deque
from datetime import datetime
from math import sqrt
from pathlib import Path
from statistics import fmean, pstdev
from time import monotonic

from ..shared.hardware import get_hat
from ..shared.notify import LandingNotifier
from ..shared.phases import Phase
from ..shared.sensors import SENSOR_CHANNELS, SensorReader
from ..shared.telemetry import DropLog
from .ekf import EKF
from .live_stream import PlotSender

# CSV columns
GROUPS = ('imu', 'mag', 'pressure', 'humidity')
EKF_VALUES = ('ekf_displacement_m', 'ekf_vertical_velocity_m_s', 'ekf_accel_bias_m_s2', 'ekf_displacement_std_m', 'ekf_velocity_std_m_s')
HEADER = (
    'sequence', 'time_s', 'iteration_dt_s', 'flight_elapsed_s', 'phase', 'events',
    *SENSOR_CHANNELS,
    *(group + '_sample_s' for group in GROUPS),
    *(group + '_status' for group in GROUPS),
    *(group + '_fresh' for group in GROUPS),
    'accel_magnitude_g', 'gyro_magnitude_rad_s', 'settling_duration_s',
    'imu_valid', 'release_detected', 'settling', 'settling_started_s', 'landing_confirmed_s',
    *EKF_VALUES, 'ekf_updated', 'ekf_imu_gap', 'ekf_pressure_updated',
)


class DropDetector:
    def __init__(self, settings=None):
        from ..run import EARTH_SETTINGS
        self.settings = EARTH_SETTINGS['detection'].copy()
        if settings:
            self.settings.update(settings)
        self.phase = Phase.INITIALIZING
        self.release_at = None
        self.settling_at = None
        self.confirmed_at = None
        self.last_imu_at = None
        self.accel_magnitude = None
        self.gyro_magnitude = None
        self.baseline_g = 1.0
        self.gyro_bias = (0.0, 0.0, 0.0)
        self.up_body = (0.0, 0.0, 1.0)
        self.release_candidate_at = None
        self.calibration_samples = []
        self.accel_window = deque()

    def update(self, now, readings, fresh, valid):
        if self.phase == Phase.LANDED:
            return []

        # Fresh IMU data
        stale = self.last_imu_at is not None and now - self.last_imu_at > self.settings['max_imu_gap_s']
        if not valid or stale:
            self.settling_at = None
            self.release_candidate_at = None
            self.calibration_samples.clear()
            self.accel_window.clear()
        if not fresh or not valid:
            return []
        self.last_imu_at = now
        accel = tuple(readings['accel_' + axis] for axis in 'xyz')
        gyro = tuple(readings['gyro_' + axis] for axis in 'xyz')
        self.accel_magnitude = sqrt(sum(value**2 for value in accel))
        self.gyro_magnitude = sqrt(sum((value - bias)**2 for value, bias in zip(gyro, self.gyro_bias)))

        # Mission phase
        if self.phase == Phase.INITIALIZING:
            return self.calibrate(now, accel, gyro)
        if self.phase == Phase.WAITING_FOR_RELEASE:
            return self.detect_release(now)
        return self.detect_landing(now, accel)

    def calibrate(self, now, accel, gyro):
        # Stationary readings
        settings = self.settings
        moving = (abs(self.accel_magnitude - 1.0) > settings['accel_tolerance_g']
                  or self.gyro_magnitude > settings['gyro_tolerance_rad_s'])
        if moving:
            self.calibration_samples.clear()
            return []
        self.calibration_samples.append((now, accel, gyro))
        if now - self.calibration_samples[0][0] < settings['calibration_s']:
            return []
        samples = self.calibration_samples
        if any(pstdev(sample[1][axis] for sample in samples) > settings['accel_std_g'] for axis in range(3)):
            self.calibration_samples = [(now, accel, gyro)]
            return []

        # Bias and initial orientation
        self.baseline_g = fmean(sqrt(sum(v*v for v in sample[1])) for sample in samples)
        self.gyro_bias = tuple(fmean(sample[2][axis] for sample in samples) for axis in range(3))
        mean_accel = tuple(fmean(sample[1][axis] for sample in samples) for axis in range(3))
        length = sqrt(sum(value**2 for value in mean_accel))
        self.up_body = tuple(value / length for value in mean_accel)
        self.calibration_samples.clear()
        self.phase = Phase.WAITING_FOR_RELEASE
        return ['ready']

    def detect_release(self, now):
        # Low-g interval
        if self.accel_magnitude >= self.settings['release_fraction'] * self.baseline_g:
            self.release_candidate_at = None
            return []
        if self.release_candidate_at is None:
            self.release_candidate_at = now
        if now - self.release_candidate_at < self.settings['release_duration_s']:
            return []
        self.release_at = now
        self.phase = Phase.LANDING
        self.accel_window.clear()
        return ['release']

    def detect_landing(self, now, accel):
        # Settling window
        settings = self.settings
        self.accel_window.append((now, accel))
        while len(self.accel_window) > 1 and self.accel_window[1][0] <= now - settings['settling_window_s']:
            self.accel_window.popleft()
        quiet = (
            abs(self.accel_magnitude - self.baseline_g) <= settings['accel_tolerance_g']
            and self.gyro_magnitude <= settings['gyro_tolerance_rad_s']
            and now - self.accel_window[0][0] >= settings['settling_window_s']
            and all(pstdev(sample[1][axis] for sample in self.accel_window) <= settings['accel_std_g'] for axis in range(3))
        )
        if not quiet:
            self.settling_at = None
            return []
        if self.settling_at is None:
            self.settling_at = now
        if now - self.settling_at < settings['settling_duration_s']:
            return []
        self.confirmed_at = now
        self.phase = Phase.LANDED
        return ['landing']

# csv output
def make_row(sequence, elapsed, dt, reader, detector, estimator, events):
    # Timing and detection
    flight_time = None if detector.release_at is None else elapsed - detector.release_at
    settling_time = 0.0 if detector.settling_at is None else elapsed - detector.settling_at
    row = {
        'sequence': sequence, 'time_s': elapsed, 'iteration_dt_s': dt,
        'flight_elapsed_s': flight_time, 'phase': detector.phase.value,
        'events': ';'.join(events),
        'accel_magnitude_g': detector.accel_magnitude,
        'gyro_magnitude_rad_s': detector.gyro_magnitude,
        'settling_duration_s': settling_time,
        'release_detected': detector.release_at is not None,
        'settling': detector.settling_at is not None,
        'settling_started_s': detector.settling_at,
        'landing_confirmed_s': detector.confirmed_at,
    }

    # Raw sensors and poll status
    for name in SENSOR_CHANNELS:
        row[name] = reader.readings.get(name)
    for group in GROUPS:
        sample_time = reader.sample_times_s.get(group)
        status = 'unavailable' if sample_time is None else 'ok'
        row[group + '_sample_s'] = sample_time
        row[group + '_status'] = reader.errors.get(group, status)
        row[group + '_fresh'] = group in reader.updated_groups

    # Estimated state
    estimates = dict.fromkeys(EKF_VALUES) if estimator is None else estimator.snapshot()
    row.update(estimates)
    return row

# main run code
def run_live_drop(sense, output_path, notifier=None, stream_host=None,
                   stream_port=8765, settings=None, stop_requested=None, ekf_use_pressure=True):
    # Recording setup
    reader = SensorReader(sense)
    detector = DropDetector(settings)
    sender = PlotSender(stream_host, stream_port) if stream_host is not None else None
    estimator = None
    last_estimate_at = None
    started_at = previous_at = monotonic()
    sequence = 0
    previous_led = None
    sense.clear()
    print(f'Live drop recording: {output_path}', flush=True)

    try:
        with DropLog(output_path, fieldnames=HEADER, flush_interval_s=0.25) as log:
            while True:
                # Sensor acquisition
                now = monotonic()
                elapsed = now - started_at
                readings = reader.read(elapsed, recover_errors=True)
                imu_time = reader.sample_times_s.get('imu')
                valid = (imu_time is not None and 'imu' not in reader.errors
                         and elapsed - imu_time <= detector.settings['max_imu_gap_s'])
                fresh = 'imu' in reader.updated_groups
                events = detector.update(elapsed, readings, fresh, valid)

                # EKF initialization
                updated = gap = pressure_updated = False
                if 'ready' in events:
                    estimator = EKF(detector.up_body, detector.gyro_bias,
                                               detector.baseline_g, use_pressure=ekf_use_pressure)
                    last_estimate_at = elapsed
                    updated = True

                # Fresh measurement updates
                if estimator is not None:
                    if valid and fresh:
                        dt = elapsed - last_estimate_at
                        gap = dt > detector.settings['max_imu_gap_s']
                        accel = [readings['accel_' + axis] for axis in 'xyz']
                        gyro = [readings['gyro_' + axis] for axis in 'xyz']
                        estimator.predict(dt, accel, gyro, imu_gap=gap)
                        if detector.phase == Phase.WAITING_FOR_RELEASE and detector.release_candidate_at is None:
                            estimator.constrain_waiting(accel)
                        last_estimate_at = elapsed
                        updated = True
                    if 'pressure' in reader.updated_groups:
                        estimator.update_pressure(readings['pressure'] * 100.0,
                                                  waiting=detector.release_at is None)
                        pressure_updated = estimator.pressure_updated
                        updated = updated or pressure_updated
                    if 'landing' in events:
                        estimator.confirm_landed()
                        updated = True

                # Green landing indicator
                if detector.release_at is not None:
                    led_on = detector.phase == Phase.LANDED or (elapsed - detector.release_at) % 1.0 < 0.5
                    if led_on != previous_led:
                        sense.set_pixel(7, 7, 0, 255 if led_on else 0, 0)
                        previous_led = led_on

                # CSV and laptop snapshot
                stopped = stop_requested is not None and stop_requested()
                if stopped and detector.phase != Phase.LANDED:
                    events.append('stopped')
                row = make_row(sequence, elapsed, now-previous_at, reader, detector, estimator, events)
                row.update(imu_valid=valid, ekf_updated=updated, ekf_imu_gap=gap, ekf_pressure_updated=pressure_updated)
                log.write(row)
                if sender is not None:
                    sender.publish(row, now, force=detector.phase == Phase.LANDED or stopped)
                previous_at = now
                sequence += 1
                if detector.phase == Phase.LANDED or stopped:
                    break
    except KeyboardInterrupt:
        print('Live drop recording stopped.', flush=True)
    finally:
        # Recording cleanup
        if sender is not None:
            sender.close()
        if detector.phase != Phase.LANDED:
            sense.clear()

    # Landing notification
    if detector.phase == Phase.LANDED:
        flight_time = detector.confirmed_at - detector.release_at
        print(f'Landing confirmed after {flight_time:.3f} s, including settling.', flush=True)
        if notifier is not None:
            message = (f'Landing confirmed at T+{detector.confirmed_at:.3f} s.\n'
                       f'Flight timer: {flight_time:.3f} s, including settling.\n'
                       f'CSV: {output_path}')
            notifier.send('EDL landing: ', message)
            notifier.wait()
    return detector


def main(settings=None):
    if settings is None:
        from ..run import EARTH_SETTINGS
        settings = EARTH_SETTINGS

    # Hardware and output
    output_path = Path(__file__).resolve().parents[3] / 'logs' / ('drop_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '.csv')

    sense = get_hat(force_emulator=settings['use_emulator'],
                    require_hardware=not settings['use_emulator'])

    run_live_drop(sense, output_path, notifier=LandingNotifier(),
                   stream_host=settings['laptop_ip'] if settings['live_plotting'] else None,
                   stream_port=settings['stream_port'], settings=settings['detection'],
                   ekf_use_pressure=settings['ekf_use_pressure'])
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
