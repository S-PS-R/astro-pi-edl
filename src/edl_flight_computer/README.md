# edl_flight_computer

| Module | Role |
|---|---|
| hat.py | Sense HAT handle; simulator if the board is missing |
| sensors.py | Temperature, IMU accel, barometer |
| gpio_io.py | LED-matrix event discretes; joystick input |
| sequence.py | EDL phase timeline |
| display.py | HDMI status + matrix wash |
| landing.py | Touchdown from IMU magnitude |
| comms.py | Touchdown UDP downlink |
| log.py | CSV / UDP telemetry |
| record.py | 50 Hz free-flight capture |
| plot.py | Telemetry PNG |
| faults.py | Safe mode |
| demo.py | Sequenced EDL demonstration |

## Planned pitch, roll, and yaw estimation

This section is an implementation plan, not an existing feature. The flight
computer currently reads acceleration, temperature, and pressure in `sensors.py`;
it does not yet estimate orientation. The emulator supplies raw sensor readings
and deliberately has no orientation getters. Orientation estimation belongs here
so the same flight code can process hardware readings and emulator recordings.

Pitch, roll, and yaw describe orientation, not rotation speed. Roll is about X,
pitch about Y, and yaw about Z under the convention used below. Gyro measures
angular velocity about the board axes. Sensor fusion uses gyro to **predict**
orientation, then uses trustworthy acceleration and magnetic readings to
**correct** that prediction. **Rejection** prevents an unreliable measurement
from applying a misleading correction.

### 1. Add the inputs and define conventions

Extend `sensors.py` with gyro and magnetometer readers using the same `get_hat()`
instance as the other readers:

| Input | Sense HAT / SenseEmu method | Raw API unit | Purpose |
| --- | --- | --- | --- |
| Accelerometer X/Y/Z | `get_accelerometer_raw()` | g | Tilt reference when conditions permit |
| Gyroscope X/Y/Z | `get_gyroscope_raw()` | rad/s | Predict rotation over time |
| Magnetometer X/Y/Z | `get_compass_raw()` | µT | Magnetic heading reference |
| Sample timestamp | Acquisition clock or recording timestamp | seconds | Actual elapsed time between updates |

The emulator GUI, monitor, raw API, and CSV all use rad/s for gyro. Keep angular
rates in rad/s and estimated angles in radians throughout the flight computer.
If a third-party fusion library requires other units, isolate the conversion in
its adapter and convert its angle outputs back to radians.

Document the board-to-vehicle mounting transform, axis signs, quaternion order,
and reference frame before implementing the filter. Apply the same mounting
transform to all three sensor vectors. For the equations below, use a
right-handed frame, a scalar-first quaternion `q = (w, x, y, z)` mapping body to
reference coordinates, and `R = Rz(yaw) Ry(pitch) Rx(roll)`.

Accelerometer g units refer to standard gravity: multiply by 9.80665 to obtain
m/s². On another planet, the expected stationary magnitude in g is
`local_gravity_m_s2 / 9.80665`, not automatically 1. The accelerometer measures
specific force; ideal free fall produces approximately zero, not a direct
measurement of gravitational acceleration.

### 2. Initialize and calibrate

Start with a known initial attitude, or initialize tilt during a verified quiet
period using acceleration. Initialize heading from a calibrated, usable magnetic
field or a supplied starting reference. Without an absolute heading reference,
label yaw as relative to the initial heading rather than geographic north.

Estimate gyro zero-rate bias while verified stationary. Calibrate accelerometer
offsets/scales and magnetometer hard-iron offsets and soft-iron distortion.
Do not learn a gyro bias merely because readings are steady: steady rotation is
also possible. Freeze stationary bias learning during flight unless the chosen
filter has a validated in-flight bias estimator.

Use a quaternion internally to represent 3D rotation. This avoids the singularity
of pitch/roll/yaw near a pi/2-radian pitch; the displayed Euler angles can still
become ambiguous there.

### 3. Prediction: propagate orientation using gyro

For every new gyro sample, use its timestamp:

```text
dt = current_timestamp - previous_timestamp
omega = measured_gyro - estimated_gyro_bias
angle = norm(omega) * dt

if norm(omega) is near zero:
    delta_q = (1, 0, 0, 0)
else:
    delta_q = (cos(angle/2), normalize(omega) * sin(angle/2))

q_predicted = normalize(q_previous ⊗ delta_q)
```

Here `⊗` means quaternion multiplication. This increment assumes constant angular
velocity during the interval. It handles simultaneous body-axis rotation without
incorrectly treating each gyro component as an independent Euler-angle derivative.
For a simple single-axis example, 0.5 rad/s for 0.1 seconds gives 0.05 rad.
A constant gyro error of 0.001 rad/s accumulates 0.06 rad of error in a minute.

Reject nonpositive timestamp differences. Flag missing/stale samples and large
time gaps; do not integrate the last rate across an arbitrary gap as though it
were measured throughout. Define a maximum usable gap and a recovery policy.
Gyro saturation also means the true rate is unknown: mark degraded orientation,
rather than claiming the clipped rate captures the motion.

The GUI refresh rate is unrelated to this calculation. `record.py` currently
polls nominally at 50 Hz and sleeps after its work, so actual intervals differ
from 20 ms. The emulator acceleration/gyro model ticks at 952 Hz and magnetometer at 80 Hz;
exact CSV replay instead follows the source timestamps (20 Hz in the Earth drop example). Use actual timestamps and
document any sample-and-hold approximation when samples are missed or repeated;
do not assume a faster polling loop creates new measurements.

### 4. Rejection: decide which corrections are trustworthy

An **innovation** is the difference between a measured reference and what the
predicted attitude says it should be. For example, compare a normalized measured
acceleration vector with the predicted stationary specific-force direction in
board coordinates. Use angular disagreement for direction checks and separate
magnitude checks; zero-length vectors must not be normalized.

| Measurement | Reasons to reject or reduce its correction |
| --- | --- |
| Acceleration | Nonfinite/stale values, clipping, near-zero magnitude in free fall, magnitude inconsistent with local stationary conditions, excessive directional disagreement, or a flight phase with significant aerodynamic/thrust acceleration |
| Magnetic field | Nonfinite/stale values, clipping, weak field, abrupt field changes, implausible magnitude relative to a validated local baseline, or excessive heading disagreement |
| Gyro prediction | Nonfinite/stale values, invalid timing, or saturation; these impair prediction itself, not just a reference correction |

An acceleration magnitude near the expected stationary value is **not sufficient**
to trust tilt: lateral acceleration can still redirect the vector. During EDL,
use flight-phase information and, where available, independent motion estimates
to decide whether acceleration is a valid vertical reference.

For magnetometer correction, check that the field has a usable horizontal
component. A field parallel to vertical cannot constrain heading. An unfamiliar
planet may have a weak, variable, or unknown field; do not assume an Earth field
magnitude or Earth magnetic declination. Geographic yaw is unavailable without
an appropriate reference even if local magnetic heading is measurable.

Reject acceleration and magnetic corrections independently. Retain the raw
samples and log the rejection reason. Use configurable thresholds chosen from
calibration and scenario tests, with hysteresis or several acceptable samples
before re-enabling correction. No universal threshold is justified for this EDL
profile. A large innovation may also indicate a bad estimate, so prolonged
rejection needs a deliberate recovery/reinitialization policy, not a permanent
lockout or an automatic snap to one suspect measurement.

[x-io Fusion](https://github.com/xioTechnologies/Fusion) documents acceleration and
magnetic rejection based on disagreement with its estimate. For a covariance-based
filter, innovation gates additionally account for measurement and state uncertainty;
see the [PX4 EKF guide](https://docs.px4.io/v1.15/en/advanced_config/tuning_the_ecl_ekf).
These are reference approaches, not dependencies already added to this project.

### 5. Correction: adjust prediction using accepted references

When acceleration is trustworthy, its direction corrects tilt (roll/pitch).
Acceleration alone cannot determine yaw. When magnetic measurements are
trustworthy, compensate for tilt and use the horizontal magnetic direction to
correct heading. Do not calculate heading from uncorrected board X/Y components
when the board is tilted.

Use a quaternion fusion algorithm that supports independently weighted or rejected
reference measurements. Conceptually:

```text
q = gyro_prediction(previous_q, gyro, dt)
if acceleration_is_trustworthy:
    q = apply_tilt_correction(q, acceleration, correction_strength)
if magnetic_field_is_trustworthy:
    q = apply_heading_correction(q, magnetic_field, correction_strength)
q = normalize(q)
```

This is explanatory pseudocode, not a complete algorithm. An unobserved axis must
not be forced to zero: with acceleration only, preserve predicted yaw. Blend
corrections instead of replacing the predicted attitude on every sample. Relate
correction strength to elapsed time; for an exponential response with time
constant `tau`, a per-update weight is `1 - exp(-dt/tau)`.

RTIMULib's [RTQF implementation](https://github.com/RPi-Distro/RTIMULib/blob/master/RTIMULib/RTFusionRTQF.cpp)
provides an example of gyro quaternion prediction followed by a weighted quaternion
correction. A robust EDL estimator also needs the rejection and validity handling
above; copying a blend alone does not solve high-acceleration conditions.

If both references are rejected, continue usable gyro prediction but report a
degraded, drifting estimate. If gyro is also unusable, report invalid propagation
and retain the last estimate with its age; do not silently report it as current.
Without an EKF covariance, use explicit quality flags and elapsed time since the
last accepted corrections rather than inventing a numeric confidence percentage.

### 6. Convert the result to pitch, roll, and yaw

For the normalized quaternion and convention defined above:

```text
roll  = atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
pitch = asin(clamp(2*(w*y - z*x), -1, 1))
yaw   = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
```

Keep these angles in radians for display and telemetry. Document angle wrapping,
for example roll/yaw in [-pi, pi) and pitch in [-pi/2, pi/2]. Keep the quaternion for
filter state and downstream rotation calculations. At the Euler singularity,
individual roll/yaw values are not unique even when the quaternion is valid.

### Implementation and validation checklist

1. Add gyro, magnetic, and timing inputs to `sensors.py`. For emulator tests,
   retrieve a consistent snapshot with `get_readings()` and use manual simulation
   time or recording timestamps. Plan how hardware acquisition timing is captured.
2. Add an orientation estimator in the flight-computer package, initialized once
   and updated with timestamped samples. Keep it independent of GUI callbacks.
3. Extend `record.py` and `log.py` to record gyro, magnetic field, quaternion,
   pitch/roll/yaw, sample timing, bias, rejection flags/reasons, saturation,
   and time since accepted tilt/heading corrections. The current log lacks these.
4. Test stationary operation, known single-axis rotations, combined rotations,
   angle wrapping, gyro bias, lateral acceleration without rotation, free fall,
   magnetic disturbance, saturation, timestamp gaps, and recovery after rejection.
5. Replay identical data/timestamps to verify deterministic behavior and ensure
   changing GUI refresh frequency has no effect on the estimates.

For accuracy tests, create physically consistent board-axis sensor recordings in
an external scenario generator and keep reference orientation in a separate file.
The emulator intentionally permits inconsistent raw inputs and rejects orientation
CSV columns. Compare estimated and reference quaternions using rotation-angle
error; also inspect Euler angles and correction/rejection transitions for
readability. Retest with noise enabled, but remember the emulator's synthetic
noise is not a calibrated model of a flight-qualified IMU.
