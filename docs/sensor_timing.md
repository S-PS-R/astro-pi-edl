# Fixed Sense HAT V2 timing

## What the defaults mean

SenseHat() initializes sensor drivers using RTIMULib settings. Defaults select
nominal output data rates, full-scale ranges, bandwidth and calibration/fusion
behavior. They do not set the Python program's loop frequency. Reading a register
faster can return the same sample; reading slower can miss samples.

The emulator fixes the following upstream default profile, with no constructor,
GUI, CLI or environment-variable rate controls:

| Channel | Frequency | Period |
| --- | --- | --- |
| LSM9DS1 acceleration/gyro | 119 Hz | 1/119 s |
| LSM9DS1 magnetic | 20 Hz | 0.05 s |
| LPS25H pressure + temperature | 25 Hz | 0.04 s |
| HTS221 humidity + temperature | 12.5 Hz | 0.08 s |

The IMU ranges are +/-8 g, +/-500 degrees/s and +/-4 gauss. The emulator converts
these to g, rad/s and microtesla. Earlier emulator code used maximum chip rates
and ranges; those are different from the upstream defaults.

## Sources

- [RTIMUSettings.cpp](https://github.com/RPi-Distro/RTIMULib/blob/master/RTIMULib/RTIMUSettings.cpp):
  LSM9DS1 defaults select 119 Hz gyro/acceleration, 20 Hz compass and the ranges above.
- [RTIMULSM9DS1.cpp](https://github.com/RPi-Distro/RTIMULib/blob/master/RTIMULib/IMUDrivers/RTIMULSM9DS1.cpp):
  rate settings are mapped to hardware register codes; default sensitivities are
  0.244 mg/LSB, 17.5 mdps/LSB and 0.14 mgauss/LSB.
- [RTHumidityHTS221.cpp](https://github.com/RPi-Distro/RTIMULib/blob/master/RTIMULib/IMUDrivers/RTHumidityHTS221.cpp):
  CTRL1=0x87 (12.5 Hz continuous output).
- [RTPressureLPS25H.cpp](https://github.com/RPi-Distro/RTIMULib/blob/master/RTIMULib/IMUDrivers/RTPressureLPS25H.cpp):
  CTRL_REG1=0xc4 (25 Hz output), hardware averaging/FIFO also configured.
- [HTS221 datasheet](https://www.st.com/resource/en/datasheet/hts221.pdf) and
  [LPS25HB datasheet](https://www.st.com/resource/en/datasheet/lps25hb.pdf): ODR register encodings.
- [SenseHat settings loading](https://github.com/astro-pi/python-sense-hat/blob/master/sense_hat/sense_hat.py):
  ~/.config/sense_hat/RTIMULib.ini is used; /etc/RTIMULib.ini may be copied there.

Sources checked during this cleanup. OS package versions and existing settings
can differ. An untouched SenseHat() call alone does not prove which settings a
particular Pi loaded. On your Pi run:

```sh
python tools/calibration/check_sensors.py
python tools/calibration/check_sensor_profile.py
```

The checker reads the existing file and reports mismatches without modifying
rates or calibration. It checks IMU rates/ranges; environmental ODR is set by
its driver. Matching this reference profile is a prerequisite for comparing
hardware and emulator timing, not a measurement of physical oscillator accuracy.

## Three different clocks

1. Sensor ODR: fixed independent simulated output clocks in both noise modes.
2. Scenario CSV timestamps: when physical targets change; source resolution is
   adjustable and cannot invent motion between rows.
3. Application polling/GUI refresh: when code asks for readings or paints them.

Manual emulator time gives deterministic nominal ticks. Real Python/Linux is
not a hard real-time system and physical chips have oscillator tolerances.
Neither a desktop simulation nor a sleep() loop guarantees exact wall-clock
hardware timing. Synthetic noise/moving averages are not validated hardware
filter replicas. Upper-atmosphere proxies can also exceed real sensor ranges.
