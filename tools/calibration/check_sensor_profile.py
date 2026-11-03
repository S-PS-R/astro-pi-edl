"""Check the Pi's existing RTIMULib settings against the fixed emulator profile."""

import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src'))
from sense_hat_emulator.hardware_profile import EXPECTED_IMU_SETTINGS, SENSOR_RATES_HZ


def check_profile(path):
    settings = {}
    for line in Path(path).read_text(encoding='utf-8').splitlines():
        line = line.split('#', 1)[0].strip()
        if '=' in line:
            key, value = (part.strip() for part in line.split('=', 1))
            if key in EXPECTED_IMU_SETTINGS:
                settings[key] = int(value)
    return {key: {'expected': expected, 'actual': settings.get(key)}
            for key, expected in EXPECTED_IMU_SETTINGS.items() if settings.get(key) != expected}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--settings', type=Path, default=Path.home() / '.config/sense_hat/RTIMULib.ini')
    args = parser.parse_args(argv)
    try:
        mismatches = check_profile(args.settings)
    except (OSError, ValueError) as exc:
        parser.exit(1, f'Cannot verify profile: {exc}\nRun check_sensors.py on the Pi first to initialize its driver settings.\n')
    print(f'Settings: {args.settings}')
    print('Fixed nominal rates (Hz):', dict(SENSOR_RATES_HZ))
    for key, values in mismatches.items():
        print(f"MISMATCH {key}: Pi={values['actual']}, emulator profile={values['expected']}")
    if mismatches:
        print('Hardware settings differ; do not claim an exact profile match. No settings were changed.')
        return 1
    print('IMU rate/range settings match. Environmental rates are fixed by the RTIMULib drivers.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
