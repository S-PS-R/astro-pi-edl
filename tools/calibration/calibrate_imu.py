""" Official calibration utility launcher for the Sense HAT IMU.
    Author: Samir Rathore
"""

from pathlib import Path
import argparse
import shutil
import subprocess

def main():
    argparse.ArgumentParser(description=__doc__).parse_args()
    # Paths
    home = Path.home()
    source = Path("/usr/share/librtimulib-utils/RTEllipsoidFit")
    calibration_dir = home / "RTEllipsoidFit"

    # Check that the calibration utility exists
    if not source.exists():
        print(f"ERROR: Could not find: " + str(source))
        print("Are you sure you are running on Raspberry PI 5?")
        print("Make sure the required calibration tools are installed.")
        raise SystemExit(1)

    # Copy calibration directory if it hasn't already been copied
    if not calibration_dir.exists():
        print(f"Copying calibration tools to:")
        print(calibration_dir)
        shutil.copytree(source, calibration_dir)
        print("Copy complete.")
    else:
        print(f"Calibration directory already exists:")
        print(calibration_dir)
        print("Using existing directory.")

    # Launch RTIMULibCal
    print("\nStarting RTIMULibCal...\n")

    subprocess.run(
        ["RTIMULibCal"],
        cwd=calibration_dir
    )

    print("\nRTIMULibCal closed.")


if __name__ == "__main__":
    main()
