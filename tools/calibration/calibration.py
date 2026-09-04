""" Offical calibration script for Sense HAT IMU
    Author: Samir Rathore
"""

from pathlib import Path
import shutil
import subprocess

# Patths
home = Path.home()
source = Path("/usr/share/librtimulib-utils/RTEllipsoidFit")
calFile = home / "RTEllipsoidFit"

# Check that the calibration utility exists
if not source.exists():
    print(f"ERROR: Could not find:")
    print(source)
    print()
    print("Make sure the required calibration tools are installed.")
    raise SystemExit(1)

# Copy calibration directory if it hasn't already been copied
if not calFile.exists():
    print(f"Copying calibration tools to:")
    print(calFile)
    shutil.copytree(source, calFile)
    print("Copy complete.")
else:
    print(f"Calibration directory already exists:")
    print(calFile)
    print("Using existing directory.")

# Launch RTIMULibCal
print("\nStarting RTIMULibCal...\n")

subprocess.run(
    ["RTIMULibCal"],
    cwd=calFile
)

print("\nRTIMULibCal closed.")