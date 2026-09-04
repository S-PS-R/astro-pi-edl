"""C&DH software for the Pi 5 / Sense HAT V2 EDL stack."""

from .comms import send_landing_message
from .display import render
from .faults import Fault, handle_fault, inject_fault, safe_hold
from .gpio_io import (
    PIN_PARACHUTE,
    PIN_DESCENT_ENGINE,
    PIN_HEATSHIELD,
    PIN_JOY_CENTER,
    PIN_SAFE,
    pulse,
    read_input,
    set_output,
)
from .landing import detect_touchdown
from .sensors import read_acceleration, read_pressure, read_temperature
from .sequence import advance_phase, phase_elapsed, run_terminal_descent, start_edl

PIN_CHUTE = PIN_PARACHUTE
PIN_ENGINE = PIN_DESCENT_ENGINE
read_aux = read_pressure

__all__ = [
    "start_edl",
    "advance_phase",
    "phase_elapsed",
    "run_terminal_descent",
    "read_temperature",
    "read_acceleration",
    "read_pressure",
    "read_aux",
    "set_output",
    "read_input",
    "pulse",
    "PIN_PARACHUTE",
    "PIN_DESCENT_ENGINE",
    "PIN_HEATSHIELD",
    "PIN_SAFE",
    "PIN_JOY_CENTER",
    "PIN_CHUTE",
    "PIN_ENGINE",
    "detect_touchdown",
    "inject_fault",
    "handle_fault",
    "safe_hold",
    "Fault",
    "send_landing_message",
    "render",
]
