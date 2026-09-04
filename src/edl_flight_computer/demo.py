"""Sequenced EDL run on the HAT (or simulator)."""

from __future__ import annotations

import sys
from pathlib import Path
from time import sleep

if __package__ is None or __package__ == "":
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from edl_flight_computer.comms import send_landing_message
    from edl_flight_computer.display import render
    from edl_flight_computer.faults import Fault, active_fault, handle_fault, inject_fault
    from edl_flight_computer.gpio_io import (
            PIN_DESCENT_ENGINE,
            PIN_JOY_CENTER,
            PIN_PARACHUTE,
            PIN_SAFE,
            pulse,
            read_input,
            set_output,
        )
    from edl_flight_computer.hat import is_simulated
    from edl_flight_computer.landing import accel_magnitude, detect_touchdown
    from edl_flight_computer.sensors import read_acceleration, read_pressure, read_temperature
    from edl_flight_computer.sequence import advance_phase, phase_elapsed, start_edl
    from edl_flight_computer.comms import send_landing_message
    from edl_flight_computer.display import render
    from edl_flight_computer.faults import Fault, active_fault, handle_fault, inject_fault
    from edl_flight_computer.gpio_io import (
        PIN_DESCENT_ENGINE,
        PIN_JOY_CENTER,
        PIN_PARACHUTE,
        PIN_SAFE,
        pulse,
        read_input,
        set_output,
    )
    from edl_flight_computer.hat import is_simulated
    from edl_flight_computer.landing import accel_magnitude, detect_touchdown
    from edl_flight_computer.sensors import read_acceleration, read_pressure, read_temperature
    from edl_flight_computer.sequence import advance_phase, phase_elapsed, start_edl
else:
    from .comms import send_landing_message
    from .display import render
    from .faults import Fault, active_fault, handle_fault, inject_fault
    from .gpio_io import (
        PIN_DESCENT_ENGINE,
        PIN_JOY_CENTER,
        PIN_PARACHUTE,
        PIN_SAFE,
        pulse,
        read_input,
        set_output,
    )
    from .hat import is_simulated
    from .landing import accel_magnitude, detect_touchdown
    from .sensors import read_acceleration, read_pressure, read_temperature
    from .sequence import advance_phase, phase_elapsed, start_edl
    from .comms import send_landing_message
    from .display import render
    from .faults import Fault, active_fault, handle_fault, inject_fault
    from .gpio_io import (
        PIN_DESCENT_ENGINE,
        PIN_JOY_CENTER,
        PIN_PARACHUTE,
        PIN_SAFE,
        pulse,
        read_input,
        set_output,
    )
    from .hat import is_simulated
    from .landing import accel_magnitude, detect_touchdown
    from .sensors import read_acceleration, read_pressure, read_temperature
    from .sequence import advance_phase, phase_elapsed, start_edl

PHASE_DWELL_S = 1.5
_POLL_S = 0.1


def _snapshot() -> dict[str, float]:
    accel = read_acceleration()
    return {
        "T_C": round(read_temperature(), 2),
        "ax": round(accel[0], 3),
        "ay": round(accel[1], 3),
        "az": round(accel[2], 3),
        "a_mag_g": round(accel_magnitude(accel), 3),
        "P_mbar": round(read_pressure(), 2),
    }


def _show(phase: str) -> None:
    fault = active_fault().value
    render(phase, phase_elapsed(), _snapshot(), None if fault == "none" else fault)


def _dwell() -> bool:
    waited = 0.0
    while waited < PHASE_DWELL_S:
        if _maybe_fault():
            return True
        sleep(_POLL_S)
        waited += _POLL_S
    return False


def _act(phase: str) -> None:
    if phase == "parachute":
        pulse(PIN_PARACHUTE, 0.4)
    elif phase == "terminal_descent":
        set_output(PIN_DESCENT_ENGINE, True)


def _try_land() -> bool:
    accel = read_acceleration()
    landed = detect_touchdown(accel)
    if is_simulated() and not landed:
        landed = True
        accel = (0.1, 0.1, 2.4)
    if not landed:
        print(f"no touchdown (|a|={accel_magnitude(accel):.2f} g)")
        return False
    set_output(PIN_DESCENT_ENGINE, False)
    set_output(PIN_PARACHUTE, False)
    advance_phase()
    _show("touchdown")
    send_landing_message(
        f"TOUCHDOWN |a|={accel_magnitude(accel):.2f}g P={read_pressure():.1f}mbar"
    )
    return True


def main() -> None:
    state = start_edl()
    print(f"EDL start: {state.phase}  hat={'sim' if is_simulated() else 'real'}")

    while True:
        _act(state.phase)
        if _dwell():
            return
        _show(state.phase)
        if state.phase == "terminal_descent":
            _try_land()
            return
        if state.phase == "touchdown":
            return
        advance_phase()


def _maybe_fault() -> bool:
    try:
        pressed = read_input(PIN_JOY_CENTER)
    except ValueError:
        return False
    if not pressed:
        return False
    inject_fault(Fault.SENSOR)
    handle_fault()
    render("safe_hold", phase_elapsed(), _snapshot(), Fault.SENSOR.value)
    return True


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        set_output(PIN_SAFE, True)
        print("\nsafe")
