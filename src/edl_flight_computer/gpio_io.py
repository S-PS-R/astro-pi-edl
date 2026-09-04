"""EDL event discretes.

Outputs 0-7 are LED-matrix columns (parachute, descent engine, heatshield jettison, safe).
Inputs 10-14 are the HAT joystick.
"""

from __future__ import annotations

from time import sleep

from .hat import get_hat

PIN_PARACHUTE = 0
PIN_DESCENT_ENGINE = 1
PIN_HEATSHIELD = 2
PIN_SAFE = 7

PIN_CHUTE = PIN_PARACHUTE
PIN_ENGINE = PIN_DESCENT_ENGINE

PIN_JOY_UP = 10
PIN_JOY_DOWN = 11
PIN_JOY_LEFT = 12
PIN_JOY_RIGHT = 13
PIN_JOY_CENTER = 14

_OUTPUT_COLOR = {
    PIN_PARACHUTE: (0, 180, 0),
    PIN_DESCENT_ENGINE: (255, 90, 0),
    PIN_HEATSHIELD: (255, 140, 0),
    PIN_SAFE: (180, 0, 0),
}

_JOY_PINS = {
    "up": PIN_JOY_UP,
    "down": PIN_JOY_DOWN,
    "left": PIN_JOY_LEFT,
    "right": PIN_JOY_RIGHT,
    "middle": PIN_JOY_CENTER,
}

_lines: dict[int, bool] = {}
_joy: dict[int, bool] = {pin: False for pin in _JOY_PINS.values()}


def output_state() -> dict[int, bool]:
    return dict(_lines)


def column_color(pin: int) -> tuple[int, int, int]:
    return _OUTPUT_COLOR.get(pin, (0, 0, 180))


def set_output(pin: int, value: bool) -> None:
    if pin < 0 or pin > 7:
        raise ValueError(f"discrete {pin} is not a matrix column (0-7)")
    _lines[pin] = bool(value)
    _paint_column(pin, bool(value))


def read_input(pin: int) -> bool:
    _drain_stick()
    if pin in _joy:
        return _joy[pin]
    raise ValueError(f"input {pin} is not a joystick channel (10-14)")


def pulse(pin: int, seconds: float) -> None:
    set_output(pin, True)
    sleep(max(seconds, 0.0))
    set_output(pin, False)


def _paint_column(pin: int, on: bool) -> None:
    hat = get_hat()
    color = column_color(pin) if on else (0, 0, 0)
    for y in range(8):
        hat.set_pixel(pin, y, *color)


def _drain_stick() -> None:
    stick = getattr(get_hat(), "stick", None)
    if stick is None:
        return
    for event in stick.get_events():
        pin = _JOY_PINS.get(getattr(event, "direction", ""))
        if pin is None:
            continue
        _joy[pin] = getattr(event, "action", "") in {"pressed", "held"}
