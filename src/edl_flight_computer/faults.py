"""Fault flags and safe-mode discretes."""

from __future__ import annotations

from enum import Enum

from .gpio_io import PIN_DESCENT_ENGINE, PIN_PARACHUTE, PIN_SAFE, set_output


class Fault(Enum):
    NONE = "none"
    SENSOR = "sensor"
    GPIO = "gpio"
    COMMS = "comms"


_active: Fault = Fault.NONE


def inject_fault(kind: Fault | str) -> Fault:
    global _active
    _active = kind if isinstance(kind, Fault) else Fault(kind)
    return _active


def handle_fault(kind: Fault | None = None) -> Fault:
    global _active
    if kind is not None:
        _active = kind
    if _active is not Fault.NONE:
        safe_hold()
    return _active


def active_fault() -> Fault:
    return _active


def safe_hold() -> None:
    set_output(PIN_PARACHUTE, False)
    set_output(PIN_DESCENT_ENGINE, False)
    set_output(PIN_SAFE, True)
    print(f"[safe] {_active.value}")
