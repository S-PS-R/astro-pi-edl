"""HDMI status line and LED-matrix phase wash."""

from __future__ import annotations

from typing import Any

from .gpio_io import column_color, output_state
from .hat import get_hat, is_simulated

_PHASE_COLOR = {
    "preentry": (20, 20, 40),
    "entry": (0, 40, 140),
    "peak_heating": (180, 50, 0),
    "parachute": (0, 140, 40),
    "terminal_descent": (200, 200, 200),
    "touchdown": (255, 255, 255),
}

_FAULT_COLOR = (160, 0, 0)


def render(
    phase: str,
    elapsed_s: float,
    sensors: dict[str, Any] | None = None,
    fault: str | None = None,
) -> None:
    sensors = sensors or {}
    src = "sim" if is_simulated() else "hat"
    print(
        f"[{src}] phase={phase:18s} t={elapsed_s:6.1f}s "
        f"fault={fault or 'none':8s} {sensors}"
    )
    _paint_matrix(phase, fault)


def _paint_matrix(phase: str, fault: str | None) -> None:
    hat = get_hat()
    fill = _FAULT_COLOR if fault and fault != "none" else _PHASE_COLOR.get(
        phase, (30, 30, 30)
    )
    for x in range(8):
        for y in range(8):
            hat.set_pixel(x, y, *fill)
    for pin, on in output_state().items():
        if on:
            color = column_color(pin)
            for y in range(8):
                hat.set_pixel(pin, y, *color)
