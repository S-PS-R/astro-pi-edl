"""Sense HAT V2 handle. Falls back to SimulatedHat if I2C is unavailable."""

from __future__ import annotations

from typing import Any

_hat: Any = None
_simulated = False


class SimulatedHat:
    def __init__(self) -> None:
        self._pixels = [(0, 0, 0)] * 64
        self._stick = _SimStick()

    @property
    def stick(self) -> "_SimStick":
        return self._stick

    def get_temperature(self) -> float:
        return 28.4

    def get_temperature_from_humidity(self) -> float:
        return 28.4

    def get_temperature_from_pressure(self) -> float:
        return 29.1

    def get_pressure(self) -> float:
        return 1013.25

    def get_accelerometer_raw(self) -> dict[str, float]:
        return {"x": 0.02, "y": -0.01, "z": 1.00}

    def set_pixel(self, x: int, y: int, r: int, g: int, b: int) -> None:
        self._pixels[y * 8 + x] = (r, g, b)

    def clear(self, *color: int) -> None:
        fill = (color[0], color[1], color[2]) if color else (0, 0, 0)
        self._pixels = [fill] * 64

    def show_message(self, text: str, **_: Any) -> None:
        print(f"[hat sim] {text}")


class _SimStick:
    def get_events(self) -> list:
        return []


def get_hat() -> Any:
    global _hat, _simulated
    if _hat is not None:
        return _hat
    try:
        from sense_hat import SenseHat  # type: ignore

        _hat = SenseHat()
        _simulated = False
    except Exception as exc:
        print(f"[hat] no Sense HAT ({exc!r}); simulator")
        _hat = SimulatedHat()
        _simulated = True
    return _hat


def is_simulated() -> bool:
    get_hat()
    return _simulated
