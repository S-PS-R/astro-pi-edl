"""Sense HAT V2 handle. Uses SenseEmu when the real library or hardware is unavailable."""

from __future__ import annotations
from typing import Any

_hat: Any = None
_simulated = False


def get_hat(*, noise=True, seed=None, force_emulator=False) -> Any:
    global _hat, _simulated
    if _hat is not None:
        if force_emulator and not _simulated:
            raise RuntimeError('Hardware is already in use; start a new process for CSV replay')
        return _hat
    if force_emulator:
        from sense_hat_emulator import SenseEmu
        _hat = SenseEmu(noise=noise, seed=seed)
        _simulated = True
        return _hat
    try:
        from sense_hat import SenseHat  # type: ignore

        _hat = SenseHat()
        _simulated = False
    except Exception as exc:
        print(f"[hat] no Sense HAT ({exc!r}); simulator")
        from sense_hat_emulator import SenseEmu

        _hat = SenseEmu(noise=noise, seed=seed)
        _simulated = True
    return _hat


def is_simulated() -> bool:
    get_hat()
    return _simulated
