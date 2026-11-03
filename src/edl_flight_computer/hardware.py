"""Shared Sense HAT connection, with explicit hardware or emulator selection."""

from __future__ import annotations
from typing import Any

_hat: Any = None
_simulated = False


def get_hat(*, noise=True, seed=None, force_emulator=False, require_hardware=False) -> Any:
    """Get the shared board, optionally refusing any emulator fallback."""
    global _hat, _simulated
    if require_hardware and force_emulator:
        raise ValueError('Hardware cannot be required when forcing the emulator')
    if _hat is not None:
        if require_hardware and _simulated:
            raise RuntimeError('Real Sense HAT hardware is required; emulator is in use')
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
        if require_hardware:
            raise RuntimeError(
                'Sense HAT unavailable. Run this on a Raspberry Pi with a connected '
                'Sense HAT and the sense-hat library installed.'
            ) from exc
        print(f"[hat] no Sense HAT ({exc!r}); simulator")
        from sense_hat_emulator import SenseEmu

        _hat = SenseEmu(noise=noise, seed=seed)
        _simulated = True
    return _hat


def is_simulated() -> bool:
    get_hat()
    return _simulated
