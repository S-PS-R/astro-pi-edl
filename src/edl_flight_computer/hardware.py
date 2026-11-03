"""Open one shared real Sense HAT or emulator for the flight program."""

_hat = None
_simulated = False


def get_hat(*, noise=True, seed=None, force_emulator=False, require_hardware=False):
    """Return the existing board, or connect once when first called.

    force_emulator selects simulation. require_hardware prevents fallback.
    With neither option, try the real board first and fall back if unavailable.
    """
    global _hat, _simulated

    if force_emulator and require_hardware:
        raise ValueError('Choose either hardware or emulator, not both')

    # Reuse the board so all callers see the same sensors and LEDs.
    if _hat is not None:
        if require_hardware and _simulated:
            raise RuntimeError('Real Sense HAT required; the emulator is already in use')
        if force_emulator and not _simulated:
            raise RuntimeError('Real hardware is already in use; start a new process for simulation')
        return _hat

    if not force_emulator:
        try:
            # Installed on the Pi; desktop runs use the emulator instead.
            from sense_hat import SenseHat  # pyright: ignore[reportMissingImports]
            _hat = SenseHat()
        except (ImportError, OSError, RuntimeError) as error:
            if require_hardware:
                raise RuntimeError(
                    'Sense HAT unavailable. Run on a Raspberry Pi with the HAT '
                    'connected and the sense-hat library installed.'
                ) from error
            print(f'Cannot connect to Sense HAT: {error}. Using emulator.')
        else:
            _simulated = False
            return _hat

    from sense_hat_emulator import SenseEmu
    _hat = SenseEmu(noise=noise, seed=seed)
    _simulated = True
    return _hat


def is_simulated():
    get_hat()
    return _simulated
