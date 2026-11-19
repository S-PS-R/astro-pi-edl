"""Mission phase names shared by Earth and Mars."""

from enum import Enum

class Phase(Enum):
    INITIALIZING = 'initializing'
    WAITING_FOR_RELEASE = 'waiting for release'
    ENTRY = 'entry'
    PARACHUTE_DESCENT = 'parachute descent'
    TERMINAL_DESCENT = 'terminal descent'
    LANDING = 'landing'
    LANDED = 'landed'
