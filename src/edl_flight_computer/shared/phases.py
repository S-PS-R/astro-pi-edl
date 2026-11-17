"""Mission phase names shared by Earth and Mars."""

from enum import Enum

class Phase(Enum):
    INITIALIZING = 'initializing'
    WAITING_FOR_RELEASE = 'waiting for release'
    ENTRY = 'entry'
    LANDING = 'landing'
    LANDED = 'landed'
