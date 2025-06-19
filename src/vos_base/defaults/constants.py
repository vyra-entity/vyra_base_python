from dataclasses import dataclass
from enum import Enum
from typing import Final
from typing import NamedTuple


class RequiredVersion(Enum):
    """ Constants used in the vOS universe.
        According to "Pyright" Annotations like 'Final[str]' are not allowed.
    """
    PYTHON_MIN = (3, 9, 0)


@dataclass(slots=True, frozen=True)
class FeederConstants:
    MAX_DEFAULT_LENGTH: Final[int] = 1000
    MAX_PULLREQUESTSFEED_LENGTH: Final[int] = 1000
    MAX_NEWSFEED_LENGTH: Final[int] = 1000
    MAX_ERRORFEED_LENGTH: Final[int] = 1000
    MAX_STATEFEED_LENGTH: Final[int] = 1000


@dataclass(slots=True, frozen=True)
class LoggerConstants:
    MAX_QUEUE_LENGTH: Final[int] = 1000


@dataclass(slots=True, frozen=True)
class TimeoutConstants:
    """ Sets all constants for handling timeout and message length of the 
        different connection types like async, udp. 
    """
    CONNECT: Final[int] = 5
    SEND: Final[int] = 5
    RECEIVE: Final[int] = 5
    CLOSED: Final[int] = 5


@dataclass(slots=True, frozen=True)
class MessageLengthConstants:
    RECEIVE: Final[int] = 4096


@dataclass(slots=True, frozen=True)
class CyclicRefreshConstants:
    """ Set the cyclic refresh time for the different modules in ms. """
    NEWSFEED: Final[int] = 100
    LOGGER: Final[int] = 20
