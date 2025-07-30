from dataclasses import dataclass
from enum import Enum
from typing import Final, NamedTuple
import uuid

# Namespace to generate unique uuidv5 for a specific module id
MODULE_ID_NAMESPACE: uuid.UUID = uuid.uuid5(uuid.NAMESPACE_DNS, "VYRA")

class RequiredVersion(Enum):
    """
    Constants used in the V.Y.R.A. universe.

    According to "Pyright", annotations like ``Final[str]`` are not allowed.
    """
    PYTHON_MIN = (3, 9, 0)


@dataclass(slots=True, frozen=True)
class FeederConstants:
    """
    Constants for feeder limits.

    :cvar MAX_DEFAULT_LENGTH: Default maximum length.
    :cvar MAX_PULLREQUESTSFEED_LENGTH: Maximum length for pull requests feed.
    :cvar MAX_NEWSFEED_LENGTH: Maximum length for news feed.
    :cvar MAX_ERRORFEED_LENGTH: Maximum length for error feed.
    :cvar MAX_STATEFEED_LENGTH: Maximum length for state feed.
    """
    MAX_DEFAULT_LENGTH: Final[int] = 1000
    MAX_PULLREQUESTSFEED_LENGTH: Final[int] = 1000
    MAX_NEWSFEED_LENGTH: Final[int] = 1000
    MAX_ERRORFEED_LENGTH: Final[int] = 1000
    MAX_STATEFEED_LENGTH: Final[int] = 1000


@dataclass(slots=True, frozen=True)
class LoggerConstants:
    """
    Constants for logger configuration.

    :cvar MAX_QUEUE_LENGTH: Maximum queue length for logger.
    """
    MAX_QUEUE_LENGTH: Final[int] = 1000


@dataclass(slots=True, frozen=True)
class TimeoutConstants:
    """
    Sets all constants for handling timeout and message length of the different connection types like async, UDP.

    :cvar CONNECT: Timeout for connect (seconds).
    :cvar SEND: Timeout for send (seconds).
    :cvar RECEIVE: Timeout for receive (seconds).
    :cvar CLOSED: Timeout for closed state (seconds).
    """
    CONNECT: Final[int] = 5
    SEND: Final[int] = 5
    RECEIVE: Final[int] = 5
    CLOSED: Final[int] = 5


@dataclass(slots=True, frozen=True)
class MessageLengthConstants:
    """
    Constants for message length.

    :cvar RECEIVE: Maximum receive message length (bytes).
    """
    RECEIVE: Final[int] = 4096


@dataclass(slots=True, frozen=True)
class CyclicRefreshConstants:
    """
    Set the cyclic refresh time for the different modules in milliseconds.

    :cvar NEWSFEED: Refresh time for news feed (ms).
    :cvar LOGGER: Refresh time for logger (ms).
    """
    NEWSFEED: Final[int] = 100
    LOGGER: Final[int] = 20
