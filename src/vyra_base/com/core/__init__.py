"""
Core Communication Components

Provides base types, exceptions, and utilities for all communication protocols.
"""
from vyra_base.com.core.exceptions import (
    CommunicationError,
    ProtocolUnavailableError,
    ProtocolNotInitializedError,
    TransportError,
    ConnectionError,
    TimeoutError,
    SerializationError,
    ProviderError,
    ProviderNotFoundError,
    ProviderRegistrationError,
    InterfaceError,
    CallableError,
    SpeakerError,
    JobError,
    IndustrialBusError,
    BusConnectionError,
    BusTimeoutError,
    BusDiagnosticError,
)

from vyra_base.com.core.types import (
    ProtocolType,
    InterfaceType,
    AccessLevel,
    DisplayStyle,
    InterfaceMetadata,
    VyraInterface,
    VyraCallable,
    VyraSpeaker,
    VyraJob,
)

from vyra_base.com.core.factory import InterfaceFactory

from vyra_base.com.core.decorators import (
    remote_callable,
    remote_speaker,
    remote_job,
    get_decorated_methods,
)

__all__ = [
    # Exceptions
    "CommunicationError",
    "ProtocolUnavailableError",
    "ProtocolNotInitializedError",
    "TransportError",
    "ConnectionError",
    "TimeoutError",
    "SerializationError",
    "ProviderError",
    "ProviderNotFoundError",
    "ProviderRegistrationError",
    "InterfaceError",
    "CallableError",
    "SpeakerError",
    "JobError",
    "IndustrialBusError",
    "BusConnectionError",
    "BusTimeoutError",
    "BusDiagnosticError",
    # Types
    "ProtocolType",
    "InterfaceType",
    "AccessLevel",
    "DisplayStyle",
    "InterfaceMetadata",
    "VyraInterface",
    "VyraCallable",
    "VyraSpeaker",
    "VyraJob",
    # Factory
    "InterfaceFactory",
    # Decorators
    "remote_callable",
    "remote_speaker",
    "remote_job",
    "get_decorated_methods",
]
