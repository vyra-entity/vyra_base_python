"""
Communication Layer Exceptions

Centralized exception hierarchy for all communication protocols.
"""


class CommunicationError(Exception):
    """Base exception for all communication errors."""
    pass


class ProtocolUnavailableError(CommunicationError):
    """Raised when a requested protocol is not available or not installed."""
    pass


class ProtocolNotInitializedError(CommunicationError):
    """Raised when trying to use a protocol that hasn't been initialized."""
    pass


class TransportError(CommunicationError):
    """Base exception for transport layer errors."""
    pass


class ConnectionError(TransportError):
    """Raised when connection to remote endpoint fails."""
    pass


class TimeoutError(TransportError):
    """Raised when operation exceeds timeout."""
    pass


class SerializationError(TransportError):
    """Raised when serialization/deserialization fails."""
    pass


class ProviderError(CommunicationError):
    """Base exception for provider-related errors."""
    pass


class ProviderNotFoundError(ProviderError):
    """Raised when requested provider is not registered."""
    pass


class ProviderRegistrationError(ProviderError):
    """Raised when provider registration fails."""
    pass


class InterfaceError(CommunicationError):
    """Base exception for interface creation/management errors."""
    pass


class CallableError(InterfaceError):
    """Raised when callable creation or invocation fails."""
    pass


class SpeakerError(InterfaceError):
    """Raised when speaker creation or publishing fails."""
    pass


class JobError(InterfaceError):
    """Raised when job creation or execution fails."""
    pass


class IndustrialBusError(CommunicationError):
    """Base exception for industrial bus communication."""
    pass


class BusConnectionError(IndustrialBusError):
    """Raised when connection to industrial bus fails."""
    pass


class BusTimeoutError(IndustrialBusError):
    """Raised when bus operation times out."""
    pass


class BusDiagnosticError(IndustrialBusError):
    """Raised when bus diagnostic detects failure."""
    pass
