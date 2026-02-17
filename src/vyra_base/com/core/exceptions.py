"""
Communication Layer Exceptions

Centralized exception hierarchy for all communication protocols.
Provides detailed error handling for transport, protocol, and industrial communication layers.
"""
from typing import Optional, Any, Dict


# ==============================================================================
# Base Communication Exceptions
# ==============================================================================

class CommunicationError(Exception):
    """
    Base exception for all communication errors.
    
    Attributes:
        message: Error description
        details: Optional additional error context
        original_exception: Original exception if wrapped
    """
    def __init__(
        self,
        message: str,
        details: Optional[Dict[str, Any]] = None,
        original_exception: Optional[Exception] = None
    ):
        super().__init__(message)
        self.message = message
        self.details = details or {}
        self.original_exception = original_exception
    
    def __str__(self) -> str:
        result = self.message
        if self.details:
            result += f" | Details: {self.details}"
        if self.original_exception:
            result += f" | Caused by: {type(self.original_exception).__name__}: {self.original_exception}"
        return result


# ==============================================================================
# Protocol Layer Exceptions
# ==============================================================================

class ProtocolError(CommunicationError):
    """Base exception for protocol-specific errors."""
    pass


class ProtocolUnavailableError(ProtocolError):
    """Raised when a requested protocol is not available or not installed."""
    def __init__(self, protocol: str, message: Optional[str] = None):
        super().__init__(
            message or f"Protocol '{protocol}' is not available",
            details={"protocol": protocol}
        )
        self.protocol = protocol


class ProtocolNotInitializedError(ProtocolError):
    """Raised when trying to use a protocol that hasn't been initialized."""
    def __init__(self, protocol: str, component: Optional[str] = None):
        details = {"protocol": protocol}
        if component:
            details["component"] = component
        super().__init__(
            f"Protocol '{protocol}' not initialized" + (f" for {component}" if component else ""),
            details=details
        )
        self.protocol = protocol
        self.component = component


class ProtocolConfigurationError(ProtocolError):
    """Raised when protocol configuration is invalid or incomplete."""
    def __init__(self, protocol: str, config_issue: str):
        super().__init__(
            f"Configuration error in protocol '{protocol}': {config_issue}",
            details={"protocol": protocol, "issue": config_issue}
        )


class ProtocolVersionMismatchError(ProtocolError):
    """Raised when protocol versions are incompatible."""
    def __init__(self, protocol: str, expected: str, actual: str):
        super().__init__(
            f"Version mismatch in protocol '{protocol}': expected {expected}, got {actual}",
            details={"protocol": protocol, "expected": expected, "actual": actual}
        )


# ==============================================================================
# Transport Layer Exceptions
# ==============================================================================

class TransportError(CommunicationError):
    """Base exception for transport layer errors."""
    pass


class ConnectionError(TransportError):
    """Raised when connection to remote endpoint fails."""
    def __init__(
        self,
        endpoint: str,
        reason: Optional[str] = None,
        original_exception: Optional[Exception] = None
    ):
        super().__init__(
            f"Connection to '{endpoint}' failed" + (f": {reason}" if reason else ""),
            details={"endpoint": endpoint, "reason": reason},
            original_exception=original_exception
        )
        self.endpoint = endpoint


class ConnectionTimeoutError(ConnectionError):
    """Raised when connection attempt times out."""
    def __init__(self, endpoint: str, timeout: float):
        super().__init__(
            endpoint,
            f"timeout after {timeout}s",
            None
        )
        self.timeout = timeout


class ConnectionClosedError(ConnectionError):
    """Raised when connection is unexpectedly closed."""
    def __init__(self, endpoint: str, reason: Optional[str] = None):
        super().__init__(endpoint, f"connection closed: {reason}" if reason else "connection closed")


class TimeoutError(TransportError):
    """Raised when operation exceeds timeout."""
    def __init__(self, operation: str, timeout: float, details: Optional[Dict] = None):
        super().__init__(
            f"Operation '{operation}' timed out after {timeout}s",
            details={"operation": operation, "timeout": timeout, **(details or {})}
        )
        self.operation = operation
        self.timeout = timeout


class SerializationError(TransportError):
    """Raised when serialization/deserialization fails."""
    def __init__(
        self,
        data_type: str,
        operation: str,  # "serialize" or "deserialize"
        reason: Optional[str] = None,
        original_exception: Optional[Exception] = None
    ):
        super().__init__(
            f"Failed to {operation} {data_type}" + (f": {reason}" if reason else ""),
            details={"data_type": data_type, "operation": operation},
            original_exception=original_exception
        )


class MessageError(TransportError):
    """Base exception for message-related errors."""
    pass


class MessageTooLargeError(MessageError):
    """Raised when message exceeds size limit."""
    def __init__(self, size: int, max_size: int):
        super().__init__(
            f"Message size {size} bytes exceeds maximum {max_size} bytes",
            details={"size": size, "max_size": max_size}
        )


class MessageCorruptedError(MessageError):
    """Raised when received message is corrupted or invalid."""
    def __init__(self, reason: Optional[str] = None):
        super().__init__(
            f"Message corrupted" + (f": {reason}" if reason else ""),
            details={"reason": reason}
        )


# ==============================================================================
# Provider Layer Exceptions
# ==============================================================================

class ProviderError(CommunicationError):
    """Base exception for provider-related errors."""
    pass


class ProviderNotFoundError(ProviderError):
    """Raised when requested provider is not registered."""
    def __init__(self, provider_name: str, protocol: Optional[str] = None):
        details = {"provider": provider_name}
        if protocol:
            details["protocol"] = protocol
        super().__init__(
            f"Provider '{provider_name}' not found" + (f" for protocol '{protocol}'" if protocol else ""),
            details=details
        )


class ProviderRegistrationError(ProviderError):
    """Raised when provider registration fails."""
    def __init__(self, provider_name: str, reason: str):
        super().__init__(
            f"Failed to register provider '{provider_name}': {reason}",
            details={"provider": provider_name, "reason": reason}
        )


class ProviderAlreadyRegisteredError(ProviderError):
    """Raised when attempting to register a provider that already exists."""
    def __init__(self, provider_name: str):
        super().__init__(
            f"Provider '{provider_name}' is already registered",
            details={"provider": provider_name}
        )


# ==============================================================================
# Interface Layer Exceptions
# ==============================================================================

class InterfaceError(CommunicationError):
    """Base exception for interface creation/management errors."""
    pass


class InterfaceNotFoundError(InterfaceError):
    """Raised when requested interface does not exist."""
    def __init__(self, interface_name: str, interface_type: Optional[str] = None):
        details = {"interface": interface_name}
        if interface_type:
            details["type"] = interface_type
        super().__init__(
            f"Interface '{interface_name}' not found" + (f" (type: {interface_type})" if interface_type else ""),
            details=details
        )


class InterfaceAlreadyExistsError(InterfaceError):
    """Raised when attempting to create an interface that already exists."""
    def __init__(self, interface_name: str):
        super().__init__(
            f"Interface '{interface_name}' already exists",
            details={"interface": interface_name}
        )


class InterfaceNotInitializedError(InterfaceError):
    """Raised when attempting to use an uninitialized interface."""
    def __init__(self, interface_name: str):
        super().__init__(
            f"Interface '{interface_name}' not initialized",
            details={"interface": interface_name}
        )


class TServerError(InterfaceError):
    """Raised when transport server creation or invocation fails."""
    pass


class TServerInvocationError(TServerError):
    """Raised when transport server invocation fails."""
    def __init__(
        self,
        server_name: str,
        reason: Optional[str] = None,
        original_exception: Optional[Exception] = None
    ):
        super().__init__(
            f"Transport server '{server_name}' invocation failed" + (f": {reason}" if reason else ""),
            details={"server": server_name},
            original_exception=original_exception
        )


class TServerTimeoutError(TServerError):
    """Raised when transport server invocation times out."""
    def __init__(self, server_name: str, timeout: float):
        super().__init__(
            f"Transport server '{server_name}' timed out after {timeout}s",
            details={"server": server_name, "timeout": timeout}
        )


class TPublisherError(InterfaceError):
    """Raised when transport publisher creation or publishing fails."""
    pass


class TPublisherPublishError(TPublisherError):
    """Raised when message publication fails."""
    def __init__(
        self,
        publisher_name: str,
        reason: Optional[str] = None,
        original_exception: Optional[Exception] = None
    ):
        super().__init__(
            f"Publisher '{publisher_name}' publish failed" + (f": {reason}" if reason else ""),
            details={"publisher": publisher_name},
            original_exception=original_exception
        )

class TSubscriberError(InterfaceError):
    """Raised when transport subscriber creation or subscription fails."""
    pass

class TSubscriberSubscribeError(TSubscriberError):
    """Raised when subscription fails."""
    def __init__(self, subscriber_name: str, reason: Optional[str] = None):
        super().__init__(
            f"Subscriber '{subscriber_name}' subscription failed" + (f": {reason}" if reason else ""),
            details={"subscriber": subscriber_name}
        )


class ActionServerError(InterfaceError):
    """Raised when actionServer creation or execution fails."""
    pass


class ActionServerExecutionError(ActionServerError):
    """Raised when actionServer execution fails."""
    def __init__(
        self,
        job_name: str,
        reason: Optional[str] = None,
        original_exception: Optional[Exception] = None
    ):
        super().__init__(
            f"Job '{job_name}' execution failed" + (f": {reason}" if reason else ""),
            details={"job": job_name},
            original_exception=original_exception
        )


class ActionServerCancellationError(ActionServerError):
    """Raised when actionServer cancellation fails."""
    def __init__(self, job_name: str, reason: Optional[str] = None):
        super().__init__(
            f"ActionServer '{job_name}' cancellation failed" + (f": {reason}" if reason else ""),
            details={"job": job_name}
        )


class ActionServerTimeoutError(ActionServerError):
    """Raised when actionServer execution times out."""
    def __init__(self, job_name: str, timeout: float):
        super().__init__(
            f"ActionServer '{job_name}' timed out after {timeout}s",
            details={"job": job_name, "timeout": timeout}
        )


# ==============================================================================
# Industrial Bus Exceptions
# ==============================================================================

class IndustrialBusError(CommunicationError):
    """Base exception for industrial bus communication."""
    pass


class BusConnectionError(IndustrialBusError):
    """Raised when connection to industrial bus fails."""
    def __init__(
        self,
        bus_type: str,
        endpoint: str,
        reason: Optional[str] = None,
        original_exception: Optional[Exception] = None
    ):
        super().__init__(
            f"{bus_type} connection to '{endpoint}' failed" + (f": {reason}" if reason else ""),
            details={"bus_type": bus_type, "endpoint": endpoint},
            original_exception=original_exception
        )


class BusTimeoutError(IndustrialBusError):
    """Raised when bus operation times out."""
    def __init__(self, bus_type: str, operation: str, timeout: float):
        super().__init__(
            f"{bus_type} operation '{operation}' timed out after {timeout}s",
            details={"bus_type": bus_type, "operation": operation, "timeout": timeout}
        )


class BusDiagnosticError(IndustrialBusError):
    """Raised when bus diagnostic detects failure."""
    def __init__(self, bus_type: str, diagnostic_code: str, message: str):
        super().__init__(
            f"{bus_type} diagnostic error {diagnostic_code}: {message}",
            details={"bus_type": bus_type, "code": diagnostic_code}
        )


class BusConfigurationError(IndustrialBusError):
    """Raised when bus configuration is invalid."""
    def __init__(self, bus_type: str, config_issue: str):
        super().__init__(
            f"{bus_type} configuration error: {config_issue}",
            details={"bus_type": bus_type, "issue": config_issue}
        )


class ModbusError(IndustrialBusError):
    """Base exception for Modbus-specific errors."""
    pass


class ModbusExceptionError(ModbusError):
    """Raised when Modbus device returns an exception."""
    def __init__(self, function_code: int, exception_code: int, message: str):
        super().__init__(
            f"Modbus exception 0x{exception_code:02X} on function 0x{function_code:02X}: {message}",
            details={"function_code": function_code, "exception_code": exception_code}
        )


class OPCUAError(IndustrialBusError):
    """Base exception for OPC-UA-specific errors."""
    pass


class OPCUABadStatusError(OPCUAError):
    """Raised when OPC-UA operation returns bad status."""
    def __init__(self, status_code: str, operation: str):
        super().__init__(
            f"OPC-UA bad status {status_code} during {operation}",
            details={"status_code": status_code, "operation": operation}
        )


# ==============================================================================
# External Communication Exceptions
# ==============================================================================

class ExternalCommError(CommunicationError):
    """Base exception for external communication layers (REST, gRPC, WebSocket, etc.)."""
    pass


class RESTError(ExternalCommError):
    """Base exception for REST API errors."""
    pass


class HTTPError(RESTError):
    """Raised when HTTP request fails."""
    def __init__(self, status_code: int, reason: str, url: str):
        super().__init__(
            f"HTTP {status_code} error: {reason}",
            details={"status_code": status_code, "url": url}
        )
        self.status_code = status_code
        self.url = url


class GRPCError(ExternalCommError):
    """Base exception for gRPC errors."""
    pass


class GRPCStatusError(GRPCError):
    """Raised when gRPC call fails with status code."""
    def __init__(self, status_code: str, details: str):
        super().__init__(
            f"gRPC error {status_code}: {details}",
            details={"status_code": status_code}
        )


class WebSocketError(ExternalCommError):
    """Base exception for WebSocket errors."""
    pass


class WebSocketConnectionError(WebSocketError):
    """Raised when WebSocket connection fails."""
    def __init__(self, url: str, reason: Optional[str] = None):
        super().__init__(
            f"WebSocket connection to '{url}' failed" + (f": {reason}" if reason else ""),
            details={"url": url}
        )


class MQTTError(ExternalCommError):
    """Base exception for MQTT errors."""
    pass


class MQTTConnectionError(MQTTError):
    """Raised when MQTT broker connection fails."""
    def __init__(self, broker: str, port: int, reason: Optional[str] = None):
        super().__init__(
            f"MQTT connection to {broker}:{port} failed" + (f": {reason}" if reason else ""),
            details={"broker": broker, "port": port}
        )


class SharedMemoryError(ExternalCommError):
    """Base exception for shared memory communication errors."""
    pass


class SharedMemoryAccessError(SharedMemoryError):
    """Raised when shared memory access fails."""
    def __init__(self, segment_name: str, operation: str, reason: Optional[str] = None):
        super().__init__(
            f"Shared memory '{segment_name}' {operation} failed" + (f": {reason}" if reason else ""),
            details={"segment": segment_name, "operation": operation}
        )


# ==============================================================================
# Retry and Recovery Exceptions
# ==============================================================================

class RetryableError(CommunicationError):
    """
    Exception indicating the operation can be retried.
    
    Used for transient errors where retry might succeed.
    """
    def __init__(self, message: str, retry_after: Optional[float] = None):
        super().__init__(message, details={"retry_after": retry_after})
        self.retry_after = retry_after


class NonRetryableError(CommunicationError):
    """
    Exception indicating the operation should not be retried.
    
    Used for permanent errors where retry will always fail.
    """
    pass
