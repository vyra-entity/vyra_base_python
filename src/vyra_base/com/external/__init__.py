"""
External Communication Layer

Communication protocols for external system integration:
- Shared Memory: Zero-copy local IPC via POSIX shared memory
- gRPC: High-performance RPC
- MQTT: IoT Messaging
- REST: HTTP APIs
- WebSocket: Real-time bidirectional communication
- Registry: Central management and monitoring for all external protocols

All protocols optional with graceful degradation.
"""
import logging

from vyra_base.com.core.exceptions import (
    ProtocolUnavailableError,
    ProviderError,
    CommunicationError,
)

logger = logging.getLogger(__name__)

# Try importing registry (always available, no external deps)
try:
    from vyra_base.com.external.registry import (
        ExternalRegistry,
        ProtocolConnection,
        ProtocolStatus,
        get_global_registry,
    )
    REGISTRY_AVAILABLE = True
    logger.debug("✅ External protocol registry available")
except ImportError as e:
    ExternalRegistry = None
    ProtocolConnection = None
    ProtocolStatus = None
    get_global_registry = None
    REGISTRY_AVAILABLE = False
    logger.debug(f"⚠️  External protocol registry unavailable: {e}")

# Optional protocol imports
try:
    from vyra_base.com.external.shared_memory import SharedMemoryProvider
except ImportError:
    SharedMemoryProvider = None

try:
    from vyra_base.com.external.grpc import GrpcProvider
except ImportError:
    GrpcProvider = None

try:
    from vyra_base.com.external.mqtt import MqttProvider
except ImportError:
    MqttProvider = None

try:
    from vyra_base.com.external.rest import RestProvider
except ImportError:
    RestProvider = None

try:
    from vyra_base.com.external.websocket import WebSocketProvider
except ImportError:
    WebSocketProvider = None

__all__ = [
    # Core exceptions
    "ProtocolUnavailableError",
    "ProviderError",
    "CommunicationError",
    # Registry
    "ExternalRegistry",
    "ProtocolConnection",
    "ProtocolStatus",
    "get_global_registry",
    "REGISTRY_AVAILABLE",
    # Protocol providers
    "SharedMemoryProvider",
    "GrpcProvider",
    "MqttProvider",
    "RestProvider",
    "WebSocketProvider",
]
