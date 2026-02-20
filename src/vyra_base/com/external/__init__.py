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
    from vyra_base.com.external.shared_memory import (
        SharedMemorySegment,
        SegmentInfo,
        POSIX_IPC_AVAILABLE,
        SharedMemorySerializer,
        SerializationFormat,
        MessageType,
        calculate_segment_size,
        SharedMemoryDiscovery,
        SegmentDiscoveryInfo,
        get_discovery,
        DISCOVERY_DIR,
    )
except ImportError:
    SharedMemorySegment = None
    SegmentInfo = None
    POSIX_IPC_AVAILABLE = False
    SharedMemorySerializer = None
    SerializationFormat = None
    MessageType = None
    calculate_segment_size = None
    SharedMemoryDiscovery = None
    SegmentDiscoveryInfo = None
    get_discovery = None
    DISCOVERY_DIR = None
    logger.debug("⚠️  Shared Memory protocol unavailable")

try:
    from vyra_base.com.external.grpc import (
        GrpcClient,
        GrpcServer,
        GRPC_CLIENT_AVAILABLE,
        GRPC_SERVER_AVAILABLE,
    )
except ImportError:
    GrpcClient = None
    GrpcServer = None
    GRPC_CLIENT_AVAILABLE = False
    GRPC_SERVER_AVAILABLE = False
    logger.debug("⚠️  gRPC protocol unavailable")

try:
    from vyra_base.com.external.mqtt import (
        MqttClient,
        MQTT_AVAILABLE,
    )
except ImportError:
    MqttClient = None
    MQTT_AVAILABLE = False
    logger.debug("⚠️  MQTT protocol unavailable")

try:
    from vyra_base.com.external.rest import (
        RestClient,
        REST_AVAILABLE,
    )
except ImportError:
    RestClient = None
    REST_AVAILABLE = False
    logger.debug("⚠️  REST protocol unavailable")

try:
    from vyra_base.com.external.websocket import (
        WebSocketClient,
        WEBSOCKET_AVAILABLE,
    )
except ImportError:
    WebSocketClient = None
    WEBSOCKET_AVAILABLE = False
    logger.debug("⚠️  WebSocket protocol unavailable")

# TCP and UDP (asyncio stdlib — always available)
try:
    from vyra_base.com.external.tcp import AsyncTcpClient, AsyncTcpServer
    TCP_AVAILABLE = True
except ImportError as e:
    AsyncTcpClient = None
    AsyncTcpServer = None
    TCP_AVAILABLE = False
    logger.debug(f"⚠️  TCP unavailable: {e}")

try:
    from vyra_base.com.external.udp import AsyncUdpClient, AsyncUdpServer
    UDP_AVAILABLE = True
except ImportError as e:
    AsyncUdpClient = None
    AsyncUdpServer = None
    UDP_AVAILABLE = False
    logger.debug(f"⚠️  UDP unavailable: {e}")

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
    # External interfaces
    # Shared Memory
    "SharedMemorySegment",
    "SegmentInfo",
    "POSIX_IPC_AVAILABLE",
    "SharedMemorySerializer",
    "SerializationFormat",
    "MessageType",
    "calculate_segment_size",
    "SharedMemoryDiscovery",
    "SegmentDiscoveryInfo",
    "get_discovery",
    "DISCOVERY_DIR",
    # gRPC
    "GrpcClient",
    "GrpcServer",
    "GRPC_CLIENT_AVAILABLE",
    "GRPC_SERVER_AVAILABLE",
    # MQTT
    "MqttClient",
    "MQTT_AVAILABLE",
    # REST
    "RestClient",
    "REST_AVAILABLE",
    # WebSocket
    "WebSocketClient",
    "WEBSOCKET_AVAILABLE",
    # TCP
    "AsyncTcpClient",
    "AsyncTcpServer",
    "TCP_AVAILABLE",
    # UDP
    "AsyncUdpClient",
    "AsyncUdpServer",
    "UDP_AVAILABLE",
]
