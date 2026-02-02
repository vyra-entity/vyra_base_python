"""
External Communication Layer

Communication protocols for external system integration:
- Redis: Pub/Sub, Key-Value Storage
- gRPC: High-performance RPC
- MQTT: IoT Messaging
- REST: HTTP APIs
- WebSocket: Real-time bidirectional communication

All protocols optional with graceful degradation.
"""
from vyra_base.com.core.exceptions import (
    ProtocolUnavailableError,
    ProviderError,
    CommunicationError,
)

# Optional protocol imports
try:
    from vyra_base.com.external.redis import RedisProvider
except ImportError:
    RedisProvider = None

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
    "ProtocolUnavailableError",
    "ProviderError",
    "CommunicationError",
    "RedisProvider",
    "GrpcProvider",
    "MqttProvider",
    "RestProvider",
    "WebSocketProvider",
]
