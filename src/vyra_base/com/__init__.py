"""
VYRA Base Communication Module (COM)

Multi-protocol communication system with automatic protocol selection and fallback.

Architecture
------------
Transport Layer  (in-process / low-latency)
    ROS2  · Zenoh  · Redis  · UDS

External Layer   (out-of-process / cross-service)
    gRPC  · MQTT  · REST  · WebSocket  · Shared Memory

Industrial Layer
    Modbus  · OPC UA

Public API
----------
Decorators (use on component methods):
    @remote_service       — expose a method as a callable service (request/response)
    @remote_publisher     — expose a method as a publisher (fire-and-forget)
    @remote_subscriber    — register a method as a message subscriber
    @remote_actionServer  — expose a method group as an action server (goal/feedback/result)

Factory:
    InterfaceFactory      — create server/client/publisher/subscriber/action objects

Example::

    from vyra_base.com import remote_service, remote_publisher, InterfaceFactory, ProtocolType

    class MyComponent:
        @remote_service(name="ping", protocols=[ProtocolType.ZENOH])
        async def ping(self, request, response=None):
            return {"pong": True}
"""

# ============================================================================
# Core Components (always available)
# ============================================================================

from vyra_base.com.core import (
    # Exceptions
    CommunicationError,
    ProtocolUnavailableError,
    ProtocolNotInitializedError,
    TransportError,
    ProviderError,
    ProviderNotFoundError,
    ProviderRegistrationError,
    InterfaceError,
    TServerError,
    TSubscriberError,
    ActionServerError,
    # Types
    ProtocolType,
    InterfaceType,
    AccessLevel,
    ActionStatus,
    VyraPublisher,
    VyraSubscriber,
    VyraServer,
    VyraClient,
    VyraActionServer,
    VyraActionClient,
    # Abstract Handlers (implement these in your component)
    IServiceHandler,
    IActionHandler,
    IGoalHandle,
    # Factory
    InterfaceFactory,
    # Decorators
    remote_service,
    remote_publisher,
    remote_subscriber,
    remote_actionServer,
    get_decorated_methods,
    bind_decorated_callbacks,
    # Blueprint Infrastructure
    HandlerBlueprint,
    ServiceBlueprint,
    PublisherBlueprint,
    SubscriberBlueprint,
    ActionBlueprint,
    CallbackRegistry,
)

# Topic Builder for naming conventions
from vyra_base.com.core.topic_builder import (
    TopicBuilder,
    TopicComponents,
    InterfaceType as TopicInterfaceType,
    create_topic_builder,
    build_topic,
    parse_topic,
)

# Interface path registry & loader
from vyra_base.com.core import (
    InterfacePathRegistry,
    get_interface_registry,
    InterfaceLoader,
)

# Provider Registry
from vyra_base.com.providers import (
    AbstractProtocolProvider,
    ProviderRegistry,
)

# ============================================================================
# Feeders — automatic publication of state / news / errors (optional)
# ============================================================================

try:
    from vyra_base.com.feeder.feeder import BaseFeeder
    from vyra_base.com.feeder.state_feeder import StateFeeder
    from vyra_base.com.feeder.news_feeder import NewsFeeder
    from vyra_base.com.feeder.error_feeder import ErrorFeeder
except ImportError:
    BaseFeeder = None
    StateFeeder = None
    NewsFeeder = None
    ErrorFeeder = None

# ============================================================================
# Transport Layer — ROS2, Zenoh, Redis, UDS  (all optional at import time)
# ============================================================================

# ROS2
try:
    from vyra_base.com.transport.t_ros2.communication.publisher import ROS2Publisher
    from vyra_base.com.transport.t_ros2.communication.subscriber import ROS2Subscriber
    from vyra_base.com.transport.t_ros2.node import VyraNode, CheckerNode, NodeSettings
    from vyra_base.com.transport.t_ros2.communication.action_client import ROS2ActionClient
    from vyra_base.com.transport.t_ros2.communication.action_server import ROS2ActionServer
    from vyra_base.com.transport.t_ros2.communication.service_client import ROS2ServiceClient
    from vyra_base.com.transport.t_ros2.communication.service_server import ROS2ServiceServer
except ImportError:
    ROS2Publisher = None
    ROS2Subscriber = None
    VyraNode = None
    CheckerNode = None
    NodeSettings = None
    ROS2ActionClient = None
    ROS2ActionServer = None
    ROS2ServiceClient = None
    ROS2ServiceServer = None

# Zenoh
try:
    from vyra_base.com.transport.t_zenoh.provider import ZenohProvider
    from vyra_base.com.transport.t_zenoh.communication.session import ZenohSession, SessionConfig, SessionMode
except ImportError:
    ZenohProvider = None
    ZenohSession = None
    SessionConfig = None
    SessionMode = None

# Redis transport
try:
    from vyra_base.com.transport.t_redis.provider import RedisProvider
    from vyra_base.com.transport.t_redis.communication.redis_client import RedisClient
except ImportError:
    RedisProvider = None
    RedisClient = None

# UDS (Unix Domain Sockets)
try:
    from vyra_base.com.transport.t_uds.provider import UDSProvider
except ImportError:
    UDSProvider = None

# ============================================================================
# External Layer — gRPC, MQTT, REST, WebSocket, Shared Memory (all optional)
# ============================================================================

try:
    from vyra_base.com.external.grpc import GrpcServer, GrpcClient
except ImportError:
    GrpcServer = None
    GrpcClient = None

try:
    from vyra_base.com.external.mqtt import MqttClient
except ImportError:
    MqttClient = None

try:
    from vyra_base.com.external.rest import RestClient
except ImportError:
    RestClient = None

try:
    from vyra_base.com.external.websocket import WebSocketClient
except ImportError:
    WebSocketClient = None

try:
    from vyra_base.com.external.shared_memory import (
        SharedMemorySegment,
        SharedMemorySerializer,
        SharedMemoryDiscovery,
    )
except ImportError:
    SharedMemorySegment = None
    SharedMemorySerializer = None
    SharedMemoryDiscovery = None

# ============================================================================
# Public API declaration
# ============================================================================

__all__ = [
    # ------------------------------------------------------------------
    # Exceptions
    # ------------------------------------------------------------------
    "CommunicationError",
    "ProtocolUnavailableError",
    "ProtocolNotInitializedError",
    "TransportError",
    "ProviderError",
    "ProviderNotFoundError",
    "ProviderRegistrationError",
    "InterfaceError",
    "TServerError",
    "TSubscriberError",
    "ActionServerError",
    # ------------------------------------------------------------------
    # Types & Enums
    # ------------------------------------------------------------------
    "ProtocolType",
    "InterfaceType",
    "AccessLevel",
    "ActionStatus",
    "VyraPublisher",
    "VyraSubscriber",
    "VyraServer",
    "VyraClient",
    "VyraActionServer",
    "VyraActionClient",
    # ------------------------------------------------------------------
    # Abstract Handlers
    # ------------------------------------------------------------------
    "IServiceHandler",
    "IActionHandler",
    "IGoalHandle",
    # ------------------------------------------------------------------
    # Factory
    # ------------------------------------------------------------------
    "InterfaceFactory",
    # ------------------------------------------------------------------
    # Decorators
    # ------------------------------------------------------------------
    "remote_service",
    "remote_publisher",
    "remote_subscriber",
    "remote_actionServer",
    "get_decorated_methods",
    "bind_decorated_callbacks",
    # ------------------------------------------------------------------
    # Blueprint Infrastructure
    # ------------------------------------------------------------------
    "HandlerBlueprint",
    "ServiceBlueprint",
    "PublisherBlueprint",
    "SubscriberBlueprint",
    "ActionBlueprint",
    "CallbackRegistry",
    # ------------------------------------------------------------------
    # Provider Registry
    # ------------------------------------------------------------------
    "AbstractProtocolProvider",
    "ProviderRegistry",
    # ------------------------------------------------------------------
    # Topic Builder
    # ------------------------------------------------------------------
    "TopicBuilder",
    "TopicComponents",
    "TopicInterfaceType",
    "create_topic_builder",
    "build_topic",
    "parse_topic",
    # ------------------------------------------------------------------
    # Interface Loading
    # ------------------------------------------------------------------
    "InterfacePathRegistry",
    "get_interface_registry",
    "InterfaceLoader",
    # ------------------------------------------------------------------
    # Feeders
    # ------------------------------------------------------------------
    "BaseFeeder",
    "StateFeeder",
    "NewsFeeder",
    "ErrorFeeder",
    # ------------------------------------------------------------------
    # Transport Layer — ROS2
    # ------------------------------------------------------------------
    "ROS2Publisher",
    "ROS2Subscriber",
    "VyraNode",
    "CheckerNode",
    "NodeSettings",
    "ROS2ActionClient",
    "ROS2ActionServer",
    "ROS2ServiceClient",
    "ROS2ServiceServer",
    # ------------------------------------------------------------------
    # Transport Layer — Zenoh
    # ------------------------------------------------------------------
    "ZenohProvider",
    "ZenohSession",
    "SessionConfig",
    "SessionMode",
    # ------------------------------------------------------------------
    # Transport Layer — Redis
    # ------------------------------------------------------------------
    "RedisProvider",
    "RedisClient",
    # ------------------------------------------------------------------
    # Transport Layer — UDS
    # ------------------------------------------------------------------
    "UDSProvider",
    # ------------------------------------------------------------------
    # External Layer
    # ------------------------------------------------------------------
    "GrpcServer",
    "GrpcClient",
    "MqttClient",
    "RestClient",
    "WebSocketClient",
    "SharedMemorySegment",
    "SharedMemorySerializer",
    "SharedMemoryDiscovery",
]
