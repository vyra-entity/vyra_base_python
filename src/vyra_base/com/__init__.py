"""
VYRA Base Communication Module (COM)

Multi-protocol communication system with automatic protocol selection and fallback.

Provides:
- Multi-protocol support (ROS2, Redis, gRPC, MQTT, REST, WebSocket, Modbus, OPC UA)
- Automatic protocol fallback
- Protocol-agnostic decorators (@remove_server, @remote_publisher, @remote_job)
- Backward compatibility with legacy ROS2-only datalayer API

New Code (Recommended):
    from vyra_base.com import remove_server, InterfaceFactory, ProtocolType
    
Legacy Code (Still Supported):
    from vyra_base.com import remove_server
"""

# ============================================================================
# NEW MULTI-PROTOCOL API (Recommended)
# ============================================================================

# Core components
from vyra_base.com.core import (
    # Exceptions
    CommunicationError,
    ProtocolUnavailableError,
    ProtocolNotInitializedError,
    TransportError,
    ProviderError,
    InterfaceError,
    TServerError,
    TSubscriberError,
    ActionServerError,
    # Types
    ProtocolType,
    InterfaceType,
    # Factory & Decorators
    InterfaceFactory,
    remove_server,
    remote_publisher,
    remote_actionServer,
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

# Provider Registry
from vyra_base.com.providers import (
    AbstractProtocolProvider,
    ProviderRegistry,
)

# Feeders - Multi-protocol support (optional)
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
# LEGACY COMPATIBILITY (Deprecated - use InterfaceFactory instead)
# ============================================================================
#
# NOTE: Legacy functions (create_vyra_callable, create_vyra_speaker, etc.) have been removed.
# External modules using them should migrate to:
#   from vyra_base.com import InterfaceFactory, remove_server, ProtocolType
#
# The @remove_server decorator now supports multi-protocol.
# Old code using it will continue to work but gets ROS2-only behavior.
#

# ROS2 infrastructure - Using transport/t_ros2 implementations (optional)
try:
    from vyra_base.com.transport.t_ros2.communication.publisher import ROS2Publisher
    from vyra_base.com.transport.t_ros2.communication.subscriber import ROS2Subscriber
    from vyra_base.com.transport.t_ros2.node import VyraNode, CheckerNode, NodeSettings
    from vyra_base.com.transport.t_ros2.communication.action_client import ROS2ActionClient
    from vyra_base.com.transport.t_ros2.communication.action_server import ROS2ActionServer
    from vyra_base.com.transport.t_ros2.communication.service_client import ROS2ServiceClient
    from vyra_base.com.transport.t_ros2.communication.service_server import ROS2ServiceServer
except ImportError:
    ROS2Subscriber = None
    VyraNode = None
    CheckerNode = None
    NodeSettings = None
    ROS2ActionClient = None
    ROS2ActionServer = None
    ROS2ServiceClient = None
    ROS2ServiceServer = None

# IPC - Inter-Process Communication (moved to external/grpc)
try:
    from vyra_base.com.external.grpc import GrpcServer, GrpcClient
except ImportError:
    GrpcServer = None
    GrpcClient = None

__all__ = [
    # ========================================================================
    # NEW MULTI-PROTOCOL API (Recommended)
    # ========================================================================
    
    # Exceptions
    "CommunicationError",
    "ProtocolUnavailableError",
    "ProtocolNotInitializedError",
    "TransportError",
    "ProviderError",
    "InterfaceError",
    "TServerError",
    "TSubscriberError",
    "ActionServerError",
    
    # Types
    "ProtocolType",
    "InterfaceType",
    
    # Factory
    "InterfaceFactory",
    
    # Decorators (Multi-protocol)
    "remote_service",
    "remote_publisher",
    "remote_actionServer",
    
    # Providers
    "AbstractProtocolProvider",
    "ProviderRegistry",
    
    # Topic Builder
    "TopicBuilder",
    "TopicComponents",
    "TopicInterfaceType",
    "create_topic_builder",
    "build_topic",
    "parse_topic",
    
    # Feeders (Multi-protocol)
    "BaseFeeder",
    "StateFeeder",
    "NewsFeeder",
    "ErrorFeeder",
    
    # ========================================================================
    # LEGACY ROS2 API (Minimal exports - migrate to InterfaceFactory)
    # ========================================================================
    
    # ROS2 Classes
    "ROS2Job",
    "ROS2Callable",
    "ROS2Speaker",
    "ROS2Publisher",
    "ROS2Subscriber",
    "VyraNode",
    "CheckerNode",
    "NodeSettings",
    "ROS2ActionClient",
    "ROS2ActionServer",
    "ROS2ServiceClient",
    "ROS2ServiceServer",
    
    # IPC Classes
    "GrpcServer",
    "GrpcClient",
]
