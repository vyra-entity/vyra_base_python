"""
VYRA Base Communication Module (COM)

Multi-protocol communication system with automatic protocol selection and fallback.

Provides:
- Multi-protocol support (ROS2, Redis, gRPC, MQTT, REST, WebSocket, Modbus, OPC UA)
- Automatic protocol fallback
- Protocol-agnostic decorators (@remote_callable, @remote_speaker, @remote_job)
- Backward compatibility with legacy ROS2-only datalayer API

New Code (Recommended):
    from vyra_base.com import remote_callable, InterfaceFactory, ProtocolType
    
Legacy Code (Still Supported):
    from vyra_base.com import remote_callable
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
    CallableError,
    SpeakerError,
    JobError,
    # Types
    ProtocolType,
    InterfaceType,
    # Factory & Decorators
    InterfaceFactory,
    remote_callable,
    remote_speaker,
    remote_job,
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
#   from vyra_base.com import InterfaceFactory, remote_callable, ProtocolType
#
# The @remote_callable decorator now supports multi-protocol.
# Old code using it will continue to work but gets ROS2-only behavior.
#

# Use new ROS2 implementations (optional)
try:
    from vyra_base.com.transport.t_ros2.vyra_models.callable import ROS2Callable
    from vyra_base.com.transport.t_ros2.vyra_models.speaker import ROS2Speaker
    from vyra_base.com.transport.t_ros2.vyra_models.job import ROS2Job
except ImportError:
    ROS2Callable = None
    ROS2Speaker = None
    ROS2Job = None

# ROS2 infrastructure - Using transport/t_ros2 implementations (optional)
try:
    from vyra_base.com.transport.t_ros2.communication.publisher import VyraPublisher
    from vyra_base.com.transport.t_ros2.communication.subscriber import VyraSubscriber
    from vyra_base.com.transport.t_ros2.node import VyraNode, CheckerNode, NodeSettings
    from vyra_base.com.transport.t_ros2.communication.action_client import VyraActionClient
    from vyra_base.com.transport.t_ros2.communication.action_server import VyraActionServer
    from vyra_base.com.transport.t_ros2.communication.service_client import VyraServiceClient
    from vyra_base.com.transport.t_ros2.communication.service_server import VyraServiceServer
except ImportError:
    VyraSubscriber = None
    VyraNode = None
    CheckerNode = None
    NodeSettings = None
    VyraActionClient = None
    VyraActionServer = None
    VyraServiceClient = None
    VyraServiceServer = None

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
    "CallableError",
    "SpeakerError",
    "JobError",
    
    # Types
    "ProtocolType",
    "InterfaceType",
    
    # Factory
    "InterfaceFactory",
    
    # Decorators (Multi-protocol)
    "remote_callable",
    "remote_speaker",
    "remote_job",
    
    # Providers
    "AbstractProtocolProvider",
    "ProviderRegistry",
    
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
    "VyraPublisher",
    "VyraSubscriber",
    "VyraNode",
    "CheckerNode",
    "NodeSettings",
    "VyraActionClient",
    "VyraActionServer",
    "VyraServiceClient",
    "VyraServiceServer",
    
    # IPC Classes
    "GrpcServer",
    "GrpcClient",
]
