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
    TServerError,
    TSubscriberError,
    ActionServerError,
    IndustrialBusError,
    BusConnectionError,
    BusTimeoutError,
    BusDiagnosticError,
)

from vyra_base.com.core.types import (
    ProtocolType,
    InterfaceType,
    AccessLevel,
    ActionStatus,
    DisplayStyle,
    InterfaceMetadata,
    VyraTransport,
    # New unified types
    VyraPublisher,
    VyraSubscriber,
    VyraServer,
    VyraClient,
    VyraActionServer,
    VyraActionClient
)

from vyra_base.com.core.factory import InterfaceFactory

from vyra_base.com.core.blueprints import (
    HandlerBlueprint,
    ServiceBlueprint,
    PublisherBlueprint,
    SubscriberBlueprint,
    ActionBlueprint,
    InterfaceType as BlueprintInterfaceType,
    AnyBlueprint,
)

from vyra_base.com.core.abstract_handlers import (
    IServiceHandler,
    IActionHandler,
    IGoalHandle,
)

from vyra_base.com.core.callback_registry import CallbackRegistry

from vyra_base.com.core.decorators import (
    remote_service,
    remote_publisher,
    remote_subscriber,
    remote_actionServer,
    get_decorated_methods,
    bind_decorated_callbacks,
    # Backward compatibility aliases
    remote_callable,
    remote_callable_ros2,
    remote_speaker,
    remote_listener,
    remote_job,
)

from vyra_base.com.core.topic_builder import (
    TopicBuilder,
    TopicComponents,
    InterfaceType as TopicInterfaceType,
    create_topic_builder,
    build_topic,
    parse_topic,
)

from vyra_base.com.core.interface_path_registry import (
    InterfacePathRegistry,
    get_interface_registry,
)

from vyra_base.com.core.interface_loader import InterfaceLoader

from vyra_base.helper.ros2_env_helper import (
    update_ament_prefix_path,
    update_python_path,
    ensure_interface_package_discoverable,
    ensure_workspace_discoverable,
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
    "TServerError",
    "TSubscriberError",
    "ActionServerError",
    "IndustrialBusError",
    "BusConnectionError",
    "BusTimeoutError",
    "BusDiagnosticError",
    # Types
    "ProtocolType",
    "InterfaceType",
    "AccessLevel",
    "ActionStatus",
    "DisplayStyle",
    "InterfaceMetadata",
    "VyraTransport",
    # New unified types
    "VyraPublisher",
    "VyraSubscriber",
    "VyraServer",
    "VyraClient",
    "VyraActionServer",
    "VyraActionClient",
    # Blueprints
    "HandlerBlueprint",
    "ServiceBlueprint",
    "PublisherBlueprint",
    "SubscriberBlueprint",
    "ActionBlueprint",
    "AnyBlueprint",
    "CallbackRegistry",
    # Abstract Handlers (REQUIRED)
    "IServiceHandler",
    "IActionHandler",
    "IGoalHandle",
    # Factory
    "InterfaceFactory",
    # Decorators
    "remote_service",
    "remote_publisher",
    "remote_subscriber",
    "remote_actionServer",
    "get_decorated_methods",
    "bind_decorated_callbacks",
    # Topic Builder
    "TopicBuilder",
    "TopicComponents",
    "TopicInterfaceType",
    "create_topic_builder",
    "build_topic",
    "parse_topic",
    # Interface Loading
    "InterfacePathRegistry",
    "get_interface_registry",
    "InterfaceLoader",
    # ROS2 Environment Helper
    "update_ament_prefix_path",
    "update_python_path",
    "ensure_interface_package_discoverable",
    "ensure_workspace_discoverable",
]
