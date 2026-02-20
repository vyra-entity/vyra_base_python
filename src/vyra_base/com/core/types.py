"""
Core Communication Types

Defines base data structures for all communication protocols.
VYRA uses three communication paradigms:
- Callable: Request-Response (ROS2 Service, gRPC Unary, etc.)
- Speaker: Publish-Subscribe (ROS2 Topic, MQTT, Redis Pub/Sub)
- Job: Long-Running Task (ROS2 Action, async Task)
"""
import json
from abc import ABC, abstractmethod
from dataclasses import dataclass, field, asdict
from enum import Enum
from typing import Any, Callable, Coroutine, Optional, Dict, List
from datetime import datetime

from vyra_base.com.core.topic_builder import TopicBuilder

class ProtocolType(str, Enum):
    """Supported communication protocols."""
    # Communication Abstraction Layer (CAL)
    ROS2 = "ros2"
    ZENOH = "zenoh"
    REDIS = "redis"
    UDS = "uds"
    
    # External Communication Layer
    SHARED_MEMORY = "sharedmemory"
    MQTT = "mqtt"
    GRPC = "grpc"
    REST = "rest"
    WEBSOCKET = "websocket"
    
    # Industrial Buses (SCADA/MES Level - Northbound)
    MODBUS = "modbus"
    OPCUA = "opcua"


class InterfaceType(str, Enum):
    """VYRA communication interface types."""
    # New unified types
    PUBLISHER = "publisher"        # Publish-only (no callback)
    SUBSCRIBER = "subscriber"      # Subscribe with callback
    SERVER = "server"              # Request-Response server
    CLIENT = "client"              # Request-Response client
    ACTION_SERVER = "actionServer" # Long-running task server
    ACTION_CLIENT = "actionClient" # Long-running task client

class CallbackType(str, Enum):
    """Types of callbacks for communication interfaces."""
    SUBSCRIBER = "subscriber_callback"
    SERVER = "response_callback"
    ACTION_SERVER_GOAL = "handle_goal_request"
    ACTION_SERVER_CANCEL = "handle_cancel_request"
    ACTION_SERVER_EXECUTION = "execution_callback"
    ACTION_CLIENT_DIRECT_RESPONSE = "direct_response_callback"
    ACTION_CLIENT_FEEDBACK = "feedback_callback"
    ACTION_CLIENT_GOAL = "goal_callback"

class AccessLevel(str, Enum):
    """Access control levels for interfaces."""
    PUBLIC = "public"
    PROTECTED = "protected"
    PRIVATE = "private"
    INTERNAL = "internal"


class ActionStatus(Enum):
    """
    Status values for action goals (ROS2 Action compatible).
    
    Used to track the lifecycle state of action server goals.
    """
    UNKNOWN = 0       # Unknown state
    ACCEPTED = 1      # Goal accepted by server
    EXECUTING = 2     # Goal is currently executing
    CANCELING = 3     # Goal is being canceled
    SUCCEEDED = 4     # Goal completed successfully
    CANCELED = 5      # Goal was canceled
    ABORTED = 6       # Goal aborted due to error


@dataclass
class DisplayStyle:
    """Display configuration for interface visibility."""
    visible: bool = True
    icon: str = ""
    color: str = ""
    category: str = ""


@dataclass
class InterfaceMetadata:
    """Metadata for communication interfaces."""
    name: str
    interface_type: InterfaceType
    protocol: ProtocolType
    description: str = ""
    access_level: AccessLevel = AccessLevel.PUBLIC
    display_style: DisplayStyle = field(default_factory=DisplayStyle)
    created_at: datetime = field(default_factory=datetime.now)
    tags: List[str] = field(default_factory=list)
    custom_data: Dict[str, Any] = field(default_factory=dict)
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary representation."""
        return asdict(self)
    
    def to_json(self) -> str:
        """Convert to JSON string."""
        data = self.to_dict()
        # Convert datetime to ISO format
        data['created_at'] = data['created_at'].isoformat()
        # Convert display_style
        if isinstance(data.get('display_style'), DisplayStyle):
            data['display_style'] = asdict(data['display_style'])
        return json.dumps(data)


class VyraTransport(ABC):
    """
    Abstract base class for all VYRA communication interfaces.
    
    All communication paradigms (Publisher, Server, ActionServer) inherit from this.
    """
    
    def __init__(
        self,
        name: str,
        protocol: ProtocolType,
        interface_type: InterfaceType,
        **kwargs
    ):
        # Extract known InterfaceMetadata parameters
        metadata_params: Dict[str, str | ProtocolType | InterfaceType | Any] = {
            'name': name,
            'protocol': protocol,
            'interface_type': interface_type
        }
        
        # Map known kwargs to InterfaceMetadata fields
        known_fields = {'description', 'access_level', 'display_style', 'created_at', 'tags', 'custom_data'}
        for key in known_fields:
            if key in kwargs:
                metadata_params[key] = kwargs.pop(key)
        
        # Store remaining kwargs in custom_data
        if kwargs:
            custom_data: dict[str, Any] = metadata_params.get('custom_data', {})
            custom_data.update(kwargs)
            metadata_params['custom_data'] = custom_data
        
        self.metadata = InterfaceMetadata(**metadata_params)
        self._initialized = False
    
    @property
    def name(self) -> str:
        """Get interface name."""
        return self.metadata.name
    
    @property
    def protocol(self) -> ProtocolType:
        """Get protocol type."""
        return self.metadata.protocol
    
    @property
    def interface_type(self) -> InterfaceType:
        """Get interface type."""
        return self.metadata.interface_type
    
    @abstractmethod
    async def initialize(self) -> bool:
        """
        Initialize the interface.
        
        Returns:
            bool: True if initialization successful
        """
        pass
    
    @abstractmethod
    async def shutdown(self) -> None:
        """Shutdown the interface and cleanup resources."""
        pass
    
    def is_initialized(self) -> bool:
        """Check if interface is initialized."""
        return self._initialized

    def is_connected(self) -> bool:
        """Check if transport layer is connected (override if applicable)."""
        return self._initialized

# ============================================================================
# NEW UNIFIED TRANSPORT LAYER CLASSES (Async-first)
# ============================================================================

@dataclass
class VyraPublisher(VyraTransport):
    """
    Publisher interface for one-way message publishing.
    
    Represents publish-only communication (no callbacks):
    - ROS2: Topic Publisher
    - Zenoh: Publisher
    - Redis: Pub/Sub Publisher
    - UDS: Datagram Socket Sender
    """
    
    def __init__(
        self,
        name: str,
        topic_builder: TopicBuilder,
        protocol: ProtocolType = ProtocolType.ROS2,
        **kwargs
    ):
        super().__init__(name, protocol, InterfaceType.PUBLISHER, **kwargs)
        if not topic_builder:
            raise InterfaceError("TopicBuilder is required for Publisher")
        self.topic_builder: TopicBuilder = topic_builder
        self._transport_handle: Optional[Any] = None
    
    async def initialize(self) -> bool:
        """Initialize publisher with transport layer."""
        self._initialized = True
        return True
    
    async def shutdown(self) -> None:
        """Shutdown publisher."""
        self._initialized = False
        self._transport_handle = None
    
    async def publish(self, message: Any) -> bool:
        """
        Publish a message (async).
        
        Args:
            message: Message data to publish
            
        Returns:
            bool: True if published successfully
        """
        if not self._initialized:
            raise InterfaceError(f"Publisher '{self.name}' not initialized")
        
        # Will be implemented by transport-specific subclasses
        raise NotImplementedError


@dataclass
class VyraSubscriber(VyraTransport):
    """
    Subscriber interface for receiving messages with callback.
    
    Represents subscribe-only communication:
    - ROS2: Topic Subscriber
    - Zenoh: Subscriber
    - Redis: Pub/Sub Subscriber
    - UDS: Datagram Socket Receiver
    """
    
    def __init__(
        self,
        name: str,
        topic_builder: TopicBuilder,
        subscriber_callback: Optional[Callable] = None,
        protocol: ProtocolType = ProtocolType.ROS2,
        **kwargs
    ):
        super().__init__(name, protocol, InterfaceType.SUBSCRIBER, **kwargs)
        if not topic_builder:
            raise InterfaceError("TopicBuilder is required for Subscriber")
        self.topic_builder: TopicBuilder = topic_builder
        self.subscriber_callback = subscriber_callback  # async def callback(msg: Any) -> None
        self._transport_handle: Optional[Any] = None
    
    async def initialize(self) -> bool:
        """Initialize subscriber with transport layer."""
        self._initialized = True
        return True
    
    async def shutdown(self) -> None:
        """Shutdown subscriber."""
        self._initialized = False
        self._transport_handle = None
    
    async def subscribe(self) -> None:
        """
        Start subscribing to messages.
        Calls subscriber_callback for each received message.
        """
        if not self._initialized:
            raise InterfaceError(f"Subscriber '{self.name}' not initialized")
        
        # Will be implemented by transport-specific subclasses
        raise NotImplementedError


@dataclass
class VyraServer(VyraTransport):
    """
    Server interface for request-response communication.
    
    Represents service server:
    - ROS2: Service Server
    - Zenoh: Queryable
    - Redis: Request-Response Pattern
    - UDS: Stream Socket RPC Server
    """
    
    def __init__(
        self,
        name: str,
        topic_builder: TopicBuilder,
        response_callback: Optional[Callable[[Any], Coroutine[Any, Any, Any]]] = None,
        protocol: ProtocolType = ProtocolType.ROS2,
        **kwargs
    ):
        super().__init__(name, protocol, InterfaceType.SERVER, **kwargs)
        if not topic_builder:
            raise InterfaceError("TopicBuilder is required for Server")
        self.topic_builder: TopicBuilder = topic_builder
        self.response_callback = response_callback  # async def callback(request: Any) -> Any
        self._transport_handle: Optional[Any] = None
    
    async def initialize(self) -> bool:
        """Initialize server with transport layer."""
        self._initialized = True
        return True
    
    async def shutdown(self) -> None:
        """Shutdown server."""
        self._initialized = False
        self._transport_handle = None
    
    async def serve(self) -> None:
        """
        Start serving requests.
        Calls response_callback for each request and returns response.
        """
        if not self._initialized:
            raise InterfaceError(f"Server '{self.name}' not initialized")
        
        # Will be implemented by transport-specific subclasses
        raise NotImplementedError


@dataclass
class VyraClient(VyraTransport):
    """
    Client interface for request-response communication.
    
    Represents service client:
    - ROS2: Service Client
    - Zenoh: Query Client
    - Redis: Request-Response Pattern
    - UDS: Stream Socket RPC Client
    """
    
    def __init__(
        self,
        name: str,
        topic_builder: TopicBuilder,
        request_callback: Optional[Callable[[Any], Coroutine[Any, Any, Any]]] = None,  # Optional for async pattern
        protocol: ProtocolType = ProtocolType.ROS2,
        **kwargs
    ):
        super().__init__(name, protocol, InterfaceType.CLIENT, **kwargs)
        if not topic_builder:
            raise InterfaceError("TopicBuilder is required for Client")
        self.topic_builder: TopicBuilder = topic_builder
        self.request_callback = request_callback  # async def callback(response: Any) -> Any
        self._transport_handle: Optional[Any] = None
    
    async def initialize(self) -> bool:
        """Initialize client with transport layer."""
        self._initialized = True
        return True
    
    async def shutdown(self) -> None:
        """Shutdown client."""
        self._initialized = False
        self._transport_handle = None
    
    async def call(self, request: Any, timeout: float = 5.0) -> Any:
        """
        Send request and await response (async).
        
        Args:
            request: Request data
            timeout: Timeout in seconds
            
        Returns:
            Response data
        """
        if not self._initialized:
            raise InterfaceError(f"Client '{self.name}' not initialized")
        
        # Will be implemented by transport-specific subclasses
        raise NotImplementedError


@dataclass
class VyraActionServer(VyraTransport):
    """
    Action Server interface for long-running tasks.
    
    Represents async task server with progress feedback:
    - ROS2: Action Server
    - Zenoh: Callable + Publishers (control/feedback/result)
    - Redis: State tracking with Pub/Sub
    - UDS: Stream-based state messages
    """
    
    def __init__(
        self,
        name: str,
        topic_builder: TopicBuilder,
        handle_goal_request: Optional[Callable[[Any], Coroutine[Any, Any, bool]]] = None,
        handle_cancel_request: Optional[Callable[[Any], Coroutine[Any, Any, bool]]] = None,
        execution_callback: Optional[Callable[[Any], Coroutine[Any, Any, Any]]] = None,
        protocol: ProtocolType = ProtocolType.ROS2,
        **kwargs
    ):
        super().__init__(name, protocol, InterfaceType.ACTION_SERVER, **kwargs)
        if not topic_builder:
            raise InterfaceError("TopicBuilder is required for ActionServer")
        self.topic_builder: TopicBuilder = topic_builder
        
        # Three callbacks for action lifecycle
        self.handle_goal_request = handle_goal_request          # async def(goal: Any) -> bool
        self.handle_cancel_request = handle_cancel_request      # async def(goal_handle: Any) -> bool
        self.execution_callback = execution_callback            # async def(goal_handle: Any) -> Any
        
        self._transport_handle: Optional[Any] = None
    
    async def initialize(self) -> bool:
        """Initialize action server with transport layer."""
        self._initialized = True
        return True
    
    async def shutdown(self) -> None:
        """Shutdown action server."""
        self._initialized = False
        self._transport_handle = None
    
    async def start(self) -> None:
        """
        Start action server.
        Handles goal requests, cancellations, and executions.
        """
        if not self._initialized:
            raise InterfaceError(f"ActionServer '{self.name}' not initialized")
        
        # Will be implemented by transport-specific subclasses
        raise NotImplementedError


@dataclass
class VyraActionClient(VyraTransport):
    """
    Action Client interface for long-running tasks.
    
    Represents async task client:
    - ROS2: Action Client
    - Zenoh: Query + Subscribers (control/feedback/result)
    - Redis: State tracking with Pub/Sub
    - UDS: Stream-based state messages
    """
    
    def __init__(
        self,
        name: str,
        topic_builder: TopicBuilder,
        direct_response_callback: Optional[Callable[[Any], Coroutine[Any, Any, Any]]] = None,
        feedback_callback: Optional[Callable[[Any], Coroutine[Any, Any, Any]]] = None,
        goal_callback: Optional[Callable[[Any], Coroutine[Any, Any, Any]]] = None,
        protocol: ProtocolType = ProtocolType.ROS2,
        **kwargs
    ):
        super().__init__(name, protocol, InterfaceType.ACTION_CLIENT, **kwargs)
        if not topic_builder:
            raise InterfaceError("TopicBuilder is required for ActionClient")
        self.topic_builder: TopicBuilder = topic_builder
        
        # Three callbacks for action lifecycle
        self.direct_response_callback = direct_response_callback  # async def(accepted: bool) -> None
        self.feedback_callback = feedback_callback                # async def(feedback: Any) -> None
        self.goal_callback = goal_callback                        # async def(result: Any) -> None
        
        self._transport_handle: Optional[Any] = None
    
    async def initialize(self) -> bool:
        """Initialize action client with transport layer."""
        self._initialized = True
        return True
    
    async def shutdown(self) -> None:
        """Shutdown action client."""
        self._initialized = False
        self._transport_handle = None
    
    async def send_goal(self, goal: Any, **kwargs) -> Any:
        """
        Send goal to action server.
        
        Args:
            goal: Goal data
            **kwargs: Additional parameters (timeout, etc.)
            
        Returns:
            Goal handle or result
        """
        if not self._initialized:
            raise InterfaceError(f"ActionClient '{self.name}' not initialized")
        
        # Will be implemented by transport-specific subclasses
        raise NotImplementedError
    
    async def cancel_goal(self, goal_handle: Any) -> bool:
        """
        Cancel a running goal.
        
        Args:
            goal_handle: Handle returned from send_goal
            
        Returns:
            bool: True if cancellation accepted
        """
        if not self._initialized:
            raise InterfaceError(f"ActionClient '{self.name}' not initialized")
        
        # Will be implemented by transport-specific subclasses
        raise NotImplementedError


# Import exceptions for convenience
from vyra_base.com.core.exceptions import InterfaceError
