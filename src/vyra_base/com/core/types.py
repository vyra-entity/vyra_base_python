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
from typing import Any, Callable, Optional, Dict, List
from datetime import datetime


class ProtocolType(str, Enum):
    """Supported communication protocols."""
    # Communication Abstraction Layer (CAL)
    ROS2 = "ros2"
    SHARED_MEMORY = "shared_memory"
    UDS = "uds"
    
    # External Communication Layer
    REDIS = "redis"
    MQTT = "mqtt"
    GRPC = "grpc"
    REST = "rest"
    WEBSOCKET = "websocket"
    
    # Industrial Buses (SCADA/MES Level - Northbound)
    MODBUS = "modbus"
    OPCUA = "opcua"


class InterfaceType(str, Enum):
    """VYRA communication interface types."""
    CALLABLE = "callable"  # Request-Response
    SPEAKER = "speaker"    # Publish-Subscribe
    JOB = "job"            # Long-Running Task


class AccessLevel(str, Enum):
    """Access control levels for interfaces."""
    PUBLIC = "public"
    PROTECTED = "protected"
    PRIVATE = "private"
    INTERNAL = "internal"


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


class VyraInterface(ABC):
    """
    Abstract base class for all VYRA communication interfaces.
    
    All communication paradigms (Callable, Speaker, Job) inherit from this.
    """
    
    def __init__(
        self,
        name: str,
        protocol: ProtocolType,
        interface_type: InterfaceType,
        **kwargs
    ):
        self.metadata = InterfaceMetadata(
            name=name,
            protocol=protocol,
            interface_type=interface_type,
            **kwargs
        )
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


@dataclass
class VyraCallable(VyraInterface):
    """
    Callable interface for request-response communication.
    
    Represents synchronous/asynchronous service calls across protocols:
    - ROS2: Service
    - gRPC: Unary RPC
    - REST: HTTP Request
    """
    
    def __init__(
        self,
        name: str,
        callback: Optional[Callable] = None,
        protocol: ProtocolType = ProtocolType.ROS2,
        **kwargs
    ):
        super().__init__(name, protocol, InterfaceType.CALLABLE, **kwargs)
        self.callback = callback
        self._transport_handle: Optional[Any] = None
    
    async def initialize(self) -> bool:
        """Initialize callable with transport layer."""
        self._initialized = True
        return True
    
    async def shutdown(self) -> None:
        """Shutdown callable."""
        self._initialized = False
        self._transport_handle = None
    
    async def call(self, request: Any, timeout: float = 5.0) -> Any:
        """
        Invoke the callable remotely.
        
        Args:
            request: Request data
            timeout: Timeout in seconds
            
        Returns:
            Response data
        """
        if not self._initialized:
            raise InterfaceError(f"Callable '{self.name}' not initialized")
        
        # Will be implemented by transport-specific subclasses
        raise NotImplementedError


@dataclass
class VyraSpeaker(VyraInterface):
    """
    Speaker interface for publish-subscribe communication.
    
    Represents one-way broadcasting across protocols:
    - ROS2: Topic Publisher
    - MQTT: Publisher
    - Redis: Pub/Sub Publisher
    """
    
    def __init__(
        self,
        name: str,
        protocol: ProtocolType = ProtocolType.ROS2,
        **kwargs
    ):
        super().__init__(name, protocol, InterfaceType.SPEAKER, **kwargs)
        self._transport_handle: Optional[Any] = None
    
    async def initialize(self) -> bool:
        """Initialize speaker with transport layer."""
        self._initialized = True
        return True
    
    async def shutdown(self) -> None:
        """Shutdown speaker."""
        self._initialized = False
        self._transport_handle = None
    
    async def shout(self, message: Any) -> bool:
        """
        Publish a message.
        
        Args:
            message: Message data to publish
            
        Returns:
            bool: True if published successfully
        """
        if not self._initialized:
            raise InterfaceError(f"Speaker '{self.name}' not initialized")
        
        # Will be implemented by transport-specific subclasses
        raise NotImplementedError
    
    async def listen(
        self,
        callback: Callable[[Any], None],
        **kwargs
    ) -> None:
        """
        Subscribe and listen for messages.
        
        Args:
            callback: Callback function to process messages
            **kwargs: Additional protocol-specific parameters
        """
        if not self._initialized:
            raise InterfaceError(f"Speaker '{self.name}' not initialized")
        
        # Will be implemented by transport-specific subclasses
        raise NotImplementedError


@dataclass
class VyraJob(VyraInterface):
    """
    Job interface for long-running tasks.
    
    Represents asynchronous tasks with progress feedback:
    - ROS2: Action
    - gRPC: Server Streaming
    - Async Task: Python asyncio
    """
    
    def __init__(
        self,
        name: str,
        callback: Optional[Callable] = None,
        protocol: ProtocolType = ProtocolType.ROS2,
        **kwargs
    ):
        super().__init__(name, protocol, InterfaceType.JOB, **kwargs)
        self.callback = callback
        self._transport_handle: Optional[Any] = None
    
    async def initialize(self) -> bool:
        """Initialize job with transport layer."""
        self._initialized = True
        return True
    
    async def shutdown(self) -> None:
        """Shutdown job."""
        self._initialized = False
        self._transport_handle = None
    
    async def execute(
        self,
        goal: Any,
        feedback_callback: Optional[Callable] = None
    ) -> Any:
        """
        Execute the job.
        
        Args:
            goal: Goal/input data
            feedback_callback: Optional callback for progress updates
            
        Returns:
            Result data
        """
        if not self._initialized:
            raise InterfaceError(f"Job '{self.name}' not initialized")
        
        # Will be implemented by transport-specific subclasses
        raise NotImplementedError


# Import exceptions for convenience
from vyra_base.com.core.exceptions import InterfaceError
