"""
Protocol Provider Interface

Abstract base class for all communication protocol providers.
Implements provider pattern for pluggable protocols.
"""
import logging
from abc import ABC, abstractmethod
from typing import Any, Callable, Optional, Dict, List

from vyra_base.com.core.types import (
    ProtocolType,
    # New unified types
    VyraPublisher,
    VyraSubscriber,
    VyraServer,
    VyraClient,
    VyraActionServer,
    VyraActionClient,

)
from vyra_base.com.core.exceptions import (
    ProtocolUnavailableError,
    ProviderError,
)
from vyra_base.com.core.topic_builder import TopicBuilder

logger = logging.getLogger(__name__)


class AbstractProtocolProvider(ABC):
    """
    Abstract base class for all protocol providers.
    
    Each protocol (ROS2, Shared Memory, MQTT, etc.) implements this interface
    to provide consistent callable/speaker/job creation across transports.
    """
    
    def __init__(self, protocol: ProtocolType):
        self.protocol = protocol
        self._available = False
        self._initialized = False
        self._config: Dict[str, Any] = {}
        
        self._topic_builder: TopicBuilder
    
    @property
    def name(self) -> str:
        """Get protocol name."""
        return self.protocol.value
    
    @abstractmethod
    async def check_availability(self) -> bool:
        """
        Check if protocol is available (libraries installed, services running).
        
        Returns:
            bool: True if protocol is available
        """
        pass
    
    @abstractmethod
    async def initialize(self, config: Optional[Dict[str, Any]] = None) -> bool:
        """
        Initialize the protocol provider.
        
        Args:
            config: Optional configuration dictionary
            
        Returns:
            bool: True if initialization successful
        """
        pass
    
    @abstractmethod
    async def shutdown(self) -> None:
        """Shutdown the protocol provider and cleanup resources."""
        pass

    # ========================================================================
    # NEW UNIFIED INTERFACE CREATION METHODS
    # ========================================================================

    @abstractmethod
    async def create_publisher(
        self,
        name: str,
        **kwargs
    ) -> VyraPublisher:
        """
        Create a publisher interface.
        
        Args:
            name: Publisher name
            **kwargs: Protocol-specific parameters (message_type, qos, etc.)
            
        Returns:
            VyraPublisher: Created publisher interface
        """
        pass

    @abstractmethod
    async def create_subscriber(
        self,
        name: str,
        subscriber_callback: Optional[Callable],
        **kwargs
    ) -> VyraSubscriber:
        """
        Create a subscriber interface.
        
        Args:
            name: Subscriber name
            subscriber_callback: Callback for received messages (async)
            **kwargs: Protocol-specific parameters (message_type, qos, etc.)
            
        Returns:
            VyraSubscriber: Created subscriber interface
        """
        pass

    @abstractmethod
    async def create_server(
        self,
        name: str,
        response_callback: Optional[Callable],
        **kwargs
    ) -> VyraServer:
        """
        Create a server interface.
        
        Args:
            name: Server name
            response_callback: Callback for requests (async)
            **kwargs: Protocol-specific parameters (service_type, qos, etc.)
            
        Returns:
            VyraServer: Created server interface
        """
        pass

    @abstractmethod
    async def create_client(
        self,
        name: str,
        **kwargs
    ) -> VyraClient:
        """
        Create a client interface.
        
        Args:
            name: Client name
            **kwargs: Protocol-specific parameters (service_type, qos, etc.)
            
        Returns:
            VyraClient: Created client interface
        """
        pass

    @abstractmethod
    async def create_action_server(
        self,
        name: str,
        handle_goal_request: Optional[Callable] = None,
        handle_cancel_request: Optional[Callable] = None,
        execution_callback: Optional[Callable] = None,
        **kwargs
    ) -> VyraActionServer:
        """
        Create an action server interface.
        
        Args:
            name: Action server name
            handle_goal_request: Callback to accept/reject goals (async)
            handle_cancel_request: Callback to handle cancellations (async)
            execution_callback: Callback for goal execution (async)
            **kwargs: Protocol-specific parameters (action_type, qos, etc.)
            
        Returns:
            VyraActionServer: Created action server interface
        """
        pass

    @abstractmethod
    async def create_action_client(
        self,
        name: str,
        direct_response_callback: Optional[Callable] = None,
        feedback_callback: Optional[Callable] = None,
        goal_callback: Optional[Callable] = None,
        **kwargs
    ) -> VyraActionClient:
        """
        Create an action client interface.
        
        Args:
            name: Action client name
            direct_response_callback: Callback for goal acceptance (async)
            feedback_callback: Callback for feedback updates (async)
            goal_callback: Callback for final result (async)
            **kwargs: Protocol-specific parameters (action_type, qos, etc.)
            
        Returns:
            VyraActionClient: Created action client interface
        """
        pass

    def create_topic_builder(self, module_name: str, module_id: str) -> TopicBuilder:
        """
        Create a TopicBuilder for consistent naming conventions.
        
        Args:
            module_name: Name of the module (e.g., "v2_modulemanager")
            module_id: Unique module ID (e.g., "abc123")
            
        Returns:
            TopicBuilder instance configured with module info
        """
        self._topic_builder = TopicBuilder(
            module_name=module_name, module_id=module_id)
        
        return self._topic_builder
    
    def is_available(self) -> bool:
        """Check if protocol is available."""
        return self._available
    
    def is_initialized(self) -> bool:
        """Check if provider is initialized."""
        return self._initialized
    
    def get_config(self) -> Dict[str, Any]:
        """Get current configuration."""
        return self._config.copy()
    
    def update_config(self, config: Dict[str, Any]) -> None:
        """Update configuration."""
        self._config.update(config)
    
    def require_availability(self) -> None:
        """Raise exception if protocol is not available."""
        if not self._available:
            raise ProtocolUnavailableError(
                f"Protocol '{self.protocol}' is not available. "
                f"Check installation and dependencies."
            )
    
    def require_initialization(self) -> None:
        """Raise exception if provider is not initialized."""
        if not self._initialized:
            raise ProviderError(
                f"Provider '{self.protocol}' is not initialized. "
                f"Call initialize() first."
            )
