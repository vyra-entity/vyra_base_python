"""
UDS Protocol Provider

Implements AbstractProtocolProvider for Unix Domain Socket transport.
Provides low-latency local IPC via stream sockets.
"""
import logging
from typing import Any, Callable, Optional, Dict

from vyra_base.com.core.topic_builder import TopicBuilder
from vyra_base.com.core.types import (
    ProtocolType,
    VyraPublisher,
    VyraSubscriber,
    VyraServer,
    VyraActionServer,
    VyraClient,
    VyraActionClient,
)
from vyra_base.com.core.exceptions import (
    ProtocolUnavailableError,
    ProviderError,
)
from vyra_base.com.providers.protocol_provider import AbstractProtocolProvider
from vyra_base.com.transport.t_uds.communication import UDS_SOCKET_DIR
from vyra_base.com.transport.t_uds.vyra_models import (
    UdsPublisherImpl,
    UdsSubscriberImpl,
    UdsServerImpl,
    VyraClientImpl,
    VyraActionServerImpl,
    VyraActionClientImpl,
)

logger = logging.getLogger(__name__)


class UDSProvider(AbstractProtocolProvider):
    """
    Protocol provider for Unix Domain Socket transport.
    
    Features:
    - Low-latency local IPC
    - Stream-based communication
    - Automatic connection management
    - No serialization overhead (JSON)
    
    Requirements:
    - Unix-like OS (Linux, macOS)
    - File system access to /tmp/vyra_sockets
    
    Limitations:
    - Local machine only
    - No pub/sub pattern (use Callable for request-response)
    
    Example:
        >>> # Initialize provider
        >>> provider = UDSProvider(ProtocolType.UDS)
        >>> 
        >>> if await provider.check_availability():
        ...     await provider.initialize()
        ...     
        ...     # Create callable (server)
        ...     async def handle_request(req):
        ...         return {"result": req["value"] * 2}
        ...     
        ...     callable = await provider.create_callable(
        ...         "calculate",
        ...         handle_request,
        ...         module_name="math_service"
        ...     )
        ...     
        ...     # Create client
        ...     client = await provider.create_callable(
        ...         "calculate",
        ...         None,  # No callback for client
        ...         module_name="math_service"
        ...     )
        ...     result = await client.call({"value": 21})
    """
    
    def __init__(
        self,
        module_name: str,
        module_id: str,
        protocol: ProtocolType = ProtocolType.UDS,
    ):
        """
        Initialize UDS provider.
        
        Args:
            protocol: Protocol type (must be UDS)
            module_name: Default module name for interfaces
        """
        super().__init__(protocol)
        self.module_name = module_name
        
        # Default configuration
        self._config = {
            "socket_dir": str(UDS_SOCKET_DIR),
            "connect_timeout": 5.0,
            "call_timeout": 5.0,
        }

        # Topic builder for consistent naming
        self._topic_builder = TopicBuilder(module_name, module_id)
    
    async def check_availability(self) -> bool:
        """
        Check if UDS transport is available.
        
        Returns:
            bool: Always True on Unix-like systems
        """
        import platform
        system = platform.system()
        
        # UDS available on Unix-like systems
        self._available = system in ('Linux', 'Darwin', 'FreeBSD', 'OpenBSD')
        
        if not self._available:
            logger.warning(
                f"⚠️ UDS transport not available on {system}. "
                f"UDS requires Unix-like OS."
            )
        else:
            logger.info("✅ UDS transport available")
        
        return self._available
    
    async def initialize(self, config: Optional[Dict[str, Any]] = None) -> bool:
        """
        Initialize UDS provider.
        
        Args:
            config: Optional configuration
                - socket_dir: Socket directory (default: /tmp/vyra_sockets)
                - connect_timeout: Connection timeout
                - call_timeout: Default call timeout
                
        Returns:
            bool: True if initialization successful
        """
        if not self._available:
            raise ProtocolUnavailableError(
                "UDS transport not available. Requires Unix-like OS."
            )
        
        if self._initialized:
            logger.warning("⚠️ Provider already initialized")
            return True
        
        # Update configuration
        if config:
            self._config.update(config)
        
        logger.info(
            f"🚀 Initializing UDS provider for module: {self.module_name}"
        )
        
        try:
            # Ensure socket directory exists
            UDS_SOCKET_DIR.mkdir(parents=True, exist_ok=True)
            
            self._initialized = True
            logger.info(
                f"✅ UDS provider initialized for {self.module_name}"
            )
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to initialize provider: {e}")
            return False
    
    async def shutdown(self) -> None:
        """Shutdown the provider and cleanup resources."""
        if not self._initialized:
            return
        
        logger.info(f"🛑 Shutting down UDS provider: {self.module_name}")
        
        # Note: Individual sockets cleanup themselves
        # No global cleanup needed
        
        self._initialized = False
        logger.info("✅ UDS provider shutdown complete")
    
    # ============================================================================
    # UNIFIED TRANSPORT LAYER METHODS
    # ============================================================================
    
    async def create_publisher(
        self,
        name: str,
        topic_builder: TopicBuilder,
        message_type: type,
        **kwargs
    ) -> VyraPublisher:
        """
        Create UDS Publisher (datagram sockets).
        
        Args:
            name: Publisher name
            topic_builder: TopicBuilder instance
            message_type: Message type class
            **kwargs: Additional publisher options (module_name override)
            
        Returns:
            UdsPublisherImpl instance
        """
        if not self._initialized:
            raise ProviderError("Provider not initialized")
        
        module_name = kwargs.pop('module_name', self.module_name)
        
        try:
            publisher = UdsPublisherImpl(
                name=name,
                topic_builder=topic_builder,
                message_type=message_type,
                module_name=module_name,
                **kwargs
            )
            
            await publisher.initialize()
            
            logger.info(f"✅ UDS Publisher created: {module_name}.{name}")
            return publisher
            
        except Exception as e:
            logger.error(f"❌ Failed to create UDS Publisher '{name}': {e}")
            raise ProviderError(f"Failed to create publisher: {e}")
    
    async def create_subscriber(
        self,
        name: str,
        topic_builder: TopicBuilder,
        subscriber_callback: Callable,
        message_type: type,
        **kwargs
    ) -> VyraSubscriber:
        """
        Create UDS Subscriber (datagram sockets).
        
        Args:
            name: Subscriber name
            topic_builder: TopicBuilder instance
            subscriber_callback: Async callback for received messages
            message_type: Message type class
            **kwargs: Additional subscriber options (module_name override)
            
        Returns:
            UdsSubscriberImpl instance
        """
        if not self._initialized:
            raise ProviderError("Provider not initialized")
        
        module_name = kwargs.pop('module_name', self.module_name)
        
        try:
            subscriber = UdsSubscriberImpl(
                name=name,
                topic_builder=topic_builder,
                subscriber_callback=subscriber_callback,
                message_type=message_type,
                module_name=module_name,
                **kwargs
            )
            
            await subscriber.initialize()
            await subscriber.subscribe()
            
            logger.info(f"✅ UDS Subscriber created: {module_name}.{name}")
            return subscriber
            
        except Exception as e:
            logger.error(f"❌ Failed to create UDS Subscriber '{name}': {e}")
            raise ProviderError(f"Failed to create subscriber: {e}")
    
    async def create_server(
        self,
        name: str,
        topic_builder: TopicBuilder,
        response_callback: Callable,
        service_type: type,
        **kwargs
    ) -> VyraServer:
        """
        Create UDS Server (stream sockets).
        
        Args:
            name: Server name
            topic_builder: TopicBuilder instance
            response_callback: Async callback for handling requests
            service_type: Service type class
            **kwargs: Additional server options (module_name override)
            
        Returns:
            UdsServerImpl instance
        """
        if not self._initialized:
            raise ProviderError("Provider not initialized")
        
        module_name = kwargs.pop('module_name', self.module_name)
        
        try:
            server = UdsServerImpl(
                name=name,
                topic_builder=topic_builder,
                response_callback=response_callback,
                service_type=service_type,
                module_name=module_name,
                **kwargs
            )
            
            await server.initialize()
            await server.serve()
            
            logger.info(f"✅ UDS Server created: {module_name}.{name}")
            return server
            
        except Exception as e:
            logger.error(f"❌ Failed to create UDS Server '{name}': {e}")
            raise ProviderError(f"Failed to create server: {e}")
    
    async def create_client(
        self,
        name: str,
        topic_builder: TopicBuilder,
        service_type: type,
        request_callback: Optional[Callable] = None,
        **kwargs
    ) -> VyraClient:
        """
        Create UDS Client (stream sockets).
        
        Args:
            name: Client name
            topic_builder: TopicBuilder instance
            service_type: Service type class
            request_callback: Optional async callback for responses
            **kwargs: Additional client options (module_name override)
            
        Returns:
            UdsClientImpl instance
        """
        if not self._initialized:
            raise ProviderError("Provider not initialized")
        
        module_name = kwargs.pop('module_name', self.module_name)
        
        try:
            client = VyraClientImpl(
                name=name,
                topic_builder=topic_builder,
                request_callback=request_callback,
                service_type=service_type,
                module_name=module_name,
                **kwargs
            )
            
            await client.initialize()
            
            logger.info(f"✅ UDS Client created: {module_name}.{name}")
            return client
            
        except Exception as e:
            logger.error(f"❌ Failed to create UDS Client '{name}': {e}")
            raise ProviderError(f"Failed to create client: {e}")
    
    async def create_action_server(
        self,
        name: str,
        topic_builder: TopicBuilder,
        handle_goal_request: Callable,
        handle_cancel_request: Callable,
        execution_callback: Callable,
        action_type: type,
        **kwargs
    ) -> VyraActionServer:
        """
        Create UDS Action Server (stream sockets + state messages).
        
        Args:
            name: Action server name
            topic_builder: TopicBuilder instance
            handle_goal_request: Async callback for goal requests
            handle_cancel_request: Async callback for cancel requests
            execution_callback: Async callback for goal execution
            action_type: Action type class
            **kwargs: Additional action server options (module_name override)
            
        Returns:
            UdsActionServerImpl instance
        """
        if not self._initialized:
            raise ProviderError("Provider not initialized")
        
        module_name = kwargs.pop('module_name', self.module_name)
        
        try:
            action_server = VyraActionServerImpl(
                name=name,
                topic_builder=topic_builder,
                handle_goal_request=handle_goal_request,
                handle_cancel_request=handle_cancel_request,
                execution_callback=execution_callback,
                action_type=action_type,
                module_name=module_name,
                **kwargs
            )
            
            await action_server.initialize()
            
            logger.info(f"✅ UDS Action Server created: {module_name}.{name}")
            return action_server
            
        except Exception as e:
            logger.error(f"❌ Failed to create UDS Action Server '{name}': {e}")
            raise ProviderError(f"Failed to create action server: {e}")
    
    async def create_action_client(
        self,
        name: str,
        topic_builder: TopicBuilder,
        action_type: type,
        direct_response: Optional[Callable] = None,
        feedback_callback: Optional[Callable] = None,
        goal_response_callback: Optional[Callable] = None,
        **kwargs
    ) -> VyraActionClient:
        """
        Create UDS Action Client.
        
        Args:
            name: Action client name
            topic_builder: TopicBuilder instance
            action_type: Action type class
            direct_response: Optional async callback for results
            feedback_callback: Optional async callback for feedback
            goal_response_callback: Optional async callback for goal responses
            **kwargs: Additional action client options (module_name override)
            
        Returns:
            UdsActionClientImpl instance
        """
        if not self._initialized:
            raise ProviderError("Provider not initialized")
        
        module_name = kwargs.pop('module_name', self.module_name)
        
        try:
            action_client = VyraActionClientImpl(
                name=name,
                topic_builder=topic_builder,
                direct_response=direct_response,
                feedback_callback=feedback_callback,
                goal_response_callback=goal_response_callback,
                action_type=action_type,
                module_name=module_name,
                **kwargs
            )
            
            await action_client.initialize()
            
            logger.info(f"✅ UDS Action Client created: {module_name}.{name}")
            return action_client
            
        except Exception as e:
            logger.error(f"❌ Failed to create UDS Action Client '{name}': {e}")
            raise ProviderError(f"Failed to create action client: {e}")
