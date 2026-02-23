"""
Zenoh Protocol Provider

Implements AbstractProtocolProvider for Zenoh transport.
Provides efficient, scalable communication via Zenoh router.
"""
from __future__ import annotations

import asyncio
import logging
from typing import Any, Callable, Optional, Dict

from vyra_base.com.core.topic_builder import TopicBuilder
from vyra_base.com.core.types import (
    ProtocolType,
    VyraPublisher,
    VyraSubscriber,
    VyraServer,
    VyraClient,
    VyraActionServer,
    VyraActionClient
)
from vyra_base.com.core.exceptions import (
    ProtocolUnavailableError,
    ProviderError,
)
from vyra_base.com.transport.t_zenoh.communication.session import ZenohSession, SessionConfig, SessionMode
from vyra_base.com.transport.t_zenoh.vyra_models import (
    VyraPublisherImpl,
    VyraSubscriberImpl,
    VyraServerImpl,
    VyraClientImpl,
    VyraActionServerImpl,
    VyraActionClientImpl
)
from vyra_base.com.transport.t_zenoh.communication.serializer import SerializationFormat
from vyra_base.com.providers.protocol_provider import AbstractProtocolProvider

logger = logging.getLogger(__name__)

# Check if zenoh is available
try:
    import zenoh
    ZENOH_AVAILABLE = True
except ImportError:
    ZENOH_AVAILABLE = False
    logger.warning(
        "⚠️ zenoh-python not available. Zenoh transport disabled. "
        "Install via: pip install eclipse-zenoh"
    )


class ZenohProvider(AbstractProtocolProvider):
    """
    Protocol provider for Zenoh transport.
    
    Features:
    - Efficient Pub/Sub with zero-copy capabilities
    - Query/Reply for request-response patterns
    - Router-based scalability
    - Built-in discovery and fault tolerance
    - Multi-protocol support (TCP, UDP, shared memory)
    
    Requirements:
    - eclipse-zenoh Python package
    - Zenoh router (typically Docker service)
    
    Example:
        >>> # Initialize provider
        >>> provider = ZenohProvider(ProtocolType.ZENOH)
        >>> 
        >>> if await provider.check_availability():
        ...     await provider.initialize(config={
        ...         "mode": "client",
        ...         "connect": ["tcp/zenoh-router:7447"]
        ...     })
        ...     
        ...     # Create callable (query/reply)
        ...     async def handle_request(req):
        ...         return {"result": req["value"] * 2}
        ...     
        ...     callable = await provider.create_callable(
        ...         "/calculate",
        ...         handle_request
        ...     )
        ...     
        ...     # Create publisher (pub/sub)
        ...     publisher = await provider.create_publisher("/sensor_data")
    """
    
    def __init__(
        self,
        module_name: str,
        module_id: str,
        protocol: ProtocolType = ProtocolType.ZENOH
    ):
        """
        Initialize Zenoh provider.
        
        Args:
            protocol: Protocol type (must be ZENOH)
        """
        super().__init__(protocol)
        self._session: Optional[ZenohSession] = None
        self._format = SerializationFormat.JSON

        # Topic builder for consistent naming
        self._topic_builder = TopicBuilder(module_name, module_id)
    
    async def check_availability(self) -> bool:
        """
        Check if Zenoh is available.
        
        Returns:
            bool: True if zenoh-python is installed
        """
        self._available = ZENOH_AVAILABLE
        
        if not self._available:
            logger.warning("Zenoh not available")
        else:
            logger.debug("✅ Zenoh is available")
        
        return self._available
    
    async def initialize(self, config: Optional[Dict[str, Any]] = None) -> bool:
        """
        Initialize Zenoh provider and open session.
        
        Args:
            config: Configuration dictionary:
                - mode: "peer", "client", or "router" (default: "client")
                - connect: List of endpoints (default: ["tcp/zenoh-router:7447"])
                - listen: List of listen endpoints
                - format: Serialization format (default: "json")
                
        Returns:
            bool: True if initialization successful
        """
        if self._initialized:
            logger.warning("Zenoh provider already initialized")
            return True
        
        if not self._available:
            raise ProtocolUnavailableError("Zenoh is not available")
        
        try:
            logger.info("🔧 Initializing Zenoh provider...")
            
            # Parse configuration
            config = config or {}
            self._config.update(config)
            
            mode_str = config.get("mode", "client")
            mode = SessionMode(mode_str)
            
            connect = config.get("connect", ["tcp/zenoh-router:7447"])
            listen = config.get("listen", [])
            
            format_str = config.get("format", "json")
            self._format = SerializationFormat(format_str)
            
            # Create session config
            session_config = SessionConfig(
                mode=mode,
                connect=connect,
                listen=listen
            )
            
            # Create and open session
            self._session = ZenohSession(session_config)
            await self._session.open()
            
            self._initialized = True
            logger.info("✅ Zenoh provider initialized")
            try:
                session_info = self._session.session.info()
                # zenoh 1.x: zid is a property, not callable
                zid = session_info.zid() if callable(session_info.zid) else session_info.zid
                logger.info(f"Session ID: {zid}")
            except Exception:
                logger.info("✅ Zenoh provider initialized (session ID unavailable)")
            
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to initialize Zenoh provider: {e}")
            raise ProviderError(f"Zenoh initialization failed: {e}")
    
    async def shutdown(self) -> None:
        """Shutdown Zenoh provider and close session."""
        if not self._initialized:
            logger.debug("Zenoh provider not initialized, nothing to shutdown")
            return
        
        try:
            logger.info("🛑 Shutting down Zenoh provider...")
            
            if self._session:
                await self._session.close()
                self._session = None
            
            self._initialized = False
            logger.info("✅ Zenoh provider shutdown complete")
            
        except Exception as e:
            logger.error(f"❌ Error shutting down Zenoh provider: {e}")
            raise
    
    # ============================================================================
    # UNIFIED TRANSPORT LAYER METHODS
    # ============================================================================
    
    async def create_publisher(
        self,
        name: str,
        topic_builder: Optional[TopicBuilder] = None,
        message_type: Optional[type] = None,
        **kwargs
    ) -> VyraPublisher:
        """
        Create Zenoh Publisher.
        
        Args:
            name: Publisher name
            topic_builder: TopicBuilder instance
            message_type: Message type class
            **kwargs: Additional publisher options
            
        Returns:
            ZenohPublisherImpl instance
        """
        self.require_initialization()
        
        try:
            if not self._session or not self._session.is_open:
                raise ProviderError("Zenoh session not open")
            
            effective_topic_builder = topic_builder or self._topic_builder
            publisher = VyraPublisherImpl(
                name=name,
                topic_builder=effective_topic_builder,
                zenoh_session=self._session.session,
                message_type=message_type,
                **kwargs
            )
            
            await publisher.initialize()
            
            logger.info(f"✅ Zenoh Publisher created: {name}")
            return publisher
            
        except Exception as e:
            logger.error(f"❌ Failed to create Zenoh Publisher '{name}': {e}")
            raise ProviderError(f"Failed to create publisher: {e}")
    
    async def create_subscriber(
        self,
        name: str,
        topic_builder: Optional[TopicBuilder] = None,
        subscriber_callback: Optional[Callable] = None,
        message_type: Optional[type] = None,
        **kwargs
    ) -> VyraSubscriber:
        """
        Create Zenoh Subscriber.
        
        Args:
            name: Subscriber name
            topic_builder: TopicBuilder instance
            subscriber_callback: Async callback for received messages
            message_type: Message type class
            **kwargs: Additional subscriber options
            
        Returns:
            ZenohSubscriberImpl instance
        """
        self.require_initialization()
        
        try:
            if not self._session or not self._session.is_open:
                raise ProviderError("Zenoh session not open")

            effective_topic_builder = topic_builder or self._topic_builder
            subscriber = VyraSubscriberImpl(
                name=name,
                topic_builder=effective_topic_builder,
                subscriber_callback=subscriber_callback,
                zenoh_session=self._session.session,
                message_type=message_type,
                **kwargs
            )
            
            await subscriber.initialize()
            await subscriber.subscribe()
            
            logger.info(f"✅ Zenoh Subscriber created: {name}")
            return subscriber
            
        except Exception as e:
            logger.error(f"❌ Failed to create Zenoh Subscriber '{name}': {e}")
            raise ProviderError(f"Failed to create subscriber: {e}")
    
    async def create_server(
        self,
        name: str,
        topic_builder: Optional[TopicBuilder] = None,
        response_callback: Optional[Callable] = None,
        service_type: Optional[type] = None,
        **kwargs
    ) -> VyraServer:
        """
        Create Zenoh Server (Queryable).
        
        Args:
            name: Server name
            topic_builder: TopicBuilder instance
            response_callback: Async callback for handling requests
            service_type: Service type class
            **kwargs: Additional server options
            
        Returns:
            ZenohServerImpl instance
        """
        self.require_initialization()
        
        try:
            if not self._session or not self._session.is_open:
                raise ProviderError("Zenoh session not open")
            
            effective_topic_builder = topic_builder or self._topic_builder
            server = VyraServerImpl(
                name=name,
                topic_builder=effective_topic_builder,
                response_callback=response_callback,
                zenoh_session=self._session.session,
                service_type=service_type,
                **kwargs
            )
            
            await server.initialize()
            await server.serve()
            
            logger.info(f"✅ Zenoh Server created: {name}")
            return server
            
        except Exception as e:
            logger.error(f"❌ Failed to create Zenoh Server '{name}': {e}")
            raise ProviderError(f"Failed to create server: {e}")
    
    async def create_client(
        self,
        name: str,
        topic_builder: Optional[TopicBuilder] = None,
        service_type: Optional[type] = None,
        request_callback: Optional[Callable] = None,
        **kwargs
    ) -> VyraClient:
        """
        Create Zenoh Client (Query sender).
        
        Args:
            name: Client name
            topic_builder: TopicBuilder instance (uses provider's default if omitted)
            service_type: Optional service type class (ignored in Zenoh – schema-less)
            request_callback: Optional async callback for responses
            **kwargs: Additional client options
            
        Returns:
            ZenohClientImpl instance
        """
        self.require_initialization()
        
        try:
            if not self._session or not self._session.is_open:
                raise ProviderError("Zenoh session not open")
            
            # Use provider's topic_builder if none provided
            effective_topic_builder = topic_builder or self._topic_builder
            
            client = VyraClientImpl(
                name=name,
                topic_builder=effective_topic_builder,
                request_callback=request_callback,
                zenoh_session=self._session.session,
                service_type=service_type,
                **kwargs
            )
            
            await client.initialize()
            
            logger.info(f"✅ Zenoh Client created: {name}")
            return client
            
        except Exception as e:
            logger.error(f"❌ Failed to create Zenoh Client '{name}': {e}")
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
        Create Zenoh Action Server.
        
        Args:
            name: Action server name
            topic_builder: TopicBuilder instance
            handle_goal_request: Async callback for goal requests
            handle_cancel_request: Async callback for cancel requests
            execution_callback: Async callback for goal execution
            action_type: Action type class
            **kwargs: Additional action server options
            
        Returns:
            ZenohActionServerImpl instance
        """
        self.require_initialization()
        
        try:
            if not self._session or not self._session.is_open:
                raise ProviderError("Zenoh session not open")
            
            action_server = VyraActionServerImpl(
                name=name,
                topic_builder=topic_builder,
                handle_goal_request=handle_goal_request,
                handle_cancel_request=handle_cancel_request,
                execution_callback=execution_callback,
                zenoh_session=self._session.session,
                action_type=action_type,
                **kwargs
            )
            
            await action_server.initialize()
            
            logger.info(f"✅ Zenoh Action Server created: {name}")
            return action_server
            
        except Exception as e:
            logger.error(f"❌ Failed to create Zenoh Action Server '{name}': {e}")
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
        Create Zenoh Action Client.
        
        Args:
            name: Action client name
            topic_builder: TopicBuilder instance
            action_type: Action type class
            direct_response: Optional async callback for results
            feedback_callback: Optional async callback for feedback
            goal_response_callback: Optional async callback for goal responses
            **kwargs: Additional action client options
            
        Returns:
            ZenohActionClientImpl instance
        """
        self.require_initialization()
        
        try:
            if not self._session or not self._session.is_open:
                raise ProviderError("Zenoh session not open")
            
            action_client = VyraActionClientImpl(
                name=name,
                topic_builder=topic_builder,
                direct_response=direct_response,
                feedback_callback=feedback_callback,
                goal_response_callback=goal_response_callback,
                zenoh_session=self._session.session,
                action_type=action_type,
                **kwargs
            )
            
            await action_client.initialize()
            
            logger.info(f"✅ Zenoh Action Client created: {name}")
            return action_client
            
        except Exception as e:
            logger.error(f"❌ Failed to create Zenoh Action Client '{name}': {e}")
            raise ProviderError(f"Failed to create action client: {e}")
    
    def require_initialization(self) -> None:
        """
        Ensure provider is initialized.
        
        Raises:
            ProviderError: If not initialized
        """
        if not self._initialized:
            raise ProviderError("Zenoh provider not initialized. Call initialize() first.")
        
        if not self._session or not self._session.is_open:
            raise ProviderError("Zenoh session not open")
    
    def get_session(self) -> Optional[ZenohSession]:
        """Get the underlying Zenoh session."""
        return self._session
