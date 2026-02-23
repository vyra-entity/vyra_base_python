"""
Redis Transport Provider

Implements AbstractProtocolProvider for Redis transport.
Provides message queue and key-value storage via Redis.
"""
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
    VyraActionClient,
)
from vyra_base.com.core.exceptions import (
    ProtocolUnavailableError,
    ProviderError,
)
from vyra_base.com.providers.protocol_provider import AbstractProtocolProvider
from vyra_base.com.transport.t_redis.communication.redis_client import RedisClient
from vyra_base.com.transport.t_redis.vyra_models import (
    RedisPublisherImpl,
    RedisSubscriberImpl,
    RedisServerImpl,
    RedisClientImpl,
    RedisActionServerImpl,
    RedisActionClientImpl,
)

logger = logging.getLogger(__name__)


class RedisProvider(AbstractProtocolProvider):
    """
    Protocol provider for Redis communication.
    
    Features:
    - Pub/Sub messaging via Publisher interface
    - Key-Value storage via Callable interface (get/set)
    - TLS support with ACL authentication
    - Streaming support (Redis Streams)
    
    Wraps existing vyra_base.com.transport.redis.communication.RedisClient
    for seamless integration with established infrastructure.
    
    Example:
        >>> # Initialize provider
        >>> provider = RedisProvider(
        ...     protocol=ProtocolType.REDIS,
        ...     module_name="my_module"
        ... )
        >>> 
        >>> if await provider.check_availability():
        ...     await provider.initialize()
        ...     
        ...     # Create publisher (Pub/Sub)
        ...     publisher = await provider.create_publisher(
        ...         "sensor_updates",
        ...         module_name="robot"
        ...     )
        ...     await publisher.publish({"temperature": 23.5})
        ...     
        ...     # Listen for messages
        ...     async def on_message(data):
        ...         print(f"Received: {data}")
        ...     await publisher.listen(on_message)
    """
    
    def __init__(
        self,
        module_name: str,
        module_id: str,
        protocol: ProtocolType = ProtocolType.REDIS,
        **redis_kwargs
    ):
        """
        Initialize Redis provider.
        
        Args:
            protocol: Protocol type (must be REDIS)
            module_name: Module name for Redis namespace
            module_id: Module ID for Redis namespace
                - host: Redis host (default: from env REDIS_HOST)
                - port: Redis port (default: from env REDIS_PORT)
                - username: Redis ACL username
                - password: Redis ACL password
                - use_tls: Enable TLS (default: from env)
        """
        super().__init__(protocol)
        self.module_name = module_name
        self._redis_kwargs = redis_kwargs
        self._client: Optional[Any] = None  # RedisClient instance
        self._topic_builder = TopicBuilder(module_name, module_id)

        
    async def check_availability(self) -> bool:
        """
        Check if Redis is available.
        
        Returns:
            bool: True if Redis client can be imported
        """
        
        logger.debug("✅ Redis available")
        self._available = True
        return True
    
    async def initialize(self, config: Optional[Dict[str, Any]] = None) -> bool:
        """
        Initialize Redis client.
        
        Args:
            config: Optional configuration overrides
            
        Returns:
            bool: True if initialization successful
            
        Raises:
            ProviderError: If initialization fails
        """
        if not await self.check_availability():
            raise ProtocolUnavailableError("Redis not available")
        
        try:
            # Merge config
            redis_config = {**self._redis_kwargs}
            if config:
                redis_config.update(config)
            
            # Create Redis client
            self._client = RedisClient(
                module_name=self.module_name,
                **redis_config
            )
            
            # Test connection
            await self._client.connect()
            if not await self._client.health_check():
                raise ProviderError("Redis health check failed")
            
            self._initialized = True
            logger.info(f"✅ Redis provider initialized for module: {self.module_name}")
            return True
            
        except Exception as e:
            raise ProviderError(f"Redis initialization failed: {e}")
    
    async def shutdown(self) -> None:
        """Shutdown Redis client."""
        if self._client:
            try:
                await self._client.disconnect()
                logger.info("✅ Redis provider shutdown complete")
            except Exception as e:
                logger.error(f"❌ Redis shutdown error: {e}")
        
        self._initialized = False
        self._client = None
    
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
        Create Redis Publisher (Pub/Sub).
        
        Args:
            name: Publisher name
            topic_builder: TopicBuilder instance
            message_type: Message type class
            **kwargs: Additional publisher options
            
        Returns:
            RedisPublisherImpl instance
        """
        if not self._initialized or not self._client:
            raise ProviderError("Provider not initialized")
        
        try:
            publisher = RedisPublisherImpl(
                name=name,
                topic_builder=topic_builder,
                redis_client=self._client,
                message_type=message_type,
                **kwargs
            )
            
            await publisher.initialize()
            
            logger.info(f"✅ Redis Publisher created: {name}")
            return publisher
            
        except Exception as e:
            logger.error(f"❌ Failed to create Redis Publisher '{name}': {e}")
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
        Create Redis Subscriber (Pub/Sub).
        
        Args:
            name: Subscriber name
            topic_builder: TopicBuilder instance
            subscriber_callback: Async callback for received messages
            message_type: Message type class
            **kwargs: Additional subscriber options
            
        Returns:
            RedisSubscriberImpl instance
        """
        if not self._initialized or not self._client:
            raise ProviderError("Provider not initialized")
        
        try:
            subscriber = RedisSubscriberImpl(
                name=name,
                topic_builder=topic_builder,
                subscriber_callback=subscriber_callback,
                redis_client=self._client,
                message_type=message_type,
                **kwargs
            )
            
            await subscriber.initialize()
            await subscriber.subscribe()
            
            logger.info(f"✅ Redis Subscriber created: {name}")
            return subscriber
            
        except Exception as e:
            logger.error(f"❌ Failed to create Redis Subscriber '{name}': {e}")
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
        Create Redis Server (request/response pattern).
        
        Args:
            name: Server name
            topic_builder: TopicBuilder instance
            response_callback: Async callback for handling requests
            service_type: Service type class
            **kwargs: Additional server options
            
        Returns:
            RedisServerImpl instance
        """
        if not self._initialized or not self._client:
            raise ProviderError("Provider not initialized")
        
        try:
            server = RedisServerImpl(
                name=name,
                topic_builder=topic_builder,
                response_callback=response_callback,
                redis_client=self._client,
                service_type=service_type,
                **kwargs
            )
            
            await server.initialize()
            await server.serve()
            
            logger.info(f"✅ Redis Server created: {name}")
            return server
            
        except Exception as e:
            logger.error(f"❌ Failed to create Redis Server '{name}': {e}")
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
        Create Redis Client (request/response pattern).
        
        Args:
            name: Client name
            topic_builder: TopicBuilder instance
            service_type: Service type class
            request_callback: Optional async callback for responses
            **kwargs: Additional client options
            
        Returns:
            RedisClientImpl instance
        """
        if not self._initialized or not self._client:
            raise ProviderError("Provider not initialized")
        
        try:
            client = RedisClientImpl(
                name=name,
                topic_builder=topic_builder,
                request_callback=request_callback,
                redis_client=self._client,
                service_type=service_type,
                **kwargs
            )
            
            await client.initialize()
            
            logger.info(f"✅ Redis Client created: {name}")
            return client
            
        except Exception as e:
            logger.error(f"❌ Failed to create Redis Client '{name}': {e}")
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
        Create Redis Action Server (state tracking + pub/sub).
        
        Args:
            name: Action server name
            topic_builder: TopicBuilder instance
            handle_goal_request: Async callback for goal requests
            handle_cancel_request: Async callback for cancel requests
            execution_callback: Async callback for goal execution
            action_type: Action type class
            **kwargs: Additional action server options
            
        Returns:
            RedisActionServerImpl instance
        """
        if not self._initialized or not self._client:
            raise ProviderError("Provider not initialized")
        
        try:
            action_server = RedisActionServerImpl(
                name=name,
                topic_builder=topic_builder,
                handle_goal_request=handle_goal_request,
                handle_cancel_request=handle_cancel_request,
                execution_callback=execution_callback,
                redis_client=self._client,
                action_type=action_type,
                **kwargs
            )
            
            await action_server.initialize()
            
            logger.info(f"✅ Redis Action Server created: {name}")
            return action_server
            
        except Exception as e:
            logger.error(f"❌ Failed to create Redis Action Server '{name}': {e}")
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
        Create Redis Action Client.
        
        Args:
            name: Action client name
            topic_builder: TopicBuilder instance
            action_type: Action type class
            direct_response: Optional async callback for results
            feedback_callback: Optional async callback for feedback
            goal_response_callback: Optional async callback for goal responses
            **kwargs: Additional action client options
            
        Returns:
            RedisActionClientImpl instance
        """
        if not self._initialized or not self._client:
            raise ProviderError("Provider not initialized")
        
        try:
            action_client = RedisActionClientImpl(
                name=name,
                topic_builder=topic_builder,
                direct_response=direct_response,
                feedback_callback=feedback_callback,
                goal_response_callback=goal_response_callback,
                redis_client=self._client,
                action_type=action_type,
                **kwargs
            )
            
            await action_client.initialize()
            
            logger.info(f"✅ Redis Action Client created: {name}")
            return action_client
            
        except Exception as e:
            logger.error(f"❌ Failed to create Redis Action Client '{name}': {e}")
            raise ProviderError(f"Failed to create action client: {e}")
    
    def get_client(self) -> Any:
        """
        Get underlying Redis client for advanced operations.
        
        Returns:
            RedisClient: Underlying vyra_base Redis client
            
        Raises:
            ProviderError: If provider not initialized
        """
        if not self._initialized or not self._client:
            raise ProviderError("Provider not initialized")
        
        return self._client
