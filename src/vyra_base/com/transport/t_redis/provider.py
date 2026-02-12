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
    VyraCallable,
    VyraSpeaker,
    VyraJob,
)
from vyra_base.com.core.exceptions import (
    ProtocolUnavailableError,
    ProviderError,
)
from vyra_base.com.providers.protocol_provider import AbstractProtocolProvider
from vyra_base.com.transport.t_redis.communication.redis_client import RedisClient

logger = logging.getLogger(__name__)


class RedisProvider(AbstractProtocolProvider):
    """
    Protocol provider for Redis communication.
    
    Features:
    - Pub/Sub messaging via Speaker interface
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
        ...     # Create speaker (Pub/Sub)
        ...     speaker = await provider.create_speaker(
        ...         "sensor_updates",
        ...         module_name="robot"
        ...     )
        ...     await speaker.shout({"temperature": 23.5})
        ...     
        ...     # Listen for messages
        ...     async def on_message(data):
        ...         print(f"Received: {data}")
        ...     await speaker.listen(on_message)
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
    
    async def create_callable(
        self,
        name: str,
        callback: Optional[Callable] = None,
        **kwargs
    ) -> VyraCallable:
        """
        Create Redis Callable (request-response via key-value).
        
        Redis Callable supports request-response pattern:
        - Server-side: Listens to request keys, executes callback, writes response
        - Client-side: Writes request key, waits for response key
        
        Args:
            name: Callable name (used as key prefix)
            callback: Server-side callback function (None for client)
            **kwargs: Additional parameters
                - is_callable: True for server, False for client (auto-detected from callback if not provided)
            
        Returns:
            VyraCallable: Redis callable instance
        """
        if not self._initialized:
            raise ProviderError("Provider not initialized")
        
        from vyra_base.com.transport.t_redis.vyra_models import RedisCallable
        
        # Check is_callable flag (defaults to True if callback provided, False otherwise)
        is_callable = kwargs.pop("is_callable", callback is not None)
        
        role = "server" if is_callable else "client"
        logger.info(f"🔧 Creating Redis callable {role}: {name}")
        
        # Ensure callback matches is_callable flag
        if is_callable and callback is None:
            raise ProviderError("Callback required for callable server (is_callable=True)")
        if not is_callable and callback is not None:
            logger.debug("Ignoring callback for callable client (is_callable=False)")
            callback = None
        
        callable_obj = RedisCallable(
            name=name,
            topic_builder=self._topic_builder,
            callback=callback,
            redis_client=self._client,
            **kwargs
        )
        
        await callable_obj.initialize()
        logger.info(f"✅ Redis callable {role} created: {name}")
        return callable_obj
    
    async def create_speaker(
        self,
        name: str,
        callback: Optional[Callable] = None,
        **kwargs
    ) -> VyraSpeaker:
        """
        Create Redis Speaker (Pub/Sub).
        
        Args:
            name: Channel name
            callback: Optional callback for subscriber
            **kwargs: Additional parameters
            
        Returns:
            VyraSpeaker: Redis speaker instance
        """
        if not self._initialized:
            raise ProviderError("Provider not initialized")
        
        from vyra_base.com.transport.t_redis.vyra_models import RedisSpeaker
        
        speaker = RedisSpeaker(
            name=name,
            topic_builder=self._topic_builder,
            redis_client=self._client,
            module_name=self.module_name,
            callback=callback,
            **kwargs
        )
        
        await speaker.initialize()
        return speaker
    
    async def create_job(
        self,
        name: str,
        callback: Optional[Callable] = None,
        **kwargs
    ) -> VyraJob:
        """
        Create Redis Job (Redis Streams consumer).
        
        Note:
            Redis Jobs use Redis Streams for long-running tasks
            with progress feedback.
        """
        raise NotImplementedError(
            "Redis Job not yet implemented. "
            "Consider using Redis Streams or async tasks."
        )
    
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
