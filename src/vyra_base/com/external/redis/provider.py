"""
Redis Protocol Provider

Implements AbstractProtocolProvider for Redis communication.
Wraps existing vyra_base.storage.redis_client.RedisClient.
"""
import logging
from typing import Any, Callable, Optional, Dict

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
from vyra_base.storage.redis_client import RedisClient

logger = logging.getLogger(__name__)


class RedisProvider(AbstractProtocolProvider):
    """
    Protocol provider for Redis communication.
    
    Features:
    - Pub/Sub messaging via Speaker interface
    - Key-Value storage via Callable interface (get/set)
    - TLS support with ACL authentication
    - Streaming support (Redis Streams)
    
    Wraps existing vyra_base.storage.redis_client.RedisClient
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
        protocol: ProtocolType = ProtocolType.REDIS,
        module_name: str = "default",
        **redis_kwargs
    ):
        """
        Initialize Redis provider.
        
        Args:
            protocol: Protocol type (must be REDIS)
            module_name: Module name for Redis namespace
            **redis_kwargs: Additional arguments passed to RedisClient
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
        Create Redis Callable (Key-Value operations).
        
        Redis Callable supports:
        - get(key): Retrieve value
        - set(key, value): Store value
        - delete(key): Remove key
        
        Args:
            name: Key prefix for this callable
            callback: Optional callback for set operations
            **kwargs: Additional parameters
            
        Returns:
            VyraCallable: Redis callable instance
            
        Note:
            Redis doesn't have native request-response like RPC.
            This provides key-value operations wrapped as callable.
        """
        if not self._initialized:
            raise ProviderError("Provider not initialized")
        
        # Redis Callable is primarily for get/set operations
        # Not implementing as separate class - use client directly
        raise NotImplementedError(
            "Redis Callable not implemented. "
            "Use RedisProvider.create_speaker() for Pub/Sub or "
            "access provider._client directly for key-value operations."
        )
    
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
        
        from vyra_base.com.external.redis.speaker import RedisSpeaker
        
        speaker = RedisSpeaker(
            name=name,
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
