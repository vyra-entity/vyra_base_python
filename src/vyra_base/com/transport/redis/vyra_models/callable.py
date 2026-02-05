"""
Redis Callable Implementation

Concrete implementation of VyraCallable for Redis key-value operations.
Provides request-response pattern via Redis GET/SET operations.
"""
import asyncio
import logging
import json
import uuid

from typing import Any, Callable, Optional

from vyra_base.com.core.types import VyraCallable, ProtocolType
from vyra_base.com.core.exceptions import InterfaceError
from vyra_base.com.transport.redis.communication.redis_client import RedisClient

logger = logging.getLogger(__name__)


class RedisCallable(VyraCallable):
    """
    Redis-specific implementation of VyraCallable using key-value operations.
    
    Server-side: Listens to request key pattern, executes callback, writes response
    Client-side: Writes request to key, waits for response key
    
    Example:
        >>> # Server-side
        >>> async def handle_request(request):
        ...     return {"result": request["value"] * 2}
        >>> 
        >>> callable = RedisCallable(
        ...     name="calculate",
        ...     callback=handle_request,
        ...     redis_client=client
        ... )
        >>> await callable.initialize()
        >>> 
        >>> # Client-side
        >>> callable = RedisCallable(
        ...     name="calculate",
        ...     redis_client=client
        ... )
        >>> response = await callable.call({"value": 42})
    """
    
    def __init__(
        self,
        name: str,
        callback: Optional[Callable] = None,
        redis_client: Optional[RedisClient] = None,
        request_key_pattern: str = "request:{name}:{id}",
        response_key_pattern: str = "response:{name}:{id}",
        ttl: int = 300,  # 5 minutes
        **kwargs
    ):
        """
        Initialize Redis callable.
        
        Args:
            name: Callable name
            callback: Server-side callback function
            redis_client: RedisClient instance
            request_key_pattern: Pattern for request keys
            response_key_pattern: Pattern for response keys
            ttl: Time-to-live for keys in seconds
        """
        super().__init__(name, callback, ProtocolType.REDIS, **kwargs)
        self.redis_client: RedisClient | None = redis_client
        self.request_key_pattern = request_key_pattern
        self.response_key_pattern = response_key_pattern
        self.ttl = ttl
        self._listener_task: Optional[Any] = None
        self._running = False
    
    async def initialize(self) -> bool:
        """Initialize Redis callable."""
        if self._initialized:
            logger.warning(f"RedisCallable '{self.name}' already initialized")
            return True
        
        if not self.redis_client:
            raise InterfaceError("redis_client is required for RedisCallable")
        
        try:
            if self.callback:
                # Server-side: start listening for requests
                logger.info(f"🔧 Starting Redis callable server: {self.name}")
                await self._start_server()
                logger.info(f"✅ Redis callable server started: {self.name}")
            else:
                # Client-side: just mark as initialized
                logger.info(f"🔧 Redis callable client ready: {self.name}")
            
            self._initialized = True
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to initialize RedisCallable '{self.name}': {e}")
            raise InterfaceError(f"Failed to initialize RedisCallable: {e}")
    
    async def _start_server(self):
        """Start server-side request listener."""
        import asyncio
        
        self._running = True
        pattern = self.request_key_pattern.format(name=self.name, id="*")
        
        if self.redis_client is None:
            raise InterfaceError("redis_client is not set")

        # Subscribe to request key pattern
        await self.redis_client.subscribe_pattern(pattern)
        
        # Start background task
        self._listener_task = asyncio.create_task(self._process_requests())
    
    async def _process_requests(self):
        """Background task to process incoming requests."""
        logger.info(f"📡 Redis callable '{self.name}' listening for requests")
        
        if self.redis_client is None:
            raise InterfaceError("redis_client is not set")

        if self.callback is None:
            raise InterfaceError("callback is not set for server-side RedisCallable")

        while self._running:
            try:
                # This would need integration with RedisClient's pubsub listener
                # For now, polling approach:
                await asyncio.sleep(0.1)
                
                # Get pending requests
                pattern = self.request_key_pattern.format(name=self.name, id="*")
                keys = await self.redis_client.get_keys_by_pattern(pattern)
                
                for key in keys:
                    # Extract request ID
                    request_id = key.split(":")[-1]
                    
                    # Get request data
                    request_data = await self.redis_client.get(key)
                    if request_data is None:
                        continue
                    
                    # Parse request
                    if isinstance(request_data, str):
                        request_data = json.loads(request_data)
                    
                    # Execute callback
                    response_data = await self.callback(request_data)
                    
                    # Write response
                    response_key = self.response_key_pattern.format(
                        name=self.name, 
                        id=request_id
                    )
                    await self.redis_client.set(response_key, response_data)
                    
                    # Delete request key
                    await self.redis_client.delete(key)
                    
            except Exception as e:
                logger.error(f"❌ Error processing Redis request: {e}")
    
    async def shutdown(self) -> None:
        """Shutdown and cleanup Redis callable resources."""
        if not self._initialized:
            return
        
        logger.info(f"🛑 Shutting down RedisCallable: {self.name}")
        
        self._running = False
        
        if self._listener_task:
            self._listener_task.cancel()
            try:
                await self._listener_task
            except:
                pass
            self._listener_task = None
        
        self._initialized = False
        logger.info(f"✅ RedisCallable '{self.name}' shutdown complete")
    
    async def call(self, request: Any, timeout: float = 5.0) -> Any:
        """
        Call the Redis callable (client-side).
        
        Args:
            request: Request data
            timeout: Timeout in seconds
            
        Returns:
            Response data
            
        Raises:
            InterfaceError: If call fails or times out
        """
        if not self._initialized:
            raise InterfaceError(f"RedisCallable '{self.name}' not initialized")
        
        # Generate unique request ID
        request_id = str(uuid.uuid4())
        
        # Build keys
        request_key = self.request_key_pattern.format(name=self.name, id=request_id)
        response_key = self.response_key_pattern.format(name=self.name, id=request_id)
        
        if self.redis_client is None:
            raise InterfaceError("redis_client is not set")

        try:
            logger.debug(f"📞 Calling Redis callable: {self.name}")
            
            # Write request
            await self.redis_client.set(request_key, request)
            
            # Wait for response (polling)
            start_time = asyncio.get_event_loop().time()
            while True:
                # Check for response
                response = await self.redis_client.get(response_key)
                if response is not None:
                    # Clean up response key
                    await self.redis_client.delete(response_key)
                    
                    logger.debug(f"✅ Redis callable call succeeded: {self.name}")
                    return response
                
                # Check timeout
                if asyncio.get_event_loop().time() - start_time > timeout:
                    # Clean up request key
                    await self.redis_client.delete(request_key)
                    raise TimeoutError(f"Redis callable call timed out after {timeout}s")
                
                # Sleep before next poll
                await asyncio.sleep(0.05)
            
        except Exception as e:
            logger.error(f"❌ Redis callable call failed '{self.name}': {e}")
            raise InterfaceError(f"Callable call failed: {e}")
