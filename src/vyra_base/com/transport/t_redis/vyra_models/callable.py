"""
Redis Callable Implementation

Concrete implementation of VyraCallable for Redis key-value operations.
Provides request-response pattern via Redis GET/SET operations.

Supports both JSON (default) and Protocol Buffer message formats.
"""
import asyncio
import logging
import json
import uuid

from typing import Any, Callable, Optional

from vyra_base.com.core.types import VyraCallable, ProtocolType
from vyra_base.com.core.exceptions import InterfaceError
from vyra_base.com.transport.t_redis.communication.redis_client import RedisClient
from vyra_base.com.core.topic_builder import TopicBuilder, InterfaceType
from vyra_base.com.converters.protobuf_converter import ProtobufConverter

logger = logging.getLogger(__name__)


class RedisCallable(VyraCallable):
    """
    Redis-specific implementation of VyraCallable using key-value operations.
    
    Server-side: Listens to request key pattern, executes callback, writes response
    Client-side: Writes request to key, waits for response key
    
    Naming Convention:
        Uses TopicBuilder for consistent naming: <module_name>_<module_id>/<function_name>
        Example: request:v2_modulemanager_abc123/calculate:12345
    
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
        topic_builder: TopicBuilder,
        callback: Optional[Callable] = None,
        redis_client: Optional[RedisClient] = None,
        request_key_pattern: str = "request:{name}:{id}",
        response_key_pattern: str = "response:{name}:{id}",
        ttl: int = 300,  # 5 minutes
        protobuf_type: Optional[Any] = None,  # Protocol Buffer message class
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
            protobuf_type: Optional Protocol Buffer message class for serialization
            topic_builder: TopicBuilder for naming convention
        """
        # Apply topic builder if provided
        if topic_builder:
            name = topic_builder.build(name, interface_type=InterfaceType.CALLABLE)
            logger.debug(f"Applied TopicBuilder: {name}")
        
        super().__init__(name, topic_builder, callback, ProtocolType.REDIS, **kwargs)
        self.redis_client: RedisClient | None = redis_client
        self.request_key_pattern = request_key_pattern
        self.response_key_pattern = response_key_pattern
        self.ttl = ttl
        self.protobuf_type = protobuf_type
        self._protobuf_converter: Optional[ProtobufConverter] = None
        self._listener_task: Optional[Any] = None
        self._running = False
    
    async def initialize(self) -> bool:
        """Initialize Redis callable with optional Protobuf support."""
        if self._initialized:
            logger.warning(f"RedisCallable '{self.name}' already initialized")
            return True
        
        if not self.redis_client:
            raise InterfaceError("redis_client is required for RedisCallable")
        
        # Dynamic protobuf interface loading if not provided
        if not self.protobuf_type and self.topic_builder:
            logger.debug(f"protobuf_type not provided for '{self.name}', attempting dynamic loading")
            
            try:
                components = self.topic_builder.parse(self.name)
                function_name = components.function_name
                
                # Load protobuf interface dynamically (for redis/zenoh/uds)
                self.protobuf_type = self.topic_builder.load_interface_type(
                    function_name,
                    protocol="redis"  # Uses .proto files
                )
                
                if self.protobuf_type:
                    logger.info(
                        f"✓ Dynamically loaded protobuf type for '{self.name}': "
                        f"{self.protobuf_type}"
                    )
            except Exception as e:
                logger.debug(
                    f"Could not dynamically load protobuf type for '{self.name}': {e}. "
                    f"Falling back to JSON mode."
                )
        
        # Initialize protobuf converter if protobuf type available
        if self.protobuf_type:
            self._protobuf_converter = ProtobufConverter()
            
            if not self._protobuf_converter or not self._protobuf_converter.is_available():
                logger.warning(
                    f"⚠️ Protobuf converter unavailable for '{self.name}'. "
                    f"Falling back to JSON mode."
                )
                self._protobuf_converter = None
                self.protobuf_type = None
            else:
                logger.info(f"✓ Protobuf mode enabled for RedisCallable '{self.name}'")
        
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
                    if self._protobuf_converter:
                        # Use protobuf deserialization
                        if isinstance(request_data, str):
                            request_data = request_data.encode()
                        request = self._protobuf_converter.deserialize(request_data, self.protobuf_type)
                    else:
                        # Fallback to JSON deserialization
                        if isinstance(request_data, str):
                            request = json.loads(request_data)
                        else:
                            request = request_data
                    
                    # Execute callback
                    response_data = await self.callback(request)
                    
                    # Serialize response
                    if self._protobuf_converter:
                        # Use protobuf serialization
                        response_data = self._protobuf_converter.serialize(response_data)
                    
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
            
            # Serialize request
            if self._protobuf_converter:
                # Use protobuf serialization
                request_data = self._protobuf_converter.serialize(request)
            else:
                # Direct storage (existing behavior)
                request_data = request
            
            # Write request
            await self.redis_client.set(request_key, request_data)
            
            # Wait for response (polling)
            start_time = asyncio.get_event_loop().time()
            while True:
                # Check for response
                response_data = await self.redis_client.get(response_key)
                if response_data is not None:
                    # Clean up response key
                    await self.redis_client.delete(response_key)
                    
                    # Deserialize response
                    if self._protobuf_converter:
                        # Use protobuf deserialization
                        response = self._protobuf_converter.deserialize(response_data, self.protobuf_type)
                    else:
                        # Direct return (existing behavior)
                        response = response_data
                    
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
