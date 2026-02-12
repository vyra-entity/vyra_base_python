"""
Redis Speaker Implementation

Pub/Sub communication via Redis channels.
"""
import asyncio
import json
import logging
from typing import Any, Callable, Optional

from vyra_base.com.core.types import VyraSpeaker, ProtocolType
from vyra_base.com.core.exceptions import SpeakerError, TransportError
from vyra_base.com.core.topic_builder import TopicBuilder, InterfaceType
from vyra_base.com.converters.protobuf_converter import ProtobufConverter

logger = logging.getLogger(__name__)


class RedisSpeaker(VyraSpeaker):
    """
    Redis Speaker for Pub/Sub communication.
    
    Features:
    - Publish to Redis channels
    - Subscribe to Redis channels with callback
    - Automatic JSON serialization
    - Pattern subscriptions (wildcards)
    
    Naming Convention:
        Uses TopicBuilder for consistent naming: <module_name>_<module_id>/<function_name>
        Example: v2_modulemanager_abc123/sensor_data
    
    Example:
        >>> # Publisher
        >>> speaker = RedisSpeaker(
        ...     "sensor_data",
        ...     redis_client=client,
        ...     module_name="robot"
        ... )
        >>> await speaker.initialize()
        >>> await speaker.shout({"temperature": 23.5})
        >>> 
        >>> # Subscriber
        >>> async def on_data(message):
        ...     print(f"Received: {message}")
        >>> 
        >>> speaker = RedisSpeaker(
        ...     "sensor_data",
        ...     redis_client=client,
        ...     callback=on_data
        ... )
        >>> await speaker.initialize()
        >>> await speaker.listen(on_data)
    """
    
    def __init__(
        self,
        name: str,
        topic_builder: TopicBuilder,
        redis_client: Any,
        module_name: str = "default",
        callback: Optional[Callable] = None,
        pattern: bool = False,
        protobuf_type: Optional[Any] = None,
        **kwargs
    ):
        """
        Initialize Redis speaker.
        
        Args:
            name: Channel name (or pattern if pattern=True)
            redis_client: vyra_base RedisClient instance
            module_name: Module name for namespacing
            callback: Optional[Callable] = None,
            pattern: bool = False,
            protobuf_type: Optional protobuf message type for serialization
            topic_builder: TopicBuilder,
            **kwargs
        """       
        super().__init__(name, topic_builder, ProtocolType.REDIS, **kwargs)
        self._client = redis_client
        self.module_name = module_name
        self._callback = callback
        self._pattern = pattern
        self._listening = False
        self._listener_task: Optional[asyncio.Task] = None
        self.protobuf_type = protobuf_type
        self._protobuf_converter: Optional[ProtobufConverter] = None
        
        # Statistics
        self._publish_count = 0
        self._receive_count = 0
    
    async def initialize(self) -> bool:
        """Initialize Redis speaker."""
        if not self._client:
            raise SpeakerError("Redis client not provided")
        
        # Dynamic interface loading if protobuf_type not provided
        if not self.protobuf_type and self.topic_builder:
            try:
                components = self.topic_builder.parse(self.name)
                function_name = components.function_name
                if function_name:
                    self.protobuf_type = self.topic_builder.load_interface_type(
                        function_name, protocol="redis"
                    )
                    logger.debug(
                        f"🔄 Dynamically loaded protobuf type for Redis speaker '{self.name}': "
                        f"{self.protobuf_type.__name__ if self.protobuf_type else 'None'}"
                    )
            except Exception as e:
                logger.warning(
                    f"⚠️ Could not load protobuf type for Redis speaker '{self.name}': {e}. "
                    "Falling back to JSON mode."
                )
        
        # Initialize protobuf converter if type available
        if self.protobuf_type:
            self._protobuf_converter = ProtobufConverter()
            if not self._protobuf_converter or not self._protobuf_converter.is_available():
                logger.warning(
                    "⚠️ ProtobufConverter not available. Falling back to JSON mode."
                )
                self._protobuf_converter = None
        
        # Ensure Redis connection
        if not self._client._connected:
            await self._client.connect()
        
        self._initialized = True
        logger.debug(f"✅ Redis speaker initialized: {self.name}")
        return True
    
    async def shutdown(self) -> None:
        """Shutdown Redis speaker."""
        if self._listening:
            await self.stop_listening()
        
        self._initialized = False
        logger.debug(f"✅ Redis speaker shutdown: {self.name}")
    
    async def shout(self, message: Any) -> bool:
        """
        Publish message to Redis channel.
        
        Args:
            message: Message to publish (will be JSON-serialized)
            
        Returns:
            bool: True if published successfully
            
        Raises:
            SpeakerError: If not initialized or publish fails
        """
        if not self._initialized:
            raise SpeakerError(f"Speaker '{self.name}' not initialized")
        
        try:
            # Serialize message
            if self._protobuf_converter:
                # Use protobuf serialization
                message_data = self._protobuf_converter.serialize(message)
                # Redis publish expects string, so decode if bytes
                if isinstance(message_data, bytes):
                    message_str = message_data.decode('utf-8', errors='ignore')
                else:
                    message_str = message_data
            elif isinstance(message, (dict, list)):
                message_str = json.dumps(message)
            elif isinstance(message, str):
                message_str = message
            else:
                message_str = str(message)
            
            # Publish to channel
            # RedisClient.publish returns number of subscribers
            subscriber_count = await self._client.publish(self.name, message_str)
            
            self._publish_count += 1
            logger.debug(
                f"📤 Published to Redis channel '{self.name}': "
                f"{subscriber_count} subscribers"
            )
            
            return True
            
        except Exception as e:
            raise SpeakerError(f"Failed to publish to '{self.name}': {e}")
    
    async def listen(
        self,
        callback: Callable[[Any], None],
        **kwargs
    ) -> None:
        """
        Subscribe to Redis channel and listen for messages.
        
        Args:
            callback: Callback function for received messages
            **kwargs: Additional parameters (e.g., timeout)
            
        Raises:
            SpeakerError: If not initialized or already listening
        """
        if not self._initialized:
            raise SpeakerError(f"Speaker '{self.name}' not initialized")
        
        if self._listening:
            logger.warning(f"⚠️ Already listening on '{self.name}'")
            return
        
        self._callback = callback
        self._listening = True
        
        # Start listener task
        self._listener_task = asyncio.create_task(
            self._listen_loop(**kwargs)
        )
        
        logger.info(f"👂 Started listening on Redis channel: {self.name}")
    
    async def stop_listening(self) -> None:
        """Stop listening for messages."""
        if not self._listening:
            return
        
        self._listening = False
        
        if self._listener_task:
            self._listener_task.cancel()
            try:
                await self._listener_task
            except asyncio.CancelledError:
                pass
            self._listener_task = None
        
        # Unsubscribe from channel
        try:
            if self._pattern:
                await self._client.remove_listener_patterns([self.name])
            else:
                await self._client.remove_listener_channels([self.name])
        except Exception as e:
            logger.error(f"❌ Error unsubscribing: {e}")
        
        logger.info(f"🛑 Stopped listening on Redis channel: {self.name}")
    
    async def _listen_loop(self, timeout: float = 1.0) -> None:
        """
        Internal listener loop.
        
        Uses RedisClient's create_pubsub_listener for efficient listening.
        """
        try:
            # Define message handler
            async def message_handler(message_data):
                """Handle incoming Redis message."""
                try:
                    # Parse message
                    if isinstance(message_data, bytes):
                        raw_data = message_data
                    elif isinstance(message_data, dict):
                        # Already parsed by Redis
                        raw_data = message_data.get('data', message_data)
                        if isinstance(raw_data, str):
                            raw_data = raw_data.encode('utf-8')
                    else:
                        raw_data = str(message_data).encode('utf-8')
                    
                    # Deserialize message
                    if self._protobuf_converter:
                        # Use protobuf deserialization
                        message = self._protobuf_converter.deserialize(raw_data, self.protobuf_type)
                    else:
                        # Fallback to JSON deserialization
                        message_str = raw_data.decode('utf-8')
                        try:
                            message = json.loads(message_str)
                        except (json.JSONDecodeError, TypeError):
                            message = message_str
                    
                    self._receive_count += 1
                    logger.debug(f"📥 Received message on '{self.name}': {message}")
                    
                    # Call user callback
                    if self._callback:
                        if asyncio.iscoroutinefunction(self._callback):
                            await self._callback(message)
                        else:
                            self._callback(message)
                    
                except Exception as e:
                    logger.error(f"❌ Error processing message: {e}")
            
            # Subscribe using RedisClient's listener
            channels = [self.name]
            await self._client.create_pubsub_listener(
                channels=channels,
                callback_handler=message_handler
            )
            
            # Keep loop alive
            while self._listening:
                await asyncio.sleep(timeout)
            
        except asyncio.CancelledError:
            logger.debug("🛑 Listener loop cancelled")
            raise
        except Exception as e:
            logger.exception(f"❌ Listener loop error: {e}")
            raise SpeakerError(f"Listener loop failed: {e}")
    
    def get_statistics(self) -> dict:
        """Get speaker statistics."""
        return {
            "name": self.name,
            "module": self.module_name,
            "protocol": "redis",
            "initialized": self._initialized,
            "listening": self._listening,
            "publish_count": self._publish_count,
            "receive_count": self._receive_count,
            "pattern": self._pattern,
        }
