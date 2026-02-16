"""
Redis Publisher Implementation

Uses Redis Pub/Sub channels for message broadcasting.
"""

import logging
import json
from typing import Optional, Any

from vyra_base.com.core.types import VyraPublisher, ProtocolType
from vyra_base.com.core.topic_builder import TopicBuilder
from vyra_base.com.transport.t_redis.communication.redis_client import RedisClient

logger = logging.getLogger(__name__)

class RedisPublisherImpl(VyraPublisher):
    """
    Redis-based publisher implementation using Pub/Sub.
    
    Pattern: Publishes messages to Redis channel at topic name.
    """
    
    def __init__(
        self,
        name: str,
        topic_builder: TopicBuilder,
        redis_client: RedisClient,
        message_type: type,
        **kwargs
    ):
        super().__init__(name, topic_builder, ProtocolType.REDIS, **kwargs)
        self._redis: RedisClient = redis_client
        self.message_type = message_type
        
    async def initialize(self) -> bool:
        """Initialize Redis publisher (no setup needed for pub)."""
        try:
            topic_name = self.topic_builder.build(self.name)
            self._topic_name = topic_name
            
            # Test Redis connection
            await self._redis.ping()
            
            logger.info(f"✅ RedisPublisher initialized: {topic_name}")
            self._initialized = True
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to initialize RedisPublisher: {e}")
            return False
    
    async def publish(self, message: Any) -> bool:
        """
        Publish message to Redis channel.
        
        Args:
            message: Message instance or dict
            
        Returns:
            True on success
        """
        try:
            # Serialize message to JSON  
            # TODO: Support protobuf serialization
            if hasattr(message, 'to_dict'):
                data = message.to_dict()
            elif isinstance(message, dict):
                data = message
            else:
                data = {'data': str(message)}
                
            message_json = json.dumps(data)
            
            # Publish to Redis channel
            num_subscribers = await self._redis.publish_message(
                self._topic_name, message_json)
            
            logger.debug(f"Published to {num_subscribers} subscribers on {self._topic_name}")
            
            return True
            
        except Exception as e:
            logger.error(f"❌ Publish failed: {e}")
            return False
    
    async def cleanup(self):
        """Cleanup resources (none needed for publisher)."""
        logger.info(f"🔄 RedisPublisher cleaned up: {self.name}")
    
    async def shutdown(self) -> None:
        """Shutdown publisher."""
        await self.cleanup()
        self._initialized = False
