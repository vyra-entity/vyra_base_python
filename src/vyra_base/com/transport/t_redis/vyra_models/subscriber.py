"""
Redis Subscriber Implementation

Uses Redis Pub/Sub channels for message reception.
"""

import logging
import json
import asyncio
from typing import Optional, Any, Callable, Awaitable

from vyra_base.com.core.types import VyraSubscriber, ProtocolType
from vyra_base.com.core.topic_builder import TopicBuilder
from vyra_base.com.transport.t_redis.vyra_models.client import RedisClient

logger = logging.getLogger(__name__)


class RedisSubscriberImpl(VyraSubscriber):
    """
    Redis-based subscriber implementation using Pub/Sub.
    
    Pattern: Subscribes to Redis channel and listens for messages.
    """
    
    def __init__(
        self,
        name: str,
        topic_builder: TopicBuilder,
        subscriber_callback: Callable[[Any], Awaitable[None]],
        redis_client: RedisClient,
        message_type: type,
        **kwargs
    ):
        super().__init__(name, topic_builder, subscriber_callback, ProtocolType.REDIS, **kwargs)
        self._redis = redis_client
        self.message_type = message_type
        self.subscriber_callback = subscriber_callback
        
    async def initialize(self) -> bool:
        """Initialize Redis subscriber."""
        try:
            self._topic_name = self.topic_builder.build(self.name, namespace=self.namespace, subsection=self.subsection)
            
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to initialize RedisSubscriber: {e}")
            return False
    
    async def subscribe(self) -> bool:
        """
        Start subscribing to Redis channel.
        
        Returns:
            True on success
        """            
        try:
            # Create pubsub instance
            await self._redis.subscribe_channel(self._topic_name)
            logger.info(f"✅ RedisSubscriber initialized: {self._topic_name}")

            # Wrap the user callback to match the Redis transport signature
            # (message_dict, context) and extract only the data payload.
            user_cb = self.subscriber_callback

            async def _redis_callback(message: dict, context: Any = None) -> None:
                raw = message.get("data", "{}")
                if isinstance(raw, (bytes, bytearray)):
                    raw = raw.decode("utf-8")
                try:
                    data = json.loads(raw) if isinstance(raw, str) else raw
                except (json.JSONDecodeError, TypeError):
                    data = raw
                if asyncio.iscoroutinefunction(user_cb):
                    await user_cb(data)
                else:
                    user_cb(data)

            await self._redis.create_pubsub_listener(
                self._topic_name, _redis_callback
            )
            
            logger.info(f"📡 Subscribed to Redis channel: {self._topic_name}")
            return True
            
        except Exception as e:
            logger.error(f"❌ Subscribe failed: {e}")
            return False
    
    async def cleanup(self):
        """Cleanup Redis resources."""
        if self._redis:
            await self._redis.remove_listener_channels(self._topic_name)
            
        logger.info(f"🔄 RedisSubscriber cleaned up: {self.name}")
    
    async def shutdown(self) -> None:
        """Shutdown subscriber."""
        await self.cleanup()
        self._initialized = False
