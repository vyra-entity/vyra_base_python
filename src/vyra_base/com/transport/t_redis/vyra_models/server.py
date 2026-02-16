"""
Redis Server Implementation

Uses request/response pattern with Redis keys and channels.
"""

import logging
import json
import asyncio
import uuid
from typing import Optional, Any, Callable, Awaitable

from vyra_base.com.core.types import VyraServer, ProtocolType
from vyra_base.com.core.topic_builder import TopicBuilder
from vyra_base.com.transport.t_redis.communication.redis_client import RedisClient

logger = logging.getLogger(__name__)


class RedisServerImpl(VyraServer):
    """
    Redis-based server implementation using request/response pattern.
    
    Pattern:
    - Listens on channel "srv:{service_name}:requests"
    - Receives requests with {request_id, data, response_channel}
    - Processes request via callback
    - Publishes response to response_channel with {request_id, data}
    """
    
    def __init__(
        self,
        name: str,
        topic_builder: TopicBuilder,
        response_callback: Callable[[Any], Awaitable[Any]],
        redis_client: RedisClient,
        service_type: type,
        **kwargs
    ):
        super().__init__(name, topic_builder, response_callback, ProtocolType.REDIS, **kwargs)
        self._redis = redis_client
        self.service_type = service_type
        self._pubsub = None
        self._listen_task: Optional[asyncio.Task] = None

    async def initialize(self) -> bool:
        """Initialize Redis server."""
        try:
            service_name = self.topic_builder.build_topic(self.name)
            self._service_name = service_name
        
            self._pubsub = self._redis.pubsub()
            
            logger.info(f"✅ RedisServer initialized: {service_name}")
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to initialize RedisServer: {e}")
            return False
    
    async def serve(self) -> bool:
        """
        Start serving requests.
        
        Returns:
            True on success
        """
        if not self._pubsub:
            logger.error("Server not initialized")
            return False
            
        try:
            # Subscribe to request channel
            await self._pubsub.subscribe(self._request_channel)
            
            # Start listening task
            self._listen_task = asyncio.create_task(self._listen_loop())
            
            logger.info(f"📡 RedisServer serving on: {self._request_channel}")
            return True
            
        except Exception as e:
            logger.error(f"❌ Serve failed: {e}")
            return False
    
    async def _listen_loop(self):
        """Background task to handle incoming requests."""
        try:
            while True:
                message = await self._pubsub.get_message(ignore_subscribe_messages=True)
                
                if message and message['type'] == 'message':
                    # Handle request in separate task to avoid blocking
                    asyncio.create_task(self._handle_request(message['data']))
                
                await asyncio.sleep(0.01)
                
        except asyncio.CancelledError:
            logger.debug("Server listen loop canceled")
        except Exception as e:
            logger.error(f"❌ Server listen loop failed: {e}")
    
    async def _handle_request(self, raw_data: bytes):
        """Process individual request."""
        try:
            # Parse request
            request_msg = json.loads(raw_data)
            request_id = request_msg.get('request_id')
            request_data = request_msg.get('data')
            response_channel = request_msg.get('response_channel')
            
            if not all([request_id, response_channel]):
                logger.error("Invalid request format")
                return
            
            # TODO: Deserialize request_data to srv_type.Request
            
            # Call user callback
            response = await self.response_callback(request_data)
            
            # TODO: Serialize response from srv_type.Response
            
            # Publish response
            response_msg = json.dumps({
                'request_id': request_id,
                'data': response if isinstance(response, dict) else {'result': str(response)}
            })
            
            await self._redis.publish(response_channel, response_msg)
            
        except Exception as e:
            logger.error(f"❌ Request handling failed: {e}")
    
    async def cleanup(self):
        """Cleanup Redis resources."""
        if self._listen_task:
            self._listen_task.cancel()
            try:
                await self._listen_task
            except asyncio.CancelledError:
                pass
            self._listen_task = None
            
        if self._pubsub:
            await self._pubsub.unsubscribe(self._request_channel)
            await self._pubsub.close()
            self._pubsub = None
            
        logger.info(f"🔄 RedisServer cleaned up: {self.name}")
    
    async def shutdown(self) -> None:
        """Shutdown interface."""
        await self.cleanup()
        self._initialized = False
    