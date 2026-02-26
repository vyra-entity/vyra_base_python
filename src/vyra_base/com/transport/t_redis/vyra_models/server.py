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
            service_name = self.topic_builder.build(self.name, namespace=self.namespace, subsection=self.subsection)
            self._service_name = service_name
            
            self._request_channel = f"srv:{service_name}:requests"
            await self._redis.subscribe_channel(self._request_channel)

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
        if not self._request_channel:
            logger.error("Server not initialized")
            return False
            
        try:
            # Subscribe to request channel
            await self._redis.create_pubsub_listener(
                self._request_channel, self._handle_request
            )
            
            logger.info(f"📡 RedisServer serving on: {self._request_channel}")
            return True
            
        except Exception as e:
            logger.error(f"❌ Serve failed: {e}")
            return False
    
    async def _handle_request(self, raw_data, context=None):
        """Process individual request."""
        try:
            # raw_data is the full Redis PubSub message dict: {type, channel, data, ...}
            # Extract actual payload from 'data' field
            if isinstance(raw_data, dict) and 'data' in raw_data:
                payload = raw_data['data']
            else:
                payload = raw_data

            # Parse request - handle both raw bytes/str and already-parsed dict
            if isinstance(payload, dict):
                request_msg = payload
            elif isinstance(payload, (bytes, str)):
                request_msg = json.loads(payload)
            else:
                request_msg = json.loads(str(payload))
            request_id = request_msg.get('request_id')
            request_data = request_msg.get('data')
            response_channel = request_msg.get('response_channel')
            
            if not all([request_id, response_channel]):
                logger.error("Invalid request format")
                return

            if not isinstance(response_channel, str):
                logger.error(f"Invalid response channel format: {type(response_channel)}")
                return
            
            if not self.response_callback:
                logger.error(f"No response callback defined for incoming data: {request_data}")
                response_msg = json.dumps({
                    'request_id': request_id,
                    'data': {'error': 'No response callback defined'}
                })
                await self._redis.publish_message(response_channel, response_msg)
                return

            # TODO: Deserialize request_data to srv_type.Request
            
            # Call user callback - support both (request, response) ROS2-style
            # and plain (request) callbacks. Use a dynamic response holder.
            class _ResponseHolder:
                """Dynamic attribute container for ROS2-style response objects."""
                def __getattr__(self, name):
                    return None
                def __setattr__(self, name, value):
                    object.__setattr__(self, name, value)
                def to_dict(self):
                    return {k: v for k, v in self.__dict__.items()
                            if not k.startswith('_')}

            import inspect
            sig = inspect.signature(self.response_callback)
            num_params = len(sig.parameters)

            if num_params >= 2:
                # ROS2-style: callback(request, response)
                response_holder = _ResponseHolder()
                await self.response_callback(request_data, response_holder)
                response_data = response_holder.to_dict()
            else:
                # Plain callback: callback(request)
                result = await self.response_callback(request_data)
                if isinstance(result, dict):
                    response_data = result
                elif result is not None:
                    response_data = {'result': str(result)}
                else:
                    response_data = {}
            
            # TODO: Serialize response from srv_type.Response
            
            # Publish response
            response_msg = json.dumps({
                'request_id': request_id,
                'data': response_data
            }, default=str)
            
            await self._redis.publish_message(response_channel, response_msg)
            
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
            
        if self._redis and self._request_channel:
            await self._redis.remove_listener_channels(self._request_channel)
            self._request_channel = None
            
        logger.info(f"🔄 RedisServer cleaned up: {self.name}")
    
    async def shutdown(self) -> None:
        """Shutdown interface."""
        await self.cleanup()
        self._initialized = False
    