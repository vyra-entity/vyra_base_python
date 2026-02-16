"""
Redis Client Implementation

Uses request/response pattern with Redis channels.
"""

import logging
import json
import asyncio
import uuid
from typing import Optional, Any, Callable, Awaitable

from vyra_base.com.core.types import VyraClient, ProtocolType
from vyra_base.com.core.topic_builder import TopicBuilder
from vyra_base.com.transport.t_redis.communication import RedisClient

logger = logging.getLogger(__name__)


class RedisClientImpl(VyraClient):
    """
    Redis-based client implementation using request/response pattern.
    
    Pattern:
    - Publishes request to "srv:{service_name}:requests" with unique response_channel
    - Listens on own response_channel for response
    - Matches responses by request_id
    """
    
    def __init__(
        self,
        name: str,
        topic_builder: TopicBuilder,
        request_callback: Optional[Callable[[Any], Awaitable[None]]] = None,
        redis_client: RedisClient = None,
        service_type: type = None,
        **kwargs
    ):
        super().__init__(name, topic_builder, request_callback, ProtocolType.REDIS, **kwargs)
        self._redis = redis_client
        self.service_type = service_type
        self._client_id = str(uuid.uuid4())[:8]
        self._response_channel = f"srv:{service_name}:response:{self._client_id}"
        self._pubsub = None
        self._pending_requests = {}  # request_id -> asyncio.Future
        self._listen_task: Optional[asyncio.Task] = None
        
    async def initialize(self) -> bool:
        """Initialize Redis client."""
        try:
            service_name = self.topic_builder.build_topic(self.name)
            self._service_name = service_name
            self._response_channel = f"srv:{service_name}:response:{self._client_id}"
            
            # Create pubsub for responses
            self._pubsub = self._redis.pubsub()
            await self._pubsub.subscribe(self._response_channel)
            
            # Start listening for responses
            self._listen_task = asyncio.create_task(self._listen_loop())
            
            logger.info(f"✅ RedisClient initialized: {self.service_name}")
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to initialize RedisClient: {e}")
            return False
    
    async def call(self, request: Any, timeout: float = 5.0) -> Optional[Any]:
        """
        Send request and wait for response.
        
        Args:
            request: Request message instance or dict
            timeout: Response timeout in seconds
            
        Returns:
            Response object on success, None on failure
        """
        if not self._pubsub:
            logger.error("Client not initialized")
            return None
            
        try:
            # Generate request ID
            request_id = str(uuid.uuid4())
            
            # Create future for response
            response_future = asyncio.Future()
            self._pending_requests[request_id] = response_future
            
            # Serialize request
            # TODO: Support srv_type.Request serialization
            if hasattr(request, 'to_dict'):
                request_data = request.to_dict()
            elif isinstance(request, dict):
                request_data = request
            else:
                request_data = {'request': str(request)}
            
            # Build request message
            request_msg = json.dumps({
                'request_id': request_id,
                'data': request_data,
                'response_channel': self._response_channel
            })
            
            # Publish request
            await self._redis.publish(f"srv:{self._service_name}:requests", request_msg)
            
            # Wait for response with timeout
            try:
                response_data = await asyncio.wait_for(response_future, timeout=timeout)
                
                # Optional: Call request_callback
                if self.request_callback:
                    await self.request_callback(response_data)
                
                return response_data
                
            except asyncio.TimeoutError:
                logger.error(f"❌ Request timeout after {timeout}s")
                return None
                
        except Exception as e:
            logger.error(f"❌ Call failed: {e}")
            return None
            
        finally:
            # Cleanup pending request
            self._pending_requests.pop(request_id, None)
    
    async def _listen_loop(self):
        """Background task to receive responses."""
        try:
            while True:
                message = await self._pubsub.get_message(ignore_subscribe_messages=True)
                
                if message and message['type'] == 'message':
                    try:
                        response_msg = json.loads(message['data'])
                        request_id = response_msg.get('request_id')
                        response_data = response_msg.get('data')
                        
                        # Resolve pending request
                        if request_id in self._pending_requests:
                            future = self._pending_requests[request_id]
                            if not future.done():
                                future.set_result(response_data)
                                
                    except Exception as e:
                        logger.error(f"❌ Response handling failed: {e}")
                
                await asyncio.sleep(0.01)
                
        except asyncio.CancelledError:
            logger.debug("Client listen loop canceled")
        except Exception as e:
            logger.error(f"❌ Client listen loop failed: {e}")
    
    async def cleanup(self):
        """Cleanup Redis resources."""
        # Cancel pending requests
        for future in self._pending_requests.values():
            if not future.done():
                future.cancel()
        self._pending_requests.clear()
        
        if self._listen_task:
            self._listen_task.cancel()
            try:
                await self._listen_task
            except asyncio.CancelledError:
                pass
            self._listen_task = None
            
        if self._pubsub:
            await self._pubsub.unsubscribe(self._response_channel)
            await self._pubsub.close()
            self._pubsub = None
            
        logger.info(f"🔄 RedisClient cleaned up: {self.name}")
    
    async def shutdown(self) -> None:
        """Shutdown interface."""
        await self.cleanup()
        self._initialized = False
    