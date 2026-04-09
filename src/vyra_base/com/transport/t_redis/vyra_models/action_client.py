"""
Redis Action Client Implementation

Uses Redis keys for state tracking and pub/sub for updates.
"""

import logging
import json
import asyncio
import uuid
from typing import Optional, Any, Callable, Awaitable

from vyra_base.com.core.types import VyraActionClient, ProtocolType
from vyra_base.com.core.topic_builder import TopicBuilder
from vyra_base.com.core.exceptions import InterfaceError
from vyra_base.com.transport.t_redis.communication import RedisClient

logger = logging.getLogger(__name__)


class RedisActionClientImpl(VyraActionClient):
    """
    Redis-based action client with state tracking.
    
    Architecture:
    - Publishes goals to ``{fn}/goal`` channel
    - Subscribes to ``{fn}/{goal_id}/updates`` for feedback/result
    - Reads ``{fn}/{goal_id}/feedback`` and ``{fn}/{goal_id}/result`` keys
    
    All keys are built via :py:meth:`_action_channel` following VYRA naming
    conventions.
    """
    
    def __init__(
        self,
        name: str,
        topic_builder: TopicBuilder,
        direct_response: Optional[Callable[[Any], Awaitable[None]]] = None,
        feedback_callback: Optional[Callable[[Any], Awaitable[None]]] = None,
        goal_response_callback: Optional[Callable[[Any], Awaitable[None]]] = None,
        redis_client: RedisClient = None,
        action_type: type = None,
        **kwargs
    ):
        super().__init__(
            name, topic_builder,
            direct_response, feedback_callback, goal_response_callback,
            ProtocolType.REDIS, **kwargs
        )
        self.action_type = action_type
        self._redis: RedisClient | None = redis_client
        self._client_id = str(uuid.uuid4())[:8]
        self._active_goal_id: Optional[str] = None
        self._listen_task: Optional[asyncio.Task] = None
        self._goal_response_future: Optional[asyncio.Future] = None
        self.direct_response_callback = direct_response
        self.feedback_callback = feedback_callback

    async def initialize(self) -> bool:
        """Initialize Redis action client."""
        try:
            action_name = self.topic_builder.build(self.name, namespace=self.namespace, subsection=self.subsection)
            self._action_name = action_name
            self._key_goal = self._action_channel("goal")
            self._key_cancel = self._action_channel("cancel")
            logger.info(f"✅ RedisActionClient initialized: {action_name}")
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to initialize RedisActionClient: {e}")
            return False
    
    async def send_goal(self, goal: Any, timeout: float = 5.0) -> Optional[str]:
        """
        Send goal to action server.
        
        Args:
            goal: Goal message instance or dict
            timeout: Goal acceptance timeout
            
        Returns:
            goal_id on success, None on failure
        """
        try:
            # Generate goal ID on the client side so we can subscribe to updates
            # BEFORE sending the goal (prevents race condition where the server
            # completes execution and publishes the result notification before we
            # have subscribed to the updates channel).
            goal_id = str(uuid.uuid4())

            if not self._redis:
                raise InterfaceError("Redis client not initialized")

            # Subscribe to updates channel FIRST (using our pre-generated goal_id)
            await self._subscribe_to_updates(goal_id)

            # Create response future
            self._goal_response_future = asyncio.Future()
            response_channel = self._action_channel(f"response/{self._client_id}")

            # Subscribe to this specific response (for goal acceptance)
            await self._redis.subscribe_channel(response_channel)

            # Start listening for response
            async def wait_response(message, context):
                if not self._redis:
                    raise InterfaceError("Redis client not initialized")

                try:
                    response = json.loads(message['data'])
                    if self._goal_response_future and not self._goal_response_future.done():
                        self._goal_response_future.set_result(response)
                finally:
                    await self._redis.remove_listener_channels(response_channel)

            await self._redis.create_pubsub_listener(
                response_channel,
                wait_response
            )

            # Serialize goal
            # Convert action_type.Goal to dict if needed
            if self.action_type and hasattr(goal, '__dict__'):
                # Object with attributes
                if hasattr(goal, 'to_dict'):
                    goal_data = goal.to_dict()
                else:
                    # Extract public attributes
                    goal_data = {k: v for k, v in goal.__dict__.items() if not k.startswith('_')}
            elif hasattr(goal, 'to_dict'):
                goal_data = goal.to_dict()
            elif isinstance(goal, dict):
                goal_data = goal
            else:
                goal_data = {'goal': str(goal)}

            # Publish goal request (server will reuse our goal_id)
            goal_msg = json.dumps({
                'goal_id': goal_id,
                'goal': goal_data,
                'response_channel': response_channel
            })
            await self._redis.publish_message(self._key_goal, goal_msg)

            # Wait for acceptance
            try:
                response = await asyncio.wait_for(self._goal_response_future, timeout=timeout)

                if response.get('accepted'):
                    self._active_goal_id = response.get('goal_id', goal_id)
                    return self._active_goal_id
                else:
                    logger.warning(f"Goal rejected by server")
                    return None

            except asyncio.TimeoutError:
                logger.error(f"❌ Goal acceptance timeout")
                return None

        except Exception as e:
            logger.error(f"❌ Send goal failed: {e}")
            return None
    
    async def _subscribe_to_updates(self, goal_id: str):
        """Subscribe to feedback and result updates."""
        if not self._redis:
            logger.error("Redis client not initialized")
            return
        
        try:
            # Use RedisClient's subscribe_channel method
            await self._redis.subscribe_channel(self._action_channel(f"{goal_id}/updates"))
            
            await self._redis.create_pubsub_listener(
                self._action_channel(f"{goal_id}/updates"),
                self._listen_updates,
                callback_context={'goal_id': goal_id}
            )
            
        except Exception as e:
            logger.error(f"❌ Failed to subscribe to updates: {e}")
    
    async def _listen_updates(self, message, context: dict):
        """Listen for feedback and result notifications."""
        try:
            goal_id = context.get('goal_id')

            if not self._redis:
                logger.error("Redis client not initialized")
                return

            try:
                update = json.loads(message['data'])
                update_type = update.get('type')

                if update_type == 'feedback' and self.feedback_callback:
                    # Fetch feedback payload from Redis key set by the action server
                    raw = await self._redis.get(self._action_channel(f"{goal_id}/feedback"))
                    if raw:
                        await self.feedback_callback(json.loads(raw))

                elif update_type == 'result':
                    # Fetch result payload from Redis key set by the action server
                    raw = await self._redis.get(self._action_channel(f"{goal_id}/result"))
                    if raw and self.direct_response_callback:
                        await self.direct_response_callback(json.loads(raw))

            except Exception as e:
                logger.error(f"❌ Update handling failed: {e}")

            await asyncio.sleep(0.01)

        except asyncio.CancelledError:
            pass
        except Exception as e:
            logger.error(f"❌ Updates listen loop failed: {e}")
    
    async def cancel_goal(self, goal_id: str, timeout: float = 5.0) -> bool:
        """
        Cancel active goal.
        
        Args:
            goal_id: ID of goal to cancel
            timeout: Cancel timeout
            
        Returns:
            True on successful cancellation
        """
        try:
            # Create response future
            response_future = asyncio.Future()
            response_channel = self._action_channel(f"cancel_response/{self._client_id}")
            
            if not self._redis:
                raise InterfaceError("Redis client not initialized")
            
            # Subscribe to cancel response
            await self._redis.subscribe_channel(response_channel)
            
            # Wait for cancel response message
            async def wait_cancel_response(message, context):
                if not self._redis:
                    raise InterfaceError("Redis client not initialized")
                try:
                    response = json.loads(message['data'])
                    if not response_future.done():
                        response_future.set_result(response)
                finally:
                    await self._redis.remove_listener_channels(response_channel)
            
            # Send cancel request
            cancel_msg = json.dumps({
                'goal_id': goal_id,
                'response_channel': response_channel
            })

            await self._redis.create_pubsub_listener(
                self._key_cancel,
                wait_cancel_response,
                cancel_msg
            )
            return True
                
        except Exception as e:
            logger.error(f"❌ Cancel goal failed: {e}")
            return False
    
    async def cleanup(self):
        """Cleanup Redis resources.

        Unsubscribes channels registered by this client but does NOT close the
        underlying Redis connection because it is shared with other components
        (action servers, publishers, etc.) managed by the provider.
        """
        if self._listen_task:
            self._listen_task.cancel()
            try:
                await self._listen_task
            except asyncio.CancelledError:
                pass

        # Unsubscribe only the channels this client registered, leaving the
        # shared Redis connection open for other users.
        if self._redis and self._active_goal_id:
            updates_channel = self._action_channel(f"{self._active_goal_id}/updates")
            try:
                await self._redis.remove_listener_channels(updates_channel)
            except Exception as e:
                logger.debug(f"⚠️ Could not remove updates channel on cleanup: {e}")

        logger.info(f"🔄 RedisActionClient cleaned up: {self.name}")
    
    async def shutdown(self) -> None:
        """Shutdown interface."""
        await self.cleanup()
        self._initialized = False
    