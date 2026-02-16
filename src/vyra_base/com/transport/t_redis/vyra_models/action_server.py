"""
Redis Action Server Implementation

Uses Redis keys for state tracking and pub/sub for communication.
"""

import logging
import json
import asyncio
import uuid
from typing import Optional, Any, Callable, Awaitable, Dict
from enum import Enum

from vyra_base.com.core.types import VyraActionServer, ProtocolType
from vyra_base.com.core.topic_builder import TopicBuilder
from vyra_base.com.core.exceptions import InterfaceError
from vyra_base.com.transport.t_redis.communication import RedisClient

logger = logging.getLogger(__name__)


class GoalStatus(Enum):
    """Action goal execution status."""
    UNKNOWN = 0
    ACCEPTED = 1
    EXECUTING = 2
    SUCCEEDED = 3
    ABORTED = 4
    CANCELED = 5


class RedisActionServerImpl(VyraActionServer):
    """
    Redis-based action server with state tracking.
    
    Architecture:
    Keys:
    - action:{name}:{goal_id}:state → JSON: {status, timestamp}
    - action:{name}:{goal_id}:feedback → JSON: Latest feedback
    - action:{name}:{goal_id}:result → JSON: Final result
    
    Channels:
    - action:{name}:goal → Goal requests
    - action:{name}:cancel → Cancel requests
    - action:{name}:{goal_id}:updates → Feedback/result notifications
    """
    
    def __init__(
        self,
        name: str,
        topic_builder: TopicBuilder,
        handle_goal_request: Callable[[Any], Awaitable[bool]],
        handle_cancel_request: Callable[[Any], Awaitable[bool]],
        execution_callback: Callable[[Any], Awaitable[Any]],
        redis_client: RedisClient = None,
        action_type: type = None,
        **kwargs
    ):
        super().__init__(
            name, topic_builder,
            handle_goal_request, handle_cancel_request, execution_callback,
            ProtocolType.REDIS, **kwargs
        )
        self.action_type = action_type
        self._redis: RedisClient | None = redis_client
        self._active_goals: Dict[str, asyncio.Task] = {}  # goal_id -> execution_task
        
    async def initialize(self) -> bool:
        """Initialize Redis action server."""
        if not self._redis:
            logger.error("Redis client not initialized")
            return False
        
        try:
            action_name = self.topic_builder.build(self.name)
            self._action_name = action_name
                        
            # Subscribe to goal and cancel channels
            await self._redis.subscribe_channel(
                f"action:{action_name}:goal",
                f"action:{action_name}:cancel"
            )
            
            await self._redis.create_pubsub_listener(
                f"action:{action_name}:goal",
                self._handle_goal_request
            )

            await self._redis.create_pubsub_listener(
                f"action:{action_name}:cancel",
                self._handle_cancel_request
            )
            
            logger.info(f"✅ RedisActionServer initialized: {self._action_name}")
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to initialize RedisActionServer: {e}")
            return False
    
    async def _handle_goal_request(self, raw_data: bytes):
        """Handle incoming goal request."""
        try:
            goal_msg = json.loads(raw_data)
            goal_id = goal_msg.get('goal_id', str(uuid.uuid4()))
            goal_data = goal_msg.get('goal')
            response_channel = goal_msg.get('response_channel')
            
            # TODO: Deserialize goal_data to action_type.Goal
            
            # Call handle_goal callback
            if self.handle_goal_request:
                accepted = await self.handle_goal_request(goal_data)
            else:
                accepted = True
            
            if accepted:
                # Set initial state
                await self._set_goal_state(goal_id, GoalStatus.ACCEPTED)
                
                # Start execution task
                execution_task = asyncio.create_task(
                    self._execute_goal(goal_id, goal_data)
                )
                self._active_goals[goal_id] = execution_task
                
                # Send acceptance response
                if response_channel:
                    if not self._redis:
                        raise InterfaceError("Redis client not initialized")
                    await self._redis.publish_message(response_channel, json.dumps({
                        'goal_id': goal_id,
                        'accepted': True
                    }))
            else:
                # Send rejection response
                if response_channel:
                    if not self._redis:
                        raise InterfaceError("Redis client not initialized")
                    await self._redis.publish_message(response_channel, json.dumps({
                        'goal_id': goal_id,
                        'accepted': False
                    }))
                    
        except Exception as e:
            logger.error(f"❌ Goal request handling failed: {e}")
    
    async def _handle_cancel_request(self, raw_data: bytes):
        """Handle cancel request."""
        try:
            assert self._redis is not None, "Redis client not initialized"
            cancel_msg = json.loads(raw_data)
            goal_id = cancel_msg.get('goal_id')
            response_channel = cancel_msg.get('response_channel')
            
            if goal_id in self._active_goals:
                # TODO: Create goal_handle object for callback
                if self.handle_cancel_request:
                    canceled = await self.handle_cancel_request(goal_id)
                else:
                    canceled = True
                
                if canceled:
                    # Cancel execution task
                    task = self._active_goals[goal_id]
                    task.cancel()
                    
                    # Update state
                    await self._set_goal_state(goal_id, GoalStatus.CANCELED)
                    
                    # Send response
                    if response_channel:
                        await self._redis.publish_message(response_channel, json.dumps({
                            'goal_id': goal_id,
                            'canceled': True
                        }))
            else:
                if response_channel:
                    await self._redis.publish_message(response_channel, json.dumps({
                        'goal_id': goal_id,
                        'canceled': False,
                        'error': 'Goal not found'
                    }))
                    
        except Exception as e:
            logger.error(f"❌ Cancel request handling failed: {e}")
    
    async def _execute_goal(self, goal_id: str, goal: Any):
        """Execute goal in background."""
        try:
            assert self._redis is not None, "Redis client not initialized"
            # Update state to executing
            await self._set_goal_state(goal_id, GoalStatus.EXECUTING)
            
            # TODO: Create goal_handle with feedback publisher
            # goal_handle = GoalHandle(goal_id, self.publish_feedback)
            
            # Call execution callback
            if not self.execution_callback:
                raise InterfaceError("No execution callback defined")
            result = await self.execution_callback(goal)
            
            # Store result
            await self._redis.set(
                f"action:{self._action_name}:{goal_id}:result",
                json.dumps(result if isinstance(result, dict) else {'result': str(result)}),
                ex=3600  # Expire after 1 hour
            )
            
            # Update state to succeeded
            await self._set_goal_state(goal_id, GoalStatus.SUCCEEDED)
            
            # Notify via updates channel
            await self._redis.publish_message(
                f"action:{self._action_name}:{goal_id}:updates",
                json.dumps({'type': 'result', 'goal_id': goal_id})
            )
            
        except asyncio.CancelledError:
            await self._set_goal_state(goal_id, GoalStatus.CANCELED)
            raise
        except Exception as e:
            logger.error(f"❌ Goal execution failed: {e}")
            await self._set_goal_state(goal_id, GoalStatus.ABORTED)
        finally:
            self._active_goals.pop(goal_id, None)
    
    async def _set_goal_state(self, goal_id: str, status: GoalStatus):
        """Update goal state in Redis."""
        assert self._redis is not None, "Redis client not initialized"
        import time
        await self._redis.set(
            f"action:{self._action_name}:{goal_id}:state",
            json.dumps({'status': status.name, 'timestamp': time.time()}),
            ex=3600
        )
    
    async def publish_feedback(self, goal_id: str, feedback: Any):
        """Publish feedback for active goal."""
        try:
            assert self._redis is not None, "Redis client not initialized"
            # Store latest feedback
            await self._redis.set(
                f"action:{self._action_name}:{goal_id}:feedback",
                json.dumps(feedback if isinstance(feedback, dict) else {'feedback': str(feedback)}),
                ex=3600
            )
            
            # Notify via updates channel
            await self._redis.publish_message(
                f"action:{self._action_name}:{goal_id}:updates",
                json.dumps({'type': 'feedback', 'goal_id': goal_id})
            )
            
        except Exception as e:
            logger.error(f"❌ Feedback publish failed: {e}")
    
    async def cleanup(self):
        """Cleanup Redis resources."""
        # Cancel all active goals
        for task in self._active_goals.values():
            task.cancel()
        self._active_goals.clear()
    
    async def shutdown(self) -> None:
        """Shutdown interface."""
        await self.cleanup()
        self._initialized = False
    