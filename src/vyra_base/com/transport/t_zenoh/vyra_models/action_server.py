"""
Zenoh Action Server Implementation

Hybrid pattern: Queryable for goal/cancel requests + Publisher for feedback.
"""

import logging
import zenoh

from typing import Coroutine, Optional, Any, Callable, Awaitable, Dict, TypedDict
from enum import Enum

from vyra_base.com.core.types import VyraActionServer, ProtocolType
from vyra_base.com.core.topic_builder import TopicBuilder
from vyra_base.com.core.exceptions import InterfaceError
from vyra_base.com.transport.t_zenoh.communication.serializer import (
    ZenohSerializer,
    SerializationFormat
)
import uuid
import asyncio

logger = logging.getLogger(__name__)


class GoalStatus(Enum):
    """Action goal execution status."""
    UNKNOWN = 0
    ACCEPTED = 1
    EXECUTING = 2
    SUCCEEDED = 3
    ABORTED = 4
    CANCELED = 5


class GoalHandle(TypedDict, total=False):
    """Type definition for goal handle dictionary."""
    goal_id: str
    goal: Any
    status: GoalStatus
    feedback_publisher: Any  # zenoh.Publisher


class VyraActionServerImpl(VyraActionServer):
    """
    Vyra-based action server implementation.
    
    Architecture:
    - Queryable at "action/{name}/goal" for goal requests
    - Queryable at "action/{name}/cancel" for cancel requests  
    - Publisher at "action/{name}/{goal_id}/feedback" for feedback
    - Publisher at "action/{name}/{goal_id}/result" for results
    
    TODO: Implement using ZenohQueryable + ZenohPublisher combo.
    """
    
    def __init__(
        self,
        name: str,
        topic_builder: TopicBuilder,
        handle_goal_request: Callable[[Any], Coroutine[Any, Any, Any]],
        handle_cancel_request: Callable[[Any], Coroutine[Any, Any, bool]],
        execution_callback: Callable[[Any], Coroutine[Any, Any, Any]],
        zenoh_session: zenoh.Session = None,  # zenoh.Session
        action_type: type = None,
        **kwargs
    ):
        super().__init__(
            name, topic_builder,
            handle_goal_request, handle_cancel_request, execution_callback,
            ProtocolType.ZENOH, **kwargs
        )
        self._zenoh_session = zenoh_session
        self.action_type = action_type
        self._goal_queryable = None
        self._cancel_queryable = None
        self._active_goals: Dict[str, GoalHandle] = {}  # goal_id -> GoalHandle
        self._serializer = ZenohSerializer()
        self._format = SerializationFormat.JSON
        
    async def initialize(self) -> bool:
        """Initialize Zenoh action server."""
        try:
            action_name = self.topic_builder.build(self.name)
            self._action_name = action_name
            
            if not self._zenoh_session:
                raise InterfaceError("Zenoh session not initialized")
            
            # Create queryable for goal requests
            loop = asyncio.get_event_loop()
            
            def _create_goal_queryable():
                if not self._zenoh_session:
                    raise InterfaceError("Zenoh session not initialized")
            
                return self._zenoh_session.declare_queryable(
                    f"action/{action_name}/goal",
                    lambda query: self._handle_goal_request_sync(query)
                )
            
            self._goal_queryable = await loop.run_in_executor(None, _create_goal_queryable)
            
            # Create queryable for cancel requests
            def _create_cancel_queryable():
                if not self._zenoh_session:
                    raise InterfaceError("Zenoh session not initialized")
            
                return self._zenoh_session.declare_queryable(
                    f"action/{action_name}/cancel",
                    lambda query: self._handle_cancel_request_sync(query)
                )
            
            self._cancel_queryable = await loop.run_in_executor(None, _create_cancel_queryable)
            
            logger.info(f"✅ ZenohActionServer initialized: {action_name}")
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to initialize ZenohActionServer: {e}")
            return False
    
    def _handle_goal_request_sync(self, query: Any):
        """Sync wrapper for async goal handler."""
        try:
            loop = asyncio.get_event_loop()
        except RuntimeError:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
        
        loop.run_until_complete(self._handle_goal_request(query))
    
    async def _handle_goal_request(self, query: Any):
        """
        Handle incoming goal request.
        """
        try:            
            # Deserialize goal
            goal = self._serializer.deserialize(query.value.payload, format=self._format)
            
            if self.handle_goal_request is None:
                logger.warning("No goal handler defined, rejecting goal")
                response = {"accepted": False, "error": "No goal handler defined"}
                response_bytes = self._serializer.serialize(response, format=self._format)
                query.reply(f"action/{self._action_name}/goal", response_bytes)
                return

            # Call handle_goal callback
            accepted = await self.handle_goal_request(goal)
            
            if accepted:
                # Generate goal ID
                goal_id = str(uuid.uuid4())
                
                # Start execution task
                asyncio.create_task(self._execute_goal(goal_id, goal))
                
                # Reply with acceptance
                response = {"goal_id": goal_id, "accepted": True}
                response_bytes = self._serializer.serialize(response, format=self._format)
                query.reply(f"action/{self._action_name}/goal", response_bytes)
            else:
                # Reply with rejection
                response = {"accepted": False}
                response_bytes = self._serializer.serialize(response, format=self._format)
                query.reply(f"action/{self._action_name}/goal", response_bytes)
        except Exception as e:
            logger.error(f"❌ Goal handling failed: {e}")
            error_response = {"accepted": False, "error": str(e)}
            error_bytes = self._serializer.serialize(error_response, format=self._format)
            query.reply(f"action/{self._action_name}/goal", error_bytes)
    
    def _handle_cancel_request_sync(self, query: Any):
        """Sync wrapper for async cancel handler."""
        try:
            loop = asyncio.get_event_loop()
        except RuntimeError:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
        
        loop.run_until_complete(self._handle_cancel_request(query))
    
    async def _handle_cancel_request(self, query: Any):
        """
        Handle incoming cancel request.
        """
        try:
            # Extract goal_id from request
            request = self._serializer.deserialize(query.value.payload, format=self._format)
            goal_id = request.get('goal_id')
            
            if goal_id in self._active_goals:
                # Call cancel callback
                goal_handle = self._active_goals[goal_id]

                if self.handle_cancel_request is None:
                    logger.warning("No cancel handler defined, rejecting cancel request")
                    response = {'canceled': False, 'error': "No cancel handler defined"}
                    response_bytes = self._serializer.serialize(response, format=self._format)
                    query.reply(f"action/{self._action_name}/cancel", response_bytes)
                    return
                
                canceled = await self.handle_cancel_request(goal_handle)
                
                if canceled:
                    goal_handle['status'] = GoalStatus.CANCELED
                    response = {'canceled': True}
                else:
                    response = {'canceled': False}
            else:
                response = {'canceled': False, 'error': 'Goal not found'}
            
            # Reply
            response_bytes = self._serializer.serialize(response, format=self._format)
            query.reply(f"action/{self._action_name}/cancel", response_bytes)
            
        except Exception as e:
            logger.error(f"❌ Cancel handling failed: {e}")
            error_response = {'canceled': False, 'error': str(e)}
            error_bytes = self._serializer.serialize(error_response, format=self._format)
            query.reply(f"action/{self._action_name}/cancel", error_bytes)
    
    async def _execute_goal(self, goal_id: str, goal: Any):
        """
        Execute goal in background task.
        """
        try:
            # Create goal handle
            goal_handle: GoalHandle = {
                'goal_id': goal_id,
                'goal': goal,
                'status': GoalStatus.EXECUTING,
                'feedback_publisher': None
            }
            self._active_goals[goal_id] = goal_handle
            
            # Create publisher for feedback
            loop = asyncio.get_event_loop()
            
            def _create_feedback_pub():
                if not self._zenoh_session:
                    raise InterfaceError("Zenoh session not initialized")
                
                return self._zenoh_session.declare_publisher(
                    f"action/{self._action_name}/{goal_id}/feedback"
                )
            
            goal_handle['feedback_publisher'] = await loop.run_in_executor(None, _create_feedback_pub)
            
            if not self.execution_callback:
                raise InterfaceError("No execution callback defined for action server")
            
            # Execute goal
            result = await self.execution_callback(goal_handle)
            
            # Publish result
            goal_handle['status'] = GoalStatus.SUCCEEDED
            result_bytes = self._serializer.serialize(result, format=self._format)
            
            
            
            def _create_result_pub():
                if not self._zenoh_session:
                    raise InterfaceError("Zenoh session not initialized")
            
                pub = self._zenoh_session.declare_publisher(  # type: ignore[union-attr]  # type: ignore[union-attr]
                    f"action/{self._action_name}/{goal_id}/result"
                )
                pub.put(result_bytes)
                pub.undeclare()
            
            await loop.run_in_executor(None, _create_result_pub)
            
            # Cleanup
            if goal_handle['feedback_publisher']:  # type: ignore[misc]
                fb_publisher = goal_handle['feedback_publisher']  # pyright: ignore[reportGeneralTypeIssues]
                await loop.run_in_executor(None, fb_publisher.undeclare)  # pyright: ignore[reportGeneralTypeIssues]
            
            del self._active_goals[goal_id]
            
        except Exception as e:
            logger.error(f"❌ Goal execution failed: {e}")
            if goal_id in self._active_goals:
                self._active_goals[goal_id]['status'] = GoalStatus.ABORTED
    
    async def publish_feedback(self, goal_id: str, feedback: Any):
        """
        Publish feedback for active goal.
        """
        try:
            if goal_id not in self._active_goals:
                logger.warning(f"Goal {goal_id} not active")
                return
            
            goal_handle = self._active_goals[goal_id]
            publisher = goal_handle.get('feedback_publisher')
            
            if publisher:
                feedback_bytes = self._serializer.serialize(feedback, format=self._format)
                loop = asyncio.get_event_loop()
                await loop.run_in_executor(None, publisher.put, feedback_bytes)
            
        except Exception as e:
            logger.error(f"❌ Feedback publish failed: {e}")
    
    async def cleanup(self):
        """Cleanup Zenoh resources."""
        loop = asyncio.get_event_loop()
        
        # Cancel all active goals
        for goal_id, goal_handle in list(self._active_goals.items()):
            goal_handle['status'] = GoalStatus.CANCELED  # type: ignore[misc]
            fb_pub = goal_handle.get('feedback_publisher')  # pyright: ignore[reportGeneralTypeIssues]
            if fb_pub:
                await loop.run_in_executor(None, fb_pub.undeclare)  # pyright: ignore[reportGeneralTypeIssues]
        
        self._active_goals.clear()
        
        # Cleanup queryables
        if self._goal_queryable:
            await loop.run_in_executor(None, self._goal_queryable.undeclare)
            self._goal_queryable = None
        
        if self._cancel_queryable:
            await loop.run_in_executor(None, self._cancel_queryable.undeclare)
            self._cancel_queryable = None
        
        logger.info(f"🔄 ZenohActionServer cleaned up: {self.name}")
    
    async def shutdown(self) -> None:
        """Shutdown action server."""
        await self.cleanup()
        self._initialized = False
