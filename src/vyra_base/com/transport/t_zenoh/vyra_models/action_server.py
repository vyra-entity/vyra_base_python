"""
Zenoh Action Server Implementation

Hybrid pattern: Queryable for goal/cancel requests + Publisher for feedback.
"""

import logging
import zenoh

from typing import Coroutine, Optional, Any, Callable, Awaitable, Dict

from vyra_base.com.core.types import VyraActionServer, ProtocolType, GoalHandle
from vyra_base.com.core.topic_builder import TopicBuilder
from vyra_base.com.core.exceptions import InterfaceError
from vyra_base.com.transport.t_zenoh.communication.serializer import (
    ZenohSerializer,
    SerializationFormat
)
import uuid
import asyncio

logger = logging.getLogger(__name__)

class VyraActionServerImpl(VyraActionServer):
    """
    Vyra-based action server implementation.
    
    Architecture:
    - Queryable at ``{namespace}/{fn}/goal`` for goal requests
    - Queryable at ``{namespace}/{fn}/cancel`` for cancel requests
    - Publisher at ``{namespace}/{fn}/{goal_id}/feedback`` for feedback
    - Publisher at ``{namespace}/{fn}/{goal_id}/result`` for results
    
    Channel keys are built by :py:meth:`_action_channel` which stacks the
    protocol-level sub-channel on top of the config-level ``namespace`` /
    ``subsection`` following the VYRA topic convention.
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
        self._fb_publishers: Dict[str, Any] = {}       # goal_id -> zenoh.Publisher
        self._serializer = ZenohSerializer()
        self._format = SerializationFormat.JSON
        
    async def initialize(self) -> bool:
        """Initialize Zenoh action server."""
        try:
            self._action_name = self.topic_builder.build(self.name, namespace=self.namespace, subsection=self.subsection)
            self._key_goal = self._action_channel("goal")
            self._key_cancel = self._action_channel("cancel")
            
            if not self._zenoh_session:
                raise InterfaceError("Zenoh session not initialized")
            
            # Create queryable for goal requests
            loop = asyncio.get_event_loop()
            
            def _create_goal_queryable():
                if not self._zenoh_session:
                    raise InterfaceError("Zenoh session not initialized")
            
                return self._zenoh_session.declare_queryable(
                    self._key_goal,
                    lambda query: self._handle_goal_request_sync(query)
                )
            
            self._goal_queryable = await loop.run_in_executor(None, _create_goal_queryable)
            
            # Create queryable for cancel requests
            def _create_cancel_queryable():
                if not self._zenoh_session:
                    raise InterfaceError("Zenoh session not initialized")
            
                return self._zenoh_session.declare_queryable(
                    self._key_cancel,
                    lambda query: self._handle_cancel_request_sync(query)
                )
            
            self._cancel_queryable = await loop.run_in_executor(None, _create_cancel_queryable)
            
            logger.info(f"✅ ZenohActionServer initialized: {self._action_name}")
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
                query.reply(self._key_goal, response_bytes)
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
                query.reply(self._key_goal, response_bytes)
            else:
                # Reply with rejection
                response = {"accepted": False}
                response_bytes = self._serializer.serialize(response, format=self._format)
                query.reply(self._key_goal, response_bytes)
        except Exception as e:
            logger.error(f"❌ Goal handling failed: {e}")
            error_response = {"accepted": False, "error": str(e)}
            error_bytes = self._serializer.serialize(error_response, format=self._format)
            query.reply(self._key_goal, error_bytes)
    
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

                # Signal the executing coroutine that cancel was requested
                goal_handle.request_cancel()

                if self.handle_cancel_request is None:
                    logger.warning("No cancel handler defined, accepting cancel by default")
                    goal_handle.canceled()
                    response = {'canceled': True}
                    response_bytes = self._serializer.serialize(response, format=self._format)
                    query.reply(self._key_cancel, response_bytes)
                    return
                
                canceled = await self.handle_cancel_request(goal_handle)
                
                if canceled:
                    goal_handle.canceled()
                    response = {'canceled': True}
                else:
                    response = {'canceled': False}
            else:
                response = {'canceled': False, 'error': 'Goal not found'}
            
            # Reply
            response_bytes = self._serializer.serialize(response, format=self._format)
            query.reply(self._key_cancel, response_bytes)
            
        except Exception as e:
            logger.error(f"❌ Cancel handling failed: {e}")
            error_response = {'canceled': False, 'error': str(e)}
            error_bytes = self._serializer.serialize(error_response, format=self._format)
            query.reply(self._key_cancel, error_bytes)
    
    async def _execute_goal(self, goal_id: str, goal: Any):
        """
        Execute goal in background task.
        """
        try:
            # Create shared GoalHandle with feedback wired to this server
            goal_handle = GoalHandle(
                goal_id=goal_id,
                goal=goal,
                feedback_fn=self.publish_feedback
            )
            self._active_goals[goal_id] = goal_handle
            
            # Create zenoh publisher for feedback
            loop = asyncio.get_event_loop()
            
            def _create_feedback_pub():
                if not self._zenoh_session:
                    raise InterfaceError("Zenoh session not initialized")
                
                return self._zenoh_session.declare_publisher(
                    self._action_channel(f"{goal_id}/feedback")
                )
            
            self._fb_publishers[goal_id] = await loop.run_in_executor(None, _create_feedback_pub)
            
            if not self.execution_callback:
                raise InterfaceError("No execution callback defined for action server")
            
            # Execute goal
            result = await self.execution_callback(goal_handle)
            
            # Publish result
            goal_handle.set_succeeded(result)
            result_bytes = self._serializer.serialize(result, format=self._format)
            
            
            
            def _create_result_pub():
                if not self._zenoh_session:
                    raise InterfaceError("Zenoh session not initialized")
            
                pub = self._zenoh_session.declare_publisher(  # type: ignore[union-attr]  # type: ignore[union-attr]
                    self._action_channel(f"{goal_id}/result")
                )
                pub.put(result_bytes)
                pub.undeclare()
            
            await loop.run_in_executor(None, _create_result_pub)
            
            # Cleanup feedback publisher
            if goal_id in self._fb_publishers:
                fb_publisher = self._fb_publishers.pop(goal_id)
                await loop.run_in_executor(None, fb_publisher.undeclare)  # pyright: ignore[reportGeneralTypeIssues]
            
            del self._active_goals[goal_id]
            
        except Exception as e:
            logger.error(f"❌ Goal execution failed: {e}")
            if goal_id in self._active_goals:
                self._active_goals[goal_id].set_aborted(str(e))
    
    async def publish_feedback(self, goal_id: str, feedback: Any):
        """
        Publish feedback for active goal.
        """
        try:
            if goal_id not in self._active_goals:
                logger.warning(f"Goal {goal_id} not active")
                return
            
            publisher = self._fb_publishers.get(goal_id)
            
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
            goal_handle.set_canceled()
            fb_pub = self._fb_publishers.pop(goal_id, None)
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
