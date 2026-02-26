"""
Zenoh Action Client Implementation

Hybrid pattern: Query for goal send + Subscriber for feedback/result.
"""

import logging
from typing import Coroutine, Optional, Any, Callable, Awaitable

from vyra_base.com.core.types import VyraActionClient, ProtocolType
from vyra_base.com.core.topic_builder import TopicBuilder
from vyra_base.com.core.exceptions import InterfaceError
from vyra_base.com.transport.t_zenoh.communication.serializer import (
    ZenohSerializer,
    SerializationFormat
)
import uuid
import asyncio

logger = logging.getLogger(__name__)


class VyraActionClientImpl(VyraActionClient):
    """
    Vyra-based action client implementation.
    
    Architecture:
    - Query to ``{namespace}/{fn}/goal`` for sending goals
    - Subscriber to ``{namespace}/{fn}/{goal_id}/feedback`` for feedback
    - Subscriber to ``{namespace}/{fn}/{goal_id}/result`` for result
    - Query to ``{namespace}/{fn}/cancel`` for canceling
    
    Channel keys are built by :py:meth:`_action_channel` following the VYRA
    topic naming convention.
    """
    
    def __init__(
        self,
        name: str,
        topic_builder: TopicBuilder,
        direct_response: Optional[Callable[[Any], Coroutine[Any, Any, Any]]] = None,
        feedback_callback: Optional[Callable[[Any], Coroutine[Any, Any, Any]]] = None,
        goal_response_callback: Optional[Callable[[Any], Coroutine[Any, Any, Any]]] = None,
        zenoh_session: Any = None,  # zenoh.Session
        action_type: type = None,
        **kwargs
    ):
        super().__init__(
            name, topic_builder,
            direct_response, feedback_callback, goal_response_callback,
            ProtocolType.ZENOH, **kwargs
        )
        self._zenoh_session = zenoh_session
        self.action_type = action_type
        self._active_goal_id: Optional[str] = None
        self._feedback_subscriber = None
        self._result_subscriber = None
        self._serializer = ZenohSerializer()
        self._format = SerializationFormat.JSON
        self._loop: Optional[asyncio.AbstractEventLoop] = None  # captured in initialize()
        
    async def initialize(self) -> bool:
        """Initialize Zenoh action client."""
        try:
            action_name = self.topic_builder.build(self.name, namespace=self.namespace, subsection=self.subsection)
            self._action_name = action_name
            self._key_goal = self._action_channel("goal")
            self._key_cancel = self._action_channel("cancel")
            # Capture running event loop so sync Zenoh callbacks can schedule coroutines
            self._loop = asyncio.get_running_loop()
            # No persistent resources until goal is sent
            logger.info(f"✅ ZenohActionClient initialized: {action_name}")
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to initialize ZenohActionClient: {e}")
            return False
    
    async def send_goal(self, goal: Any) -> Optional[str]:
        """
        Send goal to action server.
        
        Args:
            goal: Goal message instance
            
        Returns:
            goal_id on success, None on failure
            
        TODO:
        1. Serialize goal
        2. Query to ``self._key_goal`` (built via _action_channel)
        3. Parse response for goal_id
        4. Subscribe to feedback and result topics
        5. Call goal_response_callback if set
        """
        try:
            if not self._zenoh_session:
                raise InterfaceError("Zenoh session not initialized")
            
            # Serialize goal
            goal_bytes = self._serializer.serialize(goal, format=self._format)
            
            # Query to action server
            loop = asyncio.get_event_loop()
            
            def _send_goal_query():
                if not self._zenoh_session:
                    raise InterfaceError("Zenoh session not initialized")
                replies = self._zenoh_session.get(  # type: ignore[union-attr]
                    self._key_goal,
                    value=goal_bytes
                )
                for reply in replies:
                    return reply.ok.payload
                return None
            
            payload = await loop.run_in_executor(None, _send_goal_query)
            
            if payload is None:
                logger.error("No response from action server")
                return None
            
            # Parse response
            response = self._serializer.deserialize(payload, format=self._format)
            
            if response.get("accepted"):
                goal_id = response.get("goal_id")
                self._active_goal_id = goal_id
                
                # Subscribe to feedback and result
                await self._subscribe_to_goal(goal_id)
                
                if self.goal_callback:
                    await self.goal_callback(response)  # type: ignore[misc]
                
                return goal_id
            else:
                logger.warning("Goal rejected by server")
                return None
            
        except Exception as e:
            logger.error(f"❌ Send goal failed: {e}")
            return None
    
    async def _subscribe_to_goal(self, goal_id: str):
        """
        Subscribe to feedback and result for specific goal.
        """
        try:
            from vyra_base.com.transport.t_zenoh.communication.subscriber import (
                ZenohSubscriber,
                SubscriberInfo
            )
            
            loop = asyncio.get_event_loop()
            
            # Subscribe to feedback
            def _create_feedback_sub():
                if not self._zenoh_session:
                    raise InterfaceError("Zenoh session not initialized")
                
                info = SubscriberInfo(
                    key_expr=self._action_channel(f"{goal_id}/feedback"),
                    format=self._format
                )
                sub = self._zenoh_session.declare_subscriber(  # type: ignore[union-attr]
                    info.key_expr,
                    lambda sample: self._handle_feedback_sync(sample)
                )
                return sub
            
            self._feedback_subscriber = await loop.run_in_executor(None, _create_feedback_sub)
            
            # Subscribe to result
            def _create_result_sub():
                if not self._zenoh_session:
                    raise InterfaceError("Zenoh session not initialized")
                
                info = SubscriberInfo(
                    key_expr=self._action_channel(f"{goal_id}/result"),
                    format=self._format
                )
                sub = self._zenoh_session.declare_subscriber(  # type: ignore[union-attr]
                    info.key_expr,
                    lambda sample: self._handle_result_sync(sample)
                )
                return sub
            
            self._result_subscriber = await loop.run_in_executor(None, _create_result_sub)
            
        except Exception as e:
            logger.error(f"❌ Failed to subscribe to goal updates: {e}")
    
    def _handle_feedback_sync(self, sample: Any):
        """Handle feedback (sync callback for Zenoh, runs in a Zenoh background thread)."""
        try:
            feedback = self._serializer.deserialize(sample.payload, format=self._format)
            if self.feedback_callback and self._loop:
                # run_coroutine_threadsafe is safe to call from any thread
                asyncio.run_coroutine_threadsafe(
                    self.feedback_callback(feedback), self._loop
                )
        except Exception as e:
            logger.error(f"❌ Feedback handling failed: {e}")
    
    def _handle_result_sync(self, sample: Any):
        """Handle result (sync callback for Zenoh, runs in a Zenoh background thread)."""
        try:
            result = self._serializer.deserialize(sample.payload, format=self._format)
            if self.direct_response_callback and self._loop:
                asyncio.run_coroutine_threadsafe(
                    self.direct_response_callback(result), self._loop
                )
        except Exception as e:
            logger.error(f"❌ Result handling failed: {e}")
    
    async def cancel_goal(self, goal_id: str) -> bool:
        """
        Cancel active goal.
        """
        try:
            cancel_request = {'goal_id': goal_id}
            cancel_bytes = self._serializer.serialize(cancel_request, format=self._format)
            
            loop = asyncio.get_event_loop()
            
            def _send_cancel_query():
                if not self._zenoh_session:
                    raise InterfaceError("Zenoh session not initialized")
                replies = self._zenoh_session.get(  # type: ignore[union-attr]
                    self._key_cancel,
                    value=cancel_bytes
                )
                for reply in replies:
                    return reply.ok.payload
                return None
            
            payload = await loop.run_in_executor(None, _send_cancel_query)
            
            if payload is None:
                return False
            
            response = self._serializer.deserialize(payload, format=self._format)
            return response.get('canceled', False)
            
        except Exception as e:
            logger.error(f"❌ Cancel goal failed: {e}")
            return False
    
    async def cleanup(self):
        """Cleanup Zenoh resources."""
        loop = asyncio.get_event_loop()
        
        if self._feedback_subscriber:
            await loop.run_in_executor(None, self._feedback_subscriber.undeclare)
            self._feedback_subscriber = None
        
        if self._result_subscriber:
            await loop.run_in_executor(None, self._result_subscriber.undeclare)
            self._result_subscriber = None
        
        logger.info(f"🔄 ZenohActionClient cleaned up: {self.name}")
    
    async def shutdown(self) -> None:
        """Shutdown action client."""
        await self.cleanup()
        self._initialized = False
