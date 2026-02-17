"""
ROS2 Action Client Implementation

Async-first action client for ROS2 with response/feedback/goal callbacks.
"""
import asyncio
import logging
from typing import _T, Any, Callable, Optional

from grpc import Future

from vyra_base.com.core.types import VyraActionClient, ProtocolType
from vyra_base.com.core.topic_builder import TopicBuilder
from vyra_base.com.core.exceptions import InterfaceError, TimeoutError
from vyra_base.com.transport.t_ros2.communication.action_client import (
    ROS2ActionClient as ROS2ActionClient,
    ActionClientInfo,
)

logger = logging.getLogger(__name__)


class VyraActionClientImpl(VyraActionClient):
    """
    Vyra Action Client implementation.
    
    Wraps Vyra action client with async callback support.
    """
    
    def __init__(
        self,
        name: str,
        topic_builder: TopicBuilder,
        node: Any,
        action_type: Any,
        direct_response_callback: Optional[Callable] = None,
        feedback_callback: Optional[Callable] = None,
        goal_callback: Optional[Callable] = None,
        **kwargs
    ):
        super().__init__(
            name, topic_builder,
            direct_response_callback, feedback_callback, goal_callback,
            ProtocolType.ROS2, **kwargs
        )
        self.node = node
        self.action_type = action_type
        self._ros2_action_client: Optional[ROS2ActionClient] = None
        
    async def initialize(self) -> bool:
        """Initialize ROS2 action client."""
        try:
            action_name = self.topic_builder.build(self.name)
            
            # Wrap async callbacks
            def feedback_cb(feedback_msg):
                if self.feedback_callback:
                    try:
                        loop = asyncio.get_event_loop()
                        if asyncio.iscoroutinefunction(self.feedback_callback):
                            loop.run_until_complete(self.feedback_callback(feedback_msg.feedback))
                        else:
                            self.feedback_callback(feedback_msg.feedback)
                    except Exception as e:
                        logger.error(f"❌ Feedback callback error: {e}")
            
            # Create action client via ROS2ActionClient
            action_info = ActionClientInfo(
                name=action_name,
                type=self.action_type,
                result_callback=self.direct_response_callback,
                feedback_callback=feedback_cb if self.feedback_callback else None
            )
            
            self._ros2_action_client = ROS2ActionClient(
                actionInfo=action_info,
                node=self.node
            )
            self._ros2_action_client.create_action_client()
            
            self._transport_handle = self._ros2_action_client
            self._initialized = True
            
            logger.info(f"✅ ROS2 ActionClient '{self.name}' initialized for '{action_name}'")
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to initialize ROS2 ActionClient '{self.name}': {e}")
            raise InterfaceError(f"ActionClient initialization failed: {e}")
    
    async def shutdown(self) -> None:
        """Shutdown ROS2 action client."""
        if self._ros2_action_client:
            self._ros2_action_client.destroy()
            self._ros2_action_client = None
        self._initialized = False
        self._transport_handle = None
        
    async def send_goal(self, goal: Any, **kwargs) -> Any:
        """
        Send goal to action server (async).
        
        Args:
            goal: Goal message
            **kwargs: Additional parameters (timeout, etc.)
            
        Returns:
            Result message
            
        Raises:
            InterfaceError: If not initialized or send fails
            TimeoutError: If goal times out
        """
        if not self._initialized or not self._ros2_action_client:
            raise InterfaceError(f"ActionClient '{self.name}' not initialized")
        
        timeout = kwargs.get('timeout', 10.0)
        
        try:
            loop = asyncio.get_event_loop()
            
            # Send goal async
            goal_coro = self._ros2_action_client.send_goal_async(goal)
            
            # Wait for acceptance
            goal_handle = await asyncio.wait_for(
                goal_coro,
                timeout=timeout
            )
            
            if not goal_handle.accepted:
                if self.direct_response_callback:
                    if asyncio.iscoroutinefunction(self.direct_response_callback):
                        await self.direct_response_callback(False)
                    else:
                        self.direct_response_callback(False)
                raise InterfaceError(f"Goal rejected by action server '{self.name}'")
            
            # Goal accepted callback
            if self.direct_response_callback:
                if asyncio.iscoroutinefunction(self.direct_response_callback):
                    await self.direct_response_callback(True)
                else:
                    self.direct_response_callback(True)
            
            # Wait for result
            result_future = await loop.run_in_executor(
                None,
                goal_handle.get_result_async
            )
            
            result = await asyncio.wait_for(
                asyncio.wrap_future(result_future),
                timeout=timeout
            )
            
            # Goal completed callback
            if self.goal_callback:
                if asyncio.iscoroutinefunction(self.goal_callback):
                    await self.goal_callback(result.result)
                else:
                    self.goal_callback(result.result)
            
            return result.result
            
        except asyncio.TimeoutError:
            raise TimeoutError(
                operation="send_goal",
                timeout=timeout,
                details={"action_name": self.name}
            )
        except Exception as e:
            logger.error(f"❌ Failed to send goal to '{self.name}': {e}")
            raise InterfaceError(f"Send goal failed: {e}")
    
    async def cancel_goal(self, goal_handle: Any) -> bool:
        """
        Cancel a running goal.
        
        Args:
            goal_handle: Handle returned from send_goal
            
        Returns:
            bool: True if cancellation accepted
        """
        if not self._initialized or not self._ros2_action_client:
            raise InterfaceError(f"ActionClient '{self.name}' not initialized")
        
        try:
            loop = asyncio.get_event_loop()
            cancel_future = await loop.run_in_executor(
                None,
                goal_handle.cancel_goal_async
            )
            
            cancel_result = await asyncio.wrap_future(cancel_future)
            return cancel_result.goals_canceling is not None
            
        except Exception as e:
            logger.error(f"❌ Failed to cancel goal on '{self.name}': {e}")
            return False
