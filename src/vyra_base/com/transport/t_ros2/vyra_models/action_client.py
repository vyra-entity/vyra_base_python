"""
ROS2 Action Client Implementation

Async-first action client for ROS2 with response/feedback/goal callbacks.
"""
import asyncio
import logging
from typing import Any, Callable, Optional

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
            action_name = self.topic_builder.build(self.name, namespace=self.namespace, subsection=self.subsection)
            
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

        The application's background spinner (ros_spinner_runner) calls
        rclpy.spin_once every ~6 ms and owns the executor — we MUST NOT call
        rclpy.spin_once here too, because concurrent spin_once calls raise
        'Executor is already spinning'. Instead we simply poll the rclpy
        Futures with asyncio.sleep() and let the background spinner resolve them.

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

        timeout = kwargs.get('timeout', 30.0)

        try:
            loop = asyncio.get_event_loop()

            # Convert dict to typed Goal if needed
            if isinstance(goal, dict) and self.action_type is not None:
                goal_obj = self.action_type.Goal()
                for key, value in goal.items():
                    if hasattr(goal_obj, key):
                        setattr(goal_obj, key, value)
                goal = goal_obj

            # Wait for server discovery (does not spin — uses DDS discovery)
            server_ready = await loop.run_in_executor(
                None,
                lambda: self._ros2_action_client._action_info.client.wait_for_server(timeout_sec=5.0)
            )
            if not server_ready:
                raise InterfaceError(f"Action server '{self.name}' not available")

            # Send goal — returns rclpy Future immediately
            send_goal_future = self._ros2_action_client._action_info.client.send_goal_async(goal)

            logger.debug(f"🎯 send_goal_async issued for '{self.name}', polling for acceptance...")
            deadline = loop.time() + timeout
            while not send_goal_future.done():
                if loop.time() > deadline:
                    raise TimeoutError(
                        operation="send_goal",
                        timeout=timeout,
                        details={"action_name": self.name}
                    )
                await asyncio.sleep(0.01)  # yield to background spinner

            goal_handle = send_goal_future.result()

            if not goal_handle.accepted:
                raise InterfaceError(f"Goal rejected by action server '{self.name}'")

            if self.goal_callback:
                if asyncio.iscoroutinefunction(self.goal_callback):
                    await self.goal_callback(True)
                else:
                    self.goal_callback(True)

            # Request the execution result
            result_future = goal_handle.get_result_async()
            logger.debug(f"🎯 get_result_async issued for '{self.name}', polling for result...")

            deadline = loop.time() + timeout
            while not result_future.done():
                if loop.time() > deadline:
                    raise TimeoutError(
                        operation="get_result",
                        timeout=timeout,
                        details={"action_name": self.name}
                    )
                await asyncio.sleep(0.01)

            wrapped_result = result_future.result()
            logger.debug(f"✅ Result received for '{self.name}'")

            # Convert ROS2 result message to dict for JSON serialization
            ros2_result = wrapped_result.result
            result_dict = {}
            try:
                for field_name in ros2_result.get_fields_and_field_types().keys():
                    result_dict[field_name] = getattr(ros2_result, field_name)
            except Exception:
                result_dict = {"raw": str(ros2_result)}

            # Goal completed callback
            if self.direct_response_callback:
                if asyncio.iscoroutinefunction(self.direct_response_callback):
                    await self.direct_response_callback(result_dict)
                else:
                    self.direct_response_callback(result_dict)

            return result_dict

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
