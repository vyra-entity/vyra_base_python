"""
ROS2 Action Server Implementation

Async-first action server for ROS2 with goal/cancel/execution callbacks.
"""
import asyncio
import logging
from typing import Any, Callable, Optional

from vyra_base.com.core.types import VyraActionServer, ProtocolType
from vyra_base.com.core.topic_builder import TopicBuilder
from vyra_base.com.core.exceptions import InterfaceError
from vyra_base.com.transport.t_ros2.communication.action_server import (
    ROS2ActionServer as ROS2ActionServer,
    ActionServerInfo
)

logger = logging.getLogger(__name__)


class VyraActionServerImpl(VyraActionServer):
    """
    Vyra Action Server implementation.
    
    Wraps Vyra action server with async callback support for goal handling.
    """
    
    def __init__(
        self,
        name: str,
        topic_builder: TopicBuilder,
        node: Any,
        action_type: Any,
        handle_goal_request: Optional[Callable] = None,
        handle_cancel_request: Optional[Callable] = None,
        execution_callback: Optional[Callable] = None,
        **kwargs
    ):
        super().__init__(
            name, topic_builder,
            handle_goal_request, handle_cancel_request, execution_callback,
            ProtocolType.ROS2, **kwargs
        )
        self.node = node
        self.action_type = action_type
        self._ros2_action_server: Optional[ROS2ActionServer] = None
        
    async def initialize(self) -> bool:
        """Initialize ROS2 action server."""
        try:
            action_name = self.topic_builder.build(self.name, namespace=self.namespace, subsection=self.subsection)
            
            # Wrap async callbacks for ROS2 sync context
            def goal_callback(goal_request):
                """Sync wrapper for handle_goal_request."""
                try:
                    if self.handle_goal_request:
                        loop = asyncio.get_event_loop()
                        if asyncio.iscoroutinefunction(self.handle_goal_request):
                            accepted = loop.run_until_complete(self.handle_goal_request(goal_request))
                        else:
                            accepted = self.handle_goal_request(goal_request)
                        return accepted
                    return True  # Accept by default
                except Exception as e:
                    logger.error(f"❌ Goal callback error: {e}")
                    return False
            
            def cancel_callback(goal_handle):
                """Sync wrapper for handle_cancel_request."""
                try:
                    if self.handle_cancel_request:
                        loop = asyncio.get_event_loop()
                        if asyncio.iscoroutinefunction(self.handle_cancel_request):
                            accepted = loop.run_until_complete(self.handle_cancel_request(goal_handle))
                        else:
                            accepted = self.handle_cancel_request(goal_handle)
                        return accepted
                    return True  # Accept cancel by default
                except Exception as e:
                    logger.error(f"❌ Cancel callback error: {e}")
                    return False
            
            def execute_callback(goal_handle):
                """Sync wrapper for execution_callback."""
                try:
                    if self.execution_callback:
                        loop = asyncio.get_event_loop()
                        if asyncio.iscoroutinefunction(self.execution_callback):
                            result = loop.run_until_complete(self.execution_callback(goal_handle))
                        else:
                            result = self.execution_callback(goal_handle)
                        return result
                    # Return empty result
                    result_msg = self.action_type.Result()
                    goal_handle.succeed()
                    return result_msg
                except Exception as e:
                    logger.error(f"❌ Execution callback error: {e}")
                    goal_handle.abort()
                    return self.action_type.Result()
            
            # Create action server via ROS2ActionServer
            action_info = ActionServerInfo(
                name=action_name,
                type=self.action_type,
                goal_callback=goal_callback,
                cancel_callback=cancel_callback,
                execute_callback=execute_callback
            )
            
            self._ros2_action_server = ROS2ActionServer(
                actionInfo=action_info,
                node=self.node
            )
            self._ros2_action_server.create_action_server()
            
            self._transport_handle = self._ros2_action_server
            self._initialized = True
            
            logger.info(f"✅ ROS2 ActionServer '{self.name}' initialized on '{action_name}'")
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to initialize ROS2 ActionServer '{self.name}': {e}")
            raise InterfaceError(f"ActionServer initialization failed: {e}")
    
    async def shutdown(self) -> None:
        """Shutdown ROS2 action server."""
        if self._ros2_action_server:
            self._ros2_action_server.destroy()
            self._ros2_action_server = None
        self._initialized = False
        self._transport_handle = None
        
    async def start(self) -> None:
        """
        Start action server (already active after initialize for ROS2).
        """
        if not self._initialized:
            raise InterfaceError(f"ActionServer '{self.name}' not initialized")
        
        logger.debug(f"ActionServer '{self.name}' is active")
