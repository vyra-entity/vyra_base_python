"""
ROS2 Job Implementation

Concrete implementation of VyraJob for ROS2 Action communication.
"""
import logging
from typing import Any, Callable, Optional

from vyra_base.com.core.types import VyraJob, ProtocolType
from vyra_base.com.core.exceptions import InterfaceError
from vyra_base.com.transport.t_ros2.node import VyraNode
from vyra_base.com.transport.t_ros2.communication.action_client import ActionClientInfo
from vyra_base.com.transport.t_ros2.communication.action_server import ActionServerInfo
from vyra_base.com.transport.t_ros2.communication import VyraActionServer, VyraActionClient

logger = logging.getLogger(__name__)


class ROS2Job(VyraJob):
    """
    ROS2-specific implementation of VyraJob using ROS2 Actions.
    
    Wraps VyraActionServer for server-side and VyraActionClient for client-side.
    """
    
    def __init__(
        self,
        name: str,
        result_callback: Optional[Callable] = None,
        feedback_callback: Optional[Callable] = None,
        node: Optional[VyraNode] = None,
        action_type: Optional[Any] = None,
        **kwargs
    ):
        super().__init__(name, result_callback, feedback_callback, ProtocolType.ROS2, **kwargs)
        self.node = node
        self.action_type = action_type
        self._action_server: Optional[VyraActionServer] = None
        self._action_client: Optional[VyraActionClient] = None
        self._last_result: Any = None
    
    async def initialize(self) -> bool:
        """
        Initialize ROS2 action.
        
        Creates either action_server (if callback provided) or action_client.
        """
        if self._initialized:
            logger.warning(f"ROS2Job '{self.name}' already initialized")
            return True
        
        if not self.node:
            raise InterfaceError("Node is required for ROS2Job")
        
        if not self.action_type:
            raise InterfaceError("action_type is required for ROS2Job")
        
        try:
            if self.result_callback and self.feedback_callback:
                # Server-side: create action server
                logger.info(f"🔧 Creating ROS2 action server: {self.name}")
                
                action_info = ActionServerInfo(
                    name=self.name,
                    type=self.action_type,
                    result_callback=self.result_callback,
                    feedback_callback=self.feedback_callback
                )

                self._action_server = VyraActionServer(
                    actionInfo=action_info,
                    node=self.node,
                )
                logger.info(f"✅ ROS2 action server created: {self.name}")
            else:
                # Client-side: create action client
                logger.info(f"🔧 Creating ROS2 action client: {self.name}")
                
                action_info = ActionClientInfo(
                    name=self.name,
                    type=self.action_type,
                    result_callback=self.result_callback,
                    feedback_callback=self.feedback_callback
                )
                
                self._action_client = VyraActionClient(
                    actionInfo=action_info,
                    node=self.node
                )
                logger.info(f"✅ ROS2 action client created: {self.name}")
            
            self._initialized = True
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to initialize ROS2Job '{self.name}': {e}")
            raise InterfaceError(f"Failed to initialize ROS2Job: {e}")
    
    async def shutdown(self) -> None:
        """Shutdown and cleanup ROS2 action resources."""
        if not self._initialized:
            return
        
        logger.info(f"🛑 Shutting down ROS2Job: {self.name}")
        
        # Cleanup action server/client
        if self._action_server:
            self._action_server.destroy()
            self._action_server = None
        
        if self._action_client:
            self._action_client.destroy()
            self._action_client = None
        
        self._initialized = False
        logger.info(f"✅ ROS2Job '{self.name}' shutdown complete")
    
    async def execute(
        self,
        goal: Any,
        feedback_callback: Optional[Callable] = None
    ) -> Any:
        """
        Execute the ROS2 action (send goal).
        
        Args:
            goal: Action goal data
            feedback_callback: Optional callback for progress feedback
            
        Returns:
            Action result
            
        Raises:
            InterfaceError: If not initialized or execution fails
        """
        if not self._initialized:
            raise InterfaceError(f"ROS2Job '{self.name}' not initialized")
        
        if not self._action_client:
            raise InterfaceError(
                f"Cannot execute ROS2Job '{self.name}': no action client. "
                "This is likely a server-side job."
            )
        
        try:
            logger.info(f"🚀 Executing ROS2 action: {self.name}")
            
            # Send goal via VyraActionClient
            goal_handle = await self._action_client.send_goal_async(
                goal=goal,
                feedback_callback=feedback_callback
            )
            
            if not goal_handle.accepted:
                raise InterfaceError(f"Action goal rejected for '{self.name}'")
            
            logger.debug(f"⏳ Waiting for ROS2 action result: {self.name}")
            
            # Wait for result
            result = await goal_handle.get_result_async()
            
            self._last_result = result
            logger.info(f"✅ ROS2 action completed: {self.name}")
            return result
            
        except Exception as e:
            logger.error(f"❌ ROS2 action execution failed '{self.name}': {e}")
            raise InterfaceError(f"Action execution failed: {e}")
    
    def get_action_server(self) -> Optional[VyraActionServer]:
        """Get the underlying ROS2 action server (server-side only)."""
        return self._action_server
    
    def get_action_client(self) -> Optional[VyraActionClient]:
        """Get the underlying ROS2 action client (client-side only)."""
        return self._action_client
