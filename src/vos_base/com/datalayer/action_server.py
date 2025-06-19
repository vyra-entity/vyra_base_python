from __future__ import annotations

from unittest import result
import rclpy
from dataclasses import dataclass
from rclpy.action import ActionServer
from typing import Any
from typing import NoReturn
from typing import Callable
from typing import Union

from vos_base.com.datalayer.node import VOSNode


def _base_callback(goal_handle) -> NoReturn:
    raise NotImplementedError("No execute callback provided for action server.")


@dataclass
class Action:
    name: str = 'vos_action_server'
    type: Any = None
    callback: Callable = _base_callback
    server: Union[ActionServer, None] = None


class VOSActionServer:
    """
    Base class for ROS2 action.
    This class will be factory created to implement specific action functionality.
    """

    def __init__(self, action: Action, node: VOSNode) -> None:
        self._action: Action = action
        self._node: VOSNode = node
    
    def create_action_server(self) -> None:
        """
        Create a service in the ROS2 node.
        This method should be called to register the service with the ROS2 node.
        """
        self._node.get_logger().info(f"Creating action: {self._action.name}")
        self._action.server = ActionServer(
            self._node,
            self._action.type,
            self._action.name,
            self.execute_callback
        )

    def execute_callback(self, goal_handle) -> None:
        """
        Add a callback to the service.
        This method should be overridden in subclasses to provide specific functionality.
        """
        self._node.get_logger().info(f"Executing action: {self._action.name}")

        self._action.callback(goal_handle)
    
# EOF