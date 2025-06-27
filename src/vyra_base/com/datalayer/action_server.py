from __future__ import annotations

import rclpy
from dataclasses import dataclass
from rclpy.action import ActionServer
from typing import Any, Callable, NoReturn, Union

from vyra_base.com.datalayer.node import VyraNode


def _base_callback(goal_handle) -> NoReturn:
    """
    Raises
    ======
    NotImplementedError
        If no execute callback is provided for the action server.
    """
    raise NotImplementedError("No execute callback provided for action server.")


@dataclass
class Action:
    """
    Represents an action for the VyraActionServer.

    Attributes
    ----------
    name : str
        Name of the action server.
    type : Any
        Type of the action.
    callback : Callable
        Callback function for the action.
    server : Union[ActionServer, None]
        The ActionServer instance or None.
    """
    name: str = 'vyra_action_server'
    type: Any = None
    callback: Callable = _base_callback
    server: Union[ActionServer, None] = None


class VyraActionServer:
    """
    Base class for ROS2 actions.

    This class is intended to be factory-created to implement specific action functionality.
    """

    def __init__(self, action: Action, node: VyraNode) -> None:
        """
        Initialize the VyraActionServer.

        Parameters
        ----------
        action : Action
            The action configuration.
        node : VyraNode
            The ROS2 node to attach the action server to.
        """
        self._action: Action = action
        self._node: VyraNode = node

    def create_action_server(self) -> None:
        """
        Create and register the action server with the ROS2 node.

        This method should be called to register the action server with the ROS2 node.
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
        Execute the action callback.

        This method should be overridden in subclasses to provide specific functionality.

        Parameters
        ----------
        goal_handle : Any
            The goal handle for the action.
        """
        self._node.get_logger().info(f"Executing action: {self._action.name}")

        self._action.callback(goal_handle)