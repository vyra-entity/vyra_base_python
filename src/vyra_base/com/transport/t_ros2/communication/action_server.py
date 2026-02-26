from __future__ import annotations

import rclpy
from dataclasses import dataclass
from rclpy.action.server import ActionServer, GoalResponse
from typing import Any, Callable, NoReturn, Optional, Union

from vyra_base.com.transport.t_ros2.node import VyraNode


def _base_callback(goal_handle) -> NoReturn:
    """
    Raises
    ======
    NotImplementedError
        If no execute callback is provided for the action server.
    """
    raise NotImplementedError("No execute callback provided for action server.")


@dataclass
class ActionServerInfo:
    """
    Represents an action for the ROS2ActionServer.

    Attributes
    ----------
    name : str
        Name of the action server.
    type : Any
        Type of the action.
    goal_callback : Callable
        Called when a new goal arrives; returns GoalResponse.ACCEPT / REJECT.
    cancel_callback : Callable
        Called when a cancel request arrives; returns CancelResponse.ACCEPT / REJECT.
    execute_callback : Callable
        Called when a goal is executing; receives goal_handle.
    server : Union[ActionServer, None]
        The ActionServer instance or None.
    """
    name: str = 'vyra_action_server'
    type: Any = None
    goal_callback: Optional[Callable] = _base_callback
    cancel_callback: Optional[Callable] = None
    execute_callback: Optional[Callable] = _base_callback
    server: Union[ActionServer, None] = None


class ROS2ActionServer:
    """
    Base class for ROS2 actions.

    This class is intended to be factory-created to implement specific action functionality.
    """

    def __init__(self, actionInfo: ActionServerInfo, node: VyraNode) -> None:
        """
        Initialize the ROS2ActionServer.

        Parameters
        ----------
        action : ActionServerInfo
            The action configuration.
        node : VyraNode
            The ROS2 node to attach the action server to.
        """
        self._action: ActionServerInfo = actionInfo
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
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
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

        if self._action.execute_callback:
            self._action.execute_callback(goal_handle)

    def goal_callback(self, goal_handle) -> GoalResponse:
        """
        Handle the goal callback.

        This method should be overridden in subclasses to provide specific functionality.

        Parameters
        ----------
        goal_handle : Any
            The goal handle for the action.
        """
        self._node.get_logger().info(f"Handling goal for action: {self._action.name}")


        if self._action.goal_callback:
            return self._action.goal_callback(goal_handle)
        return GoalResponse.REJECT

    def cancel_callback(self, goal_handle) -> Any:
        """Handle the cancel callback.

        Returns CancelResponse.ACCEPT if the cancel request is accepted,
        CancelResponse.REJECT otherwise.
        """
        from rclpy.action.server import CancelResponse
        self._node.get_logger().info(f"Cancel request for action: {self._action.name}")

        if self._action.cancel_callback:
            return self._action.cancel_callback(goal_handle)
        # Accept cancel by default if no custom handler is set
        return CancelResponse.ACCEPT
    
    def destroy(self) -> None:
        """
        Destroy the action server and cleanup resources.
        """
        if self._action.server:
            self._node.get_logger().info(f"Destroying action server: {self._action.name}")
            self._action.server.destroy()
            self._action.server = None