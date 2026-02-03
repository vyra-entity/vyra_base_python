import rclpy
from dataclasses import dataclass
from typing import Any, Callable, Final, NoReturn, Optional, Union

from rclpy.action.client import ActionClient
from rclpy.task import Future

from vyra_base.com.transport.ros2.node import VyraNode

def _base_callback(*args, **kwargs) -> NoReturn:
    """
    Default callback that raises an error if not overridden.

    :raises NotImplementedError: Always raised to indicate the callback is not implemented.
    """
    raise NotImplementedError("No execute callback provided for action server.")


@dataclass
class ActionClientInfo:
    """
    Action class to define the action name and type.

    This class is used to create an action client.

    :ivar name: Name of the action client.
    :vartype name: str
    :ivar type: Type of the action.
    :vartype type: Any
    :ivar result_callable: Callback for the result.
    :vartype result_callable: Callable
    :ivar feedback_callable: Callback for feedback.
    :vartype feedback_callable: Callable
    :ivar client: The action client instance.
    :vartype client: Union[ActionClient, None]
    :ivar TIMEOUT_SEC: Timeout for action client in seconds.
    :vartype TIMEOUT_SEC: float
    """
    name: str = 'vyra_action_client'
    type: Any = None
    result_callable: Optional[Callable] = _base_callback
    feedback_callable: Optional[Callable] = _base_callback
    client: Union[ActionClient, None] = None
    TIMEOUT_SEC: Final[float] = 1.0


class VyraActionClient:
    """
    Base class for ROS2 action client.

    This class will be factory created to call specific action functionality.
    """

    def __init__(self, actionInfo: ActionClientInfo, node: VyraNode) -> None:
        """
        Initialize the VyraActionClient.

        :param actionInfo: Information about the action.
        :type actionInfo: ActionInfo
        :param node: The ROS2 node.
        :type node: VyraNode
        """
        self._action_info: ActionClientInfo = actionInfo
        self._node: VyraNode = node

    def create_action_client(self) -> None:
        """
        Create an action client in the ROS2 node.

        This method should be called to register the action client with the ROS2 node.
        """
        self._node.get_logger().info(f"Creating action client: {self._action_info.name}")
        self._action_info.client = ActionClient(self._node, self._action_info.type, self._action_info.name)

    def send_goal(self, order) -> Future:
        """
        Send a goal to the action server.

        :param order: The order to send as part of the goal.
        :type order: Any
        :raises ValueError: If the action client has not been created.
        :return: A future representing the goal request.
        :rtype: Future
        """
        goal_msg = self._action_info.type.Goal()
        goal_msg.order = order

        self._node.get_logger().info(f"Sending goal to action client: {self._action_info.name}")

        if not self._action_info.client:
            raise ValueError("Action client must be created before sending a goal.")

        self._action_info.client.wait_for_server(timeout_sec=self._action_info.TIMEOUT_SEC)

        return self._action_info.client.send_goal_async(goal_msg)

    def goal_response_callback(self, future) -> None:
        """
        Callback for the goal response.

        :param future: The future containing the goal handle.
        :type future: Future
        """
        goal_handle: Any = future.result()
        if not goal_handle.accepted:
            self._node.get_logger().info('Goal rejected :(')
            return

        self._node.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()

        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future) -> None:
        """
        Callback for receiving the result of the action.

        :param future: The future containing the result.
        :type future: Future
        """
        result: Any = future.result().result
        self._node.get_logger().info('Action Result: {0}'.format(result.sequence))

        if self._action_info.result_callable:
            self._action_info.result_callable(future)