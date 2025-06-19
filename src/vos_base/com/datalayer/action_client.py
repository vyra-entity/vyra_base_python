
import rclpy

from dataclasses import dataclass
from rclpy.action import ActionClient
from rclpy.task import Future
from typing import Any
from typing import Callable
from typing import NoReturn
from typing import Union
from typing import Final
from vos_base.com.datalayer.node import VOSNode

def _base_callback(*args, **kwargs) -> NoReturn:
    raise NotImplementedError("No execute callback provided for action server.")


@dataclass
class ActionInfo:
    """
    Action class to define the action name and type.
    This class is used to create an action client.


    """
    name: str = 'vos_action_client'
    type: Any = None
    result_callable: Callable = _base_callback
    feedback_callable: Callable = _base_callback
    client: Union[ActionClient, None] = None
    TIMEOUT_SEC: Final[float] = 1.0  # Timeout for action client in seconds


class VOSActionClient:
    """
    Base class for ROS2 action client.
    This class will be factory created to call specific action functionality.
    """

    def __init__(self, actionInfo: ActionInfo,  node: VOSNode) -> None:
        self._action_info: ActionInfo = actionInfo
        self._node: VOSNode = node
    
    def create_action_client(self) -> None:
        """
        Create an action client in the ROS2 node.
        This method should be called to register the action client with the ROS2 node.
        """
        self._node.get_logger().info(f"Creating action client: {self._action_info.name}")
        self._action_info.client = ActionClient(self._node, self._action_info.type, self._action_info.name)

    def send_goal(self, order) -> Future:
        goal_msg = self._action_info.type.Goal()
        goal_msg.order = order

        self._node.get_logger().info(f"Sending goal to action client: {self._action_info.name}")

        if not self._action_info.client:
            raise ValueError("Action client must be created before sending a goal.")
        
        self._action_info.client.wait_for_server(timeout_sec=self._action_info.TIMEOUT_SEC)

        return self._action_info.client.send_goal_async(goal_msg)

    def goal_response_callback(self, future) -> None:
        goal_handle: Any = future.result()
        if not goal_handle.accepted:
            self._node.get_logger().info('Goal rejected :(')
            return

        self._node.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future) -> None:
        result: Any = future.result().result
        self._node.get_logger().info('Action Result: {0}'.format(result.sequence))

        self._action_info.result_callable(future)
