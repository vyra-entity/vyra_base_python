
import rclpy

from dataclasses import dataclass
from rclpy.qos import QoSProfile
from rclpy.timer import Timer
from rclpy.subscription import Subscription as rclpySubscription
from typing import Any
from typing import Callable
from typing import NoReturn
from typing import Union

from vyra_base.com.datalayer.node import VyraNode


def _base_callback(*args, **kwargs) -> NoReturn:
    raise NotImplementedError("No execute callback provided for action server.")


@dataclass
class SubscriptionInfo:
    name: str = 'vyra_service_server'
    type: Any = None
    callback: Callable = _base_callback
    qos_profile: Union[QoSProfile, int] = 10
    subscription: Union[rclpySubscription, None] = None

class VyraSubscription:
    """
    Base class for ROS2 subscription.
    This class will be factory created to implement specific subscription of a topic.
    """

    def __init__(self, subscriptionInfo: SubscriptionInfo, node: VyraNode) -> None:
        self._subscription_info: SubscriptionInfo = subscriptionInfo
        self._node = node
    
    def create_subscription(self) -> None:
        """
        Create a service in the ROS2 node.
        This method should be called to register the service with the ROS2 node.
        """
        self._node.get_logger().info(f"Creating subscription: {self._subscription_info.name}")
        if not self._subscription_info.type:
            raise ValueError("Service type must be provided.")
        
        if not self._subscription_info.name:
            raise ValueError("Service name must be provided.")
        
        self._subscription_info.subscription = self._node.create_subscription(
            self._subscription_info.type, 
            self._subscription_info.name, 
            self.callback,
            self._subscription_info.qos_profile
        )

    def callback(self, msg)-> None:
        """
        Add a callback to the service.
        This method should be overridden in subclasses to provide specific functionality.
        """
        self._node.get_logger().info(f"Received message on {self._subscription_info.name}")
        
        self._subscription_info.callback(msg)
    