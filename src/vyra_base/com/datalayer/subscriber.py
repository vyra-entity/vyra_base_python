import rclpy
from dataclasses import dataclass
from typing import Any, Callable, NoReturn, Union

from rclpy.qos import QoSProfile
from rclpy.subscription import Subscription as rclpySubscription
from rclpy.timer import Timer

from vyra_base.com.datalayer.node import VyraNode


def _base_callback(*args, **kwargs) -> NoReturn:
    """
    Default callback that raises a NotImplementedError.

    :raises NotImplementedError: Always raised to indicate no callback is provided.
    """
    raise NotImplementedError("No execute callback provided for action server.")


@dataclass
class SubscriptionInfo:
    """
    Data class for storing subscription information.

    :param name: Name of the subscription.
    :type name: str
    :param type: Message type for the subscription.
    :type type: Any
    :param callback: Callback function for the subscription.
    :type callback: Callable
    :param qos_profile: Quality of Service profile or depth.
    :type qos_profile: Union[QoSProfile, int]
    :param subscription: The actual ROS2 subscription object.
    :type subscription: Union[rclpySubscription, None]
    """
    name: str = 'vyra_service_server'
    type: Any = None
    callback: Callable = _base_callback
    qos_profile: Union[QoSProfile, int] = 10
    subscription: Union[rclpySubscription, None] = None

class VyraSubscription:
    """
    Base class for ROS2 subscriptions.

    This class is intended to be factory-created to implement specific subscriptions for topics.
    """

    def __init__(self, subscriptionInfo: SubscriptionInfo, node: VyraNode) -> None:
        """
        Initialize the VyraSubscription.

        :param subscriptionInfo: Information about the subscription.
        :type subscriptionInfo: SubscriptionInfo
        :param node: The ROS2 node to attach the subscription to.
        :type node: VyraNode
        """
        self._subscription_info: SubscriptionInfo = subscriptionInfo
        self._node = node
    
    def create_subscription(self) -> None:
        """
        Create and register the subscription with the ROS2 node.

        :raises ValueError: If the subscription type or name is not provided.
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
    
    def remove_subscription(self) -> None:
        """
        Remove the subscription from the ROS2 node.

        This method will destroy the subscription if it exists.
        """
        if self._subscription_info.subscription:
            self._node.destroy_subscription(self._subscription_info.subscription)
            self._subscription_info.subscription = None
            self._node.get_logger().info(f"Subscription '{self._subscription_info.name}' removed.")

    def callback(self, msg) -> None:
        """
        Callback method for the subscription.

        This method should be overridden in subclasses to provide specific functionality.

        :param msg: The message received from the subscription.
        :type msg: Any
        """
        self._node.get_logger().info(f"Received message on {self._subscription_info.name}")
        self._subscription_info.callback(msg)