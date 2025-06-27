import rclpy

from dataclasses import dataclass
from typing import Any, Callable, NoReturn, Union

from rclpy.qos import QoSProfile
from rclpy.timer import Timer
from rclpy.publisher import Publisher as rclpyPublisher

from vyra_base.com.datalayer.node import VyraNode

@dataclass
class PeriodicCaller:
    """
    Stores information for periodic function calls.

    :param interval_time: Interval time in seconds between calls (default: 1.0).
    :type interval_time: float or None
    :param caller: Callable to be called periodically.
    :type caller: Callable or None
    :param timer: ROS2 timer object.
    :type timer: Timer or None
    """
    interval_time: Union[float, None] = 1.0  # Default interval time in seconds
    caller: Union[Callable, None] = None
    timer: Union[Timer, None] = None

@dataclass
class PublisherInfo:
    """
    Stores information about a ROS2 publisher.

    :param name: Name of the publisher/topic.
    :type name: str
    :param type: Message type for the publisher.
    :type type: Any
    :param periodic_caller: PeriodicCaller instance for periodic publishing.
    :type periodic_caller: PeriodicCaller or None
    :param qos_profile: Quality of Service profile or depth.
    :type qos_profile: QoSProfile or int
    :param publisher: The actual ROS2 publisher object.
    :type publisher: rclpyPublisher or None
    """
    name: str = 'vyra_publisher'
    type: Any = None
    periodic_caller: Union[PeriodicCaller, None] = None
    qos_profile: Union[QoSProfile, int] = 10
    publisher: Union[rclpyPublisher, None] = None

class VyraPublisher:
    """
    Base class for ROS2 publishers.

    This class is intended to be factory-created to implement a specific publisher for a topic.
    """

    def __init__(self, publisherInfo: PublisherInfo, node: VyraNode) -> None:
        """
        Initialize the VyraPublisher.

        :param publisherInfo: PublisherInfo instance containing publisher configuration.
        :type publisherInfo: PublisherInfo
        :param node: The ROS2 node to which the publisher belongs.
        :type node: VyraNode
        """
        self.publisher_info: PublisherInfo = publisherInfo
        self._node: VyraNode = node

        self._node.get_logger().info(f"Initializing publisher: {self.publisher_info.name}")

        if (
            self.publisher_info.periodic_caller is not None
            and self.publisher_info.periodic_caller.caller is not None
            and self.publisher_info.periodic_caller.interval_time is not None
        ):
            self._node.create_timer(
                self.publisher_info.periodic_caller.interval_time, 
                self.publisher_info.periodic_caller.caller
            )
    
    def create_publisher(self) -> None:
        """
        Create a publisher in the ROS2 node.

        This method should be called to register the publisher within the ROS2 node.

        :raises ValueError: If publisher type or name is not provided.
        """
        self._node.get_logger().info(f"Creating publisher: {self.publisher_info.name}")
        if not self.publisher_info.type:
            raise ValueError("Publisher type must be provided.")
        
        if not self.publisher_info.name:
            raise ValueError("Publisher name must be provided.")
        
        self.publisher_info.publisher = self._node.create_publisher(
            self.publisher_info.type, 
            self.publisher_info.name, 
            self.publisher_info.qos_profile
        )

    def publish(self, msg: Any) -> None:
        """
        Publish a message to the topic.

        This method should be overridden in subclasses to provide specific functionality.

        :param msg: The message to publish.
        :type msg: Any
        :raises ValueError: If publisher type, name, or publisher instance is not set.
        """
        self._node.get_logger().info(f"Publishing message on {self.publisher_info.name}")
        
        if not self.publisher_info.type:
            raise ValueError("Publisher type must be provided.")
        
        if not self.publisher_info.name:
            raise ValueError("Publisher name must be provided.")
        
        if not self.publisher_info.publisher:
            raise ValueError("Publisher must be created before publishing messages.")

        self.publisher_info.publisher.publish(msg)
        self._node.get_logger().info(f"Message published on {self.publisher_info.name}")
        if self.publisher_info.periodic_caller is not None:
            self.publisher_info.periodic_caller.caller = None
            self.publisher_info.periodic_caller.timer = None

# EOF