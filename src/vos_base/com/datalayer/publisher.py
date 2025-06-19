
import rclpy

from dataclasses import dataclass
from rclpy.qos import QoSProfile
from rclpy.timer import Timer
from rclpy.publisher import Publisher as rclpyPublisher

from typing import Any
from typing import Callable
from typing import NoReturn
from typing import Union

from vos_base.com.datalayer.node import VOSNode

@dataclass
class PeriodicCaller:
    interval_time: Union[float, None] = 1.0  # Default interval time in seconds
    caller: Union[Callable, None] = None
    timer: Union[Timer, None] = None

@dataclass
class PublisherInfo:
    name: str = 'vos_publisher'
    type: Any = None
    periodic_caller: Union[PeriodicCaller, None] = None
    qos_profile: Union[QoSProfile, int] = 10
    publisher: Union[rclpyPublisher, None] = None

class VOSPublisher:
    """
    Base class for ROS2 publisher.
    This class will be factory created to implement specific publisher for a topic.
    """

    def __init__(self, publisherInfo: PublisherInfo, node: VOSNode) -> None:
        self.publisher_info: PublisherInfo = publisherInfo
        self._node: VOSNode = node

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
    