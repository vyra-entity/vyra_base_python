"""
ROS2 Communication Layer

Core ROS2/DDS functionality: Services, Topics, Actions.
This layer contains the functional implementation of ROS2 communication primitives.

Components:
    - VyraServiceServer: ROS2 service server
    - VyraServiceClient: ROS2 service client
    - VyraPublisher: ROS2 topic publisher
    - VyraSubscriber: ROS2 topic subscriber
    - VyraActionServer: ROS2 action server
    - VyraActionClient: ROS2 action client
    - Ros2TypeConverter: Type conversion utilities

Usage:
    >>> from vyra_base.com.transport.t_ros2.communication import VyraServiceServer, VyraPublisher
    >>> 
    >>> # Service server
    >>> server = VyraServiceServer(node, srv_type, "/my_service", callback)
    >>> 
    >>> # Publisher
    >>> pub = VyraPublisher(node, msg_type, "/my_topic")
    >>> pub.publish(msg)
"""
import logging

logger = logging.getLogger(__name__)

try:
    from vyra_base.com.transport.t_ros2.communication.service_server import ROS2ServiceServer
    from vyra_base.com.transport.t_ros2.communication.service_client import ROS2ServiceClient
    from vyra_base.com.transport.t_ros2.communication.publisher import ROS2Publisher
    from vyra_base.com.transport.t_ros2.communication.subscriber import ROS2Subscriber
    from vyra_base.com.transport.t_ros2.communication.action_server import ROS2ActionServer
    from vyra_base.com.transport.t_ros2.communication.action_client import ROS2ActionClient
    from vyra_base.com.transport.t_ros2.communication.typeconverter import Ros2TypeConverter
    
    ROS2_COMMUNICATION_AVAILABLE = True
    logger.debug("✅ ROS2 communication layer available")
    
except ImportError as e:
    ROS2ServiceServer = None
    ROS2ServiceClient = None
    ROS2Publisher = None
    ROS2Subscriber = None
    ROS2ActionServer = None
    ROS2ActionClient = None
    Ros2TypeConverter = None
    ROS2_COMMUNICATION_AVAILABLE = False
    logger.debug(f"⚠️  ROS2 communication layer unavailable: {e}")

__all__ = [
    "ROS2ServiceServer",
    "ROS2ServiceClient",
    "ROS2Publisher",
    "ROS2Subscriber",
    "ROS2ActionServer",
    "ROS2ActionClient",
    "Ros2TypeConverter",
    "ROS2_COMMUNICATION_AVAILABLE",
]
