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
    from vyra_base.com.transport.t_ros2.communication.service_server import VyraServiceServer
    from vyra_base.com.transport.t_ros2.communication.service_client import VyraServiceClient
    from vyra_base.com.transport.t_ros2.communication.publisher import VyraPublisher
    from vyra_base.com.transport.t_ros2.communication.subscriber import VyraSubscriber
    from vyra_base.com.transport.t_ros2.communication.action_server import VyraActionServer
    from vyra_base.com.transport.t_ros2.communication.action_client import VyraActionClient
    from vyra_base.com.transport.t_ros2.communication.typeconverter import Ros2TypeConverter
    
    ROS2_COMMUNICATION_AVAILABLE = True
    logger.debug("✅ ROS2 communication layer available")
    
except ImportError as e:
    VyraServiceServer = None
    VyraServiceClient = None
    VyraPublisher = None
    VyraSubscriber = None
    VyraActionServer = None
    VyraActionClient = None
    Ros2TypeConverter = None
    ROS2_COMMUNICATION_AVAILABLE = False
    logger.debug(f"⚠️  ROS2 communication layer unavailable: {e}")

__all__ = [
    "VyraServiceServer",
    "VyraServiceClient",
    "VyraPublisher",
    "VyraSubscriber",
    "VyraActionServer",
    "VyraActionClient",
    "Ros2TypeConverter",
    "ROS2_COMMUNICATION_AVAILABLE",
]
