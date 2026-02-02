"""
ROS2 Transport

ROS2/DDS-based distributed communication transport.
Supports services, topics, and actions via rclpy.
"""

# Check if ROS2 is available
try:
    import rclpy
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

__all__ = ["ROS2_AVAILABLE"]

# Only export ROS2 components if available
if ROS2_AVAILABLE:
    from vyra_base.com.transport.ros2.node import (
        VyraNode,
        CheckerNode,
        NodeSettings,
    )
    from vyra_base.com.transport.ros2.publisher import VyraPublisher
    from vyra_base.com.transport.ros2.subscriber import VyraSubscriber
    from vyra_base.com.transport.ros2.service_server import VyraServiceServer
    from vyra_base.com.transport.ros2.service_client import VyraServiceClient
    from vyra_base.com.transport.ros2.action_server import VyraActionServer
    from vyra_base.com.transport.ros2.action_client import VyraActionClient
    from vyra_base.com.transport.ros2.provider import ROS2Provider
    from vyra_base.com.transport.ros2.typeconverter import Ros2TypeConverter
    
    # ROS2 concrete implementations
    from vyra_base.com.transport.ros2.ros2_callable import ROS2Callable
    from vyra_base.com.transport.ros2.ros2_speaker import ROS2Speaker
    from vyra_base.com.transport.ros2.ros2_job import ROS2Job
    
    __all__.extend([
        "VyraNode",
        "CheckerNode",
        "NodeSettings",
        "VyraPublisher",
        "VyraSubscriber",
        "VyraServiceServer",
        "VyraServiceClient",
        "VyraActionServer",
        "VyraActionClient",
        "ROS2Provider",
        "Ros2TypeConverter",
        "ROS2Callable",
        "ROS2Speaker",
        "ROS2Job",
    ])

