"""
ROS2 Transport Module

Provides ROS2/DDS-based transport implementation with layered architecture:

Layers:
    - communication/: Core ROS2 functionality (Services, Topics, Actions)
    - vyra_models/: VYRA abstractions (ROS2Callable, ROS2Speaker, ROS2Job)
    - node.py: ROS2 node management and lifecycle
    - provider.py: Interface layer for VYRA integration

Features:
    - Service-based request-response (ROS2Callable)
    - Topic-based Pub/Sub (ROS2Speaker)
    - Action-based long-running tasks (ROS2Job)
    - Type conversion utilities
    - Node lifecycle management

Usage:
    from vyra_base.com.transport.t_ros2 import ROS2Provider, ROS2_AVAILABLE
    
    if ROS2_AVAILABLE:
        provider = ROS2Provider(node_name="my_node")
        await provider.initialize()
        callable = await provider.create_callable("/service", callback)
        speaker = await provider.create_speaker("/topic")
"""
import logging

logger = logging.getLogger(__name__)

# Check ROS2 availability
try:
    import rclpy
    _rclpy_available = True
except ImportError:
    _rclpy_available = False
    logger.debug("⚠️  ROS2 (rclpy) not available")

# Try importing node layer (ROS2-specific)
try:
    from vyra_base.com.transport.t_ros2.node import (
        VyraNode,
        CheckerNode,
        NodeSettings,
    )
    _node_available = True
except ImportError as e:
    VyraNode = None
    CheckerNode = None
    NodeSettings = None
    _node_available = False
    logger.debug(f"⚠️  ROS2 node layer unavailable: {e}")

# Try importing communication layer
try:
    from vyra_base.com.transport.t_ros2.communication import (
        VyraServiceServer,
        VyraServiceClient,
        VyraPublisher,
        VyraSubscriber,
        VyraActionServer,
        VyraActionClient,
        Ros2TypeConverter,
        ROS2_COMMUNICATION_AVAILABLE,
    )
    _communication_available = ROS2_COMMUNICATION_AVAILABLE
except ImportError as e:
    VyraServiceServer = None
    VyraServiceClient = None
    VyraPublisher = None
    VyraSubscriber = None
    VyraActionServer = None
    VyraActionClient = None
    Ros2TypeConverter = None
    _communication_available = False
    logger.debug(f"⚠️  ROS2 communication layer unavailable: {e}")

# Try importing VYRA models layer
try:
    from vyra_base.com.transport.t_ros2.vyra_models import (
        ROS2Callable,
        ROS2Speaker,
        ROS2Job,
        ROS2_MODELS_AVAILABLE,
    )
    _models_available = ROS2_MODELS_AVAILABLE
except ImportError as e:
    ROS2Callable = None
    ROS2Speaker = None
    ROS2Job = None
    _models_available = False
    logger.debug(f"⚠️  ROS2 VYRA models layer unavailable: {e}")

# Try importing provider (interface layer)
try:
    from vyra_base.com.transport.t_ros2.provider import ROS2Provider
    _provider_available = True
except ImportError as e:
    ROS2Provider = None
    _provider_available = False
    logger.debug(f"⚠️  ROS2 provider unavailable: {e}")

# ROS2 is fully available if all layers are available
ROS2_AVAILABLE = (
    _rclpy_available 
    and _node_available 
    and _communication_available 
    and _models_available 
    and _provider_available
)

if ROS2_AVAILABLE:
    logger.info("✅ ROS2 transport fully available (node + communication + models + provider)")
elif _communication_available:
    logger.info("⚠️  ROS2 transport partially available (communication only)")
else:
    logger.debug("❌ ROS2 transport unavailable")

__all__ = [
    # Availability flag
    "ROS2_AVAILABLE",
    # Interface layer
    "ROS2Provider",
    # Node layer
    "VyraNode",
    "CheckerNode",
    "NodeSettings",
    # Communication layer
    "VyraServiceServer",
    "VyraServiceClient",
    "VyraPublisher",
    "VyraSubscriber",
    "VyraActionServer",
    "VyraActionClient",
    "Ros2TypeConverter",
    # VYRA models layer
    "ROS2Callable",
    "ROS2Speaker",
    "ROS2Job",
]

