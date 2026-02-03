"""
ROS2 VYRA Models Layer

VYRA abstractions for ROS2 communication.
This layer wraps ROS2 functionality into VYRA's unified interface (VyraCallable, VyraSpeaker, VyraJob).

Components:
    - ROS2Callable: Request-response pattern (wraps ROS2 services)
    - ROS2Speaker: Pub/Sub pattern (wraps ROS2 topics)
    - ROS2Job: Long-running tasks (wraps ROS2 actions)

Usage:
    >>> from vyra_base.com.transport.ros2.vyra_models import ROS2Callable, ROS2Speaker
    >>> 
    >>> # Callable (service wrapper)
    >>> callable = ROS2Callable(name="/calculate", node=node, srv_type=AddTwoInts)
    >>> await callable.initialize()
    >>> result = await callable.call({"a": 1, "b": 2})
    >>> 
    >>> # Speaker (topic wrapper)
    >>> speaker = ROS2Speaker(name="/sensor_data", node=node, msg_type=Temperature)
    >>> await speaker.initialize()
    >>> await speaker.shout({"value": 23.5})
"""
import logging

logger = logging.getLogger(__name__)

try:
    from vyra_base.com.transport.ros2.vyra_models.callable import ROS2Callable
    from vyra_base.com.transport.ros2.vyra_models.speaker import ROS2Speaker
    from vyra_base.com.transport.ros2.vyra_models.job import ROS2Job
    
    ROS2_MODELS_AVAILABLE = True
    logger.debug("✅ ROS2 VYRA models layer available")
    
except ImportError as e:
    ROS2Callable = None
    ROS2Speaker = None
    ROS2Job = None
    ROS2_MODELS_AVAILABLE = False
    logger.debug(f"⚠️  ROS2 VYRA models layer unavailable: {e}")

__all__ = [
    "ROS2Callable",
    "ROS2Speaker",
    "ROS2Job",
    "ROS2_MODELS_AVAILABLE",
]
