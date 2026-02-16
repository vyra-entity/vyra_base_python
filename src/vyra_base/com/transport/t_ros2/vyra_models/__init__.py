"""
ROS2 VYRA Models Layer

VYRA abstractions for ROS2 communication.
This layer wraps ROS2 functionality into VYRA's unified interface.

Components:
    - ROS2PublisherImpl: Publish-only pattern (wraps ROS2 topic publisher)
    - ROS2SubscriberImpl: Subscribe-only pattern (wraps ROS2 topic subscriber)
    - ROS2ServerImpl: Request-response server (wraps ROS2 service server)
    - ROS2ClientImpl: Request-response client (wraps ROS2 service client)
    - ROS2ActionServerImpl: Long-running task server (wraps ROS2 action server)
    - ROS2ActionClientImpl: Long-running task client (wraps ROS2 action client)

Usage:
    >>> from vyra_base.com.transport.t_ros2.vyra_models import ROS2PublisherImpl, ROS2SubscriberImpl
    >>> 
    >>> # Publisher
    >>> publisher = ROS2PublisherImpl(name="sensor_data", node=node, message_type=Temperature)
    >>> await publisher.initialize()
    >>> await publisher.publish(temperature_msg)
    >>> 
    >>> # Subscriber with callback
    >>> async def on_message(msg):
    ...     print(f"Received: {msg}")
    >>> subscriber = ROS2SubscriberImpl(name="sensor_data", node=node, message_type=Temperature, subscriber_callback=on_message)
    >>> await subscriber.initialize()
"""
import logging

logger = logging.getLogger(__name__)

try:
    # New unified implementations
    from vyra_base.com.transport.t_ros2.vyra_models.publisher import VyraPublisherImpl
    from vyra_base.com.transport.t_ros2.vyra_models.subscriber import VyraSubscriberImpl
    from vyra_base.com.transport.t_ros2.vyra_models.server import VyraServerImpl
    from vyra_base.com.transport.t_ros2.vyra_models.client import VyraClientImpl
    from vyra_base.com.transport.t_ros2.vyra_models.action_server import VyraActionServerImpl
    from vyra_base.com.transport.t_ros2.vyra_models.action_client import VyraActionClientImpl
    
    # Legacy implementations (deprecated)
    from vyra_base.com.transport.t_ros2.vyra_models.callable import ROS2Callable
    from vyra_base.com.transport.t_ros2.vyra_models.speaker import ROS2Speaker
    from vyra_base.com.transport.t_ros2.vyra_models.job import ROS2Job
    
    ROS2_MODELS_AVAILABLE = True
    logger.debug("✅ ROS2 VYRA models layer available")
    
except ImportError as e:
    VyraPublisherImpl = None
    VyraSubscriberImpl = None
    VyraServerImpl = None
    VyraClientImpl = None
    VyraActionServerImpl = None
    VyraActionClientImpl = None
    ROS2_MODELS_AVAILABLE = False
    logger.debug(f"⚠️  ROS2 VYRA models layer unavailable: {e}")

__all__ = [
    # New unified
    "VyraPublisherImpl",
    "VyraSubscriberImpl",
    "VyraServerImpl",
    "VyraClientImpl",
    "VyraActionServerImpl",
    "VyraActionClientImpl",
    # Legacy (deprecated)
    "ROS2_MODELS_AVAILABLE",
]
