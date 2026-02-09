"""
ROS2 Speaker Implementation

Concrete implementation of VyraSpeaker for ROS2 Topic communication.
"""
from __future__ import annotations

import logging
import types
from typing import Any, Callable, Optional

from vyra_base.com.core.types import VyraSpeaker, ProtocolType
from vyra_base.com.core.exceptions import InterfaceError
from vyra_base.com.transport.t_ros2.communication import VyraPublisher, VyraSubscriber
from vyra_base.com.transport.t_ros2.node import VyraNode
from vyra_base.com.transport.t_ros2.communication.publisher import PublisherInfo
from vyra_base.com.transport.t_ros2.communication.subscriber import SubscriptionInfo
from vyra_base.com.core.topic_builder import TopicBuilder, InterfaceType

logger = logging.getLogger(__name__)


class ROS2Speaker(VyraSpeaker):
    """
    ROS2-specific implementation of VyraSpeaker using ROS2 Topics.
    
    Wraps VyraPublisher for publishing and VyraSubscriber for subscribing.
    
    Naming Convention:
        Uses TopicBuilder for consistent naming: <module_name>_<module_id>/<function_name>
        Example: v2_modulemanager_abc123/sensor_data
    """
    
    def __init__(
        self,
        name: str,
        topic_builder: TopicBuilder,
        node: Optional[VyraNode] = None,
        message_type: Optional[Any] = None,
        is_publisher: bool = True,
        
        **kwargs
    ):
        super().__init__(name, topic_builder, ProtocolType.ROS2, **kwargs)
        self.node: VyraNode | None = node
        self.message_type = message_type
        self.is_publisher = is_publisher
        self._publisher: Optional[VyraPublisher] = None
        self._subscriber: Optional[VyraSubscriber] = None
        self._last_message: Any = None
    
    async def initialize(self) -> bool:
        """
        Initialize ROS2 topic.
        
        Creates either publisher or subscriber based on is_publisher flag.
        """
        if self._initialized:
            logger.warning(f"ROS2Speaker '{self.name}' already initialized")
            return True
        
        if not self.node:
            raise InterfaceError("Node is required for ROS2Speaker")
        
        if not self.message_type:
            raise InterfaceError("message_type is required for ROS2Speaker")
        
        try:
            if self.is_publisher:
                # Publisher side
                logger.info(f"🔧 Creating ROS2 publisher: {self.name}")

                publisher_info = PublisherInfo(
                    name=self.name,
                    type=self.message_type,
                    periodic_caller=None,
                    qos_profile=10,
                    publisher=None
                )
                self._publisher = VyraPublisher(
                    publisherInfo=publisher_info,
                    node=self.node,
                )
                logger.info(f"✅ ROS2 publisher created: {self.name}")
            else:
                # Subscriber side
                logger.info(f"🔧 Creating ROS2 subscriber: {self.name}")

                subscription_info = SubscriptionInfo(
                    name=self.name,
                    type=self.message_type,
                    qos_profile=10
                )
                self._subscriber = VyraSubscriber(
                    subscriptionInfo=subscription_info,
                    node=self.node,
                )
                logger.info(f"✅ ROS2 subscriber created: {self.name}")
            
            self._initialized = True
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to initialize ROS2Speaker '{self.name}': {e}")
            raise InterfaceError(f"Failed to initialize ROS2Speaker: {e}")
    
    async def shutdown(self) -> None:
        """Shutdown and cleanup ROS2 topic resources."""
        if not self._initialized:
            return

        if not self.node:
            logger.warning("Node not available during ROS2Speaker shutdown")
            return
        
        logger.info(f"🛑 Shutting down ROS2Speaker: {self.name}")
        
        # Cleanup publisher/subscriber
        if self._publisher:
            self.node.destroy_publisher(self._publisher.publisher_info.publisher)
            self._publisher = None
        
        if self._subscriber:
            self.node.destroy_subscription(self._subscriber._subscription_info.subscription)
            self._subscriber = None
        
        self._initialized = False
        logger.info(f"✅ ROS2Speaker '{self.name}' shutdown complete")
    
    async def shout(self, message: Any) -> bool:
        """
        Publish message to ROS2 topic.
        
        Args:
            message: Message data to publish
            
        Returns:
            bool: True if published successfully
            
        Raises:
            InterfaceError: If not initialized or publisher not available
        """
        if not self._initialized:
            raise InterfaceError(f"ROS2Speaker '{self.name}' not initialized")
        
        if not self._publisher:
            raise InterfaceError(
                f"Cannot publish on ROS2Speaker '{self.name}': no publisher. "
                "This is likely a subscriber-only speaker."
            )
        
        try:
            logger.debug(f"📢 Publishing to ROS2 topic: {self.name}")
            
            # Publish via VyraPublisher
            self._publisher.publish(message)
            
            self._last_message = message
            logger.debug(f"✅ Message published to: {self.name}")
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to publish on '{self.name}': {e}")
            raise InterfaceError(f"Publish failed: {e}")
    
    async def listen(
        self,
        callback: Callable[[Any], None],
        **kwargs
    ) -> None:
        """
        Subscribe to ROS2 topic and listen for messages.
        
        Args:
            callback: Callback function to process incoming messages
            **kwargs: Additional parameters
            
        Raises:
            InterfaceError: If not initialized or subscriber not available
        """
        if not self._initialized:
            raise InterfaceError(f"ROS2Speaker '{self.name}' not initialized")
        
        if not self._subscriber:
            raise InterfaceError(
                f"Cannot subscribe on ROS2Speaker '{self.name}': no subscriber. "
                "This is likely a publisher-only speaker."
            )
        
        try:
            logger.info(f"👂 Setting up ROS2 subscription callback: {self.name}")
            
            # Set callback on VyraSubscriber
            self._subscriber.callback = types.MethodType(lambda self, msg: callback(msg), self._subscriber)
            
            logger.info(f"✅ ROS2 subscription active: {self.name}")
            
        except Exception as e:
            logger.error(f"❌ Failed to set up subscription on '{self.name}': {e}")
            raise InterfaceError(f"Subscribe failed: {e}")
    
    def get_publisher(self) -> Optional[VyraPublisher]:
        """Get the underlying ROS2 publisher (publisher-side only)."""
        return self._publisher
    
    def get_subscriber(self) -> Optional[VyraSubscriber]:
        """Get the underlying ROS2 subscriber (subscriber-side only)."""
        return self._subscriber
