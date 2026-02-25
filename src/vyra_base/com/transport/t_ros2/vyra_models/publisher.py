"""
ROS2 Publisher Implementation

Async-first publisher for ROS2 topics.
"""
import asyncio
import logging
from typing import Any, Optional

from vyra_base.com.core.types import VyraPublisher, ProtocolType
from vyra_base.com.core.topic_builder import TopicBuilder
from vyra_base.com.core.exceptions import InterfaceError
from vyra_base.com.transport.t_ros2.communication.publisher import PublisherInfo, ROS2Publisher as ROS2Publisher
from vyra_base.com.transport.t_uds.vyra_models import publisher

logger = logging.getLogger(__name__)


class VyraPublisherImpl(VyraPublisher):
    """
    Vyra Publisher implementation.
    
    Wraps Vyra topic publisher for one-way message publishing.
    """
    
    def __init__(
        self,
        name: str,
        topic_builder: TopicBuilder,
        node: Any,
        message_type: Any,
        qos_profile: Optional[Any] = None,
        **kwargs
    ):
        super().__init__(name, topic_builder, ProtocolType.ROS2, **kwargs)
        self.node = node
        self.message_type = message_type
        self.qos_profile = qos_profile
        self._ros2_publisher: Optional[ROS2Publisher] = None
        
    async def initialize(self) -> bool:
        """Initialize ROS2 publisher."""
        try:
            topic_name = self.topic_builder.build(self.name)
            
            publisher_info = PublisherInfo(
                name=topic_name,
                type=self.message_type,
                qos_profile=self.qos_profile if self.qos_profile is not None else 10
            )

            self._ros2_publisher = ROS2Publisher(
                node=self.node,
                publisherInfo=publisher_info
            )

            # Registers the rclpy publisher on the ROS2 node.
            # Without this call publisher_info.publisher remains None and every
            # subsequent publish() raises "Publisher must be created before
            # publishing messages."
            self._ros2_publisher.create_publisher()

            self._transport_handle = self._ros2_publisher
            self._initialized = True
            
            logger.info(f"✅ ROS2 Publisher '{self.name}' initialized on topic '{topic_name}'")
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to initialize ROS2 Publisher '{self.name}': {e}")
            raise InterfaceError(f"Publisher initialization failed: {e}")
    
    async def shutdown(self) -> None:
        """Shutdown ROS2 publisher."""
        if self._ros2_publisher:
            self._ros2_publisher.destroy()
            self._ros2_publisher = None
        self._initialized = False
        self._transport_handle = None
        
    async def publish(self, message: Any) -> bool:
        """
        Publish message (async).
        
        Args:
            message: Message to publish (ROS2 message instance)
            
        Returns:
            bool: True if published successfully
        """
        if not self._initialized or not self._ros2_publisher:
            raise InterfaceError(f"Publisher '{self.name}' not initialized")
        
        try:
            # ROS2 publish is sync, wrap in executor for async
            loop = asyncio.get_event_loop()
            await loop.run_in_executor(None, self._ros2_publisher.publish, message)
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to publish on '{self.name}': {e}")
            return False
