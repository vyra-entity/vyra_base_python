"""
ROS2 Subscriber Implementation

Async-first subscriber for ROS2 topics with callback adapter.
"""
import asyncio
import logging
from typing import Any, Callable, Optional

from vyra_base.com.core.types import VyraSubscriber, ProtocolType
from vyra_base.com.core.topic_builder import TopicBuilder
from vyra_base.com.core.exceptions import InterfaceError
from vyra_base.com.transport.t_ros2.communication.subscriber import (
    ROS2Subscriber as ROS2Subscriber,
    SubscriberInfo
)

logger = logging.getLogger(__name__)


class CallbackAdapter:
    """Adapter to convert async callbacks to sync for ROS2."""
    
    def __init__(self, async_callback: Callable):
        self.async_callback = async_callback
        
    def __call__(self, msg: Any):
        """Sync wrapper that runs async callback."""
        try:
            # Create new event loop if none exists (ROS2 callback context)
            try:
                loop = asyncio.get_event_loop()
                if loop.is_closed():
                    raise RuntimeError("Event loop is closed")
            except RuntimeError:
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
            
            # Run async callback
            if asyncio.iscoroutinefunction(self.async_callback):
                loop.run_until_complete(self.async_callback(msg))
            else:
                # Fallback for sync callbacks
                self.async_callback(msg)
                
        except Exception as e:
            logger.error(f"❌ Callback error: {e}")


class VyraSubscriberImpl(VyraSubscriber):
    """
    Vyra Subscriber implementation.
    
    Wraps Vyra topic subscriber with async callback support.
    """
    
    def __init__(
        self,
        name: str,
        topic_builder: TopicBuilder,
        node: Any,
        message_type: Any,
        subscriber_callback: Optional[Callable] = None,
        qos_profile: Optional[Any] = None,
        **kwargs
    ):
        super().__init__(name, topic_builder, subscriber_callback, ProtocolType.ROS2, **kwargs)
        self.node = node
        self.message_type = message_type
        self.qos_profile = qos_profile
        self._ros2_subscriber: Optional[ROS2Subscriber] = None
        self._callback_adapter: Optional[CallbackAdapter] = None
        
    async def initialize(self) -> bool:
        """Initialize ROS2 subscriber."""
        try:
            topic_name = self.topic_builder.build(self.name, namespace=self.namespace, subsection=self.subsection)
            
            # Wrap async callback for ROS2 sync context
            if self.subscriber_callback:
                self._callback_adapter = CallbackAdapter(self.subscriber_callback)
            
            subscriber_info = SubscriberInfo(
                name=topic_name,
                type=self.message_type,
                callback=self._callback_adapter.__call__ if self._callback_adapter else lambda msg: None,
                qos_profile=self.qos_profile if self.qos_profile is not None else 10
            )
            self._ros2_subscriber = ROS2Subscriber(
                node=self.node,
                subscriptionInfo=subscriber_info
            )
            
            self._transport_handle = self._ros2_subscriber
            self._initialized = True
            
            logger.info(f"✅ ROS2 Subscriber '{self.name}' initialized on topic '{topic_name}'")
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to initialize ROS2 Subscriber '{self.name}': {e}")
            raise InterfaceError(f"Subscriber initialization failed: {e}")
    
    async def shutdown(self) -> None:
        """Shutdown ROS2 subscriber."""
        if self._ros2_subscriber:
            self._ros2_subscriber.remove_subscription()
            self._ros2_subscriber = None
        self._callback_adapter = None
        self._initialized = False
        self._transport_handle = None
        
    async def subscribe(self) -> None:
        """
        Start subscribing (already active after initialize for ROS2).
        
        Note: ROS2 subscribers are automatically active after creation.
        This method is provided for API consistency.
        """
        if not self._initialized:
            raise InterfaceError(f"Subscriber '{self.name}' not initialized")
        
        # ROS2 subscribers are always listening after creation
        logger.debug(f"Subscriber '{self.name}' is active")
