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
    """
    Adapter to convert async subscriber callbacks to sync for ROS2.

    The main asyncio event loop must be provided at construction time
    (captured while ``initialize()`` runs in the async context). Callbacks
    from the ROS2 executor thread are dispatched onto this loop via
    ``asyncio.run_coroutine_threadsafe`` to avoid deadlocking a running loop.
    """

    def __init__(self, async_callback: Callable, main_loop: asyncio.AbstractEventLoop):
        self.async_callback = async_callback
        self._main_loop = main_loop

    def __call__(self, msg: Any):
        """Convert ROS2 msg to dict, then schedule async callback on the main loop."""
        try:
            # Convert ROS2 message to a plain dict so callers don't receive
            # un-serialisable ROS2 objects. Use get_fields_and_field_types()
            # which exposes the public field names (not the _private slots).
            if hasattr(msg, "get_fields_and_field_types"):
                data = {
                    field: getattr(msg, field)
                    for field in msg.get_fields_and_field_types()
                }
            elif hasattr(msg, "__slots__"):
                data = {
                    slot.lstrip("_"): getattr(msg, slot)
                    for slot in msg.__slots__
                    if slot != "_check_fields"
                }
            else:
                data = msg
            if asyncio.iscoroutinefunction(self.async_callback):
                asyncio.run_coroutine_threadsafe(
                    self.async_callback(data), self._main_loop
                )
            else:
                self.async_callback(data)
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

            # Capture the running event loop now (initialize() runs in main
            # async context) so the CallbackAdapter can dispatch onto it from
            # the ROS2 executor thread via run_coroutine_threadsafe.
            main_loop = asyncio.get_event_loop()

            # Wrap async callback for ROS2 sync context
            if self.subscriber_callback:
                self._callback_adapter = CallbackAdapter(self.subscriber_callback, main_loop)
            
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
            # Register the subscription on the ROS2 node (equivalent to
            # create_publisher() for publishers — without this the node
            # never receives messages).
            self._ros2_subscriber.create_subscription()
            
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
