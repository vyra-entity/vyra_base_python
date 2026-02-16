"""
Zenoh Subscriber Implementation

Wraps ZenohSubscriber from communication layer for async callback handling.
"""
import asyncio
import logging
from typing import Optional, Any, Callable, Awaitable

from vyra_base.com.core.types import VyraSubscriber, ProtocolType
from vyra_base.com.core.topic_builder import TopicBuilder
from vyra_base.com.core.exceptions import InterfaceError
from vyra_base.com.transport.t_zenoh.communication import ZenohSubscriber
from vyra_base.com.transport.t_zenoh.communication.subscriber import SubscriberInfo
from vyra_base.com.transport.t_zenoh.communication.serializer import ZenohSerializer, SerializationFormat

logger = logging.getLogger(__name__)


class VyraSubscriberImpl(VyraSubscriber):
    """
    Vyra-based subscriber implementation.
    
    Wraps ZenohSubscriber for consistent async interface.
    Zenoh callbacks are async-native.
    """
    
    def __init__(
        self,
        name: str,
        topic_builder: TopicBuilder,
        subscriber_callback: Callable[[Any], Awaitable[None]],
        zenoh_session: Any,  # zenoh.Session
        message_type: type,
        **kwargs
    ):
        super().__init__(name, topic_builder, subscriber_callback, ProtocolType.ZENOH, **kwargs)
        self._zenoh_session = zenoh_session
        self.message_type = message_type
        self._subscriber: Optional[ZenohSubscriber] = None
        
    async def initialize(self) -> bool:
        """Initialize Zenoh subscriber."""
        try:
            topic_name = self.topic_builder.build(self.name)
            
            # Create async wrapper for callback
            def _callback_wrapper(sample):
                if not self.subscriber_callback:
                    logger.warning("No subscriber callback defined")
                    return
                
                try:
                    loop = asyncio.get_event_loop()
                except RuntimeError:
                    loop = asyncio.new_event_loop()
                    asyncio.set_event_loop(loop)
                
                # Deserialize and call user callback
                serializer = ZenohSerializer()
                data = serializer.deserialize(sample.payload, format=SerializationFormat.JSON)
                asyncio.create_task(self.subscriber_callback(data))
            
            # Declare Zenoh subscriber directly from session
            loop = asyncio.get_event_loop()
            
            def _create_subscriber():
                if not self._zenoh_session:
                    raise InterfaceError("Zenoh session not initialized")
                return self._zenoh_session.declare_subscriber(topic_name, _callback_wrapper)  # type: ignore[union-attr]
            
            zenoh_subscriber = await loop.run_in_executor(None, _create_subscriber)
            
            subscriber_info = SubscriberInfo(
                key_expr=topic_name
            )
            
            self._subscriber = ZenohSubscriber(
                subscriber_info,
                zenoh_subscriber,
                None  # Callback already set in declare_subscriber
            )
            
            logger.info(f"✅ ZenohSubscriber initialized: {topic_name}")
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to initialize ZenohSubscriber: {e}")
            return False
    
    async def subscribe(self) -> bool:
        """
        Start subscribing to Zenoh topic.
        
        Note: Subscriber is already active after initialize().
        This method exists for API compatibility.
        
        Returns:
            True on success
        """
        if not self._subscriber:
            logger.error("Subscriber not initialized")
            return False
            
        try:
            # Subscriber is already active after initialize()
            # No additional work needed
            return True
            
        except Exception as e:
            logger.error(f"❌ Subscribe failed: {e}")
            return False
    
    async def cleanup(self):
        """Cleanup Zenoh resources."""
        if self._subscriber:
            import asyncio
            loop = asyncio.get_event_loop()
            await loop.run_in_executor(None, self._subscriber.delete)
            self._subscriber = None
            logger.info(f"🔄 ZenohSubscriber cleaned up: {self.name}")
    
    async def shutdown(self) -> None:
        """Shutdown subscriber."""
        await self.cleanup()
        self._initialized = False
