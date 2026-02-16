"""
Zenoh Publisher Implementation

Wraps ZenohPublisher from communication layer for async publish operations.
"""
import asyncio
import logging
import zenoh
from typing import Optional, Any

from vyra_base.com.core.types import VyraPublisher, ProtocolType
from vyra_base.com.core.topic_builder import TopicBuilder
from vyra_base.com.core.exceptions import InterfaceError
from vyra_base.com.transport.t_zenoh.communication import ZenohPublisher, PublisherInfo

logger = logging.getLogger(__name__)


class VyraPublisherImpl(VyraPublisher):
    """
    Vyra-based publisher implementation.
    
    Wraps ZenohPublisher for consistent async interface.
    Zenoh is async-native, no callback adapter needed.
    """
    
    def __init__(
        self,
        name: str,
        topic_builder: TopicBuilder,
        zenoh_session: Any,  # zenoh.Session
        message_type: type,
        **kwargs
    ):
        super().__init__(name, topic_builder, ProtocolType.ZENOH, **kwargs)
        self._zenoh_session: zenoh.Session = zenoh_session
        self.message_type = message_type
        self._publisher: Optional[ZenohPublisher] = None
        
    async def initialize(self) -> bool:
        """Initialize Zenoh publisher."""
        try:
            topic_name = self.topic_builder.build(self.name)
            
            # Declare Zenoh publisher directly from session
            
            loop = asyncio.get_event_loop()
            
            def _create_publisher():
                if not self._zenoh_session:
                    raise InterfaceError("Zenoh session not initialized")
                return self._zenoh_session.declare_publisher(topic_name)  # type: ignore[union-attr]
            
            zenoh_publisher = await loop.run_in_executor(None, _create_publisher)
            
            publisher_info = PublisherInfo(
                key_expr=topic_name
            )
            
            self._publisher = ZenohPublisher(
                publisher_info,
                zenoh_publisher
            )
            
            logger.info(f"✅ ZenohPublisher initialized: {topic_name}")
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to initialize ZenohPublisher: {e}")
            return False
    
    async def publish(self, message: Any) -> bool:
        """
        Publish message via Zenoh.
        
        Args:
            message: Message instance (msg_type) or dict
            
        Returns:
            True on success
        """
        if not self._publisher:
            logger.error("Publisher not initialized")
            return False
            
        try:
            self._publisher.publish(message)  # type: ignore[attr-defined]
            return True
            
        except Exception as e:
            logger.error(f"❌ Publish failed: {e}")
            return False
    
    async def cleanup(self):
        """Cleanup Zenoh resources."""
        if self._publisher:
            import asyncio
            loop = asyncio.get_event_loop()
            await loop.run_in_executor(None, self._publisher.delete)
            self._publisher = None
            logger.info(f"🔄 ZenohPublisher cleaned up: {self.name}")
    
    async def shutdown(self) -> None:
        """Shutdown publisher."""
        await self.cleanup()
        self._initialized = False
        self._transport_handle = None
        