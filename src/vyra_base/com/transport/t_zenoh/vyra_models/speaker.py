"""
Zenoh Speaker Implementation

Concrete implementation of VyraSpeaker for Zenoh Pub/Sub communication.
"""
from __future__ import annotations

import logging
from typing import Any, Callable, Optional

from vyra_base.com.core.types import VyraSpeaker, ProtocolType
from vyra_base.com.core.exceptions import InterfaceError
from vyra_base.com.transport.t_zenoh.communication import (
    ZenohPublisher,
    ZenohSubscriber,
    ZenohSerializer,
)
from vyra_base.com.transport.t_zenoh.communication.publisher import PublisherInfo
from vyra_base.com.transport.t_zenoh.communication.subscriber import SubscriberInfo
from vyra_base.com.transport.t_zenoh.communication.serializer import SerializationFormat
from vyra_base.com.transport.t_zenoh.session import ZenohSession

logger = logging.getLogger(__name__)


class ZenohSpeaker(VyraSpeaker):
    """
    Zenoh-specific implementation of VyraSpeaker using Pub/Sub.
    
    Wraps ZenohPublisher for publishing and ZenohSubscriber for subscribing.
    """
    
    def __init__(
        self,
        name: str,
        session: Optional[ZenohSession] = None,
        format: SerializationFormat = SerializationFormat.JSON,
        is_publisher: bool = True,
        **kwargs
    ):
        """
        Initialize Zenoh speaker.
        
        Args:
            name: Topic name (key expression)
            session: Zenoh session
            format: Serialization format
            is_publisher: Whether this is a publisher (True) or subscriber (False)
            **kwargs: Additional parameters
        """
        super().__init__(name, ProtocolType.ZENOH, **kwargs)
        self.session = session
        self.format = format
        self.is_publisher = is_publisher
        self._publisher: Optional[ZenohPublisher] = None
        self._subscriber: Optional[ZenohSubscriber] = None
        self._last_message: Any = None
    
    async def initialize(self) -> bool:
        """
        Initialize Zenoh topic.
        
        Creates either publisher or subscriber based on is_publisher flag.
        """
        if self._initialized:
            logger.warning(f"ZenohSpeaker '{self.name}' already initialized")
            return True
        
        if not self.session or not self.session.is_open:
            raise InterfaceError("Zenoh session is required and must be open")
        
        try:
            if self.is_publisher:
                # Publisher side
                logger.info(f"🔧 Creating Zenoh publisher: {self.name}")
                
                zenoh_pub = self.session.declare_publisher(self.name)
                
                publisher_info = PublisherInfo(
                    key_expr=self.name,
                    format=self.format
                )
                
                self._publisher = ZenohPublisher(
                    publisher_info=publisher_info,
                    zenoh_publisher=zenoh_pub
                )
                
                logger.info(f"✅ Zenoh publisher created: {self.name}")
            else:
                # Subscriber side
                logger.info(f"🔧 Creating Zenoh subscriber: {self.name}")
                
                subscriber_info = SubscriberInfo(
                    key_expr=self.name,
                    format=self.format
                )
                
                # Note: callback set later via listen()
                zenoh_sub = self.session.declare_subscriber(
                    self.name,
                    lambda sample: None  # Placeholder
                )
                
                self._subscriber = ZenohSubscriber(
                    subscriber_info=subscriber_info,
                    zenoh_subscriber=zenoh_sub
                )
                
                logger.info(f"✅ Zenoh subscriber created: {self.name}")
            
            self._initialized = True
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to initialize ZenohSpeaker '{self.name}': {e}")
            raise InterfaceError(f"Failed to initialize ZenohSpeaker: {e}")
    
    async def shutdown(self) -> None:
        """Shutdown and cleanup Zenoh topic resources."""
        if not self._initialized:
            return
        
        logger.info(f"🛑 Shutting down ZenohSpeaker: {self.name}")
        
        # Cleanup publisher/subscriber
        if self._publisher:
            self._publisher.delete()
            self._publisher = None
        
        if self._subscriber:
            self._subscriber.delete()
            self._subscriber = None
        
        self._initialized = False
        logger.info(f"✅ ZenohSpeaker '{self.name}' shutdown complete")
    
    async def shout(self, message: Any) -> bool:
        """
        Publish message to Zenoh topic.
        
        Args:
            message: Message data to publish
            
        Returns:
            bool: True if published successfully
            
        Raises:
            InterfaceError: If not initialized or publisher not available
        """
        if not self._initialized:
            raise InterfaceError(f"ZenohSpeaker '{self.name}' not initialized")
        
        if not self._publisher:
            raise InterfaceError(
                f"Cannot publish on ZenohSpeaker '{self.name}': no publisher. "
                "This is likely a subscriber-only speaker."
            )
        
        try:
            logger.debug(f"📢 Publishing to Zenoh topic: {self.name}")
            
            # Publish via ZenohPublisher
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
        Subscribe to Zenoh topic and listen for messages.
        
        Args:
            callback: Callback function to process incoming messages
            **kwargs: Additional parameters
            
        Raises:
            InterfaceError: If not initialized or subscriber not available
        """
        if not self._initialized:
            raise InterfaceError(f"ZenohSpeaker '{self.name}' not initialized")
        
        if not self._subscriber:
            raise InterfaceError(
                f"Cannot subscribe on ZenohSpeaker '{self.name}': no subscriber. "
                "This is likely a publisher-only speaker."
            )
        
        try:
            logger.info(f"👂 Setting up Zenoh subscription callback: {self.name}")
            
            # Set callback on ZenohSubscriber
            self._subscriber.set_callback(callback)
            
            # Re-declare subscriber with actual callback
            if self.session and self.session.is_open:
                self._subscriber._subscriber.undeclare()
                
                zenoh_sub = self.session.declare_subscriber(
                    self.name,
                    self._subscriber._handle_message
                )
                self._subscriber._subscriber = zenoh_sub
            
            logger.info(f"✅ Zenoh subscription active: {self.name}")
            
        except Exception as e:
            logger.error(f"❌ Failed to set up subscription on '{self.name}': {e}")
            raise InterfaceError(f"Subscribe failed: {e}")
    
    def get_publisher(self) -> Optional[ZenohPublisher]:
        """Get the underlying Zenoh publisher (publisher-side only)."""
        return self._publisher
    
    def get_subscriber(self) -> Optional[ZenohSubscriber]:
        """Get the underlying Zenoh subscriber (subscriber-side only)."""
        return self._subscriber
