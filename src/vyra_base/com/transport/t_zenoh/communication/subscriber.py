"""
Zenoh Subscriber

Implements subscription functionality for Zenoh transport.
"""
from __future__ import annotations

import logging
from typing import Any, Callable, Optional
from dataclasses import dataclass

from vyra_base.com.transport.t_zenoh.communication.serializer import (
    ZenohSerializer,
    SerializationFormat
)

logger = logging.getLogger(__name__)


@dataclass
class SubscriberInfo:
    """Information about a Zenoh subscriber."""
    key_expr: str
    format: SerializationFormat = SerializationFormat.JSON
    reliability: str = "reliable"  # "reliable" or "best_effort"


class ZenohSubscriber:
    """
    Zenoh Subscriber wrapper.
    
    Handles message reception with automatic deserialization.
    """
    
    def __init__(
        self,
        subscriber_info: SubscriberInfo,
        zenoh_subscriber: Any,  # zenoh.Subscriber
        callback: Optional[Callable[[Any], None]] = None
    ):
        """
        Initialize Zenoh subscriber.
        
        Args:
            subscriber_info: Subscriber configuration
            zenoh_subscriber: Underlying Zenoh subscriber
            callback: Callback for incoming messages
        """
        self.subscriber_info = subscriber_info
        self._subscriber = zenoh_subscriber
        self._callback = callback
        self._serializer = ZenohSerializer()
    
    def set_callback(self, callback: Callable[[Any], None]) -> None:
        """
        Set callback for incoming messages.
        
        Args:
            callback: Function to call when message received
        """
        self._callback = callback
    
    def _handle_message(self, sample: Any) -> None:
        """
        Internal handler for Zenoh samples.
        
        Args:
            sample: Zenoh sample (contains payload)
        """
        try:
            # Deserialize payload
            data = self._serializer.deserialize(
                sample.payload,
                format=self.subscriber_info.format
            )
            
            logger.debug(
                f"Received on Zenoh topic '{self.subscriber_info.key_expr}': "
                f"{len(sample.payload)} bytes"
            )
            
            # Call user callback if set
            if self._callback:
                self._callback(data)
            
        except Exception as e:
            logger.error(
                f"Error processing message on '{self.subscriber_info.key_expr}': {e}"
            )
    
    def delete(self) -> None:
        """Delete subscriber and cleanup resources."""
        try:
            if self._subscriber:
                self._subscriber.undeclare()
                logger.debug(f"Undeclared Zenoh subscriber: {self.subscriber_info.key_expr}")
        except Exception as e:
            logger.error(f"Error deleting subscriber: {e}")
    
    def __del__(self):
        """Cleanup on garbage collection."""
        try:
            self.delete()
        except Exception:
            pass
