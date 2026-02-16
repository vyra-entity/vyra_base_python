"""
Zenoh Publisher

Implements publishing functionality for Zenoh transport.
"""
from __future__ import annotations

import logging
import zenoh
from typing import Any, Optional
from dataclasses import dataclass

from vyra_base.com.transport.t_zenoh.communication.serializer import (
    ZenohSerializer,
    SerializationFormat
)

logger = logging.getLogger(__name__)


@dataclass
class PublisherInfo:
    """Information about a Zenoh publisher."""
    key_expr: str
    format: SerializationFormat = SerializationFormat.JSON
    encoding: str = "application/json"


class ZenohPublisher:
    """
    Zenoh Publisher wrapper.
    
    Handles message publishing with automatic serialization.
    """
    
    def __init__(
        self,
        publisher_info: PublisherInfo,
        zenoh_publisher: zenoh.Publisher  # zenoh.Publisher
    ):
        """
        Initialize Zenoh publisher.
        
        Args:
            publisher_info: Publisher configuration
            zenoh_publisher: Underlying Zenoh publisher
        """
        self.publisher_info = publisher_info
        self._publisher = zenoh_publisher
        self._serializer = ZenohSerializer()
    
    def publish(self, data: Any) -> None:
        """
        Publish data to Zenoh topic.
        
        Args:
            data: Data to publish (will be serialized)
        """
        try:
            # Serialize data
            payload = self._serializer.serialize(
                data,
                format=self.publisher_info.format
            )
            
            # Publish via Zenoh
            self._publisher.put(payload)
            
            logger.debug(
                f"Published to Zenoh topic '{self.publisher_info.key_expr}': "
                f"{len(payload)} bytes"
            )
            
        except Exception as e:
            logger.error(
                f"Failed to publish to '{self.publisher_info.key_expr}': {e}"
            )
            raise
    
    def delete(self) -> None:
        """Delete publisher and cleanup resources."""
        try:
            if self._publisher:
                self._publisher.undeclare()
                logger.debug(f"Undeclared Zenoh publisher: {self.publisher_info.key_expr}")
        except Exception as e:
            logger.error(f"Error deleting publisher: {e}")
    
    def __del__(self):
        """Cleanup on garbage collection."""
        try:
            self.delete()
        except Exception:
            pass
