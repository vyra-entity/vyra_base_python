"""
Zenoh Queryable (Service Server)

Implements request-response server functionality for Zenoh transport.
"""
from __future__ import annotations

import asyncio
import logging
from typing import Any, Callable, Optional
from dataclasses import dataclass

from vyra_base.com.transport.t_zenoh.communication.serializer import (
    ZenohSerializer,
    SerializationFormat
)

logger = logging.getLogger(__name__)


@dataclass
class QueryableInfo:
    """Information about a Zenoh queryable."""
    key_expr: str
    format: SerializationFormat = SerializationFormat.JSON
    complete: bool = True  # Whether queryable provides complete responses


class ZenohQueryable:
    """
    Zenoh Queryable wrapper for service servers.
    
    Handles query/reply patterns with automatic serialization.
    """
    
    def __init__(
        self,
        queryable_info: QueryableInfo,
        zenoh_queryable: Any,  # zenoh.Queryable
        callback: Optional[Callable[[Any], Any]] = None
    ):
        """
        Initialize Zenoh queryable.
        
        Args:
            queryable_info: Queryable configuration
            zenoh_queryable: Underlying Zenoh queryable
            callback: Async callback for query handling
        """
        self.queryable_info = queryable_info
        self._queryable = zenoh_queryable
        self._callback = callback
        self._serializer = ZenohSerializer()
    
    def set_callback(self, callback: Callable[[Any], Any]) -> None:
        """
        Set callback for handling queries.
        
        Args:
            callback: Async function that processes request and returns response
        """
        self._callback = callback
    
    async def _handle_query(self, query: Any) -> None:
        """
        Internal handler for Zenoh queries.
        
        Args:
            query: Zenoh query object
        """
        try:
            # Deserialize request payload
            if query.payload:
                request_data = self._serializer.deserialize(
                    query.payload,
                    format=self.queryable_info.format
                )
            else:
                request_data = {}
            
            logger.debug(
                f"Received query on '{self.queryable_info.key_expr}'"
            )
            
            # Process request with user callback
            if self._callback:
                if asyncio.iscoroutinefunction(self._callback):
                    response_data = await self._callback(request_data)
                else:
                    response_data = self._callback(request_data)
            else:
                logger.warning(f"No callback set for queryable '{self.queryable_info.key_expr}'")
                response_data = {"error": "No handler available"}
            
            # Serialize response
            response_payload = self._serializer.serialize(
                response_data,
                format=self.queryable_info.format
            )
            
            # Send reply
            query.reply(response_payload)
            
            logger.debug(
                f"Replied to query on '{self.queryable_info.key_expr}': "
                f"{len(response_payload)} bytes"
            )
            
        except Exception as e:
            logger.error(
                f"Error handling query on '{self.queryable_info.key_expr}': {e}"
            )
            
            # Send error reply
            error_payload = self._serializer.serialize(
                {"error": str(e)},
                format=self.queryable_info.format
            )
            query.reply(error_payload)
    
    def delete(self) -> None:
        """Delete queryable and cleanup resources."""
        try:
            if self._queryable:
                self._queryable.undeclare()
                logger.debug(f"Undeclared Zenoh queryable: {self.queryable_info.key_expr}")
        except Exception as e:
            logger.error(f"Error deleting queryable: {e}")
    
    def __del__(self):
        """Cleanup on garbage collection."""
        try:
            self.delete()
        except Exception:
            pass
