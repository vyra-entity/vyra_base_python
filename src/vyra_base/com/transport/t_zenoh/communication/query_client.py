"""
Zenoh Query Client

Implements request-response client functionality for Zenoh transport.
"""
from __future__ import annotations

import asyncio
import logging
from typing import Any, Optional
from dataclasses import dataclass

import zenoh

from vyra_base.com.transport.t_zenoh.communication.serializer import (
    ZenohSerializer,
    SerializationFormat
)

logger = logging.getLogger(__name__)


@dataclass
class QueryClientInfo:
    """Information about a Zenoh query client."""
    key_expr: str
    format: SerializationFormat = SerializationFormat.JSON
    timeout_ms: int = 5000


class ZenohQueryClient:
    """
    Zenoh Query Client wrapper.
    
    Handles sending queries and receiving replies with automatic serialization.
    """
    
    def __init__(
        self,
        query_client_info: QueryClientInfo,
        zenoh_session: zenoh.Session  # zenoh.Session
    ):
        """
        Initialize Zenoh query client.
        
        Args:
            query_client_info: Client configuration
            zenoh_session: Zenoh session for sending queries
        """
        self.query_client_info = query_client_info
        self._session = zenoh_session
        self._serializer = ZenohSerializer()
    
    async def call(self, request: Any, timeout: Optional[float] = None) -> Any:
        """
        Send query and wait for reply.
        
        Args:
            request: Request data (will be serialized)
            timeout: Timeout in seconds (overrides default)
            
        Returns:
            Response data (deserialized)
            
        Raises:
            TimeoutError: If no reply received within timeout
            Exception: If query fails
        """
        try:
            # Serialize request
            payload = self._serializer.serialize(
                request,
                format=self.query_client_info.format
            )
            
            logger.debug(
                f"Sending query to '{self.query_client_info.key_expr}': "
                f"{len(payload)} bytes"
            )
            
            # Determine timeout
            timeout_ms = (
                int(timeout * 1000) if timeout 
                else self.query_client_info.timeout_ms
            )
            
            # Send query (blocking, wrap in executor)
            loop = asyncio.get_event_loop()
            replies = await loop.run_in_executor(
                None,
                lambda: self._session.get(
                    self.query_client_info.key_expr,
                    payload=payload,
                    timeout=timeout_ms
                )
            )
            
            # Process replies (get first one)
            for reply in replies:
                if reply.is_ok:  # type: ignore[attr-defined]
                    response_data = self._serializer.deserialize(
                        reply.ok.payload,
                        format=self.query_client_info.format
                    )
                    
                    logger.debug(
                        f"Received reply from '{self.query_client_info.key_expr}': "
                        f"{len(reply.ok.payload)} bytes"  # type: ignore[union-attr]
                    )
                    
                    return response_data
            
            # No replies received
            raise TimeoutError(
                f"No reply received for query to '{self.query_client_info.key_expr}'"
            )
            
        except Exception as e:
            logger.error(
                f"Query to '{self.query_client_info.key_expr}' failed: {e}"
            )
            raise
