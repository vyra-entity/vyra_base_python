"""
Zenoh Client Implementation

Uses Zenoh Query (get) for request/response pattern.
"""
import asyncio
import logging
from typing import Coroutine, Optional, Any, Callable, Awaitable

from vyra_base.com.core.types import VyraClient, ProtocolType
from vyra_base.com.core.topic_builder import TopicBuilder
from vyra_base.com.core.exceptions import InterfaceError
from vyra_base.com.transport.t_zenoh.communication.serializer import (
    ZenohSerializer,
    SerializationFormat
)

logger = logging.getLogger(__name__)


class VyraClientImpl(VyraClient):
    """
    Vyra-based client implementation using Query (get).
    
    TODO: Implement using session.get() for queries.
    Pattern: Send query to key, receive replies from queryables
    """
    
    def __init__(
        self,
        name: str,
        topic_builder: TopicBuilder,
        request_callback: Optional[Callable[[Any], Coroutine[Any, Any, Any]]] = None,
        zenoh_session: Any = None,  # zenoh.Session
        service_type: type = None,
        **kwargs
    ):
        super().__init__(name, topic_builder, request_callback, ProtocolType.ZENOH, **kwargs)
        self._zenoh_session = zenoh_session
        self.service_type = service_type
        self._serializer = ZenohSerializer()
        self._format = SerializationFormat.JSON
        
    async def initialize(self) -> bool:
        """Initialize Zenoh client."""
        try:
            # If name is already a fully-qualified topic (contains '/'), use it
            # directly instead of running through topic_builder.build() which
            # only accepts plain function names without '/'.
            if '/' in self.name:
                self._service_name = self.name
            else:
                self._service_name = self.topic_builder.build(self.name)
            # No persistent resources needed for client
            logger.info(f"✅ ZenohClient initialized: {self._service_name}")
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to initialize ZenohClient: {e}")
            return False
    
    async def call(self, request: Any, timeout: float = 5.0) -> Optional[Any]:
        """
        Send query to Zenoh server and await response.
        
        Args:
            request: Request message instance
            timeout: Query timeout in seconds
            
        Returns:
            Response object on success, None on failure
        """
        if not self._zenoh_session:
            logger.error("Client not initialized (no session)")
            return None
            
        try:
            # Serialize request
            request_bytes = self._serialize_request(request)
            
            # Send query using session.get()
            # Note: This is a synchronous Zenoh call wrapped in async
            
            loop = asyncio.get_event_loop()
            
            if not self._zenoh_session:
                raise InterfaceError("Zenoh session not initialized")
            
            def _do_query():
                if not self._zenoh_session:
                    raise InterfaceError("Zenoh session not initialized")
                
                replies = self._zenoh_session.get(
                    self._service_name,
                    timeout=timeout,
                    payload=request_bytes
                )
                # Get first reply
                for reply in replies:
                    return reply.ok.payload
                return None
            
            payload = await loop.run_in_executor(None, _do_query)
            
            if payload is None:
                logger.warning(f"No reply received from {self._service_name}")
                return None
            
            # Deserialize response
            response = self._deserialize_response(payload)
            
            # Optional: Call request_callback with response
            if self.request_callback:
                await self.request_callback(response)
            
            return response
            
        except Exception as e:
            logger.error(f"❌ Call failed: {e}")
            return None
    
    def _serialize_request(self, request: Any) -> bytes:
        """Serialize request to bytes using Zenoh serializer."""
        return self._serializer.serialize(request, format=self._format)
    
    def _deserialize_response(self, payload: bytes) -> Any:
        """Deserialize response from bytes using Zenoh serializer."""
        return self._serializer.deserialize(payload, format=self._format)
    
    async def cleanup(self):
        """Cleanup resources (none for stateless client)."""
        logger.info(f"🔄 ZenohClient cleaned up: {self.name}")
    
    async def shutdown(self) -> None:
        """Shutdown client."""
        await self.cleanup()
        self._initialized = False
