"""
Zenoh Server Implementation

Uses Zenoh Queryable for request/response pattern.
"""

import logging
from typing import Coroutine, Optional, Any, Callable, Awaitable

from vyra_base.com.core.types import VyraServer, ProtocolType
from vyra_base.com.core.topic_builder import TopicBuilder
from vyra_base.com.core.exceptions import InterfaceError
from vyra_base.com.transport.t_zenoh.communication.serializer import (
    ZenohSerializer,
    SerializationFormat
)

logger = logging.getLogger(__name__)


class VyraServerImpl(VyraServer):
    """
    Vyra-based server implementation using Queryable.
    
    TODO: Implement using ZenohQueryable from communication layer.
    Pattern: Queryable receives queries and responds with query.reply()
    """
    
    def __init__(
        self,
        name: str,
        topic_builder: TopicBuilder,
        zenoh_session: Any,  # zenoh.Session
        service_type: type,
        response_callback: Optional[Callable[[Any], Coroutine[Any, Any, Any]]] = None,
        **kwargs
    ):
        super().__init__(name, topic_builder, response_callback, ProtocolType.ZENOH, **kwargs)
        self._zenoh_session = zenoh_session
        self.service_type = service_type
        self._queryable = None
        self._serializer = ZenohSerializer()
        self._format = SerializationFormat.JSON
        
    async def initialize(self) -> bool:
        """Initialize Zenoh queryable (server)."""
        try:
            service_name = self.topic_builder.build(self.name)
            self._service_name = service_name
            
            if not self._zenoh_session:
                raise InterfaceError("Zenoh session not initialized")
            
            # Create Zenoh Queryable
            import asyncio
            loop = asyncio.get_event_loop()
            
            def _create_queryable():
                return self._zenoh_session.declare_queryable(
                    service_name,
                    self._handle_query_sync
                )
            
            self._queryable = await loop.run_in_executor(None, _create_queryable)
            
            logger.info(f"✅ ZenohServer initialized: {service_name}")
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to initialize ZenohServer: {e}")
            return False
    
    def _handle_query_sync(self, query: Any):
        """
        Synchronous query handler (called by Zenoh).
        
        Args:
            query: Zenoh Query object
        """
        import asyncio
        try:
            # Get or create event loop
            try:
                loop = asyncio.get_event_loop()
            except RuntimeError:
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
            
            # Run async handler
            loop.run_until_complete(self._handle_query(query))
        except Exception as e:
            logger.error(f"❌ Sync query handler failed: {e}")
    
    async def _handle_query(self, query: Any):
        """
        Internal query handler that wraps user callback.
        
        Args:
            query: Zenoh Query object (has .value and .reply())
        """
        try:
            if not self.response_callback:
                logger.error("Response callback not set for query handling")
                return
            
            # Deserialize request from query.value
            request = self._deserialize_request(query.value.payload)
            
            # Call user callback
            response = await self.response_callback(request)
            
            # Serialize and reply
            response_bytes = self._serialize_response(response)
            query.reply(self._service_name, response_bytes)
            
        except Exception as e:
            logger.error(f"❌ Query handling failed: {e}")
            # Send error reply
            error_response = {'error': str(e)}
            error_bytes = self._serializer.serialize(error_response, format=self._format)
            query.reply(self._service_name, error_bytes)
    
    def _deserialize_request(self, payload: bytes) -> Any:
        """Deserialize request from bytes using Zenoh serializer."""
        return self._serializer.deserialize(payload, format=self._format)
    
    def _serialize_response(self, response: Any) -> bytes:
        """Serialize response to bytes using Zenoh serializer."""
        return self._serializer.serialize(response, format=self._format)
    
    async def serve(self) -> bool:
        """
        Start serving queries.
        
        Returns:
            True on success
        """
        if not self._queryable:
            logger.error("Server not initialized")
            return False
            
        try:
            # Queryable is already listening after initialize()
            return True
            
        except Exception as e:
            logger.error(f"❌ Serve failed: {e}")
            return False
    
    async def cleanup(self):
        """Cleanup Zenoh resources."""
        if self._queryable:
            import asyncio
            loop = asyncio.get_event_loop()
            await loop.run_in_executor(None, self._queryable.undeclare)
            self._queryable = None
            logger.info(f"🔄 ZenohServer cleaned up: {self.name}")
    
    async def shutdown(self) -> None:
        """Shutdown server."""
        await self.cleanup()
        self._initialized = False
