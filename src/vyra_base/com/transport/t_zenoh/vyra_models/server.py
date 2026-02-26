"""
Zenoh Server Implementation

Uses Zenoh Queryable for request/response pattern.
"""
import asyncio
import concurrent.futures
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


class ZenohProxyRequest(dict):
    """
    Dict subclass that also supports attribute access.

    Zenoh deserialises requests as plain Python dicts, but ROS2-style callbacks
    access fields via ``request.key`` / ``request.value`` etc.  This thin
    wrapper makes both ``request["key"]`` and ``request.key`` work.
    """

    def __getattr__(self, name: str) -> Any:
        try:
            return self[name]
        except KeyError:
            raise AttributeError(f"Request has no attribute '{name}'")

    def __setattr__(self, name: str, value: Any) -> None:
        self[name] = value


class ZenohProxyResponse:
    """
    Proxy response object for ROS2-style callbacks (request, response) pattern.
    
    Some callbacks (e.g. get_interface_list) follow the ROS2 pattern where they
    receive a mutable response object, set attributes on it, and return None.
    This proxy captures those attribute assignments and converts them to a dict
    so Zenoh can serialize and reply.
    """

    def to_dict(self) -> dict:
        """Return all captured response attributes as a plain dict."""
        return dict(vars(self))


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
        service_type: type | None = None,
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
            service_name = self.topic_builder.build(self.name, namespace=self.namespace, subsection=self.subsection)
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
        Synchronous query handler (called by Zenoh in a background thread).
        
        Args:
            query: Zenoh Query object
        """
        
        try:
            try:
                loop = asyncio.get_event_loop()
            except RuntimeError:
                loop = None

            if loop is not None and loop.is_running():
                # Called from a thread while the main loop is running — schedule safely
                future = asyncio.run_coroutine_threadsafe(self._handle_query(query), loop)
                future.result(timeout=10)
            else:
                # No running loop — create one or reuse
                if loop is None or loop.is_closed():
                    loop = asyncio.new_event_loop()
                    asyncio.set_event_loop(loop)
                loop.run_until_complete(self._handle_query(query))
        except Exception as e:
            logger.error(f"❌ Sync query handler failed: {e}")
    
    async def _handle_query(self, query: Any):
        """
        Internal query handler that wraps user callback.

        Supports two callback conventions:
        1. ``callback(request) → response_dict``  — returns a value directly
        2. ``callback(request, response)``  — ROS2-style: mutates a response object,
           returns ``None``; the proxy response's attributes are used as the reply.

        Args:
            query: Zenoh Query object (has .value and .reply())
        """
        try:
            if not self.response_callback:
                logger.error("Response callback not set for query handling")
                return

            # Deserialize request from query.payload (zenoh 1.x API)
            request = self._deserialize_request(query.payload)

            # Create proxy for ROS2-style callbacks that write to a response object
            proxy_response = ZenohProxyResponse()

            # Call user callback — support both (request,) and (request, response) signatures
            try:
                result = await self.response_callback(request, proxy_response)
            except TypeError:
                # Fallback: callback only accepts a single argument
                result = await self.response_callback(request)

            # Determine what to send back:
            # • If the callback returned a non-None, non-bool value → use it directly
            #   (booleans indicate ROS2-style success/failure, not the response payload)
            # • Otherwise fall back to whatever was written into proxy_response
            if result is not None and not isinstance(result, bool):
                response_data = result
            else:
                response_data = proxy_response.to_dict()
                if not response_data:
                    # Callback returned None/bool and wrote nothing — respond with bool status
                    response_data = {"success": bool(result)} if result is not None else {}

            # Serialize and reply
            response_bytes = self._serialize_response(response_data)
            query.reply(self._service_name, response_bytes)

        except Exception as e:
            logger.error(f"❌ Query handling failed: {e}")
            # Send error reply
            error_response = {'error': str(e)}
            error_bytes = self._serializer.serialize(error_response, format=self._format)
            query.reply(self._service_name, error_bytes)
    
    def _deserialize_request(self, payload: bytes) -> Any:
        """Deserialize request from bytes using Zenoh serializer.
        
        Returns a ZenohProxyRequest (dict subclass with attribute access) so that
        ROS2-style callbacks can access fields via ``request.key`` or ``request["key"]``.
        """
        raw = self._serializer.deserialize(payload, format=self._format)
        if isinstance(raw, dict):
            return ZenohProxyRequest(raw)
        return raw
    
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
