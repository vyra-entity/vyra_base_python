"""
Zenoh Callable Implementation

Concrete implementation of VyraCallable for Zenoh Query/Reply communication.
"""
from __future__ import annotations

import asyncio
import logging
from typing import Any, Callable, Optional

from vyra_base.com.core.types import VyraCallable, ProtocolType
from vyra_base.com.core.exceptions import InterfaceError
from vyra_base.com.transport.t_zenoh.communication import (
    ZenohQueryable,
    ZenohQueryClient,
)
from vyra_base.com.transport.t_zenoh.communication.queryable import QueryableInfo
from vyra_base.com.transport.t_zenoh.communication.query_client import QueryClientInfo
from vyra_base.com.transport.t_zenoh.communication.serializer import SerializationFormat
from vyra_base.com.transport.t_zenoh.session import ZenohSession

logger = logging.getLogger(__name__)


class ZenohCallable(VyraCallable):
    """
    Zenoh-specific implementation of VyraCallable using Query/Reply.
    
    Server side uses ZenohQueryable, client side uses ZenohQueryClient.
    """
    
    def __init__(
        self,
        name: str,
        session: Optional[ZenohSession] = None,
        callback: Optional[Callable[[Any], Any]] = None,
        format: SerializationFormat = SerializationFormat.JSON,
        is_server: bool = True,
        timeout: float = 5.0,
        **kwargs
    ):
        """
        Initialize Zenoh callable.
        
        Args:
            name: Service name (key expression)
            session: Zenoh session
            callback: Async callback for request handling (server-side)
            format: Serialization format
            is_server: Whether this is a server (True) or client (False)
            timeout: Default timeout for client calls in seconds
            **kwargs: Additional parameters
        """
        super().__init__(name, callback, ProtocolType.ZENOH, **kwargs)
        self.session = session
        self.format = format
        self.is_server = is_server
        self.timeout = timeout
        self._queryable: Optional[ZenohQueryable] = None
        self._query_client: Optional[ZenohQueryClient] = None
    
    async def initialize(self) -> bool:
        """
        Initialize Zenoh queryable or client.
        
        Creates either queryable (server) or query client based on is_server flag.
        """
        if self._initialized:
            logger.warning(f"ZenohCallable '{self.name}' already initialized")
            return True
        
        if not self.session or not self.session.is_open:
            raise InterfaceError("Zenoh session is required and must be open")
        
        try:
            if self.is_server:
                # Server side (Queryable)
                logger.info(f"🔧 Creating Zenoh queryable: {self.name}")
                
                if not self._callback:
                    raise InterfaceError("Callback is required for Zenoh queryable")
                
                queryable_info = QueryableInfo(
                    key_expr=self.name,
                    format=self.format
                )
                
                # Create async query handler wrapper
                async def query_handler(query):
                    await self._queryable._handle_query(query)  # pyright: ignore[reportOptionalMemberAccess]
                zenoh_queryable = self.session.declare_queryable(
                    self.name,
                    query_handler
                )
                
                self._queryable = ZenohQueryable(
                    queryable_info=queryable_info,
                    zenoh_queryable=zenoh_queryable,
                    callback=self._callback
                )
                
                logger.info(f"✅ Zenoh queryable created: {self.name}")
            else:
                # Client side (Query Client)
                logger.info(f"🔧 Creating Zenoh query client: {self.name}")
                
                query_client_info = QueryClientInfo(
                    key_expr=self.name,
                    format=self.format,
                    timeout_ms=int(self.timeout * 1000)
                )
                
                self._query_client = ZenohQueryClient(
                    query_client_info=query_client_info,
                    zenoh_session=self.session.session
                )
                
                logger.info(f"✅ Zenoh query client created: {self.name}")
            
            self._initialized = True
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to initialize ZenohCallable '{self.name}': {e}")
            raise InterfaceError(f"Failed to initialize ZenohCallable: {e}")
    
    async def shutdown(self) -> None:
        """Shutdown and cleanup Zenoh queryable/client resources."""
        if not self._initialized:
            return
        
        logger.info(f"🛑 Shutting down ZenohCallable: {self.name}")
        
        # Cleanup queryable/client
        if self._queryable:
            self._queryable.delete()
            self._queryable = None
        
        if self._query_client:
            self._query_client = None
        
        self._initialized = False
        logger.info(f"✅ ZenohCallable '{self.name}' shutdown complete")
    
    async def call(self, request: Any, timeout: Optional[float] = None) -> Any:
        """
        Send request to Zenoh queryable and wait for response.
        
        Args:
            request: Request data
            timeout: Optional timeout override (seconds)
            
        Returns:
            Response data
            
        Raises:
            InterfaceError: If not initialized or client not available
        """
        if not self._initialized:
            raise InterfaceError(f"ZenohCallable '{self.name}' not initialized")
        
        if not self._query_client:
            raise InterfaceError(
                f"Cannot call ZenohCallable '{self.name}': no query client. "
                "This is likely a server-only callable."
            )
        
        try:
            logger.debug(f"📞 Calling Zenoh service: {self.name}")
            
            # Call via ZenohQueryClient
            response = await self._query_client.call(
                request,
                timeout=timeout or self.timeout
            )
            
            logger.debug(f"✅ Response received from: {self.name}")
            return response
            
        except Exception as e:
            logger.error(f"❌ Failed to call '{self.name}': {e}")
            raise InterfaceError(f"Call failed: {e}")
    
    def set_callback(self, callback: Callable[[Any], Any]) -> None:
        """
        Set or update callback for server-side queryable.
        
        Args:
            callback: Async callback function
            
        Raises:
            InterfaceError: If not a server-side callable
        """
        if not self.is_server:
            raise InterfaceError("Cannot set callback on client-side callable")
        
        self._callback = callback
        
        if self._queryable:
            self._queryable.set_callback(callback)
    
    def get_queryable(self) -> Optional[ZenohQueryable]:
        """Get the underlying Zenoh queryable (server-side only)."""
        return self._queryable
    
    def get_query_client(self) -> Optional[ZenohQueryClient]:
        """Get the underlying Zenoh query client (client-side only)."""
        return self._query_client
