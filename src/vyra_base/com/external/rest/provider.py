"""
REST Protocol Provider

HTTP REST API integration using aiohttp.
Provides callable pattern for REST endpoints.
"""
import logging
from typing import Any, Callable, Optional, Dict
import asyncio

from vyra_base.com.core.types import (
    ProtocolType,
    VyraCallable,
    VyraSpeaker,
    VyraJob,
)
from vyra_base.com.core.exceptions import (
    ProtocolUnavailableError,
    ProviderError,
)
from vyra_base.com.providers.protocol_provider import AbstractProtocolProvider

logger = logging.getLogger(__name__)

# Check if aiohttp is available
try:
    import aiohttp
    from aiohttp import web
    REST_AVAILABLE = True
except ImportError:
    REST_AVAILABLE = False
    aiohttp = None
    web = None
    logger.warning(
        "⚠️ REST not available. Install with: pip install aiohttp"
    )


class RestProvider(AbstractProtocolProvider):
    """
    Protocol provider for REST HTTP APIs.
    
    Features:
    - HTTP methods (GET, POST, PUT, DELETE, PATCH)
    - JSON request/response
    - Async request handlers
    - CORS support
    - TLS/HTTPS support
    
    Example:
        >>> # Server side
        >>> provider = RestProvider(
        ...     host="0.0.0.0",
        ...     port=8080,
        ...     mode="server"
        ... )
        >>> await provider.initialize()
        >>> 
        >>> # Register endpoint
        >>> async def handle_get_data(request):
        ...     return {"status": "ok", "data": [1, 2, 3]}
        >>> 
        >>> callable = await provider.create_callable(
        ...     "get_data",
        ...     callback=handle_get_data,
        ...     method="GET"
        ... )
        >>> 
        >>> # Client side
        >>> client_provider = RestProvider(
        ...     base_url="http://localhost:8080",
        ...     mode="client"
        ... )
        >>> await client_provider.initialize()
        >>> result = await client_provider.call("get_data", method="GET")
    """
    
    def __init__(
        self,
        protocol: ProtocolType = ProtocolType.REST,
        host: str = "0.0.0.0",
        port: int = 8080,
        base_url: Optional[str] = None,
        mode: str = "server",  # "server" or "client"
        **kwargs
    ):
        """
        Initialize REST provider.
        
        Args:
            protocol: Protocol type (must be REST)
            host: Server bind address
            port: Server port
            base_url: Base URL for client mode
            mode: Operation mode ("server" or "client")
            **kwargs: Additional parameters (ssl_context, cors, etc.)
        """
        super().__init__(protocol)
        self.host = host
        self.port = port
        self.base_url = base_url or f"http://{host}:{port}"
        self.mode = mode.lower()
        
        self._app: Optional[Any] = None  # web.Application when available
        self._runner: Optional[Any] = None  # web.AppRunner when available
        self._session: Optional[Any] = None  # aiohttp.ClientSession when available
        self._routes: Dict[str, Any] = {}
        
        if self.mode not in ["server", "client"]:
            raise ValueError(f"Invalid mode: {mode}. Must be 'server' or 'client'")
    
    async def check_availability(self) -> bool:
        """
        Check if aiohttp is available.
        
        Returns:
            bool: True if aiohttp can be imported
        """
        if not REST_AVAILABLE:
            logger.warning("❌ REST not available - install aiohttp")
            return False
        
        logger.debug("✅ REST available")
        return True
    
    async def initialize(self, config: Optional[Dict[str, Any]] = None) -> bool:
        """
        Initialize REST provider.
        
        Args:
            config: Optional configuration overrides
            
        Returns:
            bool: True if initialization successful
            
        Raises:
            ProviderError: If initialization fails
        """
        if not await self.check_availability():
            raise ProtocolUnavailableError("REST not available")
        
        try:
            if self.mode == "server":
                # Create aiohttp application
                self._app = web.Application()
                
                # Start server
                self._runner = web.AppRunner(self._app)
                await self._runner.setup()
                site = web.TCPSite(self._runner, self.host, self.port)
                await site.start()
                
                logger.info(f"✅ REST server started on {self.host}:{self.port}")
                
            elif self.mode == "client":
                # Create aiohttp client session
                self._session = aiohttp.ClientSession()
                logger.info(f"✅ REST client initialized for {self.base_url}")
            
            self._initialized = True
            return True
            
        except Exception as e:
            raise ProviderError(f"REST initialization failed: {e}")
    
    async def shutdown(self) -> None:
        """Shutdown REST provider."""
        try:
            if self._runner:
                await self._runner.cleanup()
                logger.info("✅ REST server shutdown complete")
            
            if self._session:
                await self._session.close()
                logger.info("✅ REST client shutdown complete")
        except Exception as e:
            logger.error(f"❌ REST shutdown error: {e}")
        
        self._initialized = False
        self._app = None
        self._runner = None
        self._session = None
    
    async def create_callable(
        self,
        name: str,
        callback: Optional[Callable] = None,
        **kwargs
    ) -> VyraCallable:
        """
        Create REST Callable (HTTP endpoint).
        
        Args:
            name: Endpoint path (e.g., "api/data")
            callback: Server-side handler function
            **kwargs: Additional parameters (method, cors, etc.)
            
        Returns:
            VyraCallable: REST callable instance
            
        Example:
            >>> # Server-side endpoint
            >>> async def get_users(request):
            ...     return {"users": ["alice", "bob"]}
            >>> 
            >>> callable = await provider.create_callable(
            ...     "api/users",
            ...     callback=get_users,
            ...     method="GET"
            ... )
            >>> 
            >>> # Client-side call
            >>> result = await callable.call(method="GET")
        """
        if not self._initialized:
            raise ProviderError("Provider not initialized")
        
        from vyra_base.com.external.rest.callable import RestCallable
        
        callable_instance = RestCallable(
            name=name,
            rest_provider=self,
            mode=self.mode,
            callback=callback,
            **kwargs
        )
        
        await callable_instance.initialize()
        return callable_instance
    
    async def create_speaker(
        self,
        name: str,
        callback: Optional[Callable] = None,
        **kwargs
    ) -> VyraSpeaker:
        """
        Create REST Speaker.
        
        Note:
            REST doesn't support pub/sub natively.
            Consider WebSocket or Server-Sent Events (SSE) instead.
        """
        raise NotImplementedError(
            "REST Speaker not supported. "
            "Use WebSocket provider for pub/sub or consider SSE endpoints."
        )
    
    async def create_job(
        self,
        name: str,
        callback: Optional[Callable] = None,
        **kwargs
    ) -> VyraJob:
        """
        Create REST Job (long-polling or webhook).
        
        Note:
            REST jobs use long-polling or webhook patterns.
            For true async jobs, consider Redis or gRPC streaming.
        """
        raise NotImplementedError(
            "REST Job not yet implemented. "
            "Consider Redis Streams or WebSocket for async job patterns."
        )
    
    def get_app(self) -> web.Application:
        """
        Get underlying aiohttp Application (server mode).
        
        Returns:
            aiohttp.web.Application instance
            
        Raises:
            ProviderError: If provider not initialized or not in server mode
        """
        if not self._initialized or not self._app:
            raise ProviderError("Server not initialized")
        
        return self._app
    
    def get_session(self) -> aiohttp.ClientSession:
        """
        Get underlying aiohttp ClientSession (client mode).
        
        Returns:
            aiohttp.ClientSession instance
            
        Raises:
            ProviderError: If provider not initialized or not in client mode
        """
        if not self._initialized or not self._session:
            raise ProviderError("Client not initialized")
        
        return self._session
