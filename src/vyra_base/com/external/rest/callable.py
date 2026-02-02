"""
REST Callable Implementation

HTTP REST endpoint communication via aiohttp.
"""
import asyncio
import logging

from aiohttp import web
from typing import Any, Callable, Optional, Dict

from vyra_base.com.core.types import VyraCallable, ProtocolType
from vyra_base.com.core.exceptions import CallableError

logger = logging.getLogger(__name__)


class RestCallable(VyraCallable):
    """
    REST Callable for HTTP endpoint communication.
    
    Features:
    - HTTP methods (GET, POST, PUT, DELETE, PATCH)
    - JSON request/response
    - Query parameters
    - Path parameters
    - Headers
    
    Example:
        >>> # Server side
        >>> async def handle_post(request):
        ...     data = await request.json()
        ...     return {"result": data["value"] * 2}
        >>> 
        >>> callable = RestCallable(
        ...     "api/calculate",
        ...     rest_provider=server_provider,
        ...     mode="server",
        ...     callback=handle_post,
        ...     method="POST"
        ... )
        >>> await callable.initialize()
        >>> 
        >>> # Client side
        >>> callable = RestCallable(
        ...     "api/calculate",
        ...     rest_provider=client_provider,
        ...     mode="client",
        ...     method="POST"
        ... )
        >>> result = await callable.call({"value": 21})
    """
    
    def __init__(
        self,
        name: str,
        rest_provider: Any,
        mode: str,
        callback: Optional[Callable] = None,
        method: str = "POST",
        **kwargs
    ):
        """
        Initialize REST callable.
        
        Args:
            name: Endpoint path (e.g., "api/data")
            rest_provider: RestProvider instance
            mode: Operation mode ("server" or "client")
            callback: Server-side handler function
            method: HTTP method (GET, POST, PUT, DELETE, PATCH)
            **kwargs: Additional metadata (headers, timeout, etc.)
        """
        super().__init__(name, callback, protocol=ProtocolType.REST, **kwargs)
        self._provider = rest_provider
        self.mode = mode
        self.method = method.upper()
        self._route_registered = False
    
    async def initialize(self) -> bool:
        """Initialize REST callable."""
        if not self._provider:
            raise CallableError("REST provider not provided")
        
        if self.mode == "server":
            if not self.callback:
                raise CallableError("Server mode requires callback")
            
            # Register route with aiohttp
            
            
            async def aiohttp_handler(request: Any):  # web.Request
                """Wrapper for user callback."""
                try:
                    # Call user callback
                    if not self.callback:
                        return web.json_response({"error": "No callback"}, status=500)
                    result = await self.callback(request)
                    
                    # Convert result to JSON response
                    if isinstance(result, dict):
                        return web.json_response(result)
                    elif isinstance(result, web.Response):
                        return result
                    else:
                        return web.json_response({"result": result})
                        
                except Exception as e:
                    logger.error(f"❌ REST handler error: {e}")
                    return web.json_response(
                        {"error": str(e)},
                        status=500
                    )
            
            # Add route to application
            app = self._provider.get_app()
            path = f"/{self.name.lstrip('/')}"
            
            if self.method == "GET":
                app.router.add_get(path, aiohttp_handler)
            elif self.method == "POST":
                app.router.add_post(path, aiohttp_handler)
            elif self.method == "PUT":
                app.router.add_put(path, aiohttp_handler)
            elif self.method == "DELETE":
                app.router.add_delete(path, aiohttp_handler)
            elif self.method == "PATCH":
                app.router.add_patch(path, aiohttp_handler)
            else:
                raise CallableError(f"Unsupported HTTP method: {self.method}")
            
            self._route_registered = True
            logger.info(f"✅ REST endpoint registered: {self.method} {path}")
        
        elif self.mode == "client":
            # Client just needs base URL
            logger.debug(f"✅ REST client callable initialized: {self.method} {self.name}")
        
        self._initialized = True
        return True
    
    async def shutdown(self) -> None:
        """Shutdown REST callable."""
        # Routes are automatically cleaned up with app
        self._initialized = False
        self._route_registered = False
        logger.debug(f"✅ REST callable shutdown: {self.name}")
    
    async def call(
        self,
        request: Any = None,
        timeout: float = 30.0,
        **kwargs
    ) -> Any:
        """
        Invoke REST endpoint.
        
        Args:
            request: Request data (dict for JSON, None for GET)
            timeout: Timeout in seconds
            **kwargs: Additional parameters (headers, params)
            
        Returns:
            Response data
            
        Raises:
            CallableError: If not initialized or call fails
            
        Example:
            >>> # GET request
            >>> result = await callable.call(method="GET")
            >>> 
            >>> # POST with JSON body
            >>> result = await callable.call(
            ...     {"value": 42},
            ...     headers={"Authorization": "Bearer token"}
            ... )
        """
        if not self._initialized:
            raise CallableError(f"Callable '{self.name}' not initialized")
        
        if self.mode != "client":
            raise CallableError("call() only available in client mode")
        
        try:
            session = self._provider.get_session()
            url = f"{self._provider.base_url}/{self.name.lstrip('/')}"
            
            headers = kwargs.get("headers", {})
            params = kwargs.get("params", {})
            
            # Make HTTP request
            if self.method == "GET":
                async with session.get(
                    url,
                    params=params,
                    headers=headers,
                    timeout=timeout
                ) as response:
                    response.raise_for_status()
                    return await response.json()
                    
            elif self.method in ["POST", "PUT", "PATCH"]:
                async with session.request(
                    self.method,
                    url,
                    json=request,
                    params=params,
                    headers=headers,
                    timeout=timeout
                ) as response:
                    response.raise_for_status()
                    return await response.json()
                    
            elif self.method == "DELETE":
                async with session.delete(
                    url,
                    params=params,
                    headers=headers,
                    timeout=timeout
                ) as response:
                    response.raise_for_status()
                    return await response.json()
            
            else:
                raise CallableError(f"Unsupported HTTP method: {self.method}")
            
        except Exception as e:
            raise CallableError(f"REST call '{self.method} {self.name}' failed: {e}")
