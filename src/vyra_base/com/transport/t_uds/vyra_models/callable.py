"""
UDS Callable Implementation

Request-Response communication over Unix Domain Sockets.
Provides stream-based RPC with automatic connection management.
"""
import asyncio
import json
import logging
import time
from typing import Any, Callable, Optional

from vyra_base.com.core.types import VyraCallable, ProtocolType
from vyra_base.com.core.exceptions import CallableError, TimeoutError, TransportError
from vyra_base.com.transport.t_uds.communication import UnixSocket, UDS_SOCKET_DIR

logger = logging.getLogger(__name__)


class UDSCallable(VyraCallable):
    """
    Callable interface over Unix Domain Sockets.
    
    Features:
    - Stream-based request-response
    - JSON serialization
    - Automatic connection management
    - Server-side request handling
    
    Example:
        >>> # Server side
        >>> async def handle_request(request):
        ...     return {"result": request["value"] * 2}
        >>> 
        >>> callable = UDSCallable(
        ...     "calculate",
        ...     callback=handle_request,
        ...     module_name="math_service"
        ... )
        >>> await callable.initialize()
        >>> 
        >>> # Client side
        >>> callable = UDSCallable(
        ...     "calculate",
        ...     module_name="math_service"
        ... )
        >>> await callable.initialize()
        >>> result = await callable.call({"value": 21}, timeout=5.0)
    """
    
    def __init__(
        self,
        name: str,
        callback: Optional[Callable] = None,
        module_name: str = "default",
        **kwargs
    ):
        """
        Initialize UDS callable.
        
        Args:
            name: Callable name
            callback: Server-side callback function
            module_name: Module name
            **kwargs: Additional metadata
        """
        super().__init__(name, callback, ProtocolType.UDS, **kwargs)
        self.module_name = module_name
        
        # Socket path
        socket_filename = f"{module_name}_{name}.sock"
        self._socket_path = UDS_SOCKET_DIR / socket_filename
        
        # Unix socket
        self._socket: Optional[UnixSocket] = None
        
        # Server mode flag
        self._is_server = callback is not None
    
    async def initialize(self) -> bool:
        """
        Initialize UDS callable.
        
        For server: Creates socket and starts listening
        For client: Connects to existing socket
        
        Returns:
            bool: True if initialization successful
        """
        try:
            self._socket = UnixSocket(str(self._socket_path))
            
            if self._is_server:
                return await self._initialize_server()
            else:
                return await self._initialize_client()
        except Exception as e:
            logger.error(f"❌ Failed to initialize callable '{self.name}': {e}")
            return False
    
    async def _initialize_server(self) -> bool:
        """Initialize as server."""
        logger.info(
            f"🚀 Initializing UDS server: {self.module_name}.{self.name}"
        )
        
        # Start listening
        if self._socket is None:
            logger.error("Could not start listener, no uds socket")
            raise RuntimeError("UDS socket not initialized")
        else:
            await self._socket.listen(self._handle_request)
        
        self._initialized = True
        logger.info(f"✅ UDS server ready: {self.module_name}.{self.name}")
        return True
    
    async def _initialize_client(self) -> bool:
        """Initialize as client."""
        logger.info(
            f"🔍 Connecting to UDS: {self.module_name}.{self.name}"
        )
        
        # Connect to server
        if self._socket is None:
            logger.error("Could not connect client, no uds socket")
            raise RuntimeError("UDS socket not initialized")
        else:
            await self._socket.connect()
        
        self._initialized = True
        logger.info(f"✅ Connected to UDS: {self.module_name}.{self.name}")
        return True
    
    async def _handle_request(self, request_bytes: bytes) -> bytes:
        """Handle incoming request (server side)."""
        try:
            # Deserialize request
            request_data = json.loads(request_bytes.decode())
            
            logger.debug(f"📥 Received request on {self.name}")
            
            # Call user callback
            if asyncio.iscoroutinefunction(self.callback):
                response = await self.callback(request_data)
            elif self.callback is None:
                logger.warning("Could not execute callback. Callback is None")
            else:
                response = self.callback(request_data)
            
            # Serialize response
            response_bytes = json.dumps(response).encode()
            
            logger.debug(f"📤 Sent response on {self.name}")
            return response_bytes
            
        except Exception as e:
            # Return error response
            error_response = {
                "error": str(e),
                "type": type(e).__name__
            }
            return json.dumps(error_response).encode()
    
    async def call(self, request: Any, timeout: float = 5.0) -> Any:
        """
        Call the remote callable.
        
        Args:
            request: Request data (must be JSON-serializable)
            timeout: Timeout in seconds (default: 5.0)
            
        Returns:
            Response data
            
        Raises:
            CallableError: If not initialized or not a client
            TimeoutError: If timeout exceeded
            TransportError: If communication fails
        """
        if not self._initialized:
            raise CallableError(f"Callable '{self.name}' not initialized")
        
        if self._is_server:
            raise CallableError("Cannot call from server side")
        
        start_time = time.time()
        
        try:
            # Serialize request
            request_bytes = json.dumps(request).encode()
            
            logger.debug(f"📤 Sending request to {self.name}")
            
            # Send request
            if self._socket is None:
                logger.error("Could not send, no uds socket")
                raise RuntimeError("UDS socket not initialized")
            else:
                await self._socket.send(request_bytes)
            
            # Receive response
            if self._socket is None:
                logger.error("Could not receive, no uds socket")
                raise RuntimeError("UDS socket not initialized")
            else:
                response_bytes = await self._socket.receive(timeout=timeout)
            
            if response_bytes is None:
                elapsed = time.time() - start_time
                raise TimeoutError(
                    f"Callable '{self.name}' timeout after {elapsed:.2f}s"
                )
            
            # Deserialize response
            response = json.loads(response_bytes.decode())
            
            # Check for error response
            if isinstance(response, dict) and "error" in response:
                raise CallableError(
                    f"Remote error: {response['error']} ({response.get('type', 'Unknown')})"
                )
            
            elapsed = time.time() - start_time
            logger.debug(f"✅ Call completed in {elapsed*1000:.2f}ms")
            
            return response
            
        except TimeoutError:
            raise
        except Exception as e:
            raise TransportError(f"Failed to call '{self.name}': {e}")
    
    async def shutdown(self) -> None:
        """Shutdown callable and cleanup resources."""
        logger.info(f"🛑 Shutting down UDS callable: {self.name}")
        
        if self._socket:
            await self._socket.close()
        
        self._initialized = False
        logger.info(f"✅ UDS callable shutdown complete: {self.name}")
