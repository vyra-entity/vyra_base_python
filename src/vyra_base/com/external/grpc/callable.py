"""
gRPC Callable Implementation

Unary RPC communication via gRPC over Unix Domain Sockets.
"""
import asyncio
import logging
from typing import Any, Callable, Optional

from vyra_base.com.core.types import VyraCallable, ProtocolType
from vyra_base.com.core.exceptions import CallableError

logger = logging.getLogger(__name__)


class GrpcCallable(VyraCallable):
    """
    gRPC Callable for Unary RPC communication.
    
    Features:
    - Request-response pattern over gRPC
    - Unix Domain Socket transport
    - Automatic serialization/deserialization
    - Server and client modes
    
    Example:
        >>> # Server side
        >>> async def handle_calculate(request):
        ...     return {"result": request["value"] * 2}
        >>> 
        >>> callable = GrpcCallable(
        ...     "calculate",
        ...     grpc_instance=server,
        ...     mode="server",
        ...     callback=handle_calculate
        ... )
        >>> await callable.initialize()
        >>> 
        >>> # Client side
        >>> callable = GrpcCallable(
        ...     "calculate",
        ...     grpc_instance=client,
        ...     mode="client"
        ... )
        >>> await callable.initialize()
        >>> result = await callable.call({"value": 21})
    """
    
    def __init__(
        self,
        name: str,
        grpc_instance: Any,
        mode: str,
        callback: Optional[Callable] = None,
        **kwargs
    ):
        """
        Initialize gRPC callable.
        
        Args:
            name: Method name
            grpc_instance: GrpcUdsServer or GrpcUdsClient instance
            mode: Operation mode ("server" or "client")
            callback: Server-side callback function
            **kwargs: Additional metadata
        """
        super().__init__(name, callback, protocol=ProtocolType.GRPC, **kwargs)
        self._grpc = grpc_instance
        self.mode = mode
        self._server_task: Optional[asyncio.Task] = None
    
    async def initialize(self) -> bool:
        """Initialize gRPC callable."""
        if not self._grpc:
            raise CallableError("gRPC instance not provided")
        
        if self.mode == "server":
            if not self.callback:
                raise CallableError("Server mode requires callback")
            
            # Register servicer method
            # Note: Actual implementation depends on your protobuf definitions
            # This is a placeholder for the registration logic
            logger.debug(f"✅ gRPC server callable registered: {self.name}")
        
        elif self.mode == "client":
            # Client just needs to ensure connection
            logger.debug(f"✅ gRPC client callable initialized: {self.name}")
        
        self._initialized = True
        return True
    
    async def shutdown(self) -> None:
        """Shutdown gRPC callable."""
        if self._server_task:
            self._server_task.cancel()
            try:
                await self._server_task
            except asyncio.CancelledError:
                pass
        
        self._initialized = False
        logger.debug(f"✅ gRPC callable shutdown: {self.name}")
    
    async def call(self, request: Any, timeout: float = 5.0) -> Any:
        """
        Invoke gRPC method.
        
        Args:
            request: Request data
            timeout: Timeout in seconds
            
        Returns:
            Response data
            
        Raises:
            CallableError: If not initialized or call fails
        """
        if not self._initialized:
            raise CallableError(f"Callable '{self.name}' not initialized")
        
        if self.mode != "client":
            raise CallableError("call() only available in client mode")
        
        try:
            # Use gRPC client to make unary call
            # Note: Actual implementation depends on protobuf definitions
            response = await self._grpc.call_unary(
                method_name=self.name,
                request=request,
                timeout=timeout
            )
            
            logger.debug(f"📞 gRPC call '{self.name}' completed")
            return response
            
        except Exception as e:
            raise CallableError(f"gRPC call '{self.name}' failed: {e}")
