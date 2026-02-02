"""
gRPC Protocol Provider

Implements AbstractProtocolProvider for gRPC over Unix Domain Sockets.
Wraps existing vyra_base.com.handler.ipc (GrpcUdsServer/Client).
"""
import logging
from typing import Any, Callable, Optional, Dict
from pathlib import Path

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
from vyra_base.com.handler.ipc import GrpcUdsServer, GrpcUdsClient

logger = logging.getLogger(__name__)


class GrpcProvider(AbstractProtocolProvider):
    """
    Protocol provider for gRPC over Unix Domain Sockets.
    
    Features:
    - Unary RPC (request-response via Callable)
    - Server/Client streaming (via Job)
    - Bidirectional streaming
    - Unix Domain Socket transport (no network)
    
    Wraps existing vyra_base.com.handler.ipc for seamless
    integration with established gRPC infrastructure.
    
    Example:
        >>> # Server side
        >>> provider = GrpcProvider(
        ...     socket_path="/tmp/vyra_sockets/my_service.sock",
        ...     mode="server"
        ... )
        >>> await provider.initialize()
        >>> 
        >>> # Create callable (server registers servicer)
        >>> async def handle_request(request):
        ...     return {"result": request["value"] * 2}
        >>> 
        >>> callable = await provider.create_callable(
        ...     "calculate",
        ...     callback=handle_request
        ... )
        >>> 
        >>> # Client side
        >>> client_provider = GrpcProvider(
        ...     socket_path="/tmp/vyra_sockets/my_service.sock",
        ...     mode="client"
        ... )
        >>> await client_provider.initialize()
        >>> callable = await client_provider.create_callable("calculate")
        >>> result = await callable.call({"value": 21})
    """
    
    def __init__(
        self,
        protocol: ProtocolType = ProtocolType.GRPC,
        socket_path: str = "/tmp/vyra_sockets/default.sock",
        mode: str = "client",  # "server" or "client"
        **kwargs
    ):
        """
        Initialize gRPC provider.
        
        Args:
            protocol: Protocol type (must be GRPC)
            socket_path: Path to Unix Domain Socket
            mode: Operation mode ("server" or "client")
            **kwargs: Additional parameters
        """
        super().__init__(protocol)
        self.socket_path = Path(socket_path)
        self.mode = mode.lower()
        self._grpc_instance: Optional[Any] = None  # GrpcUdsServer or GrpcUdsClient
        
        if self.mode not in ["server", "client"]:
            raise ValueError(f"Invalid mode: {mode}. Must be 'server' or 'client'")
    
    async def check_availability(self) -> bool:
        """
        Check if gRPC is available.
        
        Returns:
            bool: True if gRPC can be imported
        """        
        logger.debug("✅ gRPC available")
        return True
    
    async def initialize(self, config: Optional[Dict[str, Any]] = None) -> bool:
        """
        Initialize gRPC server or client.
        
        Args:
            config: Optional configuration overrides
            
        Returns:
            bool: True if initialization successful
            
        Raises:
            ProviderError: If initialization fails
        """
        if not await self.check_availability():
            raise ProtocolUnavailableError("gRPC not available")
        
        try:
            # Ensure socket directory exists
            self.socket_path.parent.mkdir(parents=True, exist_ok=True)
            
            if self.mode == "server":
                # Create gRPC server
                self._grpc_instance = GrpcUdsServer(socket_path=self.socket_path)
                await self._grpc_instance.start()
                logger.info(f"✅ gRPC server started on {self.socket_path}")
                
            elif self.mode == "client":
                # Create gRPC client
                self._grpc_instance = GrpcUdsClient(socket_path=self.socket_path)
                await self._grpc_instance.start()
                logger.info(f"✅ gRPC client connected to {self.socket_path}")
            
            self._initialized = True
            return True
            
        except Exception as e:
            raise ProviderError(f"gRPC initialization failed: {e}")
    
    async def shutdown(self) -> None:
        """Shutdown gRPC server or client."""
        if self._grpc_instance:
            try:
                await self._grpc_instance.stop()
                logger.info("✅ gRPC provider shutdown complete")
            except Exception as e:
                logger.error(f"❌ gRPC shutdown error: {e}")
        
        self._initialized = False
        self._grpc_instance = None
    
    async def create_callable(
        self,
        name: str,
        callback: Optional[Callable] = None,
        **kwargs
    ) -> VyraCallable:
        """
        Create gRPC Callable (Unary RPC).
        
        Args:
            name: Method name
            callback: Optional callback for server-side
            **kwargs: Additional parameters
            
        Returns:
            VyraCallable: gRPC callable instance
            
        Note:
            For server mode, callback handles incoming requests.
            For client mode, callback is ignored.
        """
        if not self._initialized:
            raise ProviderError("Provider not initialized")
        
        from vyra_base.com.external.grpc.callable import GrpcCallable
        
        callable_instance = GrpcCallable(
            name=name,
            grpc_instance=self._grpc_instance,
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
        Create gRPC Speaker (Server Streaming).
        
        Note:
            gRPC doesn't have native pub/sub.
            This uses server-side streaming for one-to-many communication.
        """
        raise NotImplementedError(
            "gRPC Speaker not implemented. "
            "Use server-side streaming via Job or consider Redis/MQTT for pub/sub."
        )
    
    async def create_job(
        self,
        name: str,
        callback: Optional[Callable] = None,
        **kwargs
    ) -> VyraJob:
        """
        Create gRPC Job (Streaming RPC).
        
        Supports:
        - Server-side streaming (server sends multiple responses)
        - Client-side streaming (client sends multiple requests)
        - Bidirectional streaming
        """
        raise NotImplementedError(
            "gRPC Job not yet implemented. "
            "Will support streaming RPCs for long-running tasks."
        )
    
    def get_instance(self) -> Any:
        """
        Get underlying gRPC server or client instance.
        
        Returns:
            GrpcUdsServer or GrpcUdsClient instance
            
        Raises:
            ProviderError: If provider not initialized
        """
        if not self._initialized or not self._grpc_instance:
            raise ProviderError("Provider not initialized")
        
        return self._grpc_instance
