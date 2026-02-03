"""
gRPC Client for vyra_base

High-level wrapper for gRPC communication.
Provides simple interface for unary RPC calls.

Example:
    >>> client = GrpcClient(target="localhost:50051")
    >>> await client.connect()
    >>> response = await client.call_method("/service/Method", request_bytes)
    >>> await client.close()
"""
from __future__ import annotations

import asyncio
import logging

from pathlib import Path
from typing import Any, Optional

from vyra_base.helper.logger import Logger
from vyra_base.helper.error_handler import ErrorTraceback

logger = logging.getLogger(__name__)

# Check if grpcio is available
try:
    import grpc
    from grpc import aio
    GRPC_AVAILABLE = True
except ImportError:
    GRPC_AVAILABLE = False


class GrpcClient:
    """
    High-level gRPC Client wrapper.
    
    Provides simple interface for gRPC communication without requiring
    proto files or code generation.
    
    Features:
    - Unary RPC (request-response)
    - Automatic connection management
    - SSL/TLS support
    
    Args:
        target: gRPC server target (e.g., "localhost:50051")
        credentials: Optional SSL credentials
        options: Optional channel options
        timeout: Default timeout for operations
    
    Example:
        >>> client = GrpcClient(target="192.168.1.10:50051")
        >>> await client.connect()
        >>> 
        >>> # Unary call
        >>> response = await client.call_method("/service/Method", request_bytes)
        >>> 
        >>> await client.close()
    """
    
    @ErrorTraceback.w_check_error_exist
    def __init__(
        self,
        target: str | Path,
        credentials: Optional[Any] = None,
        options: Optional[list] = None,
        timeout: float = 30.0,
    ):
        """
        Initialize gRPC client.
        
        Args:
            target: Server target (host:port)
            credentials: Optional SSL credentials
            options: Optional channel options
            timeout: Default timeout in seconds
        """
        if not GRPC_AVAILABLE:
            raise ImportError("grpcio not installed. Install with: pip install grpcio")
        
        if isinstance(target, Path):
            target = f"unix://{target.as_posix()}"
            
        self.target = target
        self.credentials = credentials
        self.options = options or []
        self.timeout = timeout
        
        self._channel: Optional[aio.Channel] = None
        self._connected = False
    
    @property
    def is_connected(self) -> bool:
        """Check if client is connected."""
        return self._connected
    
    @ErrorTraceback.w_check_error_exist
    async def connect(self) -> None:
        """Establish connection to gRPC server."""
        try:
            Logger.info(f"🔌 Connecting to gRPC server: {self.target}")
            
            # Create channel
            if self.credentials:
                self._channel = aio.secure_channel(
                    self.target,
                    self.credentials,
                    options=self.options
                )
            else:
                self._channel = aio.insecure_channel(
                    self.target,
                    options=self.options
                )
            
            # Wait for channel to be ready (with timeout)
            try:
                await asyncio.wait_for(
                    self._channel.channel_ready(),
                    timeout=self.timeout
                )
            except asyncio.TimeoutError:
                raise ConnectionError(f"Connection timeout to {self.target}")
            
            self._connected = True
            Logger.info(f"✅ Connected to gRPC server: {self.target}")
            
        except Exception as e:
            Logger.error(f"❌ Failed to connect to gRPC server: {e}")
            self._connected = False
            if self._channel:
                await self._channel.close()
                self._channel = None
            raise
    
    @ErrorTraceback.w_check_error_exist
    async def close(self) -> None:
        """Close connection to gRPC server."""
        if self._channel:
            Logger.info(f"🔌 Closing gRPC connection: {self.target}")
            await self._channel.close()
            self._channel = None
            self._connected = False
            Logger.info("✅ gRPC connection closed")
    
    def _ensure_connected(self):
        """Ensure client is connected."""
        if not self._connected or not self._channel:
            raise ConnectionError("gRPC client not connected. Call connect() first.")
    
    @ErrorTraceback.w_check_error_exist
    async def call_method(
        self,
        method: str,
        request: bytes,
        timeout: Optional[float] = None,
        metadata: Optional[list] = None,
    ) -> bytes:
        """
        Call a unary gRPC method.
        
        Args:
            method: Method path (e.g., "/service.Service/Method")
            request: Serialized request bytes
            timeout: Optional timeout override
            metadata: Optional metadata tuples
            
        Returns:
            Serialized response bytes
        """
        self._ensure_connected()
        
        if self._channel is None:
            raise RuntimeError("gRPC channel is not initialized")

        try:
            Logger.debug(f"📤 Calling gRPC method: {method}")
            
            # Create unary-unary call
            call = self._channel.unary_unary(
                method,
                request_serializer=None,  # Already serialized
                response_deserializer=lambda x: x,  # Keep as bytes
            )
            
            # Make call
            response = await call(
                request,
                timeout=timeout or self.timeout,
                metadata=metadata
            )
            
            Logger.debug(f"✅ gRPC call completed: {method}")
            return response
            
        except grpc.RpcError as e:
            Logger.error(f"❌ gRPC call failed: {method} - {e.code()}: {e.details()}")
            raise
        except Exception as e:
            Logger.error(f"❌ gRPC call failed: {method} - {e}")
            raise
    
    async def __aenter__(self):
        """Async context manager entry."""
        await self.connect()
        return self
    
    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """Async context manager exit."""
        await self.close()
