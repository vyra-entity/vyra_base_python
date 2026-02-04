"""
gRPC Server

High-performance gRPC server using Unix Domain Sockets for local IPC.
Supports all gRPC patterns: unary, server streaming, client streaming, bidirectional.
"""
from __future__ import annotations

import asyncio
import logging
from pathlib import Path
from typing import Any, Callable, Optional, TYPE_CHECKING

logger = logging.getLogger(__name__)

# Check if grpcio is available
try:
    import grpc
    from grpc import aio
    GRPC_AVAILABLE = True
except ImportError:
    GRPC_AVAILABLE = False
    logger.warning("⚠️  grpcio not installed. gRPC server functionality disabled.")


class GrpcServer:
    """
    gRPC Server over Unix Domain Socket.
    
    Supports registering servicers for handling different RPC patterns.
    
    Example::
    
        class MyServiceServicer(my_service_pb2_grpc.MyServiceServicer):
            async def UnaryMethod(self, request, context):
                return MyResponse(result="success")
                
            async def ServerStreamingMethod(self, request, context):
                for i in range(10):
                    yield MyResponse(result=f"item_{i}")
        
        server = GrpcServer("/tmp/my_service.sock")
        server.add_service(
            my_service_pb2_grpc.add_MyServiceServicer_to_server,
            MyServiceServicer()
        )
        await server.start()
    """
    
    def __init__(
        self, 
        socket_path: str | Path,
        max_workers: int = 10,
        options: Optional[list] = None
    ):
        """
        Initialize gRPC Server.
        
        Args:
            socket_path: Path to Unix Domain Socket
            max_workers: Maximum number of worker threads
            options: gRPC server options (e.g., max message size)
        """
        if not GRPC_AVAILABLE:
            raise ImportError("grpcio is not installed. Install with: pip install grpcio")
            
        self.socket_path = Path(socket_path) if isinstance(socket_path, str) else socket_path
        self.max_workers = max_workers
        self.options = options or [
            ('grpc.max_send_message_length', 50 * 1024 * 1024),  # 50 MB
            ('grpc.max_receive_message_length', 50 * 1024 * 1024),  # 50 MB
        ]
        self._server = None
        self._servicers = []
        self._running = False
        
    def add_service(
        self, 
        add_servicer_to_server_fn: Callable,
        servicer: Any
    ):
        """
        Add a servicer to the server.
        
        Args:
            add_servicer_to_server_fn: Generated function from protobuf 
                                       (e.g., add_MyServiceServicer_to_server)
            servicer: Instance of the servicer class
        """
        self._servicers.append((add_servicer_to_server_fn, servicer))
        logger.info(f"✅ Added servicer: {servicer.__class__.__name__}")
        
    async def start(self):
        """
        Start the gRPC server on Unix Domain Socket.
        
        Raises:
            RuntimeError: If no servicers are registered
        """
        if not GRPC_AVAILABLE:
            raise ImportError("grpcio is not installed")
            
        if not self._servicers:
            raise RuntimeError("No servicers registered. Use add_service() first.")
        
        # Clean up existing socket file
        if self.socket_path.exists():
            logger.warning(f"🧹 Removing existing socket: {self.socket_path}")
            self.socket_path.unlink()
        
        # Ensure parent directory exists
        self.socket_path.parent.mkdir(parents=True, exist_ok=True)
        
        # Create gRPC async server
        self._server = aio.server(options=self.options)
        
        # Register all servicers
        for add_fn, servicer in self._servicers:
            add_fn(servicer, self._server)
            
        # Bind to Unix Domain Socket
        address = f"unix://{self.socket_path.absolute()}"
        self._server.add_insecure_port(address)
        
        # Start server
        await self._server.start()
        self._running = True
        
        logger.info(f"🚀 gRPC Server started on {address}")
        
    async def wait_for_termination(self):
        """Wait for server termination (blocking)."""
        if self._server:
            await self._server.wait_for_termination()
            
    async def stop(self, grace: Optional[float] = 5.0):
        """
        Stop the gRPC server gracefully.
        
        Args:
            grace: Grace period in seconds for graceful shutdown
        """
        if self._server:
            logger.info(f"🛑 Stopping gRPC Server (grace: {grace}s)...")
            await self._server.stop(grace)
            self._running = False
            
            # Clean up socket file
            if self.socket_path.exists():
                self.socket_path.unlink()
                logger.info(f"🧹 Removed socket: {self.socket_path}")
    
    @property
    def is_running(self) -> bool:
        """Check if server is running."""
        return self._running
