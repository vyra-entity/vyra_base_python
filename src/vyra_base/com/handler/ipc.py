"""
gRPC over Unix Domain Socket (UDS) Base Handler

This module provides the generic foundation for gRPC communication over Unix Domain Sockets.
It supports all gRPC communication patterns:
- Unary RPC (request-response)
- Server-Side Streaming (one request, multiple responses)
- Client-Side Streaming (multiple requests, one response)
- Bidirectional Streaming (multiple requests, multiple responses)

Usage:
    Server:
        server = GrpcUdsServer(socket_path="/tmp/my_service.sock")
        server.add_service(MyServiceServicer())
        await server.start()
        
    Client:
        client = GrpcUdsClient(socket_path="/tmp/my_service.sock")
        await client.connect()
        response = await client.call_unary("method_name", request)
"""

import asyncio
import logging
import os
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Any, AsyncIterator, Callable, Optional, TYPE_CHECKING

logger = logging.getLogger(__name__)

# Check if grpcio is available
try:
    import grpc
    from grpc import aio
    GRPC_AVAILABLE = True
except ImportError:
    GRPC_AVAILABLE = False
    grpc = None
    aio = None
    logger.warning("⚠️  grpcio not installed. gRPC over UDS functionality disabled.")

# Type hints only
if TYPE_CHECKING and GRPC_AVAILABLE:
    from grpc import aio


class GrpcUdsBase(ABC):
    """
    Base class for gRPC over Unix Domain Socket communication.
    
    Attributes:
        socket_path: Path to the Unix Domain Socket
        channel: gRPC channel (for client) or server (for server)
    """
    
    def __init__(self, socket_path: str | Path):
        """
        Initialize gRPC UDS base.
        
        Args:
            socket_path: Path to Unix Domain Socket (e.g., "/tmp/vyra_service.sock")
        """
        self.socket_path = Path(socket_path) if isinstance(socket_path, str) else socket_path
        self._running = False
        
    @property
    def is_running(self) -> bool:
        """Check if server/client is running."""
        return self._running
    
    def _get_uds_address(self) -> str:
        """
        Get the gRPC-compatible UDS address.
        
        Returns:
            gRPC UDS address format: "unix:///path/to/socket.sock"
        """
        return f"unix://{self.socket_path.absolute()}"
    
    @abstractmethod
    async def start(self):
        """Start the gRPC service (server) or connection (client)."""
        pass
    
    @abstractmethod
    async def stop(self):
        """Stop the gRPC service (server) or connection (client)."""
        pass


class GrpcUdsServer(GrpcUdsBase):
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
        
        server = GrpcUdsServer("/tmp/my_service.sock")
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
        Initialize gRPC UDS Server.
        
        Args:
            socket_path: Path to Unix Domain Socket
            max_workers: Maximum number of worker threads
            options: gRPC server options (e.g., max message size)
        """
        super().__init__(socket_path)
        self.max_workers = max_workers
        self.options = options or [
            ('grpc.max_send_message_length', 50 * 1024 * 1024),  # 50 MB
            ('grpc.max_receive_message_length', 50 * 1024 * 1024),  # 50 MB
        ]
        self._server: Optional[.Server] = None
        self._servicers = []
        
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
        address = self._get_uds_address()
        self._server.add_insecure_port(address)
        
        # Start server
        await self._server.start()
        self._running = True
        
        logger.info(f"🚀 gRPC UDS Server started on {address}")
        
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
            logger.info(f"🛑 Stopping gRPC UDS Server (grace: {grace}s)...")
            await self._server.stop(grace)
            self._running = False
            
            # Clean up socket file
            if self.socket_path.exists():
                self.socket_path.unlink()
                logger.info(f"🧹 Removed socket: {self.socket_path}")


class GrpcUdsClient(GrpcUdsBase):
    """
    gRPC Client over Unix Domain Socket.
    
    Provides methods for all RPC patterns:
    - call_unary(): Unary RPC
    - call_server_streaming(): Server-side streaming
    - call_client_streaming(): Client-side streaming
    - call_bidirectional_streaming(): Bidirectional streaming
    
    Example::
    
        client = GrpcUdsClient("/tmp/my_service.sock")
        await client.connect()
        
        # Unary call
        response = await client.call_unary(
            stub.UnaryMethod,
            MyRequest(data="test")
        )
        
        # Server streaming
        async for response in client.call_server_streaming(
            stub.ServerStreamingMethod,
            MyRequest(data="test")
        ):
            print(response)
    """
    
    def __init__(
        self, 
        socket_path: str | Path,
        options: Optional[list] = None
    ):
        """
        Initialize gRPC UDS Client.
        
        Args:
            socket_path: Path to Unix Domain Socket
            options: gRPC channel options
        """
        super().__init__(socket_path)
        self.options = options or [
            ('grpc.max_send_message_length', 50 * 1024 * 1024),  # 50 MB
            ('grpc.max_receive_message_length', 50 * 1024 * 1024),  # 50 MB
        ]
        self._channel: Optional[aio.Channel] = None
        
    async def connect(self):
        """
        Connect to the gRPC server via Unix Domain Socket.
        
        Raises:
            FileNotFoundError: If socket file doesn't exist
        """
        if not self.socket_path.exists():
            raise FileNotFoundError(
                f"Socket not found: {self.socket_path}. "
                "Is the server running?"
            )
        
        address = self._get_uds_address()
        self._channel = aio.insecure_channel(address, options=self.options)
        self._running = True
        
        logger.info(f"🔗 gRPC UDS Client connected to {address}")
        
    async def start(self):
        """Alias for connect() for consistency with base class."""
        await self.connect()
        
    async def stop(self):
        """Close the gRPC channel."""
        if self._channel:
            logger.info("🛑 Closing gRPC UDS Client...")
            await self._channel.close()
            self._running = False
            
    @property
    def channel(self) -> aio.Channel:
        """
        Get the gRPC channel for creating stubs.
        
        Returns:
            gRPC async channel
            
        Raises:
            RuntimeError: If not connected
        """
        if not self._channel or not self._running:
            raise RuntimeError("Not connected. Call connect() first.")
        return self._channel
    
    async def call_unary(
        self, 
        method: Callable,
        request: Any,
        timeout: Optional[float] = None
    ) -> Any:
        """
        Call unary RPC method (single request -> single response).
        
        Args:
            method: Stub method to call (e.g., stub.UnaryMethod)
            request: Request message
            timeout: Optional timeout in seconds
            
        Returns:
            Response message
            
        Example::
        
            stub = MyServiceStub(client.channel)
            response = await client.call_unary(
                stub.GetData,
                GetDataRequest(id="123"),
                timeout=5.0
            )
        """
        try:
            response = await method(request, timeout=timeout)
            return response
        except grpc.RpcError as e:
            logger.error(f"❌ Unary RPC failed: {e.code()}: {e.details()}")
            raise
            
    async def call_server_streaming(
        self,
        method: Callable,
        request: Any,
        timeout: Optional[float] = None
    ) -> AsyncIterator[Any]:
        """
        Call server-side streaming RPC (single request -> multiple responses).
        
        Args:
            method: Stub method to call
            request: Request message
            timeout: Optional timeout in seconds
            
        Yields:
            Response messages
            
        Example::
        
            stub = MyServiceStub(client.channel)
            async for response in client.call_server_streaming(
                stub.StreamData,
                StreamRequest(count=10)
            ):
                print(response)
        """
        try:
            call = method(request, timeout=timeout)
            async for response in call:
                yield response
        except grpc.RpcError as e:
            logger.error(f"❌ Server streaming RPC failed: {e.code()}: {e.details()}")
            raise
            
    async def call_client_streaming(
        self,
        method: Callable,
        request_iterator: AsyncIterator[Any],
        timeout: Optional[float] = None
    ) -> Any:
        """
        Call client-side streaming RPC (multiple requests -> single response).
        
        Args:
            method: Stub method to call
            request_iterator: Async iterator of request messages
            timeout: Optional timeout in seconds
            
        Returns:
            Final response message
            
        Example::
        
            async def request_generator():
                for i in range(10):
                    yield UploadRequest(data=f"chunk_{i}")
            
            stub = MyServiceStub(client.channel)
            response = await client.call_client_streaming(
                stub.UploadData,
                request_generator()
            )
        """
        try:
            response = await method(request_iterator, timeout=timeout)
            return response
        except grpc.RpcError as e:
            logger.error(f"❌ Client streaming RPC failed: {e.code()}: {e.details()}")
            raise
            
    async def call_bidirectional_streaming(
        self,
        method: Callable,
        request_iterator: AsyncIterator[Any],
        timeout: Optional[float] = None
    ) -> AsyncIterator[Any]:
        """
        Call bidirectional streaming RPC (multiple requests -> multiple responses).
        
        Args:
            method: Stub method to call
            request_iterator: Async iterator of request messages
            timeout: Optional timeout in seconds
            
        Yields:
            Response messages
            
        Example::
        
            async def request_generator():
                for i in range(10):
                    yield ChatRequest(message=f"msg_{i}")
                    await asyncio.sleep(1)
            
            stub = MyServiceStub(client.channel)
            async for response in client.call_bidirectional_streaming(
                stub.Chat,
                request_generator()
            ):
                print(response.message)
            
        """
        try:
            call = method(request_iterator, timeout=timeout)
            async for response in call:
                yield response
        except grpc.RpcError as e:
            logger.error(f"❌ Bidirectional streaming RPC failed: {e.code()}: {e.details()}")
            raise


class GrpcUdsServiceRegistry:
    """
    Registry for managing multiple gRPC services over different sockets.
    
    Useful for modules that need to provide multiple independent services.
    
    Example::
    
        registry = GrpcUdsServiceRegistry()
        
        # Register permission service
        perm_server = GrpcUdsServer("/tmp/vyra_permission.sock")
        perm_server.add_service(add_PermissionServicer_to_server, PermissionServicer())
        await registry.register_server("permission", perm_server)
        
        # Register status service
        status_server = GrpcUdsServer("/tmp/vyra_status.sock")
        status_server.add_service(add_StatusServicer_to_server, StatusServicer())
        await registry.register_server("status", status_server)
        
        # Start all
        await registry.start_all()
    """
    
    def __init__(self):
        """Initialize service registry."""
        self._servers: dict[str, GrpcUdsServer] = {}
        self._clients: dict[str, GrpcUdsClient] = {}
        
    async def register_server(self, name: str, server: GrpcUdsServer):
        """
        Register and start a gRPC server.
        
        Args:
            name: Unique name for the server
            server: GrpcUdsServer instance
        """
        if name in self._servers:
            logger.warning(f"⚠️ Server '{name}' already registered, replacing...")
            await self._servers[name].stop()
            
        self._servers[name] = server
        logger.info(f"📝 Registered server: {name}")
        
    async def register_client(self, name: str, client: GrpcUdsClient):
        """
        Register a gRPC client.
        
        Args:
            name: Unique name for the client
            client: GrpcUdsClient instance
        """
        if name in self._clients:
            logger.warning(f"⚠️ Client '{name}' already registered, replacing...")
            await self._clients[name].stop()
            
        self._clients[name] = client
        logger.info(f"📝 Registered client: {name}")
        
    async def start_all(self):
        """Start all registered servers and connect all clients."""
        logger.info(f"🚀 Starting {len(self._servers)} servers and {len(self._clients)} clients...")
        
        # Start servers
        for name, server in self._servers.items():
            try:
                await server.start()
                logger.info(f"✅ Started server: {name}")
            except Exception as e:
                logger.error(f"❌ Failed to start server '{name}': {e}")
                
        # Connect clients
        for name, client in self._clients.items():
            try:
                await client.connect()
                logger.info(f"✅ Connected client: {name}")
            except Exception as e:
                logger.error(f"❌ Failed to connect client '{name}': {e}")
                
    async def stop_all(self):
        """Stop all registered servers and clients."""
        logger.info(f"🛑 Stopping {len(self._servers)} servers and {len(self._clients)} clients...")
        
        # Stop servers
        for name, server in self._servers.items():
            try:
                await server.stop()
                logger.info(f"✅ Stopped server: {name}")
            except Exception as e:
                logger.error(f"❌ Failed to stop server '{name}': {e}")
                
        # Close clients
        for name, client in self._clients.items():
            try:
                await client.stop()
                logger.info(f"✅ Closed client: {name}")
            except Exception as e:
                logger.error(f"❌ Failed to close client '{name}': {e}")
                
    def get_server(self, name: str) -> Optional[GrpcUdsServer]:
        """Get server by name."""
        return self._servers.get(name)
    
    def get_client(self, name: str) -> Optional[GrpcUdsClient]:
        """Get client by name."""
        return self._clients.get(name)
