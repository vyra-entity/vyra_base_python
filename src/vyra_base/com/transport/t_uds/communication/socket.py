"""
Unix Domain Socket (UDS) Management

Low-level Unix socket handling for local IPC.
Provides stream-based communication with connection management.
"""
import asyncio
import logging
import os
import socket
import struct
from pathlib import Path
from typing import Awaitable, Optional, Tuple, Callable, Union

from vyra_base.com.core.exceptions import TransportError, ConnectionError

logger = logging.getLogger(__name__)

# UDS socket directory — read from VYRA_SOCKET_DIR env var, fallback to /vyra/sockets
UDS_SOCKET_DIR = Path(os.environ.get("VYRA_SOCKET_DIR", "/vyra/sockets"))

# Message format: | Length (4B) | Data (variable) |
MESSAGE_HEADER_FORMAT = '!I'
MESSAGE_HEADER_SIZE = struct.calcsize(MESSAGE_HEADER_FORMAT)


class UnixSocket:
    """
    Unix domain socket wrapper for stream communication.
    
    Features:
    - Automatic connection management
    - Length-prefixed message framing
    - Non-blocking I/O with asyncio
    - Automatic socket cleanup
    
    Example:
        >>> # Server
        >>> server = UnixSocket("/tmp/vyra_sockets/my_service.sock")
        >>> await server.listen(handle_client)
        >>> 
        >>> # Client
        >>> client = UnixSocket("/tmp/vyra_sockets/my_service.sock")
        >>> await client.connect()
        >>> await client.send(b"Hello")
        >>> response = await client.receive()
    """
    
    def __init__(self, socket_path: str):
        """
        Initialize Unix socket.
        
        Args:
            socket_path: Path to Unix socket file
        """
        self.socket_path = Path(socket_path)
        self._socket: Optional[socket.socket] = None
        self._server_socket: Optional[socket.socket] = None
        self._is_server = False
        self._connected = False
        self._server_task: Optional[asyncio.Task] = None
        
        # Ensure socket directory exists
        self.socket_path.parent.mkdir(parents=True, exist_ok=True)
    
    async def connect(self, timeout: float = 5.0) -> bool:
        """
        Connect to Unix socket (client mode).
        
        Args:
            timeout: Connection timeout in seconds
            
        Returns:
            bool: True if connected successfully
            
        Raises:
            ConnectionError: If connection fails
        """
        if self._connected:
            logger.warning("⚠️ Already connected")
            return True
        
        if not self.socket_path.exists():
            raise ConnectionError(
                f"Socket does not exist: {self.socket_path}"
            )
        
        try:
            logger.debug(f"🔌 Connecting to {self.socket_path}")
            
            # Create socket
            self._socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            self._socket.settimeout(timeout)
            
            # Connect
            self._socket.connect(str(self.socket_path))
            
            # Set non-blocking
            self._socket.setblocking(False)
            
            self._connected = True
            logger.info(f"✅ Connected to {self.socket_path}")
            return True
            
        except socket.timeout:
            raise ConnectionError(f"Connection timeout: {self.socket_path}")
        except Exception as e:
            raise ConnectionError(f"Failed to connect: {e}")
    
    async def listen(
        self,
        handler: Callable[[bytes], Awaitable[bytes]],
        backlog: int = 5
    ) -> None:
        """
        Start listening for connections (server mode).
        
        Args:
            handler: Handler function for client requests
            backlog: Maximum queued connections
            
        Raises:
            TransportError: If listen fails
        """
        if self._is_server:
            logger.warning("⚠️ Already listening")
            return
        
        try:
            # Remove existing socket file
            if self.socket_path.exists():
                self.socket_path.unlink()
            
            logger.info(f"👂 Starting UDS server on {self.socket_path}")
            
            # Create server socket
            self._server_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            self._server_socket.bind(str(self.socket_path))
            self._server_socket.listen(backlog)
            self._server_socket.setblocking(False)
            
            # Set permissions (readable/writable by all)
            os.chmod(self.socket_path, 0o666)
            
            self._is_server = True
            
            # Start accept loop
            self._server_task = asyncio.create_task(
                self._accept_loop(handler)
            )
            
            logger.info(f"✅ UDS server listening on {self.socket_path}")
            
        except Exception as e:
            raise TransportError(f"Failed to start server: {e}")
    
    async def _accept_loop(self, handler: Callable[[bytes], Awaitable[bytes]]) -> None:
        """Accept incoming connections."""
        loop = asyncio.get_event_loop()
        
        while self._is_server:
            try:
                # Wait for connection
                if self._server_socket is None:
                    logger.warning("Could not use server socket, socket is None")
                    raise
                else:
                    client_socket, _ = await loop.sock_accept(self._server_socket)
                
                logger.debug("📥 Client connected")
                
                # Handle client in separate task
                asyncio.create_task(self._handle_client(client_socket, handler))
                
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"❌ Accept error: {e}")
                await asyncio.sleep(0.1)
    
    async def _handle_client(
        self,
        client_socket: socket.socket,
        handler: Callable[[bytes], Awaitable[bytes]]
    ) -> None:
        """Handle a single client connection."""
        try:
            while True:
                request = await self._receive_message(client_socket)
                if not request:
                    break

                try:
                    result: Union[bytes, Awaitable[bytes]] = handler(request)

                    response: bytes
                    if isinstance(result, bytes):
                        response = result
                    else:
                        response = await result

                    await self._send_message(client_socket, response)
                except Exception as e:
                    logger.error(f"❌ Handler error: {e}")
                    error_msg = f"Error: {e}".encode()
                    await self._send_message(client_socket, error_msg)
        except Exception as e:
            logger.error(f"❌ Client handler error: {e}")
        finally:
            client_socket.close()
            logger.debug("📤 Client disconnected")
    
    async def send(self, data: bytes) -> bool:
        """
        Send data over socket (client mode).
        
        Args:
            data: Data to send
            
        Returns:
            bool: True if sent successfully
            
        Raises:
            TransportError: If send fails
        """
        if not self._connected:
            raise TransportError("Not connected")
        
        try:
            if self._socket is None:
                logger.warning("Could not send message. No uds socket")
                raise
            else:
                await self._send_message(self._socket, data)
            return True
        except Exception as e:
            raise TransportError(f"Failed to send: {e}")
    
    async def receive(self, timeout: float = 5.0) -> Optional[bytes]:
        """
        Receive data from socket (client mode).
        
        Args:
            timeout: Receive timeout in seconds
            
        Returns:
            Received data or None if timeout
            
        Raises:
            TransportError: If receive fails
        """
        if not self._connected:
            raise TransportError("Not connected")
        
        try:
            if self._socket is None:
                logger.warning("Could not receive message. No uds socket")
                raise
            else:
                return await asyncio.wait_for(
                    self._receive_message(self._socket),
                    timeout=timeout
                )
        except asyncio.TimeoutError:
            return None
        except Exception as e:
            raise TransportError(f"Failed to receive: {e}")
    
    async def _send_message(self, sock: socket.socket, data: bytes) -> None:
        """Send length-prefixed message."""
        loop = asyncio.get_event_loop()
        
        # Send length header
        length = len(data)
        header = struct.pack(MESSAGE_HEADER_FORMAT, length)
        await loop.sock_sendall(sock, header)
        
        # Send data
        await loop.sock_sendall(sock, data)
    
    async def _receive_message(self, sock: socket.socket) -> Optional[bytes]:
        """Receive length-prefixed message."""
        loop = asyncio.get_event_loop()
        
        try:
            # Receive length header
            header = await loop.sock_recv(sock, MESSAGE_HEADER_SIZE)
            if not header or len(header) < MESSAGE_HEADER_SIZE:
                return None
            
            length = struct.unpack(MESSAGE_HEADER_FORMAT, header)[0]
            
            # Receive data
            data = b''
            remaining = length
            
            while remaining > 0:
                chunk = await loop.sock_recv(sock, min(remaining, 4096))
                if not chunk:
                    return None
                data += chunk
                remaining -= len(chunk)
            
            return data
            
        except Exception as e:
            logger.error(f"❌ Receive error: {e}")
            return None
    
    async def close(self) -> None:
        """Close socket and cleanup."""
        logger.debug(f"🛑 Closing socket: {self.socket_path}")
        
        # Stop server
        if self._is_server:
            self._is_server = False
            if self._server_task:
                self._server_task.cancel()
                try:
                    await self._server_task
                except asyncio.CancelledError:
                    pass
        
        # Close client socket
        if self._socket:
            self._socket.close()
            self._socket = None
        
        # Close server socket
        if self._server_socket:
            self._server_socket.close()
            self._server_socket = None
        
        # Remove socket file
        if self.socket_path.exists():
            try:
                self.socket_path.unlink()
            except Exception as e:
                logger.warning(f"⚠️ Failed to remove socket file: {e}")
        
        self._connected = False
        logger.debug("✅ Socket closed")
    
    def is_connected(self) -> bool:
        """Check if socket is connected."""
        return self._connected
    
    def is_server(self) -> bool:
        """Check if socket is in server mode."""
        return self._is_server
