"""
UDS Client Implementation

Uses Unix Stream Sockets for request/response RPC pattern.
"""

import logging
import socket
import json
import asyncio
from typing import Optional, Any, Callable, Awaitable
from pathlib import Path

from vyra_base.com.core.types import VyraClient, ProtocolType
from vyra_base.com.core.topic_builder import InterfaceType, TopicBuilder
from vyra_base.com.core.exceptions import InterfaceError
from vyra_base.com.transport.t_uds.communication import UDS_SOCKET_DIR

logger = logging.getLogger(__name__)


class VyraClientImpl(VyraClient):
    """
    Unix Domain Socket client implementation using stream sockets.
    
    Pattern: Connects to server socket, sends request, receives response.
    Server socket path: /tmp/vyra_sockets/srv_{module}_{service}.sock
    """
    
    def __init__(
        self,
        name: str,
        topic_builder: TopicBuilder,
        request_callback: Optional[Callable[[Any], Awaitable[None]]],
        service_type: type,
        module_name: str = "vyra",
        **kwargs
    ):
        super().__init__(name, topic_builder, request_callback, ProtocolType.UDS, **kwargs)
        self.service_type = service_type
        self._module_name = module_name
        self._socket_dir = UDS_SOCKET_DIR
        self._server_socket_path = None  # Set in initialize()
        
    async def initialize(self) -> bool:
        """Initialize UDS client."""
        try:
            service_name = self.topic_builder.build(self.name, namespace=self.namespace, subsection=self.subsection)
            self._server_socket_path = self._socket_dir / f"srv_{self._module_name}_{service_name}.sock"
            
            # Check if server socket exists
            if not self._server_socket_path.exists():
                logger.warning(f"Server socket not found: {self._server_socket_path}")
                # Don't fail, server might start later
            
            self._initialized = True
            logger.info(f"✅ VyraClient '{self.name}' initialized: {self._server_socket_path}")
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to initialize VyraClient '{self.name}': {e}")
            raise InterfaceError(f"Client initialization failed: {e}")
    
    async def call(self, request: Any, timeout: float = 5.0) -> Optional[Any]:
        """
        Send request and await response.
        
        Args:
            request: Request message instance or dict
            timeout: Call timeout in seconds
            
        Returns:
            Response object on success, None on failure
        """
        loop = asyncio.get_event_loop()
        client_socket = None
        
        try:
            # Create socket
            client_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            client_socket.setblocking(False)
            
            # Connect to server (with timeout)
            try:
                await asyncio.wait_for(
                    loop.sock_connect(client_socket, str(self._server_socket_path)),
                    timeout=timeout
                )
            except asyncio.TimeoutError:
                logger.error(f"❌ Connection timeout to {self._server_socket_path}")
                return None
            except FileNotFoundError:
                logger.error(f"❌ Server socket not found: {self._server_socket_path}")
                return None
                
            # Serialize request
            # TODO: Support srv_type.Request serialization
            if hasattr(request, 'to_dict'):
                request_dict = request.to_dict()
            elif isinstance(request, dict):
                request_dict = request
            else:
                request_dict = {'request': str(request)}
                
            request_bytes = json.dumps(request_dict).encode('utf-8')
            
            # Send request (with length prefix)
            length_prefix = len(request_bytes).to_bytes(4, byteorder='big')
            await loop.sock_sendall(client_socket, length_prefix + request_bytes)
            
            # Receive response (with timeout)
            try:
                # Receive length prefix
                length_bytes = await asyncio.wait_for(
                    loop.sock_recv(client_socket, 4),
                    timeout=timeout
                )
                if not length_bytes:
                    logger.error("No response received")
                    return None
                    
                response_length = int.from_bytes(length_bytes, byteorder='big')
                
                # Receive response data
                response_data = await asyncio.wait_for(
                    loop.sock_recv(client_socket, response_length),
                    timeout=timeout
                )
                
                # Deserialize response
                response_dict = json.loads(response_data.decode('utf-8'))
                
                # TODO: Convert dict to srv_type.Response
                
                # Optional: Call request_callback
                if self.request_callback:
                    await self.request_callback(response_dict)
                
                return response_dict
                
            except asyncio.TimeoutError:
                logger.error(f"❌ Response timeout after {timeout}s")
                return None
                
        except Exception as e:
            logger.error(f"❌ Call failed: {e}")
            return None
            
        finally:
            if client_socket:
                client_socket.close()
    
    async def cleanup(self):
        """Cleanup resources (stateless client, nothing to cleanup)."""
        logger.info(f"🔄 VyraClient cleaned up: {self.name}")
    
    @property
    def interface_type(self) -> InterfaceType:
        return InterfaceType.CLIENT
