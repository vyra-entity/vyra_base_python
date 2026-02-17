"""
UDS Server Implementation

Uses Unix Stream Sockets for request/response RPC pattern.
"""

import logging
import socket
import json
import asyncio
from typing import Optional, Any, Callable, Awaitable
from pathlib import Path

from vyra_base.com.core.types import InterfaceType, VyraServer, ProtocolType
from vyra_base.com.core.topic_builder import TopicBuilder
from vyra_base.com.core.exceptions import InterfaceError

logger = logging.getLogger(__name__)


class VyraServerImpl(VyraServer):
    """
    Unix Domain Socket server implementation using stream sockets.
    
    Pattern: Listens for connections, receives request, sends response.
    Socket path: /tmp/vyra_sockets/srv_{module}_{service}.sock
    """
    
    def __init__(
        self,
        name: str,
        topic_builder: TopicBuilder,
        response_callback: Callable[[Any], Awaitable[Any]],
        service_type: type,
        module_name: str = "vyra",
        **kwargs
    ):
        super().__init__(name, topic_builder, response_callback, ProtocolType.UDS, **kwargs)
        self.service_type = service_type
        self._module_name = module_name
        self._socket: Optional[socket.socket] = None
        self._socket_dir = Path("/tmp/vyra_sockets")
        self._socket_path = None  # Set in initialize()
        self._server_task: Optional[asyncio.Task] = None
        
    async def initialize(self) -> bool:
        """Initialize UDS server."""
        try:
            service_name = self.topic_builder.build(self.name)
            self._socket_path = self._socket_dir / f"srv_{self._module_name}_{service_name}.sock"
            
            # Create socket directory
            self._socket_dir.mkdir(parents=True, exist_ok=True)
            
            # Create stream socket
            self._socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            self._socket.setblocking(False)
            
            # Remove old socket file if exists
            if self._socket_path.exists():
                self._socket_path.unlink()
                
            # Bind and listen
            self._socket.bind(str(self._socket_path))
            self._socket.listen(5)
            
            self._initialized = True
            logger.info(f"✅ VyraServer '{self.name}' initialized: {self._socket_path}")
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to initialize VyraServer '{self.name}': {e}")
            raise InterfaceError(f"Server initialization failed: {e}")
    
    async def serve(self) -> bool:
        """
        Start serving requests.
        
        Returns:
            True on success
        """
        if not self._socket:
            logger.error("Server not initialized")
            return False
            
        try:
            # Start server task
            self._server_task = asyncio.create_task(self._accept_loop())
            
            logger.info(f"📡 VyraServer '{self.name}' serving on: {self._socket_path}")
            return True
            
        except Exception as e:
            logger.error(f"❌ VyraServer '{self.name}' serve failed: {e}")
            return False
    
    async def _accept_loop(self):
        """Accept incoming connections."""
        try:
            loop = asyncio.get_event_loop()
            
            if not self._socket:
                logger.error("VyraServer socket not available in accept loop")
                return

            while True:
                try:
                    # Accept connection (async)
                    client_socket, _ = await loop.sock_accept(self._socket)
                    
                    # Handle client in separate task
                    asyncio.create_task(self._handle_client(client_socket))
                    
                except Exception as e:
                    logger.error(f"❌ VyraServer '{self.name}' accept failed: {e}")
                    await asyncio.sleep(0.1)
                    
        except asyncio.CancelledError:
            logger.debug("VyraServer accept loop canceled")
        except Exception as e:
            logger.error(f"❌ VyraServer '{self.name}' accept loop failed: {e}")
    
    async def _handle_client(self, client_socket: socket.socket):
        """Handle individual client request."""
        loop = asyncio.get_event_loop()
        
        try:
            # Receive request (with length prefix: 4 bytes + data)
            length_bytes = await loop.sock_recv(client_socket, 4)
            if not length_bytes:
                return
                
            if not self.response_callback:
                logger.warning("No response callback defined, closing connection")
                return
            
            request_length = int.from_bytes(length_bytes, byteorder='big')
            request_data = await loop.sock_recv(client_socket, request_length)
            
            # Deserialize request
            request_dict = json.loads(request_data.decode('utf-8'))
            
            # Convert dict to srv_type.Request
            if self.service_type:
                # Try to instantiate service type Request from dict
                try:
                    if hasattr(self.service_type, 'Request'):
                        # ROS2-style service with Request inner class
                        request_obj = self.service_type.Request(**request_dict)
                    elif hasattr(request_dict, '__dict__'):
                        # Already an object
                        request_obj = request_dict
                    else:
                        # Fallback: use dict directly
                        request_obj = request_dict
                except Exception as e:
                    logger.warning(f"Could not convert dict to {self.service_type}.Request: {e}")
                    request_obj = request_dict
            else:
                request_obj = request_dict
            
            # Call user callback
            response = await self.response_callback(request_obj)
            
            # Convert response to srv_type.Response
            if self.service_type and hasattr(response, '__dict__'):
                # If response is an object with attributes, convert to dict
                if hasattr(response, 'to_dict'):
                    response_dict = response.to_dict()
                elif hasattr(response, '__dict__'):
                    # Extract public attributes
                    response_dict = {k: v for k, v in response.__dict__.items() if not k.startswith('_')}
                else:
                    response_dict = {'result': str(response)}
            elif isinstance(response, dict):
                response_dict = response
            else:
                response_dict = {'result': str(response)}
                
            response_bytes = json.dumps(response_dict).encode('utf-8')
            
            # Send response (with length prefix)
            length_prefix = len(response_bytes).to_bytes(4, byteorder='big')
            await loop.sock_sendall(client_socket, length_prefix + response_bytes)
            
        except Exception as e:
            logger.error(f"❌ VyraServer '{self.name}' client handling failed: {e}")
        finally:
            client_socket.close()
    
    async def cleanup(self):
        """Cleanup UDS resources."""
        if self._server_task:
            self._server_task.cancel()
            try:
                await self._server_task
            except asyncio.CancelledError:
                pass
            self._server_task = None
            
        if self._socket:
            self._socket.close()
            self._socket = None
        
        # Remove socket file
        if self._socket_path and self._socket_path.exists():
            try:
                self._socket_path.unlink()
            except Exception as e:
                logger.warning(f"Failed to remove socket file: {e}")
                
        logger.info(f"🔄 VyraServer '{self.name}' cleaned up: {self._socket_path}")
    
    @property
    def interface_type(self) -> InterfaceType:
        return InterfaceType.SERVER
