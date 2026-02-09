"""
UDS Speaker Implementation

Publish-Subscribe communication over Unix Domain Sockets.
Provides datagram-based pub/sub with automatic socket management.
"""
import asyncio
import json
import logging
from pathlib import Path
from typing import Any, Callable, Coroutine, Optional, Set
import socket as sock

from vyra_base.com.core.types import VyraSpeaker, ProtocolType
from vyra_base.com.core.exceptions import SpeakerError, TransportError
from vyra_base.helper.logger import Logger
from vyra_base.com.transport.t_uds.communication import UDS_SOCKET_DIR
from vyra_base.com.core.topic_builder import TopicBuilder, InterfaceType


class UDSSpeaker(VyraSpeaker):
    """
    Speaker interface over Unix Domain Sockets with datagram mode.
    
    Features:
    - Datagram-based pub/sub
    - JSON serialization
    - Multiple subscriber support
    - Non-blocking communication
    
    Naming Convention:
        Uses TopicBuilder for consistent naming: <module_name>_<module_id>/<function_name>
        Socket path: /tmp/vyra_uds/<module_name>_<module_id>_<function_name>_pub.sock
    
    Example:
        >>> # Publisher side
        >>> speaker = UDSSpeaker(
        ...     "sensor_data",
        ...     topic_builder=builder,
        ...     is_publisher=True
        ... )
        >>> await speaker.initialize()
        >>> await speaker.publish({"temperature": 25.5})
        >>> 
        >>> # Subscriber side
        >>> async def on_message(data):
        ...     print(f"Received: {data}")
        >>> 
        >>> speaker = UDSSpeaker(
        ...     "sensor_data",
        ...     topic_builder=builder,
        ...     callback=on_message,
        ...     is_publisher=False
        ... )
        >>> await speaker.initialize()
    """
    
    def __init__(
        self,
        name: str,
        topic_builder: TopicBuilder,
        callback: Optional[Callable[[Any], None] | Coroutine[Any, Any, None]] = None,
        is_publisher: bool = True,
        **kwargs
    ):
        """Initialize UDS Speaker."""
        # Apply topic builder if provided
        if topic_builder:
            name = topic_builder.build(name, interface_type=InterfaceType.SPEAKER)
            Logger.debug(f"Applied TopicBuilder: {name}")
        else:
            Logger.warning(f"No TopicBuilder provided for UDSSpeaker '{name}'")
        
        super().__init__(name, topic_builder, ProtocolType.UDS, **kwargs)
        
        self.callback = callback
        self.is_publisher = is_publisher
        self.topic_builder = topic_builder
        
        # Socket setup
        self._socket_path = Path(UDS_SOCKET_DIR) / f"{name.replace('/', '_')}_pub.sock"
        self._socket: Optional[sock.socket] = None
        self._subscriber_task: Optional[asyncio.Task] = None
        self._running = False
        
        # Ensure socket directory exists
        Path(UDS_SOCKET_DIR).mkdir(parents=True, exist_ok=True)
    
    async def initialize(self) -> bool:
        """Initialize UDS speaker (publisher or subscriber)."""
        if self._initialized:
            Logger.warning(f"UDSSpeaker '{self.name}' already initialized")
            return True
        
        try:
            if self.is_publisher:
                # Publisher: create datagram socket
                Logger.info(f"🔧 Creating UDS publisher: {self.name}")
                self._socket = sock.socket(sock.AF_UNIX, sock.SOCK_DGRAM)
                Logger.info(f"✅ UDS publisher created: {self.name}")
            else:
                # Subscriber: bind to socket and start listening
                if not self.callback:
                    raise SpeakerError(f"Callback required for subscriber '{self.name}'")
                
                Logger.info(f"🔧 Creating UDS subscriber: {self.name}")
                
                # Remove existing socket if present
                if self._socket_path.exists():
                    self._socket_path.unlink()
                
                # Create and bind socket
                self._socket = sock.socket(sock.AF_UNIX, sock.SOCK_DGRAM)
                self._socket.bind(str(self._socket_path))
                self._socket.setblocking(False)
                
                # Start subscriber task
                self._running = True
                self._subscriber_task = asyncio.create_task(self._subscriber_loop())
                
                Logger.info(f"✅ UDS subscriber created: {self.name}")
            
            self._initialized = True
            return True
            
        except Exception as e:
            Logger.error(f"❌ Failed to initialize UDSSpeaker '{self.name}': {e}")
            raise SpeakerError(f"Failed to initialize UDSSpeaker: {e}")
    
    async def shutdown(self) -> None:
        """Shutdown and cleanup UDS speaker resources."""
        if not self._initialized:
            return
        
        Logger.info(f"🛑 Shutting down UDSSpeaker: {self.name}")
        
        # Stop subscriber task
        if self._subscriber_task:
            self._running = False
            self._subscriber_task.cancel()
            try:
                await self._subscriber_task
            except asyncio.CancelledError:
                pass
            self._subscriber_task = None
        
        # Close socket
        if self._socket:
            self._socket.close()
            self._socket = None
        
        # Remove socket file (subscriber only)
        if not self.is_publisher and self._socket_path.exists():
            try:
                self._socket_path.unlink()
            except Exception as e:
                Logger.warning(f"Failed to remove socket file: {e}")
        
        self._initialized = False
        Logger.info(f"✅ UDSSpeaker '{self.name}' shutdown complete")
    
    async def publish(self, data: Any) -> None:
        """
        Publish data to all subscribers.
        
        Args:
            data: Data to publish (will be JSON serialized)
            
        Raises:
            SpeakerError: If not initialized or publish fails
        """
        if not self._initialized:
            raise SpeakerError(f"UDSSpeaker '{self.name}' not initialized")
        
        if not self.is_publisher:
            raise SpeakerError(f"Cannot publish on subscriber '{self.name}'")
        
        if not self._socket:
            raise SpeakerError(f"UDS socket not available for publishing on '{self.name}'")

        try:
            # Serialize data
            message = json.dumps(data).encode('utf-8')
            
            # Send to subscriber socket
            subscriber_path = self._socket_path
            if subscriber_path.exists():
                self._socket.sendto(message, str(subscriber_path))
                Logger.debug(f"📤 Published to UDS: {self.name}")
            else:
                Logger.debug(f"No subscribers for: {self.name}")
                
        except Exception as e:
            Logger.error(f"❌ Failed to publish to UDS '{self.name}': {e}")
            raise SpeakerError(f"Publish failed: {e}")
    
    async def _subscriber_loop(self) -> None:
        """Background task for receiving messages."""
        Logger.info(f"🎧 Starting UDS subscriber loop: {self.name}")
        
        while self._running:
            try:
                # Wait for data with timeout
                await asyncio.sleep(0.01)  # Prevent busy loop
                
                if not self._socket:
                    Logger.error("UDS socket not available for receiving")
                    continue

                try:
                    data, addr = self._socket.recvfrom(65536)  # Max UDP size
                    
                    # Deserialize and call callback
                    message = json.loads(data.decode('utf-8'))
                    Logger.debug(f"📥 Received UDS message: {self.name}")
                    
                    if self.callback:
                        if asyncio.iscoroutinefunction(self.callback):
                            await self.callback(message)
                        else:
                            self.callback(message) # pyright: ignore[reportCallIssue]
                            
                except BlockingIOError:
                    # No data available, continue
                    continue
                    
            except asyncio.CancelledError:
                break
            except Exception as e:
                Logger.error(f"❌ Error in subscriber loop '{self.name}': {e}")
                await asyncio.sleep(0.1)
        
        Logger.info(f"🛑 Stopped UDS subscriber loop: {self.name}")
    
    def get_socket_path(self) -> Path:
        """Get the socket file path."""
        return self._socket_path
