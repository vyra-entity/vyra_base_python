"""
WebSocket Speaker Implementation

Real-time pub/sub communication via WebSocket.
"""
import asyncio
import json
import logging
from typing import Any, Callable, Optional

from vyra_base.com.core.types import VyraSpeaker, ProtocolType
from vyra_base.com.core.exceptions import SpeakerError

logger = logging.getLogger(__name__)


class WebSocketSpeaker(VyraSpeaker):
    """
    WebSocket Speaker for real-time pub/sub messaging.
    
    Features:
    - Bidirectional communication
    - Automatic JSON serialization
    - Connection management
    - Broadcast to multiple clients
    - Message statistics
    
    Example:
        >>> # Server-side broadcaster
        >>> speaker = WebSocketSpeaker(
        ...     "events",
        ...     provider=server_provider,
        ...     callback=handle_message
        ... )
        >>> await speaker.initialize()
        >>> 
        >>> # Broadcast to all clients
        >>> await speaker.shout({"event": "update", "value": 42})
        >>> 
        >>> # Client-side subscriber
        >>> async def on_message(msg):
        ...     print(f"Received: {msg}")
        >>> 
        >>> speaker = WebSocketSpeaker(
        ...     "events",
        ...     provider=client_provider,
        ...     callback=on_message
        ... )
        >>> await speaker.listen(on_message)
    """
    
    def __init__(
        self,
        name: str,
        provider: Any,
        callback: Optional[Callable] = None,
        **kwargs
    ):
        """
        Initialize WebSocket speaker.
        
        Args:
            name: Channel/topic name
            provider: WebSocketProvider instance
            callback: Callback for received messages
            **kwargs: Additional metadata
        """
        super().__init__(name, protocol=ProtocolType.WEBSOCKET, **kwargs)
        self._provider = provider
        self._websocket: Optional[Any] = None
        self._listener_task: Optional[asyncio.Task] = None
        self._stats = {
            "send_count": 0,
            "receive_count": 0,
            "connections": 0,
        }
        self._callback = callback
    
    async def initialize(self) -> bool:
        """Initialize WebSocket speaker."""
        if not self._provider:
            raise SpeakerError("WebSocket provider not provided")
        
        # If server mode, start WebSocket server
        if self._provider.mode == "server":
            if not self._provider._server:
                from websockets.server import serve
                
                async def handler(websocket):
                    """Handle WebSocket connections."""
                    self._provider.add_connection(websocket)
                    self._stats["connections"] += 1
                    
                    try:
                        async for message in websocket:
                            # Call callback if provided
                            if self._callback:
                                try:
                                    # Parse JSON if possible
                                    try:
                                        data = json.loads(message)
                                    except json.JSONDecodeError:
                                        data = message
                                    
                                    self._stats["receive_count"] += 1
                                    
                                    if asyncio.iscoroutinefunction(self._callback):
                                        await self._callback(data)
                                    else:
                                        self._callback(data)
                                        
                                except Exception as e:
                                    logger.error(f"❌ Message handler error: {e}")
                    
                    finally:
                        self._provider.remove_connection(websocket)
                
                # Start server
                self._provider._server = await serve(
                    handler,
                    self._provider.host,
                    self._provider.port
                )
                
                logger.info(
                    f"✅ WebSocket server started: "
                    f"ws://{self._provider.host}:{self._provider.port}"
                )
        
        # If client mode, connect to server
        elif self._provider.mode == "client":
            # Connection will be established in listen()
            logger.debug(f"✅ WebSocket client speaker initialized: {self.name}")
        
        self._initialized = True
        return True
    
    async def shutdown(self) -> None:
        """Shutdown WebSocket speaker."""
        if self._listener_task:
            self._listener_task.cancel()
            try:
                await self._listener_task
            except asyncio.CancelledError:
                pass
        
        if self._websocket:
            await self._websocket.close()
        
        self._initialized = False
        logger.debug(f"✅ WebSocket speaker shutdown: {self.name}")
    
    async def shout(self, message: Any) -> bool:
        """
        Broadcast message via WebSocket.
        
        Args:
            message: Message to broadcast (dict/list serialized to JSON)
            
        Returns:
            bool: True if broadcast successful
            
        Raises:
            SpeakerError: If not initialized or broadcast fails
        """
        if not self._initialized:
            raise SpeakerError(f"Speaker '{self.name}' not initialized")
        
        try:
            # Serialize message
            if isinstance(message, (dict, list)):
                payload = json.dumps(message)
            else:
                payload = str(message)
            
            # Broadcast
            if self._provider.mode == "server":
                await self._provider.broadcast(payload)
                self._stats["send_count"] += 1
                logger.debug(f"📢 Broadcasted to {len(self._provider._connections)} clients")
                return True
                
            elif self._provider.mode == "client":
                if not self._websocket:
                    raise SpeakerError("WebSocket not connected")
                
                await self._websocket.send(payload)
                self._stats["send_count"] += 1
                logger.debug(f"📢 Sent via WebSocket: {payload[:100]}...")
                return True
                
        except Exception as e:
            raise SpeakerError(f"WebSocket broadcast failed: {e}")
    
    async def listen(self, callback: Callable) -> None:
        """
        Subscribe to WebSocket messages (client mode).
        
        Args:
            callback: Async callback function(message) to handle received messages
            
        Raises:
            SpeakerError: If subscription fails
        """
        if not self._initialized:
            raise SpeakerError(f"Speaker '{self.name}' not initialized")
        
        if self._provider.mode != "client":
            raise SpeakerError("listen() only available in client mode")
        
        try:
            from websockets.client import connect
            
            # Connect to server
            self._websocket = await connect(self._provider.uri)
            logger.info(f"✅ Connected to WebSocket server: {self._provider.uri}")
            
            # Start listener loop
            self._listener_task = asyncio.create_task(
                self._listen_loop(callback)
            )
            
        except Exception as e:
            raise SpeakerError(f"WebSocket subscription failed: {e}")
    
    async def _listen_loop(self, callback: Callable) -> None:
        """Background task for receiving messages."""
        try:
            async for message in self._websocket:
                try:
                    # Parse JSON if possible
                    try:
                        data = json.loads(message)
                    except json.JSONDecodeError:
                        data = message
                    
                    self._stats["receive_count"] += 1
                    
                    # Call callback
                    if asyncio.iscoroutinefunction(callback):
                        await callback(data)
                    else:
                        callback(data)
                        
                except Exception as e:
                    logger.error(f"❌ Message handler error: {e}")
                    
        except asyncio.CancelledError:
            logger.debug("✅ WebSocket listener stopped")
            raise
        except Exception as e:
            logger.error(f"❌ WebSocket listener error: {e}")
    
    def get_statistics(self) -> dict:
        """
        Get speaker statistics.
        
        Returns:
            dict: Statistics (send_count, receive_count, connections)
        """
        if self._provider.mode == "server":
            self._stats["connections"] = len(self._provider._connections)
        
        return self._stats.copy()
