"""
WebSocket Protocol Provider

Real-time bidirectional communication using websockets library.
Provides speaker pattern for pub/sub over WebSocket.
"""
import logging
from typing import Any, Callable, Optional, Dict, Set
import asyncio

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

logger = logging.getLogger(__name__)

# Check if websockets is available
try:
    import websockets
    from websockets.server import serve
    from websockets.client import connect
    WEBSOCKET_AVAILABLE = True
except ImportError:
    WEBSOCKET_AVAILABLE = False
    websockets = None
    serve = None
    connect = None
    logger.warning(
        "⚠️ WebSocket not available. Install with: pip install websockets"
    )


class WebSocketProvider(AbstractProtocolProvider):
    """
    Protocol provider for WebSocket communication.
    
    Features:
    - Real-time bidirectional communication
    - Pub/Sub pattern
    - Connection management
    - Automatic reconnection
    - Message broadcasting
    
    Example:
        >>> # Server side
        >>> provider = WebSocketProvider(
        ...     host="0.0.0.0",
        ...     port=8765,
        ...     mode="server"
        ... )
        >>> await provider.initialize()
        >>> 
        >>> # Create speaker
        >>> async def handle_message(message):
        ...     print(f"Received: {message}")
        >>> 
        >>> speaker = await provider.create_speaker(
        ...     "updates",
        ...     callback=handle_message
        ... )
        >>> 
        >>> # Broadcast message
        >>> await speaker.shout({"event": "update", "data": 123})
        >>> 
        >>> # Client side
        >>> client_provider = WebSocketProvider(
        ...     uri="ws://localhost:8765",
        ...     mode="client"
        ... )
        >>> await client_provider.initialize()
    """
    
    def __init__(
        self,
        protocol: ProtocolType = ProtocolType.WEBSOCKET,
        host: str = "0.0.0.0",
        port: int = 8765,
        uri: Optional[str] = None,
        mode: str = "server",  # "server" or "client"
        **kwargs
    ):
        """
        Initialize WebSocket provider.
        
        Args:
            protocol: Protocol type (must be WEBSOCKET)
            host: Server bind address
            port: Server port
            uri: Client connection URI (e.g., "ws://localhost:8765")
            mode: Operation mode ("server" or "client")
            **kwargs: Additional parameters
        """
        super().__init__(protocol)
        self.host = host
        self.port = port
        self.uri = uri or f"ws://{host}:{port}"
        self.mode = mode.lower()
        
        self._server: Optional[Any] = None
        self._websocket: Optional[Any] = None
        self._connections: Set[Any] = set()
        self._running = False
        
        if self.mode not in ["server", "client"]:
            raise ValueError(f"Invalid mode: {mode}. Must be 'server' or 'client'")
    
    async def check_availability(self) -> bool:
        """
        Check if websockets is available.
        
        Returns:
            bool: True if websockets can be imported
        """
        if not WEBSOCKET_AVAILABLE:
            logger.warning("❌ WebSocket not available - install websockets")
            return False
        
        logger.debug("✅ WebSocket available")
        return True
    
    async def initialize(self, config: Optional[Dict[str, Any]] = None) -> bool:
        """
        Initialize WebSocket provider.
        
        Args:
            config: Optional configuration overrides
            
        Returns:
            bool: True if initialization successful
            
        Raises:
            ProviderError: If initialization fails
        """
        if not await self.check_availability():
            raise ProtocolUnavailableError("WebSocket not available")
        
        try:
            if self.mode == "server":
                # Server will be started when first speaker is created
                logger.info(f"✅ WebSocket server ready on {self.host}:{self.port}")
                
            elif self.mode == "client":
                # Client connection will be established when needed
                logger.info(f"✅ WebSocket client initialized for {self.uri}")
            
            self._initialized = True
            self._running = True
            return True
            
        except Exception as e:
            raise ProviderError(f"WebSocket initialization failed: {e}")
    
    async def shutdown(self) -> None:
        """Shutdown WebSocket provider."""
        self._running = False
        
        try:
            # Close all connections
            if self._connections:
                tasks = [ws.close() for ws in self._connections]
                await asyncio.gather(*tasks, return_exceptions=True)
                self._connections.clear()
            
            # Close client connection
            if self._websocket:
                await self._websocket.close()
                self._websocket = None
            
            # Close server
            if self._server:
                self._server.close()
                await self._server.wait_closed()
                self._server = None
            
            logger.info("✅ WebSocket provider shutdown complete")
            
        except Exception as e:
            logger.error(f"❌ WebSocket shutdown error: {e}")
        
        self._initialized = False
    
    async def create_callable(
        self,
        name: str,
        callback: Optional[Callable] = None,
        **kwargs
    ) -> VyraCallable:
        """
        Create WebSocket Callable (request-response pattern).
        
        Note:
            WebSocket is primarily for pub/sub.
            For request-response, consider REST or gRPC.
        """
        raise NotImplementedError(
            "WebSocket Callable not supported. "
            "WebSocket is primarily for pub/sub. Use create_speaker() instead."
        )
    
    async def create_speaker(
        self,
        name: str,
        callback: Optional[Callable] = None,
        **kwargs
    ) -> VyraSpeaker:
        """
        Create WebSocket Speaker for pub/sub.
        
        Args:
            name: Channel/topic name
            callback: Callback for received messages
            **kwargs: Additional parameters
            
        Returns:
            VyraSpeaker: WebSocket speaker instance
        """
        if not self._initialized:
            raise ProviderError("Provider not initialized")
        
        from vyra_base.com.external.websocket.speaker import WebSocketSpeaker
        
        speaker = WebSocketSpeaker(
            name=name,
            provider=self,
            callback=callback,
            **kwargs
        )
        
        await speaker.initialize()
        return speaker
    
    async def create_job(
        self,
        name: str,
        callback: Optional[Callable] = None,
        **kwargs
    ) -> VyraJob:
        """
        Create WebSocket Job.
        
        Note:
            WebSocket doesn't support job patterns natively.
            Consider Redis Streams or gRPC streaming for job/task patterns.
        """
        raise NotImplementedError(
            "WebSocket Job not supported. "
            "Consider Redis Streams or gRPC streaming for job/task patterns."
        )
    
    async def broadcast(self, message: Any) -> None:
        """
        Broadcast message to all connected clients (server mode).
        
        Args:
            message: Message to broadcast
        """
        if not self._initialized:
            raise ProviderError("Provider not initialized")
        
        if self.mode != "server":
            raise ProviderError("Broadcast only available in server mode")
        
        if not self._connections:
            logger.debug("⚠️ No connections to broadcast to")
            return
        
        # Send to all connections
        tasks = []
        for ws in self._connections.copy():
            try:
                tasks.append(ws.send(message))
            except Exception as e:
                logger.warning(f"⚠️ Failed to send to connection: {e}")
                self._connections.discard(ws)
        
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)
    
    def add_connection(self, websocket: Any) -> None:
        """Add a client connection to the connection pool."""
        self._connections.add(websocket)
        logger.debug(f"✅ Connection added. Total: {len(self._connections)}")
    
    def remove_connection(self, websocket: Any) -> None:
        """Remove a client connection from the connection pool."""
        self._connections.discard(websocket)
        logger.debug(f"✅ Connection removed. Total: {len(self._connections)}")
