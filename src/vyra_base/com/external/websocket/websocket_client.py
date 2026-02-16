"""
WebSocket Client for vyra_base

High-level wrapper for WebSocket communication.

Example:
    >>> client = WebSocketClient(url="wss://api.example.com/ws")
    >>> await client.connect()
    >>> await client.send({"type": "subscribe", "channel": "updates"})
    >>> 
    >>> async def on_message(message):
    ...     print(f"Received: {message}")
    >>> await client.listen(on_message)
"""
from __future__ import annotations

import logging
import asyncio
from re import A
from typing import Any, Optional, Callable
import json

from vyra_base.helper.error_handler import ErrorTraceback

logger = logging.getLogger(__name__)

# Check if websockets is available
try:
    import websockets
    from websockets.asyncio.client import connect as ws_connect
    from websockets.asyncio.client import ClientConnection
    WEBSOCKET_AVAILABLE = True
except ImportError:
    WEBSOCKET_AVAILABLE = False


class WebSocketClient:
    """
    High-level WebSocket Client wrapper.
    
    Features:
    - Send/Receive messages
    - JSON serialization
    - Automatic reconnection
    - Callback-based message handling
    
    Args:
        url: WebSocket URL (ws:// or wss://)
        timeout: Connection timeout in seconds
        headers: Optional headers for connection
    """
    
    @ErrorTraceback.w_check_error_exist
    def __init__(
        self,
        url: str,
        timeout: float = 30.0,
        headers: Optional[dict] = None,
    ):
        """Initialize WebSocket client."""
        if not WEBSOCKET_AVAILABLE:
            raise ImportError("websockets not installed. Install with: pip install websockets")
        
        self.url = url
        self.timeout = timeout
        self.headers = headers or {}
        
        self._ws: Optional[ClientConnection] = None
        self._connected = False
        self._listener_task: Optional[asyncio.Task] = None
        self._message_callback: Optional[Callable] = None
    
    @ErrorTraceback.w_check_error_exist
    async def connect(self) -> None:
        """Establish WebSocket connection."""
        try:
            logger.info(f"🔌 Connecting to WebSocket: {self.url}")
            
            self._ws = await ws_connect(
                self.url,
                extra_headers=self.headers,
                ping_interval=20,
                ping_timeout=10,
            )
            
            self._connected = True
            logger.info(f"✅ Connected to WebSocket: {self.url}")
            
        except Exception as e:
            logger.error(f"❌ Failed to connect to WebSocket: {e}")
            raise
    
    @ErrorTraceback.w_check_error_exist
    async def close(self) -> None:
        """Close WebSocket connection."""
        self._connected = False
        
        if self._listener_task:
            self._listener_task.cancel()
            try:
                await self._listener_task
            except:
                pass
            self._listener_task = None
        
        if self._ws:
            await self._ws.close()
            self._ws = None
        
        logger.debug(f"WebSocket connection closed: {self.url}")
    
    async def _ensure_connected(self) -> ClientConnection:
        """Ensure WebSocket is connected."""
        if not self._connected or self._ws is None:
            await self.connect()
        
        if self._ws is None:
            raise RuntimeError("WebSocket connection failed")
        
        return self._ws
    
    @ErrorTraceback.w_check_error_exist
    async def send(self, message: Any) -> None:
        """
        Send message via WebSocket.
        
        Args:
            message: Message to send (will be JSON-encoded if dict)
        """
        ws = await self._ensure_connected()
        
        try:
            # Serialize message
            if isinstance(message, dict):
                data = json.dumps(message)
            elif isinstance(message, str):
                data = message
            else:
                data = str(message)
            
            await ws.send(data)
            logger.debug(f"📤 WebSocket sent: {data[:100]}")
            
        except Exception as e:
            logger.error(f"❌ WebSocket send failed: {e}")
            raise
    
    @ErrorTraceback.w_check_error_exist
    async def receive(self, timeout: Optional[float] = None) -> Any:
        """
        Receive single message from WebSocket.
        
        Args:
            timeout: Optional timeout override
            
        Returns:
            Received message (parsed from JSON if possible)
        """
        ws = await self._ensure_connected()
        
        try:
            timeout = timeout or self.timeout
            message = await asyncio.wait_for(ws.recv(), timeout=timeout)
            
            # Try to parse as JSON
            try:
                data = json.loads(message)
            except:
                data = message
            
            logger.debug(f"📥 WebSocket received: {str(data)[:100]}")
            return data
            
        except asyncio.TimeoutError:
            raise TimeoutError(f"WebSocket receive timed out after {timeout}s")
        except Exception as e:
            logger.error(f"❌ WebSocket receive failed: {e}")
            raise
    
    @ErrorTraceback.w_check_error_exist
    async def listen(self, callback: Callable) -> None:
        """
        Start listening for messages in background.
        
        Args:
            callback: Async callback function(message)
        """
        ws = await self._ensure_connected()
        
        self._message_callback = callback
        self._listener_task = asyncio.create_task(self._listen_loop())
        logger.info(f"📡 WebSocket listening started: {self.url}")
    
    async def _listen_loop(self) -> None:
        """Background task for listening to messages."""
        while self._connected and self._ws:
            try:
                message = await self._ws.recv()
                
                # Try to parse as JSON
                try:
                    data = json.loads(message)
                except:
                    data = message
                
                # Call callback
                if self._message_callback:
                    await self._message_callback(data)
                    
            except websockets.exceptions.ConnectionClosed:
                logger.warning("WebSocket connection closed")
                self._connected = False
                break
            except Exception as e:
                logger.error(f"❌ Error in WebSocket listener: {e}")
    
    @ErrorTraceback.w_check_error_exist
    async def health_check(self) -> bool:
        """
        Check if WebSocket is connected.
        
        Returns:
            True if connected, False otherwise
        """
        return self._connected and self._ws is not None
