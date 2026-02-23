"""
Zenoh Session Management

Manages Zenoh session lifecycle, configuration, and connection to Zenoh router.
"""
from __future__ import annotations

import asyncio
import json
import logging

from dataclasses import dataclass, field
from typing import Any, Dict, Optional, List
from enum import Enum

logger = logging.getLogger(__name__)

# Import external Eclipse Zenoh library
try:
    import zenoh
    ZENOH_AVAILABLE = True
except ImportError:
    ZENOH_AVAILABLE = False
    logger.warning(
        "⚠️ Eclipse Zenoh library not available. Zenoh transport disabled. "
        "Install via: pip install eclipse-zenoh"
    )


class SessionMode(str, Enum):
    """Zenoh session modes."""
    PEER = "peer"        # Peer-to-peer mode
    CLIENT = "client"    # Client mode (connects to router)
    ROUTER = "router"    # Router mode (not typically used in modules)


@dataclass
class SessionConfig:
    """
    Configuration for Zenoh session.
    
    Attributes:
        mode: Session mode (peer/client/router)
        connect: List of endpoints to connect to (e.g., ["tcp/zenoh-router:7447"])
        listen: List of endpoints to listen on
        id: Optional session ID
        scouting_multicast: Enable multicast scouting for peer discovery
        timeout_ms: Timeout for operations in milliseconds
    """
    mode: SessionMode = SessionMode.CLIENT
    connect: List[str] = field(default_factory=lambda: ["tcp/zenoh-router:7447"])
    listen: List[str] = field(default_factory=list)
    id: Optional[str] = None
    scouting_multicast: bool = True
    timeout_ms: int = 5000
    
    def to_zenoh_config(self) -> Dict[str, Any]:
        """Convert to Zenoh configuration dictionary."""
        config = {
            "mode": self.mode.value,
            "connect": {"endpoints": self.connect},
        }
        
        if self.listen:
            config["listen"] = {"endpoints": self.listen}
        
        if self.id:
            config["id"] = self.id
        
        # Note: scouting config structure varies by Zenoh version
        # Omit if not explicitly disabled to use Zenoh defaults
        
        return config


class ZenohSession:
    """
    Manages Zenoh session for VYRA modules.
    
    Provides:
    - Session lifecycle management
    - Connection to Zenoh router
    - Query/Reply, Pub/Sub, and Task primitives
    - Automatic reconnection
    
    Example:
        >>> config = SessionConfig(
        ...     mode=SessionMode.CLIENT,
        ...     connect=["tcp/zenoh-router:7447"]
        ... )
        >>> session = ZenohSession(config)
        >>> await session.open()
        >>> 
        >>> # Use session for communication
        >>> pub = session.declare_publisher("/topic")
        >>> pub.put("Hello Zenoh")
        >>> 
        >>> await session.close()
    """
    
    def __init__(self, config: Optional[SessionConfig] = None):
        """
        Initialize Zenoh session manager.
        
        Args:
            config: Session configuration
        """
        self.config = config or SessionConfig()
        self._session: Optional[Any] = None  # zenoh.Session
        self._opened = False
        self._lock = asyncio.Lock()
        
        if not ZENOH_AVAILABLE:
            raise ImportError(
                "Zenoh not available. Install via: pip install eclipse-zenoh"
            )
    
    @property
    def is_open(self) -> bool:
        """Check if session is open."""
        return self._opened and self._session is not None
    
    @property
    def session(self) -> Any:
        """Get underlying Zenoh session."""
        if not self.is_open:
            raise RuntimeError("Zenoh session not open. Call open() first.")
        return self._session
    
    async def open(self) -> bool:
        """
        Open Zenoh session and connect to router.
        
        Returns:
            bool: True if session opened successfully
        """
        async with self._lock:
            if self._opened:
                logger.warning("Zenoh session already open")
                return True
            
            try:
                logger.info("🔧 Opening Zenoh session...")
                logger.debug(f"Zenoh config: {self.config.to_zenoh_config()}")
                
                # Create Zenoh configuration
                zenoh_config = zenoh.Config.from_json5(
                    json.dumps(self.config.to_zenoh_config())
                )
                
                # Open session (blocking call, wrapped in executor)
                loop = asyncio.get_event_loop()
                self._session = await loop.run_in_executor(
                    None,
                    lambda: zenoh.open(zenoh_config)
                )
                
                if self._session is None:
                    raise RuntimeError("Zenoh session not initialized")
        
                self._opened = True
                logger.info("✅ Zenoh session opened successfully")
                try:
                    info = self._session.info()
                    # zenoh 1.x: zid is a property, not a callable
                    zid = info.zid() if callable(info.zid) else info.zid
                    logger.info(f"Session ID: {zid}")
                except Exception:
                    logger.info("✅ Zenoh session opened (session ID unavailable)")
                
                return True
                
            except Exception as e:
                logger.error(f"❌ Failed to open Zenoh session: {e}")
                self._session = None
                self._opened = False
                raise
    
    async def close(self) -> None:
        """Close Zenoh session and cleanup resources."""
        async with self._lock:
            if not self._opened:
                logger.debug("Zenoh session already closed")
                return
            
            try:
                logger.info("🛑 Closing Zenoh session...")
                
                if self._session:
                    # Close session (blocking call)
                    loop = asyncio.get_event_loop()
                    await loop.run_in_executor(
                        None,
                        self._session.close
                    )
                    self._session = None
                
                self._opened = False
                logger.info("✅ Zenoh session closed")
                
            except Exception as e:
                logger.error(f"❌ Error closing Zenoh session: {e}")
                raise
    
    def declare_publisher(self, key_expr: str, **kwargs) -> Any:
        """
        Declare a Zenoh publisher.
        
        Args:
            key_expr: Key expression (topic path)
            **kwargs: Additional publisher options
            
        Returns:
            Zenoh Publisher
        """
        if not self.is_open:
            raise RuntimeError("Session not open")
        
        if self._session is None:
            raise RuntimeError("Zenoh session not initialized")

        logger.debug(f"Declaring Zenoh publisher: {key_expr}")
        return self._session.declare_publisher(key_expr, **kwargs)
    
    def declare_subscriber(self, key_expr: str, callback, **kwargs) -> Any:
        """
        Declare a Zenoh subscriber.
        
        Args:
            key_expr: Key expression (topic path)
            callback: Callback for incoming messages
            **kwargs: Additional subscriber options
            
        Returns:
            Zenoh Subscriber
        """
        if not self.is_open:
            raise RuntimeError("Session not open")
        
        if self._session is None:
            raise RuntimeError("Zenoh session not initialized")

        logger.debug(f"Declaring Zenoh subscriber: {key_expr}")
        return self._session.declare_subscriber(key_expr, callback, **kwargs)
    
    def declare_queryable(self, key_expr: str, callback, **kwargs) -> Any:
        """
        Declare a Zenoh queryable (service server).
        
        Args:
            key_expr: Key expression (service path)
            callback: Callback for query handling
            **kwargs: Additional queryable options
            
        Returns:
            Zenoh Queryable
        """
        if not self.is_open:
            raise RuntimeError("Session not open")
        
        if self._session is None:
            raise RuntimeError("Zenoh session not initialized")

        logger.debug(f"Declaring Zenoh queryable: {key_expr}")
        return self._session.declare_queryable(key_expr, callback, **kwargs)
    
    async def get(self, key_expr: str, timeout: Optional[float] = None, **kwargs) -> Any:
        """
        Send a Zenoh query (client request).
        
        Args:
            key_expr: Key expression (service path)
            timeout: Query timeout in seconds
            **kwargs: Additional query parameters
            
        Returns:
            Query replies
        """
        if not self.is_open:
            raise RuntimeError("Session not open")
        
        logger.debug(f"Sending Zenoh query: {key_expr}")
        
        timeout_ms = timeout * 1000 if timeout else self.config.timeout_ms
        
        # Execute query in executor (blocking)
        loop = asyncio.get_event_loop()

        if self._session is None:
            raise RuntimeError("Zenoh session not initialized")
        
        return await loop.run_in_executor(
            None,
            lambda: self._session.get(key_expr, timeout=timeout_ms, **kwargs) # pyright: ignore[reportOptionalMemberAccess]
        )
    
    def __enter__(self):
        """Context manager support (sync)."""
        raise NotImplementedError("Use async context manager: async with ZenohSession()")
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager support (sync)."""
        pass
    
    async def __aenter__(self):
        """Async context manager entry."""
        await self.open()
        return self
    
    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """Async context manager exit."""
        await self.close()
