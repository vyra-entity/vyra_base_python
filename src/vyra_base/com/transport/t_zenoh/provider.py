"""
Zenoh Protocol Provider

Implements AbstractProtocolProvider for Zenoh transport.
Provides efficient, scalable communication via Zenoh router.
"""
from __future__ import annotations

import asyncio
import logging
from typing import Any, Callable, Optional, Dict

from vyra_base.com.core.topic_builder import TopicBuilder
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
from vyra_base.com.transport.t_zenoh.session import ZenohSession, SessionConfig, SessionMode
from vyra_base.com.transport.t_zenoh.vyra_models import ZenohSpeaker, ZenohCallable, ZenohJob
from vyra_base.com.transport.t_zenoh.communication.serializer import SerializationFormat
from vyra_base.com.providers.protocol_provider import AbstractProtocolProvider

logger = logging.getLogger(__name__)

# Check if zenoh is available
try:
    import zenoh
    ZENOH_AVAILABLE = True
except ImportError:
    ZENOH_AVAILABLE = False
    logger.warning(
        "⚠️ zenoh-python not available. Zenoh transport disabled. "
        "Install via: pip install eclipse-zenoh"
    )


class ZenohProvider(AbstractProtocolProvider):
    """
    Protocol provider for Zenoh transport.
    
    Features:
    - Efficient Pub/Sub with zero-copy capabilities
    - Query/Reply for request-response patterns
    - Router-based scalability
    - Built-in discovery and fault tolerance
    - Multi-protocol support (TCP, UDP, shared memory)
    
    Requirements:
    - eclipse-zenoh Python package
    - Zenoh router (typically Docker service)
    
    Example:
        >>> # Initialize provider
        >>> provider = ZenohProvider(ProtocolType.ZENOH)
        >>> 
        >>> if await provider.check_availability():
        ...     await provider.initialize(config={
        ...         "mode": "client",
        ...         "connect": ["tcp/zenoh-router:7447"]
        ...     })
        ...     
        ...     # Create callable (query/reply)
        ...     async def handle_request(req):
        ...         return {"result": req["value"] * 2}
        ...     
        ...     callable = await provider.create_callable(
        ...         "/calculate",
        ...         handle_request
        ...     )
        ...     
        ...     # Create speaker (pub/sub)
        ...     speaker = await provider.create_speaker("/sensor_data")
    """
    
    def __init__(
        self,
        module_name: str,
        module_id: str,
        protocol: ProtocolType = ProtocolType.ZENOH
    ):
        """
        Initialize Zenoh provider.
        
        Args:
            protocol: Protocol type (must be ZENOH)
        """
        super().__init__(protocol)
        self._session: Optional[ZenohSession] = None
        self._format = SerializationFormat.JSON

        # Topic builder for consistent naming
        self._topic_builder = TopicBuilder(module_name, module_id)
    
    async def check_availability(self) -> bool:
        """
        Check if Zenoh is available.
        
        Returns:
            bool: True if zenoh-python is installed
        """
        self._available = ZENOH_AVAILABLE
        
        if not self._available:
            logger.warning("Zenoh not available")
        else:
            logger.debug("✅ Zenoh is available")
        
        return self._available
    
    async def initialize(self, config: Optional[Dict[str, Any]] = None) -> bool:
        """
        Initialize Zenoh provider and open session.
        
        Args:
            config: Configuration dictionary:
                - mode: "peer", "client", or "router" (default: "client")
                - connect: List of endpoints (default: ["tcp/zenoh-router:7447"])
                - listen: List of listen endpoints
                - format: Serialization format (default: "json")
                
        Returns:
            bool: True if initialization successful
        """
        if self._initialized:
            logger.warning("Zenoh provider already initialized")
            return True
        
        if not self._available:
            raise ProtocolUnavailableError("Zenoh is not available")
        
        try:
            logger.info("🔧 Initializing Zenoh provider...")
            
            # Parse configuration
            config = config or {}
            self._config.update(config)
            
            mode_str = config.get("mode", "client")
            mode = SessionMode(mode_str)
            
            connect = config.get("connect", ["tcp/zenoh-router:7447"])
            listen = config.get("listen", [])
            
            format_str = config.get("format", "json")
            self._format = SerializationFormat(format_str)
            
            # Create session config
            session_config = SessionConfig(
                mode=mode,
                connect=connect,
                listen=listen
            )
            
            # Create and open session
            self._session = ZenohSession(session_config)
            await self._session.open()
            
            self._initialized = True
            logger.info("✅ Zenoh provider initialized")
            logger.info(f"Session ID: {self._session.session.info().zid()}")
            
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to initialize Zenoh provider: {e}")
            raise ProviderError(f"Zenoh initialization failed: {e}")
    
    async def shutdown(self) -> None:
        """Shutdown Zenoh provider and close session."""
        if not self._initialized:
            logger.debug("Zenoh provider not initialized, nothing to shutdown")
            return
        
        try:
            logger.info("🛑 Shutting down Zenoh provider...")
            
            if self._session:
                await self._session.close()
                self._session = None
            
            self._initialized = False
            logger.info("✅ Zenoh provider shutdown complete")
            
        except Exception as e:
            logger.error(f"❌ Error shutting down Zenoh provider: {e}")
            raise
    
    async def create_callable(
        self,
        name: str,
        callback: Optional[Callable],
        **kwargs
    ) -> VyraCallable:
        """
        Create a Zenoh callable (query/reply).
        
        Args:
            name: Service name (key expression)
            callback: Async callback for request handling (None for client)
            **kwargs: Additional parameters:
                - is_callable: True for server, False for client (auto-detected from callback if not provided)
                - timeout: Timeout for client calls in seconds (default: 5.0)
                - format: Override serialization format
                
        Returns:
            VyraCallable: Zenoh callable interface
        """
        self.require_initialization()
        
        try:
            logger.debug(f"Creating Zenoh callable: {name}")
            
            # Check is_callable flag (defaults to True if callback provided, False otherwise)
            is_callable = kwargs.pop("is_callable", callback is not None)
            is_server = is_callable  # For compatibility with existing code
            
            role = "server" if is_callable else "client"
            logger.info(f"🔧 Creating Zenoh callable {role}: {name}")
            
            # Ensure callback matches is_callable flag
            if is_callable and callback is None:
                raise ProviderError("Callback required for callable server (is_callable=True)")
            if not is_callable and callback is not None:
                logger.debug("Ignoring callback for callable client (is_callable=False)")
                callback = None
            
            timeout = kwargs.get("timeout", 5.0)
            format = kwargs.get("format", self._format)
            
            callable = ZenohCallable(
                name=name,
                topic_builder=self._topic_builder,
                session=self._session,
                callback=callback,
                format=format,
                is_server=is_server,
                timeout=timeout
            )
            
            await callable.initialize()
            
            logger.info(f"✅ Zenoh callable {role} created: {name}")
            return callable
            
        except Exception as e:
            logger.error(f"❌ Failed to create Zenoh callable '{name}': {e}")
            raise ProviderError(f"Failed to create callable: {e}")
    
    async def create_speaker(
        self,
        name: str,
        **kwargs
    ) -> VyraSpeaker:
        """
        Create a Zenoh speaker (pub/sub).
        
        Args:
            name: Topic name (key expression)
            **kwargs: Additional parameters:
                - is_publisher: True for publisher, False for subscriber (default: True)
                - format: Override serialization format
                
        Returns:
            VyraSpeaker: Zenoh speaker interface
        """
        self.require_initialization()
        
        try:
            logger.debug(f"Creating Zenoh speaker: {name}")
            
            is_publisher = kwargs.get("is_publisher", True)
            format = kwargs.get("format", self._format)
            
            speaker = ZenohSpeaker(
                name=name,
                topic_builder=self._topic_builder,
                session=self._session,
                format=format,
                is_publisher=is_publisher
            )
            
            await speaker.initialize()
            
            logger.info(f"✅ Zenoh speaker created: {name}")
            return speaker
            
        except Exception as e:
            logger.error(f"❌ Failed to create Zenoh speaker '{name}': {e}")
            raise ProviderError(f"Failed to create speaker: {e}")
    
    async def create_job(
        self,
        name: str,
        callback: Callable,
        **kwargs
    ) -> VyraJob:
        """
        Create a Zenoh job (long-running task).
        
        Args:
            name: Job name (base key expression)
            callback: Async callback for job execution (None for client)
            **kwargs: Additional parameters:
                - is_job: True for server, False for client (auto-detected from callback if not provided)
                - format: Override serialization format
                
        Returns:
            VyraJob: Zenoh job interface
        """
        self.require_initialization()
        
        try:
            logger.debug(f"Creating Zenoh job: {name}")
            
            # Check is_job flag (defaults to True if callback provided, False otherwise)
            is_job = kwargs.pop("is_job", callback is not None)
            is_server = is_job  # For compatibility with existing code
            
            role = "server" if is_job else "client"
            logger.info(f"🔧 Creating Zenoh job {role}: {name}")
            
            # Ensure callback matches is_job flag
            if is_job and callback is None:
                raise ProviderError("Callback required for job server (is_job=True)")
            if not is_job and callback is not None:
                logger.debug("Ignoring callback for job client (is_job=False)")
                callback = None
            
            format = kwargs.get("format", self._format)
            
            job = ZenohJob(
                name=name,
                topic_builder=self._topic_builder,
                session=self._session,
                callback=callback,
                format=format,
                is_server=is_server
            )
            
            await job.initialize()
            
            logger.info(f"✅ Zenoh job {role} created: {name}")
            return job
            
        except Exception as e:
            logger.error(f"❌ Failed to create Zenoh job '{name}': {e}")
            raise ProviderError(f"Failed to create job: {e}")
    
    def require_initialization(self) -> None:
        """
        Ensure provider is initialized.
        
        Raises:
            ProviderError: If not initialized
        """
        if not self._initialized:
            raise ProviderError("Zenoh provider not initialized. Call initialize() first.")
        
        if not self._session or not self._session.is_open:
            raise ProviderError("Zenoh session not open")
    
    def get_session(self) -> Optional[ZenohSession]:
        """Get the underlying Zenoh session."""
        return self._session
