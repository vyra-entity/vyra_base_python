"""
UDS Protocol Provider

Implements AbstractProtocolProvider for Unix Domain Socket transport.
Provides low-latency local IPC via stream sockets.
"""
import logging
from typing import Any, Callable, Optional, Dict

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
from vyra_base.com.transport.t_uds.communication import UDS_SOCKET_DIR
from vyra_base.com.transport.t_uds.vyra_models import UDSCallable

logger = logging.getLogger(__name__)


class UDSProvider(AbstractProtocolProvider):
    """
    Protocol provider for Unix Domain Socket transport.
    
    Features:
    - Low-latency local IPC
    - Stream-based communication
    - Automatic connection management
    - No serialization overhead (JSON)
    
    Requirements:
    - Unix-like OS (Linux, macOS)
    - File system access to /tmp/vyra_sockets
    
    Limitations:
    - Local machine only
    - No pub/sub pattern (use Callable for request-response)
    
    Example:
        >>> # Initialize provider
        >>> provider = UDSProvider(ProtocolType.UDS)
        >>> 
        >>> if await provider.check_availability():
        ...     await provider.initialize()
        ...     
        ...     # Create callable (server)
        ...     async def handle_request(req):
        ...         return {"result": req["value"] * 2}
        ...     
        ...     callable = await provider.create_callable(
        ...         "calculate",
        ...         handle_request,
        ...         module_name="math_service"
        ...     )
        ...     
        ...     # Create client
        ...     client = await provider.create_callable(
        ...         "calculate",
        ...         None,  # No callback for client
        ...         module_name="math_service"
        ...     )
        ...     result = await client.call({"value": 21})
    """
    
    def __init__(
        self,
        protocol: ProtocolType = ProtocolType.UDS,
        module_name: str = "default"
    ):
        """
        Initialize UDS provider.
        
        Args:
            protocol: Protocol type (must be UDS)
            module_name: Default module name for interfaces
        """
        super().__init__(protocol)
        self.module_name = module_name
        
        # Default configuration
        self._config = {
            "socket_dir": str(UDS_SOCKET_DIR),
            "connect_timeout": 5.0,
            "call_timeout": 5.0,
        }
    
    async def check_availability(self) -> bool:
        """
        Check if UDS transport is available.
        
        Returns:
            bool: Always True on Unix-like systems
        """
        import platform
        system = platform.system()
        
        # UDS available on Unix-like systems
        self._available = system in ('Linux', 'Darwin', 'FreeBSD', 'OpenBSD')
        
        if not self._available:
            logger.warning(
                f"⚠️ UDS transport not available on {system}. "
                f"UDS requires Unix-like OS."
            )
        else:
            logger.info("✅ UDS transport available")
        
        return self._available
    
    async def initialize(self, config: Optional[Dict[str, Any]] = None) -> bool:
        """
        Initialize UDS provider.
        
        Args:
            config: Optional configuration
                - socket_dir: Socket directory (default: /tmp/vyra_sockets)
                - connect_timeout: Connection timeout
                - call_timeout: Default call timeout
                
        Returns:
            bool: True if initialization successful
        """
        if not self._available:
            raise ProtocolUnavailableError(
                "UDS transport not available. Requires Unix-like OS."
            )
        
        if self._initialized:
            logger.warning("⚠️ Provider already initialized")
            return True
        
        # Update configuration
        if config:
            self._config.update(config)
        
        logger.info(
            f"🚀 Initializing UDS provider for module: {self.module_name}"
        )
        
        try:
            # Ensure socket directory exists
            UDS_SOCKET_DIR.mkdir(parents=True, exist_ok=True)
            
            self._initialized = True
            logger.info(
                f"✅ UDS provider initialized for {self.module_name}"
            )
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to initialize provider: {e}")
            return False
    
    async def shutdown(self) -> None:
        """Shutdown the provider and cleanup resources."""
        if not self._initialized:
            return
        
        logger.info(f"🛑 Shutting down UDS provider: {self.module_name}")
        
        # Note: Individual sockets cleanup themselves
        # No global cleanup needed
        
        self._initialized = False
        logger.info("✅ UDS provider shutdown complete")
    
    async def create_callable(
        self,
        name: str,
        callback: Callable,
        **kwargs
    ) -> VyraCallable:
        """
        Create a UDS callable.
        
        Args:
            name: Callable name
            callback: Server-side callback function (None for client)
            **kwargs: Additional parameters
                - module_name: Override default module name
                
        Returns:
            UDSCallable instance
            
        Raises:
            ProviderError: If provider not initialized
        """
        self.require_initialization()
        
        # Get module name
        module_name = kwargs.pop("module_name", self.module_name)
        
        role = "server" if callback else "client"
        logger.info(f"🔧 Creating UDS callable ({role}): {module_name}.{name}")
        
        callable_instance = UDSCallable(
            name=name,
            callback=callback,
            module_name=module_name,
            **kwargs
        )
        
        await callable_instance.initialize()
        
        logger.info(f"✅ UDS callable created ({role}): {module_name}.{name}")
        return callable_instance
    
    async def create_speaker(
        self,
        name: str,
        **kwargs
    ) -> VyraSpeaker:
        """
        Create a speaker interface.
        
        Note: UDS does not natively support pub/sub pattern.
        Use Redis or ROS2 for pub/sub communication.
        
        Raises:
            NotImplementedError: Speakers not supported on UDS
        """
        raise NotImplementedError(
            "Speakers (pub/sub) are not supported on UDS transport. "
            "Use Redis or ROS2 for publish-subscribe communication."
        )
    
    async def create_job(
        self,
        name: str,
        callback: Callable,
        **kwargs
    ) -> VyraJob:
        """
        Create a job interface.
        
        Note: Jobs are not yet implemented for UDS transport.
        Use ROS2 Actions for long-running tasks.
        
        Raises:
            NotImplementedError: Jobs not yet implemented
        """
        raise NotImplementedError(
            "Jobs are not yet implemented for UDS transport. "
            "Use ROS2 Actions for long-running tasks."
        )
