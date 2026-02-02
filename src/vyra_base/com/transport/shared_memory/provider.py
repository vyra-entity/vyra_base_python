"""
Shared Memory Protocol Provider

Implements AbstractProtocolProvider for POSIX shared memory transport.
Provides zero-copy IPC with deterministic latency for industrial applications.
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
from vyra_base.com.transport.shared_memory.callable import SharedMemoryCallable
from vyra_base.com.transport.shared_memory.speaker import SharedMemorySpeaker
from vyra_base.com.transport.shared_memory.discovery import SharedMemoryDiscovery
from vyra_base.com.transport.shared_memory.serialization import SerializationFormat

logger = logging.getLogger(__name__)

# Check if posix_ipc is available
try:
    import posix_ipc
    POSIX_IPC_AVAILABLE = True
except ImportError:
    POSIX_IPC_AVAILABLE = False
    logger.warning(
        "⚠️ posix_ipc not available. Shared memory transport disabled. "
        "Install with: pip install posix-ipc"
    )


class SharedMemoryProvider(AbstractProtocolProvider):
    """
    Protocol provider for POSIX shared memory transport.
    
    Features:
    - Zero-copy data transfer
    - Deterministic latency (<500µs)
    - No network dependencies
    - Automatic process crash detection
    - PID-based discovery
    
    Requirements:
    - POSIX-compliant OS (Linux, macOS)
    - posix_ipc package
    
    Example:
        >>> # Initialize provider
        >>> provider = SharedMemoryProvider(ProtocolType.SHARED_MEMORY)
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
        ...     # Create speaker (publisher)
        ...     speaker = await provider.create_speaker(
        ...         "sensor_data",
        ...         module_name="robot",
        ...         is_publisher=True
        ...     )
        ...     await speaker.shout({"temperature": 23.5})
    """
    
    def __init__(
        self,
        protocol: ProtocolType = ProtocolType.SHARED_MEMORY,
        module_name: str = "default"
    ):
        """
        Initialize shared memory provider.
        
        Args:
            protocol: Protocol type (must be SHARED_MEMORY)
            module_name: Default module name for interfaces
        """
        super().__init__(protocol)
        self.module_name = module_name
        self._discovery: Optional[SharedMemoryDiscovery] = None
        
        # Default configuration
        self._config = {
            "segment_size": 4096,  # 4KB default
            "serialization_format": SerializationFormat.JSON.value,
            "discovery_cleanup_interval": 60.0,  # Cleanup every 60s
            "max_segment_age": 300.0  # 5 minutes
        }
    
    async def check_availability(self) -> bool:
        """
        Check if shared memory transport is available.
        
        Returns:
            bool: True if posix_ipc is installed
        """
        self._available = POSIX_IPC_AVAILABLE
        
        if not self._available:
            logger.warning(
                "⚠️ Shared memory transport not available. "
                "Install posix-ipc: pip install posix-ipc"
            )
        else:
            logger.info("✅ Shared memory transport available")
        
        return self._available
    
    async def initialize(self, config: Optional[Dict[str, Any]] = None) -> bool:
        """
        Initialize shared memory provider.
        
        Args:
            config: Optional configuration
                - segment_size: Default segment size in bytes
                - serialization_format: Serialization format (json/msgpack)
                - discovery_cleanup_interval: Discovery cleanup interval
                - max_segment_age: Maximum segment age before cleanup
                
        Returns:
            bool: True if initialization successful
        """
        if not self._available:
            raise ProtocolUnavailableError(
                "Shared memory transport not available. "
                "Install posix-ipc: pip install posix-ipc"
            )
        
        if self._initialized:
            logger.warning("⚠️ Provider already initialized")
            return True
        
        # Update configuration
        if config:
            self._config.update(config)
        
        logger.info(
            f"🚀 Initializing shared memory provider for module: {self.module_name}"
        )
        
        try:
            # Initialize discovery service
            self._discovery = SharedMemoryDiscovery()
            
            # Cleanup stale segments
            cleaned = self._discovery.cleanup_stale_segments(
                max_age=self._config["max_segment_age"]
            )
            if cleaned > 0:
                logger.info(f"🧹 Cleaned {cleaned} stale segments")
            
            self._initialized = True
            logger.info(
                f"✅ Shared memory provider initialized for {self.module_name}"
            )
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to initialize provider: {e}")
            return False
    
    async def shutdown(self) -> None:
        """Shutdown the provider and cleanup resources."""
        if not self._initialized:
            return
        
        logger.info(f"🛑 Shutting down shared memory provider: {self.module_name}")
        
        # Cleanup module's segments
        if self._discovery:
            segments = self._discovery.discover_by_module(self.module_name)
            for segment in segments:
                self._discovery.unregister(segment.module_name, segment.interface_name)
        
        self._initialized = False
        logger.info("✅ Shared memory provider shutdown complete")
    
    async def create_callable(
        self,
        name: str,
        callback: Callable,
        **kwargs
    ) -> VyraCallable:
        """
        Create a shared memory callable.
        
        Args:
            name: Callable name
            callback: Server-side callback function
            **kwargs: Additional parameters
                - module_name: Override default module name
                - segment_size: Override default segment size
                - serialization_format: Override serialization format
                
        Returns:
            SharedMemoryCallable instance
            
        Raises:
            ProviderError: If provider not initialized
        """
        self.require_initialization()
        
        # Get module name
        module_name = kwargs.pop("module_name", self.module_name)
        
        # Get segment size
        segment_size = kwargs.pop(
            "segment_size",
            self._config["segment_size"]
        )
        
        # Get serialization format
        serialization_format_str = kwargs.pop(
            "serialization_format",
            self._config["serialization_format"]
        )
        serialization_format = SerializationFormat(serialization_format_str)
        
        logger.info(
            f"🔧 Creating callable: {module_name}.{name} "
            f"(size: {segment_size}, format: {serialization_format.value})"
        )
        
        callable_instance = SharedMemoryCallable(
            name=name,
            callback=callback,
            module_name=module_name,
            segment_size=segment_size,
            serialization_format=serialization_format,
            **kwargs
        )
        
        await callable_instance.initialize()
        
        logger.info(f"✅ Callable created: {module_name}.{name}")
        return callable_instance
    
    async def create_speaker(
        self,
        name: str,
        **kwargs
    ) -> VyraSpeaker:
        """
        Create a shared memory speaker.
        
        Args:
            name: Speaker/Topic name
            **kwargs: Additional parameters
                - module_name: Override default module name
                - is_publisher: True for publisher, False for subscriber
                - segment_size: Override default segment size
                - serialization_format: Override serialization format
                
        Returns:
            SharedMemorySpeaker instance
            
        Raises:
            ProviderError: If provider not initialized
        """
        self.require_initialization()
        
        # Get module name
        module_name = kwargs.pop("module_name", self.module_name)
        
        # Get publisher flag
        is_publisher = kwargs.pop("is_publisher", True)
        
        # Get segment size
        segment_size = kwargs.pop(
            "segment_size",
            self._config["segment_size"]
        )
        
        # Get serialization format
        serialization_format_str = kwargs.pop(
            "serialization_format",
            self._config["serialization_format"]
        )
        serialization_format = SerializationFormat(serialization_format_str)
        
        role = "publisher" if is_publisher else "subscriber"
        logger.info(
            f"🔧 Creating speaker ({role}): {module_name}.{name} "
            f"(size: {segment_size}, format: {serialization_format.value})"
        )
        
        speaker_instance = SharedMemorySpeaker(
            name=name,
            module_name=module_name,
            is_publisher=is_publisher,
            segment_size=segment_size,
            serialization_format=serialization_format,
            **kwargs
        )
        
        await speaker_instance.initialize()
        
        logger.info(f"✅ Speaker created ({role}): {module_name}.{name}")
        return speaker_instance
    
    async def create_job(
        self,
        name: str,
        callback: Callable,
        **kwargs
    ) -> VyraJob:
        """
        Create a job interface.
        
        Note: Jobs are not yet implemented for shared memory transport.
        Use ROS2 Actions for long-running tasks.
        
        Raises:
            NotImplementedError: Jobs not yet implemented
        """
        raise NotImplementedError(
            "Jobs are not yet implemented for shared memory transport. "
            "Use ROS2 Actions for long-running tasks."
        )
    
    def get_discovery_statistics(self) -> Dict:
        """
        Get discovery statistics.
        
        Returns:
            Dict with discovery statistics
        """
        if not self._discovery:
            return {}
        
        return self._discovery.get_statistics()
    
    def cleanup_stale_segments(self, max_age: Optional[float] = None) -> int:
        """
        Manually trigger stale segment cleanup.
        
        Args:
            max_age: Maximum age in seconds (default: from config)
            
        Returns:
            int: Number of cleaned segments
        """
        if not self._discovery:
            return 0
        
        max_age = max_age or self._config["max_segment_age"]
        return self._discovery.cleanup_stale_segments(max_age or 3600.0)
