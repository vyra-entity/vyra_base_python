"""
Protocol Provider Interface

Abstract base class for all communication protocol providers.
Implements provider pattern for pluggable protocols.
"""
import logging
from abc import ABC, abstractmethod
from typing import Any, Callable, Optional, Dict, List

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

logger = logging.getLogger(__name__)


class AbstractProtocolProvider(ABC):
    """
    Abstract base class for all protocol providers.
    
    Each protocol (ROS2, Shared Memory, MQTT, etc.) implements this interface
    to provide consistent callable/speaker/job creation across transports.
    """
    
    def __init__(self, protocol: ProtocolType):
        self.protocol = protocol
        self._available = False
        self._initialized = False
        self._config: Dict[str, Any] = {}
    
    @property
    def name(self) -> str:
        """Get protocol name."""
        return self.protocol.value
    
    @abstractmethod
    async def check_availability(self) -> bool:
        """
        Check if protocol is available (libraries installed, services running).
        
        Returns:
            bool: True if protocol is available
        """
        pass
    
    @abstractmethod
    async def initialize(self, config: Optional[Dict[str, Any]] = None) -> bool:
        """
        Initialize the protocol provider.
        
        Args:
            config: Optional configuration dictionary
            
        Returns:
            bool: True if initialization successful
        """
        pass
    
    @abstractmethod
    async def shutdown(self) -> None:
        """Shutdown the protocol provider and cleanup resources."""
        pass
    
    @abstractmethod
    async def create_callable(
        self,
        name: str,
        callback: Callable,
        **kwargs
    ) -> VyraCallable:
        """
        Create a callable interface.
        
        Args:
            name: Callable name
            callback: Callback function (async or sync)
            **kwargs: Protocol-specific parameters
            
        Returns:
            VyraCallable: Created callable interface
        """
        pass
    
    @abstractmethod
    async def create_speaker(
        self,
        name: str,
        **kwargs
    ) -> VyraSpeaker:
        """
        Create a speaker interface.
        
        Args:
            name: Speaker name
            **kwargs: Protocol-specific parameters
            
        Returns:
            VyraSpeaker: Created speaker interface
        """
        pass
    
    @abstractmethod
    async def create_job(
        self,
        name: str,
        callback: Callable,
        **kwargs
    ) -> VyraJob:
        """
        Create a job interface.
        
        Args:
            name: Job name
            callback: Job execution callback
            **kwargs: Protocol-specific parameters
            
        Returns:
            VyraJob: Created job interface
        """
        pass
    
    def is_available(self) -> bool:
        """Check if protocol is available."""
        return self._available
    
    def is_initialized(self) -> bool:
        """Check if provider is initialized."""
        return self._initialized
    
    def get_config(self) -> Dict[str, Any]:
        """Get current configuration."""
        return self._config.copy()
    
    def update_config(self, config: Dict[str, Any]) -> None:
        """Update configuration."""
        self._config.update(config)
    
    def require_availability(self) -> None:
        """Raise exception if protocol is not available."""
        if not self._available:
            raise ProtocolUnavailableError(
                f"Protocol '{self.protocol}' is not available. "
                f"Check installation and dependencies."
            )
    
    def require_initialization(self) -> None:
        """Raise exception if provider is not initialized."""
        if not self._initialized:
            raise ProviderError(
                f"Provider '{self.protocol}' is not initialized. "
                f"Call initialize() first."
            )
