"""
Protocol Provider Registry

Central registry for all protocol providers with discovery and factory methods.
"""
import logging
from typing import Any, Dict, List, Optional
from threading import RLock

from vyra_base.com.core.types import ProtocolType
from vyra_base.com.core.exceptions import (
    ProviderNotFoundError,
    ProviderRegistrationError,
)
from vyra_base.com.providers.protocol_provider import AbstractProtocolProvider

logger = logging.getLogger(__name__)


class ProviderRegistry:
    """
    Central registry for all protocol providers.
    
    Manages provider lifecycle, discovery, and factory methods.
    Thread-safe singleton pattern.
    """
    
    _instance: Optional['ProviderRegistry'] = None
    _lock = RLock()
    
    def __new__(cls):
        """Singleton pattern implementation."""
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
                    cls._instance._initialized = False
        return cls._instance
    
    def __init__(self):
        """Initialize registry (only once)."""
        if self._initialized:
            return
        
        self._providers: Dict[ProtocolType, AbstractProtocolProvider] = {}
        self._available_protocols: List[ProtocolType] = []
        self._initialized = True
        
        logger.info("✅ Provider Registry initialized")
    
    def register_provider(
        self,
        provider: AbstractProtocolProvider,
        force: bool = False
    ) -> None:
        """
        Register a protocol provider.
        
        Args:
            provider: Provider instance to register
            force: If True, replace existing provider
            
        Raises:
            ProviderRegistrationError: If provider already registered (unless force=True)
        """
        with self._lock:
            protocol = provider.protocol
            
            if protocol in self._providers and not force:
                raise ProviderRegistrationError(
                    f"{protocol.value}",
                    f"Provider for protocol '{protocol}' already registered. "
                    f"Use force=True to replace."
                )
            
            self._providers[protocol] = provider
            
            # Check availability
            if provider.is_available():
                if protocol not in self._available_protocols:
                    self._available_protocols.append(protocol)
            
            logger.info(
                f"✅ Provider '{protocol}' registered "
                f"(available: {provider.is_available()})"
            )
    
    def get_provider(
        self,
        protocol: ProtocolType,
        require_available: bool = True
    ) -> Optional[AbstractProtocolProvider]:
        """
        Get provider for protocol.
        
        Args:
            protocol: Protocol type
            require_available: If True, only return if protocol is available
            
        Returns:
            Provider instance or None
            
        Raises:
            ProviderNotFoundError: If provider not found and require_available=True
        """
        provider = self._providers.get(protocol)
        
        if provider is None:
            if require_available:
                raise ProviderNotFoundError(
                    f"No provider registered for protocol '{protocol}'"
                )
            return None
        
        if require_available and not provider.is_available():
            raise ProviderNotFoundError(
                f"Provider '{protocol}' is registered but not available"
            )
        
        return provider
    
    def list_registered(self) -> List[ProtocolType]:
        """Get list of all registered protocols."""
        with self._lock:
            return list(self._providers.keys())
    
    def list_available(self) -> List[ProtocolType]:
        """Get list of available protocols."""
        with self._lock:
            return self._available_protocols.copy()
    
    def is_available(self, protocol: ProtocolType) -> bool:
        """Check if protocol is available."""
        return protocol in self._available_protocols
    
    def unregister_provider(self, protocol: ProtocolType) -> bool:
        """
        Unregister a provider.
        
        Args:
            protocol: Protocol to unregister
            
        Returns:
            bool: True if provider was unregistered
        """
        with self._lock:
            if protocol in self._providers:
                del self._providers[protocol]
                if protocol in self._available_protocols:
                    self._available_protocols.remove(protocol)
                logger.info(f"Unregistered provider '{protocol}'")
                return True
            return False
    
    async def initialize_all(
        self,
        config: Optional[Dict[ProtocolType, Dict]] = None
    ) -> List[ProtocolType]:
        """
        Initialize all registered providers.
        
        Args:
            config: Optional configuration per protocol
            
        Returns:
            List of successfully initialized protocols
        """
        config = config or {}
        initialized = []
        
        for protocol, provider in self._providers.items():
            try:
                protocol_config = config.get(protocol, {})
                success = await provider.initialize(protocol_config)
                
                if success:
                    initialized.append(protocol)
                    logger.info(f"✅ Provider '{protocol}' initialized")
                else:
                    logger.warning(f"⚠️ Provider '{protocol}' initialization failed")
                    
            except Exception as e:
                logger.error(
                    f"❌ Failed to initialize provider '{protocol}': {e}",
                    exc_info=True
                )
        
        return initialized
    
    async def shutdown_all(self) -> None:
        """Shutdown all providers."""
        for protocol, provider in self._providers.items():
            try:
                await provider.shutdown()
                logger.info(f"✅ Provider '{protocol}' shutdown")
            except Exception as e:
                logger.error(
                    f"❌ Failed to shutdown provider '{protocol}': {e}",
                    exc_info=True
                )
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get registry statistics."""
        with self._lock:
            return {
                "registered": len(self._providers),
                "available": len(self._available_protocols),
                "protocols": {
                    protocol.value: {
                        "available": provider.is_available(),
                        "initialized": provider.is_initialized(),
                    }
                    for protocol, provider in self._providers.items()
                }
            }


# Global singleton instance
ProviderRegistryInstance = ProviderRegistry()
