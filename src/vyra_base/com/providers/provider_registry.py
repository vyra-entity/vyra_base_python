"""
Protocol Provider Registry

Central registry for all protocol providers with discovery and factory methods.
Providers are stored under a compound key (ProtocolType, module_id) so that
multiple providers of the same protocol – each targeting a different remote
module endpoint – can coexist without overwriting each other.

module_id == None  →  "own" / default provider (used by publisher, server, etc.)
module_id == <str> →  remote-module provider (used by subscriber, client, etc.)
"""
import logging
from typing import Any, Dict, List, Optional, Tuple
from threading import RLock

from vyra_base.com.core.types import ProtocolType
from vyra_base.com.core.exceptions import (
    ProviderNotFoundError,
    ProviderRegistrationError,
)
from vyra_base.com.providers.protocol_provider import AbstractProtocolProvider

logger = logging.getLogger(__name__)

# Compound registry key: (ProtocolType, Optional[module_id])
_ProviderKey = Tuple[ProtocolType, Optional[str]]


class ProviderRegistry:
    """
    Central registry for all protocol providers.

    Manages provider lifecycle, discovery, and factory methods.
    Thread-safe singleton pattern.

    Providers are keyed by (ProtocolType, module_id).
    "Own" / default providers are registered with module_id=None.
    Remote-module providers are registered with their target module_id.
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

        # Key: (ProtocolType, Optional[module_id])
        self._providers: Dict[_ProviderKey, AbstractProtocolProvider] = {}
        # Available keys
        self._available_keys: List[_ProviderKey] = []
        self._initialized = True

        logger.info("✅ Provider Registry initialized")

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _make_key(protocol: ProtocolType, module_id: Optional[str] = None) -> _ProviderKey:
        return (protocol, module_id)

    # ------------------------------------------------------------------
    # Registration
    # ------------------------------------------------------------------

    def register_provider(
        self,
        provider: AbstractProtocolProvider,
        module_id: Optional[str] = None,
        force: bool = False,
    ) -> None:
        """
        Register a protocol provider.

        Args:
            provider: Provider instance to register
            module_id: Target module-ID this provider is configured for.
                       Use None (default) for the "own" provider that publishes /
                       serves on behalf of the local module.
            force: If True, replace existing provider for the same key
        """
        with self._lock:
            key = self._make_key(provider.protocol, module_id)

            if key in self._providers and not force:
                proto_val = provider.protocol.value if hasattr(provider.protocol, "value") else str(provider.protocol)
                raise ProviderRegistrationError(
                    f"{proto_val}",
                    f"Provider for key {key} already registered. "
                    f"Use force=True to replace.",
                )

            self._providers[key] = provider

            if provider.is_available and key not in self._available_keys:
                self._available_keys.append(key)

            proto_str = provider.protocol.value if hasattr(provider.protocol, "value") else str(provider.protocol)
            logger.info(
                f"✅ Provider '{proto_str}' registered "
                f"(module_id={module_id!r}, available: {provider.is_available})"
            )

    # ------------------------------------------------------------------
    # Lookup
    # ------------------------------------------------------------------

    def get_provider(
        self,
        protocol: ProtocolType,
        module_id: Optional[str] = None,
        require_available: bool = True,
    ) -> Optional[AbstractProtocolProvider]:
        """
        Get provider for a (protocol, module_id) pair.

        Lookup order:
        1. Exact key (protocol, module_id)
        2. If module_id != None and not found: fall back to (protocol, None)

        Args:
            protocol: Protocol type
            module_id: Target module ID or None for the default provider
            require_available: If True, only return if protocol is available

        Returns:
            Provider instance or None
        """
        key = self._make_key(protocol, module_id)
        provider = self._providers.get(key)

        # Fallback to default provider when a specific module was requested
        if provider is None and module_id is not None:
            fallback_key = self._make_key(protocol, None)
            provider = self._providers.get(fallback_key)

        if provider is None:
            if require_available:
                raise ProviderNotFoundError(
                    f"No provider registered for protocol '{protocol}' "
                    f"(module_id={module_id!r})"
                )
            return None

        if require_available and not provider.is_available:
            raise ProviderNotFoundError(
                f"Provider '{protocol}' (module_id={module_id!r}) is registered but not available"
            )

        return provider

    def get_or_create_provider_for_module(
        self,
        protocol: ProtocolType,
        module_name: str,
        module_id: str,
    ) -> Optional[AbstractProtocolProvider]:
        """
        Return the provider for (protocol, module_id).

        If no provider is registered for that specific module_id yet, the
        default provider (module_id=None) is looked up and registered under the
        new key automatically – it will be used with an external TopicBuilder
        injected by the factory at creation time.

        Args:
            protocol: Protocol type
            module_name: Human-readable name of the target module
            module_id: Unique ID of the target module

        Returns:
            Provider instance or None if no provider is available at all
        """
        key = self._make_key(protocol, module_id)

        with self._lock:
            if key in self._providers:
                return self._providers[key]

            # Fall back to default provider and alias it for this module_id
            fallback_key = self._make_key(protocol, None)
            base_provider = self._providers.get(fallback_key)
            if base_provider is None:
                logger.debug(
                    f"⚠️ No base provider for protocol '{protocol}' – "
                    f"cannot create entry for module_id={module_id!r}"
                )
                return None

            # Alias: store the same provider instance under the remote key.
            # The factory is responsible for injecting a remote TopicBuilder
            # via kwargs when calling create_subscriber / create_client / etc.
            self._providers[key] = base_provider
            if base_provider.is_available and key not in self._available_keys:
                self._available_keys.append(key)

            proto_str = base_provider.protocol.value if hasattr(base_provider.protocol, "value") else str(base_provider.protocol)
            logger.info(
                f"✅ Provider alias created: '{proto_str}' → "
                f"module_name='{module_name}', module_id='{module_id}'"
            )
            return base_provider

    # ------------------------------------------------------------------
    # Introspection
    # ------------------------------------------------------------------

    def list_registered(self) -> List[_ProviderKey]:
        """Get list of all registered (protocol, module_id) keys."""
        with self._lock:
            return list(self._providers.keys())

    def list_available(self) -> List[_ProviderKey]:
        """Get list of available (protocol, module_id) keys."""
        with self._lock:
            return self._available_keys.copy()

    def is_available(self, protocol: ProtocolType, module_id: Optional[str] = None) -> bool:
        """Check if a provider for (protocol, module_id) is available."""
        return self._make_key(protocol, module_id) in self._available_keys

    def unregister_provider(
        self,
        protocol: ProtocolType,
        module_id: Optional[str] = None,
    ) -> bool:
        """
        Unregister a provider.

        Args:
            protocol: Protocol to unregister
            module_id: Module ID of the entry to remove (None = default provider)

        Returns:
            bool: True if provider was unregistered
        """
        with self._lock:
            key = self._make_key(protocol, module_id)
            if key in self._providers:
                del self._providers[key]
                if key in self._available_keys:
                    self._available_keys.remove(key)
                logger.info(f"Unregistered provider key={key!r}")
                return True
            return False

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    async def initialize_all(
        self,
        config: Optional[Dict[ProtocolType, Dict]] = None,
    ) -> List[_ProviderKey]:
        """
        Initialize all registered providers.

        Note: aliased entries (multiple module_ids sharing the same provider
        instance) will attempt initialization only once per unique instance.

        Args:
            config: Optional configuration per protocol

        Returns:
            List of successfully initialized (protocol, module_id) keys
        """
        config = config or {}
        initialized: List[_ProviderKey] = []
        seen_ids: set = set()  # avoid double-initializing aliased instances

        for key, provider in self._providers.items():
            if id(provider) in seen_ids:
                initialized.append(key)  # alias → already initialized
                continue

            protocol, _module_id = key
            try:
                protocol_config = config.get(protocol, {})
                success = await provider.initialize(protocol_config)

                if success:
                    initialized.append(key)
                    seen_ids.add(id(provider))
                    logger.info(f"✅ Provider key={key!r} initialized")
                else:
                    logger.warning(f"⚠️ Provider key={key!r} initialization failed")

            except Exception as e:
                logger.error(
                    f"❌ Failed to initialize provider key={key!r}: {e}",
                    exc_info=True,
                )

        return initialized

    async def shutdown_all(self) -> None:
        """Shutdown all unique provider instances."""
        seen_ids: set = set()
        for key, provider in self._providers.items():
            if id(provider) in seen_ids:
                continue
            try:
                await provider.shutdown()
                seen_ids.add(id(provider))
                logger.info(f"✅ Provider key={key!r} shutdown")
            except Exception as e:
                logger.error(
                    f"❌ Failed to shutdown provider key={key!r}: {e}",
                    exc_info=True,
                )

    def get_statistics(self) -> Dict[str, Any]:
        """Get registry statistics."""
        with self._lock:
            return {
                "registered": len(self._providers),
                "available": len(self._available_keys),
                "providers": {
                    str(key): {
                        "protocol": key[0].value if hasattr(key[0], "value") else str(key[0]),
                        "module_id": key[1],
                        "available": provider.is_available,
                        "initialized": provider.is_initialized,
                    }
                    for key, provider in self._providers.items()
                },
            }


# Global singleton instance
ProviderRegistryInstance = ProviderRegistry()
