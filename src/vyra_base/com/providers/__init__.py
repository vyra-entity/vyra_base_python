"""
Protocol Provider Pattern

Abstract protocol provider and registry for pluggable communication backends.
"""

from vyra_base.com.providers.protocol_provider import AbstractProtocolProvider
from vyra_base.com.providers.provider_registry import ProviderRegistry

__all__ = [
    "AbstractProtocolProvider",
    "ProviderRegistry",
]
