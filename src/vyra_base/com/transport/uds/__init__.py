"""
UDS Transport Module

Provides Unix Domain Socket-based transport implementation with layered architecture:

Layers:
    - communication/: Core UDS functionality (UnixSocket, socket management)
    - vyra_models/: VYRA abstractions (UDSCallable)
    - provider.py: Interface layer for VYRA integration

Features:
    - Stream-based local IPC
    - Low-latency request-response
    - Automatic connection management
    - JSON serialization

Usage:
    from vyra_base.com.transport.uds import UDSProvider, UDS_AVAILABLE
    
    if UDS_AVAILABLE:
        provider = UDSProvider(module_name="my_module")
        await provider.initialize()
        callable = await provider.create_callable("service", callback)
"""
import logging

logger = logging.getLogger(__name__)

# Try importing communication layer
try:
    from vyra_base.com.transport.uds.communication import (
        UnixSocket,
        UDS_SOCKET_DIR,
        MESSAGE_HEADER_FORMAT,
        MESSAGE_HEADER_SIZE,
        UDS_COMMUNICATION_AVAILABLE,
    )
    _communication_available = UDS_COMMUNICATION_AVAILABLE
except ImportError as e:
    UnixSocket = None
    UDS_SOCKET_DIR = None
    MESSAGE_HEADER_FORMAT = None
    MESSAGE_HEADER_SIZE = None
    _communication_available = False
    logger.debug(f"⚠️  UDS communication layer unavailable: {e}")

# Try importing VYRA models layer
try:
    from vyra_base.com.transport.uds.vyra_models import (
        UDSCallable,
        UDS_MODELS_AVAILABLE,
    )
    _models_available = UDS_MODELS_AVAILABLE
except ImportError as e:
    UDSCallable = None
    _models_available = False
    logger.debug(f"⚠️  UDS VYRA models layer unavailable: {e}")

# Try importing provider (interface layer)
try:
    from vyra_base.com.transport.uds.provider import UDSProvider
    _provider_available = True
except ImportError as e:
    UDSProvider = None
    _provider_available = False
    logger.debug(f"⚠️  UDS provider unavailable: {e}")

# UDS is fully available if all layers are available
UDS_AVAILABLE = _communication_available and _models_available and _provider_available

if UDS_AVAILABLE:
    logger.info("✅ UDS transport fully available (communication + models + provider)")
elif _communication_available:
    logger.info("⚠️  UDS transport partially available (communication only)")
else:
    logger.debug("❌ UDS transport unavailable")

__all__ = [
    # Availability flag
    "UDS_AVAILABLE",
    # Interface layer
    "UDSProvider",
    # Communication layer
    "UnixSocket",
    "UDS_SOCKET_DIR",
    "MESSAGE_HEADER_FORMAT",
    "MESSAGE_HEADER_SIZE",
    # VYRA models layer
    "UDSCallable",
]
