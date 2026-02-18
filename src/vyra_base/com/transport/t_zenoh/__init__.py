"""
Zenoh Transport Module

Provides Zenoh-based transport implementation with layered architecture:

Layers:
    - communication/: Core Zenoh functionality (Query/Reply, Pub/Sub, Tasks)
    - vyra_models/: VYRA abstractions (ZenohCallable, ZenohPublisher, ZenohJob)
    - session.py: Zenoh session management and lifecycle
    - provider.py: Interface layer for VYRA integration

Features:
    - Query/Reply pattern for request-response (ZenohCallable)
    - Pub/Sub for topic-based messaging (ZenohPublisher)
    - Task-based long-running operations (ZenohJob)
    - Zero-copy and efficient serialization
    - Router-based architecture for scalability
    - Built-in discovery and fault tolerance

Usage:
    from vyra_base.com.transport.t_zenoh import ZenohProvider, ZENOH_AVAILABLE
    
    if ZENOH_AVAILABLE:
        provider = ZenohProvider()
        await provider.initialize(config={
            "mode": "client",
            "connect": ["tcp/zenoh-router:7447"]
        })
        callable = await provider.create_callable("/service", callback)
        publisher = await provider.create_publisher("/topic")
"""

import logging
logger = logging.getLogger(__name__)

# Check Eclipse Zenoh library availability
try:
    import zenoh  # Eclipse Zenoh Python API only import to check if available
    _zenoh_available = True
except ImportError:
    _zenoh_available = False
    logger.debug("⚠️  Eclipse Zenoh library not available")

# Try importing session layer (Zenoh-specific)
try:
    from vyra_base.com.transport.t_zenoh.communication.session import (
        ZenohSession,
        SessionConfig,
        SessionMode
    )
    _session_available = True
except ImportError as e:
    ZenohSession = None
    SessionConfig = None
    _session_available = False
    logger.debug(f"⚠️  Zenoh session layer not available: {e}")

# Try importing vyra models
try:
    from vyra_base.com.transport.t_zenoh.vyra_models import (
        VyraPublisherImpl,
        VyraSubscriberImpl,
        VyraServerImpl,
        VyraClientImpl,
        VyraActionServerImpl,
        VyraActionClientImpl
    )
    _models_available = True
except ImportError as e:
    _models_available = False
    logger.debug(f"⚠️  Zenoh models not available: {e}")

# Try importing provider
try:
    from vyra_base.com.transport.t_zenoh.provider import ZenohProvider
    _provider_available = True
except ImportError as e:
    ZenohProvider = None
    _provider_available = False
    logger.debug(f"⚠️  Zenoh provider not available: {e}")

# Overall availability flag
ZENOH_AVAILABLE = all([
    _zenoh_available,
    _session_available,
    _models_available,
    _provider_available,
])

__all__ = [
    "ZENOH_AVAILABLE",
    "ZenohSession",
    "SessionConfig",
    "SessionMode",
    "VyraPublisherImpl",
    "VyraSubscriberImpl",
    "VyraServerImpl",
    "VyraClientImpl",
    "VyraActionServerImpl",
    "VyraActionClientImpl",
    "ZenohProvider",
]
