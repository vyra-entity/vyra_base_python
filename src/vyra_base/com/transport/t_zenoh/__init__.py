"""
Zenoh Transport Module

Provides Zenoh-based transport implementation with layered architecture:

Layers:
    - communication/: Core Zenoh functionality (Query/Reply, Pub/Sub, Tasks)
    - vyra_models/: VYRA abstractions (ZenohCallable, ZenohSpeaker, ZenohJob)
    - session.py: Zenoh session management and lifecycle
    - provider.py: Interface layer for VYRA integration

Features:
    - Query/Reply pattern for request-response (ZenohCallable)
    - Pub/Sub for topic-based messaging (ZenohSpeaker)
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
        speaker = await provider.create_speaker("/topic")
"""

from vyra_base.helper.logger import Logger

# Check Eclipse Zenoh library availability
try:
    import zenoh  # Eclipse Zenoh Python API only import to check if available
    _zenoh_available = True
except ImportError:
    _zenoh_available = False
    Logger.debug("⚠️  Eclipse Zenoh library not available")

# Try importing session layer (Zenoh-specific)
try:
    from vyra_base.com.transport.t_zenoh.session import (
        ZenohSession,
        SessionConfig,
        SessionMode
    )
    _session_available = True
except ImportError as e:
    ZenohSession = None
    SessionConfig = None
    _session_available = False
    Logger.debug(f"⚠️  Zenoh session layer not available: {e}")

# Try importing vyra models
try:
    from vyra_base.com.transport.t_zenoh.vyra_models import (
        ZenohCallable,
        ZenohSpeaker,
        ZenohJob,
    )
    _models_available = True
except ImportError as e:
    ZenohCallable = None
    ZenohSpeaker = None
    ZenohJob = None
    _models_available = False
    Logger.debug(f"⚠️  Zenoh models not available: {e}")

# Try importing provider
try:
    from vyra_base.com.transport.t_zenoh.provider import ZenohProvider
    _provider_available = True
except ImportError as e:
    ZenohProvider = None
    _provider_available = False
    Logger.debug(f"⚠️  Zenoh provider not available: {e}")

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
    "ZenohCallable",
    "ZenohSpeaker",
    "ZenohJob",
    "ZenohProvider",
]
