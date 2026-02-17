"""
VYRA Transport Layer

Unified communication abstraction layer supporting multiple protocols.
Implements provider pattern for pluggable transports.

Available Transports:
- ROS2: Distributed communication via DDS middleware
- Zenoh: Unified pub/sub and query/reply with zero-copy
- Redis: Message queue and key-value storage via Redis
- UDS: Stream-based local IPC via Unix domain sockets

Example Usage:
    >>> # ROS2 Transport
    >>> from vyra_base.com.transport.ros2 import ROS2Provider
    >>> 
    >>> provider = ROS2Provider(ProtocolType.ROS2)
    >>> if await provider.check_availability():
    ...     await provider.initialize()
    ...     callable = await provider.create_callable("service", callback)
    >>> 
    >>> # Zenoh Transport
    >>> from vyra_base.com.transport.zenoh import ZenohProvider
    >>> 
    >>> provider = ZenohProvider(ProtocolType.ZENOH)
    >>> if await provider.check_availability():
    ...     await provider.initialize(config={
    ...         "mode": "client",
    ...         "connect": ["tcp/zenoh-router:7447"]
    ...     })
    ...     callable = await provider.create_callable("service", callback)
    >>> 
    >>> # Redis Transport
    >>> from vyra_base.com.transport.redis import RedisProvider, RedisClient
    >>> 
    >>> provider = RedisProvider(ProtocolType.REDIS)
    >>> if await provider.check_availability():
    ...     await provider.initialize()
    ...     server = await provider.create_server("service", callback)
    ...     publisher = await provider.create_publisher("topic")
    >>> 
    >>> # UDS Transport
    >>> from vyra_base.com.transport.uds import UDSProvider
    >>> 
    >>> provider = UDSProvider(ProtocolType.UDS)
    >>> if await provider.check_availability():
    ...     await provider.initialize()
    ...     server = await provider.create_server("service", callback)
"""

# Redis Transport (optional)
try:
    from vyra_base.com.transport.t_redis import (
        RedisProvider,
        RedisClient,
        REDIS_AVAILABLE,
    )
except ImportError:
    REDIS_AVAILABLE = False

# ROS2 Transport (optional)
try:
    from vyra_base.com.transport.t_ros2 import (
        ROS2Provider,
        VyraNode,
        ROS2_AVAILABLE,
    )
except ImportError:
    ROS2_AVAILABLE = False

# Zenoh Transport (optional)
try:
    from vyra_base.com.transport.t_zenoh import (
        ZenohProvider,
        ZenohSession,
        ZENOH_AVAILABLE,
    )
except ImportError:
    ZENOH_AVAILABLE = False

# UDS Transport (always available on Unix)
try:
    from vyra_base.com.transport.t_uds import (
        UDSProvider,
        UnixSocket,
    )
    UDS_AVAILABLE = True
except ImportError:
    UDS_AVAILABLE = False

__all__ = [
    # Availability flags
    "REDIS_AVAILABLE",
    "ROS2_AVAILABLE",
    "ZENOH_AVAILABLE",
    "UDS_AVAILABLE",
]

# Export based on availability
if REDIS_AVAILABLE:
    __all__.extend([
        "RedisProvider",
        "RedisClient"
    ])

if ROS2_AVAILABLE:
    __all__.extend([
        "ROS2Provider",
        "VyraNode",
    ])

if ZENOH_AVAILABLE:
    __all__.extend([
        "ZenohProvider",
        "ZenohSession"
    ])

if UDS_AVAILABLE:
    __all__.extend([
        "UDSProvider",
        "UnixSocket",
    ])


def get_available_transports() -> dict:
    """
    Get dictionary of available transports.
    
    Returns:
        Dict mapping protocol name to availability status
    """
    return {
        "redis": REDIS_AVAILABLE,
        "ros2": ROS2_AVAILABLE,
        "zenoh": ZENOH_AVAILABLE,
        "uds": UDS_AVAILABLE,
    }


def print_transport_status() -> None:
    """Print status of all transport protocols."""
    print("VYRA Transport Layer Status:")
    print("-" * 40)
    
    transports = get_available_transports()
    for name, available in transports.items():
        status = "✅ Available" if available else "❌ Not Available"
        print(f"  {name.upper():20s} {status}")
    
    print("-" * 40)
