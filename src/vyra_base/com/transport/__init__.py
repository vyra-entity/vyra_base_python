"""
VYRA Transport Layer

Unified communication abstraction layer supporting multiple protocols.
Implements provider pattern for pluggable transports.

Available Transports:
- Shared Memory: Zero-copy local IPC via POSIX shared memory
- ROS2: Distributed communication via DDS middleware
- UDS: Stream-based local IPC via Unix domain sockets

Example Usage:
    >>> # Shared Memory Transport
    >>> from vyra_base.com.transport.shared_memory import SharedMemoryProvider
    >>> 
    >>> provider = SharedMemoryProvider(ProtocolType.SHARED_MEMORY)
    >>> if await provider.check_availability():
    ...     await provider.initialize()
    ...     callable = await provider.create_callable("service", callback)
    ...     speaker = await provider.create_speaker("topic")
    >>> 
    >>> # ROS2 Transport
    >>> from vyra_base.com.transport.ros2 import ROS2Provider
    >>> 
    >>> provider = ROS2Provider(ProtocolType.ROS2)
    >>> if await provider.check_availability():
    ...     await provider.initialize()
    ...     callable = await provider.create_callable("service", callback)
    >>> 
    >>> # UDS Transport
    >>> from vyra_base.com.transport.uds import UDSProvider
    >>> 
    >>> provider = UDSProvider(ProtocolType.UDS)
    >>> if await provider.check_availability():
    ...     await provider.initialize()
    ...     callable = await provider.create_callable("service", callback)
"""

# Shared Memory Transport (always try to import)
try:
    from vyra_base.com.transport.shared_memory import (
        SharedMemoryProvider,
        SharedMemoryCallable,
        SharedMemorySpeaker,
        SharedMemorySegment,
        SharedMemoryDiscovery,
        POSIX_IPC_AVAILABLE,
    )
    SHARED_MEMORY_AVAILABLE = POSIX_IPC_AVAILABLE
except ImportError:
    SHARED_MEMORY_AVAILABLE = False

# ROS2 Transport (optional)
try:
    from vyra_base.com.transport.ros2 import (
        ROS2Provider,
        VyraNode,
        ROS2_AVAILABLE,
    )
except ImportError:
    ROS2_AVAILABLE = False

# UDS Transport (always available on Unix)
try:
    from vyra_base.com.transport.uds import (
        UDSProvider,
        UDSCallable,
        UnixSocket,
    )
    UDS_AVAILABLE = True
except ImportError:
    UDS_AVAILABLE = False

__all__ = [
    # Availability flags
    "SHARED_MEMORY_AVAILABLE",
    "ROS2_AVAILABLE",
    "UDS_AVAILABLE",
]

# Export based on availability
if SHARED_MEMORY_AVAILABLE:
    __all__.extend([
        "SharedMemoryProvider",
        "SharedMemoryCallable",
        "SharedMemorySpeaker",
        "SharedMemorySegment",
        "SharedMemoryDiscovery",
    ])

if ROS2_AVAILABLE:
    __all__.extend([
        "ROS2Provider",
        "VyraNode",
    ])

if UDS_AVAILABLE:
    __all__.extend([
        "UDSProvider",
        "UDSCallable",
        "UnixSocket",
    ])


def get_available_transports() -> dict:
    """
    Get dictionary of available transports.
    
    Returns:
        Dict mapping protocol name to availability status
    """
    return {
        "shared_memory": SHARED_MEMORY_AVAILABLE,
        "ros2": ROS2_AVAILABLE,
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
