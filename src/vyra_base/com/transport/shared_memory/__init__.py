"""
Shared Memory Transport

POSIX IPC-based shared memory communication for zero-copy, deterministic IPC.
Ideal for industrial applications requiring <500µs latency.
"""
from vyra_base.com.transport.shared_memory.segment import (
    SharedMemorySegment,
    SegmentInfo,
    POSIX_IPC_AVAILABLE,
)
from vyra_base.com.transport.shared_memory.serialization import (
    SharedMemorySerializer,
    SerializationFormat,
    MessageType,
    calculate_segment_size,
)
from vyra_base.com.transport.shared_memory.discovery import (
    SharedMemoryDiscovery,
    SegmentDiscoveryInfo,
    get_discovery,
    DISCOVERY_DIR,
)
from vyra_base.com.transport.shared_memory.callable import SharedMemoryCallable
from vyra_base.com.transport.shared_memory.speaker import SharedMemorySpeaker
from vyra_base.com.transport.shared_memory.provider import SharedMemoryProvider

__all__ = [
    # Core segment management
    "SharedMemorySegment",
    "SegmentInfo",
    "POSIX_IPC_AVAILABLE",
    
    # Serialization
    "SharedMemorySerializer",
    "SerializationFormat",
    "MessageType",
    "calculate_segment_size",
    
    # Discovery
    "SharedMemoryDiscovery",
    "SegmentDiscoveryInfo",
    "get_discovery",
    "DISCOVERY_DIR",
    
    # Communication interfaces
    "SharedMemoryCallable",
    "SharedMemorySpeaker",
    
    # Provider
    "SharedMemoryProvider",
]
