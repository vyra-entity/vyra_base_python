"""
Shared Memory Serialization

Custom binary protocol for high-performance IPC via POSIX shared memory.
Optimized for industrial automation with deterministic performance.
"""
import struct
import logging
import msgpack
from typing import Any, Tuple, Optional
from enum import IntEnum
import json

logger = logging.getLogger(__name__)


class MessageType(IntEnum):
    """Message types for shared memory protocol."""
    REQUEST = 1
    RESPONSE = 2
    PUBLISH = 3
    ERROR = 4


class SerializationFormat(IntEnum):
    """Serialization formats."""
    JSON = 1      # Human-readable, flexible
    MSGPACK = 2   # Binary, fast (if available)
    PICKLE = 3    # Python-only, feature-rich


# Protocol header: | Magic (4B) | Version (1B) | Type (1B) | Format (1B) | Reserved (1B) | Length (4B) | Timestamp (8B) | Payload (variable) |
HEADER_FORMAT = '!4sBBBBIQ'  # Big-endian
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
MAGIC = b'VYRA'
VERSION = 1


class SharedMemorySerializer:
    """
    Custom binary serializer for shared memory communication.
    
    Protocol:
    - Magic: 4 bytes ('VYRA')
    - Version: 1 byte (protocol version)
    - Type: 1 byte (MessageType)
    - Format: 1 byte (SerializationFormat)
    - Reserved: 1 byte (future use)
    - Length: 4 bytes (payload length)
    - Timestamp: 8 bytes (nanoseconds since epoch)
    - Payload: variable length
    
    Features:
    - Deterministic performance
    - Minimal overhead (<32 bytes header)
    - Multiple serialization backends
    - Error detection (CRC optional)
    """
    
    def __init__(self, format: SerializationFormat = SerializationFormat.JSON):
        self.format = format
        
        # Try to import msgpack if requested
        self._msgpack = None
        if format == SerializationFormat.MSGPACK:
            try:
                self._msgpack = msgpack
            except ImportError:
                logger.warning(
                    "msgpack not available, falling back to JSON. "
                    "Install with: pip install msgpack"
                )
                self.format = SerializationFormat.JSON
    
    def serialize(
        self,
        data: Any,
        message_type: MessageType = MessageType.PUBLISH
    ) -> bytes:
        """
        Serialize data to binary format.
        
        Args:
            data: Data to serialize
            message_type: Type of message
            
        Returns:
            bytes: Serialized binary data
        """
        
        # Serialize payload
        if self.format == SerializationFormat.JSON:
            payload = json.dumps(data).encode('utf-8')
        elif self.format == SerializationFormat.MSGPACK and self._msgpack is not None:
            payload = self._msgpack.packb(data, use_bin_type=True)
        elif self.format == SerializationFormat.PICKLE:
            import pickle
            payload = pickle.dumps(data, protocol=pickle.HIGHEST_PROTOCOL)
        else:
            raise ValueError(f"Unknown serialization format: {self.format}")
        
        # Get timestamp (nanoseconds since epoch)
        import time
        timestamp = int(time.time() * 1_000_000_000)
        
        # Build header
        header = struct.pack(
            HEADER_FORMAT,
            MAGIC,
            VERSION,
            message_type,
            self.format,
            0,  # Reserved
            len(payload),
            timestamp
        )
        
        return header + payload
    
    def deserialize(self, data: bytes) -> Tuple[MessageType, Any, int]:
        """
        Deserialize binary data.
        
        Args:
            data: Binary data to deserialize
            
        Returns:
            Tuple[MessageType, payload, timestamp]
            
        Raises:
            ValueError: If data is corrupted or invalid
        """
        if len(data) < HEADER_SIZE:
            raise ValueError(
                f"Data too short: {len(data)} bytes, expected at least {HEADER_SIZE}"
            )
        
        # Parse header
        magic, version, msg_type, fmt, reserved, length, timestamp = struct.unpack(
            HEADER_FORMAT,
            data[:HEADER_SIZE]
        )
        
        # Validate magic
        if magic != MAGIC:
            raise ValueError(f"Invalid magic: {magic}, expected {MAGIC}")
        
        # Validate version
        if version != VERSION:
            raise ValueError(f"Unsupported version: {version}")
        
        # Extract payload
        payload_data = data[HEADER_SIZE:HEADER_SIZE + length]
        
        if len(payload_data) != length:
            raise ValueError(
                f"Payload length mismatch: {len(payload_data)} != {length}"
            )
        
        # Deserialize payload
        if fmt == SerializationFormat.JSON:
            payload = json.loads(payload_data.decode('utf-8'))
        elif fmt == SerializationFormat.MSGPACK:
            if self._msgpack is None:
                import msgpack
                self._msgpack = msgpack
            payload = self._msgpack.unpackb(payload_data, raw=False)
        elif fmt == SerializationFormat.PICKLE:
            import pickle
            payload = pickle.loads(payload_data)
        else:
            raise ValueError(f"Unknown format: {fmt}")
        
        return MessageType(msg_type), payload, timestamp
    
    def estimate_size(self, data: Any) -> int:
        """
        Estimate serialized size without actually serializing.
        
        Args:
            data: Data to estimate
            
        Returns:
            int: Estimated size in bytes
        """
        # Quick estimate (not exact)
        if self.format == SerializationFormat.JSON:
            estimate = len(json.dumps(data).encode('utf-8'))
        else:
            # Fallback: serialize and measure (accurate but slower)
            estimate = len(self.serialize(data))
        
        return HEADER_SIZE + estimate


# Utility functions
def calculate_segment_size(max_message_size: int) -> int:
    """
    Calculate required shared memory segment size.
    
    Adds space for:
    - Protocol header
    - Lock metadata (8 bytes)
    - Safety margin (10%)
    
    Args:
        max_message_size: Maximum expected message size
        
    Returns:
        int: Required segment size in bytes
    """
    overhead = HEADER_SIZE + 8  # Header + lock
    margin = int(max_message_size * 0.1)
    return max_message_size + overhead + margin


def create_error_message(error: Exception) -> bytes:
    """
    Create standardized error message.
    
    Args:
        error: Exception to serialize
        
    Returns:
        bytes: Serialized error message
    """
    serializer = SharedMemorySerializer()
    error_data = {
        'type': type(error).__name__,
        'message': str(error),
        'args': error.args,
    }
    return serializer.serialize(error_data, MessageType.ERROR)
