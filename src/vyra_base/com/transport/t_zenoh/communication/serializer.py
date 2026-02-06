"""
Zenoh Serialization Utilities

Handles data serialization/deserialization for Zenoh transport.
Supports JSON, MessagePack, Protobuf, and custom serializers.
"""
import json
import logging
from enum import Enum
from typing import Any, Optional

logger = logging.getLogger(__name__)


class SerializationFormat(str, Enum):
    """Supported serialization formats."""
    JSON = "json"
    MSGPACK = "msgpack"
    PROTOBUF = "protobuf"
    RAW = "raw"


class ZenohSerializer:
    """
    Serialization helper for Zenoh transport.
    
    Provides consistent serialization across callable/speaker/job interfaces.
    """
    
    @staticmethod
    def serialize(data: Any, format: SerializationFormat = SerializationFormat.JSON) -> bytes:
        """
        Serialize data to bytes.
        
        Args:
            data: Data to serialize
            format: Serialization format
            
        Returns:
            bytes: Serialized data
        """
        try:
            if format == SerializationFormat.JSON:
                return json.dumps(data).encode('utf-8')
            
            elif format == SerializationFormat.MSGPACK:
                try:
                    import msgpack
                    return msgpack.packb(data)
                except ImportError:
                    logger.warning("msgpack not available, falling back to JSON")
                    return json.dumps(data).encode('utf-8')
            
            elif format == SerializationFormat.PROTOBUF:
                # Assume data is already protobuf message
                if hasattr(data, 'SerializeToString'):
                    return data.SerializeToString()
                else:
                    raise ValueError("Data is not a Protobuf message")
            
            elif format == SerializationFormat.RAW:
                if isinstance(data, bytes):
                    return data
                elif isinstance(data, str):
                    return data.encode('utf-8')
                else:
                    raise ValueError("RAW format requires bytes or str")
            
            else:
                raise ValueError(f"Unsupported serialization format: {format}")
                
        except Exception as e:
            logger.error(f"Serialization error: {e}")
            raise
    
    @staticmethod
    def deserialize(data: bytes, format: SerializationFormat = SerializationFormat.JSON) -> Any:
        """
        Deserialize data from bytes.
        
        Args:
            data: Serialized data
            format: Serialization format
            
        Returns:
            Deserialized data
        """
        try:
            if format == SerializationFormat.JSON:
                return json.loads(data.decode('utf-8'))
            
            elif format == SerializationFormat.MSGPACK:
                try:
                    import msgpack
                    return msgpack.unpackb(data)
                except ImportError:
                    logger.warning("msgpack not available, falling back to JSON")
                    return json.loads(data.decode('utf-8'))
            
            elif format == SerializationFormat.PROTOBUF:
                # Caller must provide message type separately
                raise NotImplementedError(
                    "Protobuf deserialization requires message type. "
                    "Use message.ParseFromString(data) directly."
                )
            
            elif format == SerializationFormat.RAW:
                return data
            
            else:
                raise ValueError(f"Unsupported deserialization format: {format}")
                
        except Exception as e:
            logger.error(f"Deserialization error: {e}")
            raise
