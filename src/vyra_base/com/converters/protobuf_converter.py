"""
Protobuf Converter

Handles conversion between Python dictionaries and Protocol Buffer messages.
"""
import logging
from typing import Any, Optional, Type, Dict
import json

from vyra_base.com.converters.interface import ConverterInterface

logger = logging.getLogger(__name__)

try:
    from google.protobuf import json_format
    from google.protobuf.message import Message as ProtoMessage
    PROTOBUF_AVAILABLE = True
except ImportError:
    PROTOBUF_AVAILABLE = False
    logger.debug("⚠️ protobuf not available")


class ProtobufConverter(ConverterInterface):
    """
    Converter for Protocol Buffer messages.
    
    Provides bidirectional conversion between:
    - Python dict ↔ Protobuf message
    - JSON string ↔ Protobuf message
    - Protobuf message ↔ Binary (serialized)
    
    Example:
        >>> from mymodule_pb2 import MyMessage
        >>> converter = ProtobufConverter()
        >>> 
        >>> # Dict to Protobuf
        >>> data = {"field1": "value", "field2": 42}
        >>> proto_msg = converter.dict_to_proto(data, MyMessage)
        >>> 
        >>> # Protobuf to Dict
        >>> data_back = converter.proto_to_dict(proto_msg)
        >>> 
        >>> # Serialize/Deserialize
        >>> binary = converter.serialize(proto_msg)
        >>> proto_msg_back = converter.deserialize(binary, MyMessage)
    """
    
    @property
    def name(self) -> str:
        """Get converter name."""
        return "protobuf"
    
    def is_available(self) -> bool:
        """Check if protobuf is available."""
        return PROTOBUF_AVAILABLE
    
    def dict_to_proto(self, data: Dict[str, Any], message_type: Type[ProtoMessage]) -> ProtoMessage:
        """
        Convert Python dictionary to Protobuf message.
        
        Args:
            data: Python dictionary
            message_type: Protobuf message class
            
        Returns:
            Protobuf message instance
        """
        if not self.is_available():
            raise ImportError("protobuf not available")
        
        try:
            # Create message instance
            message = message_type()
            
            # Parse from JSON (dict -> JSON -> Protobuf)
            json_str = json.dumps(data)
            json_format.Parse(json_str, message)
            
            return message
            
        except Exception as e:
            logger.error(f"Failed to convert dict to protobuf: {e}")
            raise
    
    def proto_to_dict(self, message: ProtoMessage) -> Dict[str, Any]:
        """
        Convert Protobuf message to Python dictionary.
        
        Args:
            message: Protobuf message instance
            
        Returns:
            Python dictionary
        """
        if not self.is_available():
            raise ImportError("protobuf not available")
        
        try:
            # Convert to dict via JSON (Protobuf -> JSON -> dict)
            json_str = json_format.MessageToJson(message)
            return json.loads(json_str)
            
        except Exception as e:
            logger.error(f"Failed to convert protobuf to dict: {e}")
            raise
    
    def serialize(self, message: ProtoMessage) -> bytes:
        """
        Serialize Protobuf message to binary.
        
        Args:
            message: Protobuf message instance
            
        Returns:
            Binary data
        """
        if not self.is_available():
            raise ImportError("protobuf not available")
        
        try:
            return message.SerializeToString()
        except Exception as e:
            logger.error(f"Failed to serialize protobuf: {e}")
            raise
    
    def deserialize(self, data: bytes, message_type: Type[ProtoMessage]) -> ProtoMessage:
        """
        Deserialize binary data to Protobuf message.
        
        Args:
            data: Binary data
            message_type: Protobuf message class
            
        Returns:
            Protobuf message instance
        """
        if not self.is_available():
            raise ImportError("protobuf not available")
        
        try:
            message = message_type()
            message.ParseFromString(data)
            return message
            
        except Exception as e:
            logger.error(f"Failed to deserialize protobuf: {e}")
            raise
    
    def convert_to(self, data: Any, target_type: Optional[Type] = None) -> Any:
        """
        Convert data to Protobuf message.
        
        Args:
            data: Source data (dict, JSON string, or bytes)
            target_type: Protobuf message class
            
        Returns:
            Protobuf message
        """
        if target_type is None:
            raise ValueError("target_type (Protobuf message class) is required")
        
        if isinstance(data, dict):
            return self.dict_to_proto(data, target_type)
        elif isinstance(data, str):
            # Assume JSON string
            data_dict = json.loads(data)
            return self.dict_to_proto(data_dict, target_type)
        elif isinstance(data, bytes):
            return self.deserialize(data, target_type)
        else:
            raise ValueError(f"Unsupported data type: {type(data)}")
    
    def convert_from(self, data: Any, source_type: Optional[Type] = None) -> Any:
        """
        Convert Protobuf message to dictionary.
        
        Args:
            data: Protobuf message instance
            source_type: Ignored (type inferred from data)
            
        Returns:
            Python dictionary
        """
        if not isinstance(data, ProtoMessage):
            raise ValueError("Data must be a Protobuf message")
        
        return self.proto_to_dict(data)
