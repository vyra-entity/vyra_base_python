"""
Communication Converters & Helpers

Pluggable converters and helpers for transport and external communication.

This module provides:
- Protocol Buffer (Protobuf) conversion utilities
- Message format converters (JSON ↔ Protobuf ↔ MessagePack)
- ROS2 ↔ Protobuf bridges
- Zenoh ↔ Protobuf bridges
- Custom serialization helpers

Architecture:
- Each converter is a plugin that implements ConverterInterface
- Converters are registered in ConverterRegistry
- Factory pattern for easy converter creation

Usage:
    from vyra_base.com.converter import ConverterFactory, ProtobufConverter
    
    # Get protobuf converter
    converter = ConverterFactory.get_converter("protobuf")
    
    # Convert between formats
    proto_msg = converter.dict_to_proto(data_dict, MessageType)
    data_dict = converter.proto_to_dict(proto_msg)
"""
from vyra_base.com.converter.interface import ConverterInterface
from vyra_base.com.converter.registry import ConverterRegistry
from vyra_base.com.converter.factory import ConverterFactory
from vyra_base.com.converter.protobuf_converter import ProtobufConverter

__all__ = [
    "ConverterInterface",
    "ConverterRegistry",
    "ConverterFactory",
    "ProtobufConverter",
]
