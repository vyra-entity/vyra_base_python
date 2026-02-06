"""
Unit tests for Zenoh serializer.
"""
import pytest
import json

from vyra_base.com.transport.t_zenoh.communication.serializer import (
    ZenohSerializer,
    SerializationFormat
)


@pytest.mark.unit
class TestZenohSerializer:
    """Test Zenoh serialization utilities."""
    
    @pytest.fixture
    def serializer(self):
        """Create serializer instance."""
        return ZenohSerializer()
    
    def test_json_serialization(self, serializer):
        """Test JSON serialization."""
        data = {"name": "test", "value": 42}
        
        # Serialize
        serialized = serializer.serialize(data, SerializationFormat.JSON)
        
        assert isinstance(serialized, bytes)
        assert json.loads(serialized.decode('utf-8')) == data
        
        # Deserialize
        deserialized = serializer.deserialize(serialized, SerializationFormat.JSON)
        
        assert deserialized == data
    
    def test_json_serialization_nested(self, serializer):
        """Test JSON serialization with nested data."""
        data = {
            "sensor": "temp_01",
            "reading": {
                "value": 23.5,
                "unit": "celsius",
                "metadata": {
                    "timestamp": "2024-01-01T12:00:00Z"
                }
            }
        }
        
        serialized = serializer.serialize(data, SerializationFormat.JSON)
        deserialized = serializer.deserialize(serialized, SerializationFormat.JSON)
        
        assert deserialized == data
    
    @pytest.mark.skipif(
        not hasattr(__import__('importlib'), 'import_module') or
        not __import__('importlib.util').util.find_spec('msgpack'),
        reason="msgpack not available"
    )
    def test_msgpack_serialization(self, serializer):
        """Test MessagePack serialization."""
        data = {"name": "test", "value": 42, "array": [1, 2, 3]}
        
        # Serialize
        serialized = serializer.serialize(data, SerializationFormat.MSGPACK)
        
        assert isinstance(serialized, bytes)
        
        # Deserialize
        deserialized = serializer.deserialize(serialized, SerializationFormat.MSGPACK)
        
        assert deserialized == data
    
    def test_raw_serialization_bytes(self, serializer):
        """Test raw serialization with bytes."""
        data = b"raw binary data"
        
        serialized = serializer.serialize(data, SerializationFormat.RAW)
        
        assert serialized == data
        
        deserialized = serializer.deserialize(serialized, SerializationFormat.RAW)
        
        assert deserialized == data
    
    def test_raw_serialization_string(self, serializer):
        """Test raw serialization with string."""
        data = "text data"
        
        serialized = serializer.serialize(data, SerializationFormat.RAW)
        
        assert serialized == data.encode('utf-8')
        assert isinstance(serialized, bytes)
    
    def test_unsupported_format_serialize(self, serializer):
        """Test serialization with unsupported format."""
        with pytest.raises(ValueError, match="Unsupported serialization format"):
            serializer.serialize({"test": "data"}, "unsupported")
    
    def test_unsupported_format_deserialize(self, serializer):
        """Test deserialization with unsupported format."""
        with pytest.raises(ValueError, match="Unsupported deserialization format"):
            serializer.deserialize(b"data", "unsupported")
    
    def test_raw_format_invalid_type(self, serializer):
        """Test raw format with invalid data type."""
        with pytest.raises(ValueError, match="RAW format requires"):
            serializer.serialize(42, SerializationFormat.RAW)
    
    def test_protobuf_serialization_requires_message(self, serializer):
        """Test protobuf serialization requires protobuf message."""
        data = {"test": "data"}
        
        with pytest.raises(ValueError, match="not a Protobuf message"):
            serializer.serialize(data, SerializationFormat.PROTOBUF)
    
    def test_protobuf_deserialization_not_implemented(self, serializer):
        """Test protobuf deserialization is not directly supported."""
        with pytest.raises(NotImplementedError, match="message type"):
            serializer.deserialize(b"data", SerializationFormat.PROTOBUF)


@pytest.mark.unit
class TestSerializationFormat:
    """Test SerializationFormat enum."""
    
    def test_format_values(self):
        """Test all format values."""
        assert SerializationFormat.JSON.value == "json"
        assert SerializationFormat.MSGPACK.value == "msgpack"
        assert SerializationFormat.PROTOBUF.value == "protobuf"
        assert SerializationFormat.RAW.value == "raw"
