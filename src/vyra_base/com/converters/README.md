# Communication Converters & Helpers

Pluggable converter system for protocol-agnostic data transformation in VYRA communication layer.

## Overview

The converters module provides a flexible, extensible system for converting data between different formats used in transport and external communication protocols. This enables seamless integration between:

- **Transport protocols**: ROS2, Zenoh, Redis, UDS
- **External protocols**: gRPC, MQTT, REST, WebSocket
- **Data formats**: JSON, Protobuf, MessagePack, Avro, FlatBuffers

## Architecture

```
com/converters/
├── __init__.py              # Public API
├── interface.py             # ConverterInterface (ABC)
├── registry.py              # ConverterRegistry (Singleton)
├── factory.py               # ConverterFactory
├── protobuf_converter.py    # Protobuf ↔ Dict/JSON/Binary
└── README.md                # This file
```

### Design Pattern

1. **Plugin Architecture**: Each converter implements `ConverterInterface`
2. **Registry Pattern**: Centralized converter management
3. **Factory Pattern**: Convenient converter instantiation
4. **Lazy Initialization**: Converters loaded on-demand

## Usage

### Basic Usage

```python
from vyra_base.com.converters import ConverterFactory

# Get Protobuf converter
converter = ConverterFactory.get_converter("protobuf")

# Check availability
if converter and converter.is_available():
    # Dict to Protobuf
    data = {"name": "sensor1", "value": 42.5}
    proto_msg = converter.dict_to_proto(data, SensorData)
    
    # Protobuf to Dict
    data_back = converter.proto_to_dict(proto_msg)
    
    # Binary serialization
    binary = converter.serialize(proto_msg)
    proto_msg_back = converter.deserialize(binary, SensorData)
```

### With Transport Layers

#### Zenoh + Protobuf

```python
from vyra_base.com.transport.t_zenoh import ZenohProvider
from vyra_base.com.converters import ConverterFactory

# Get converter
proto_converter = ConverterFactory.get_converter("protobuf")

# Create Zenoh speaker
provider = ZenohProvider()
await provider.initialize()
speaker = await provider.create_speaker("/sensor_data")

# Publish with Protobuf
data = {"temperature": 23.5, "humidity": 65.0}
proto_msg = proto_converter.dict_to_proto(data, SensorReading)
binary_data = proto_converter.serialize(proto_msg)
await speaker.shout(binary_data)
```

#### ROS2 ↔ Protobuf Bridge

```python
from vyra_base.com.transport.t_ros2 import ROS2Provider
from vyra_base.com.converters import ConverterFactory

proto_converter = ConverterFactory.get_converter("protobuf")

# Subscribe to ROS2 topic
ros2_provider = ROS2Provider()
await ros2_provider.initialize()
subscriber = await ros2_provider.create_speaker("/ros2_topic", is_publisher=False)

async def handle_ros2_message(ros2_msg):
    # Convert ROS2 msg to dict
    data = {
        "field1": ros2_msg.field1,
        "field2": ros2_msg.field2
    }
    
    # Convert to Protobuf for external system
    proto_msg = proto_converter.dict_to_proto(data, ExternalMessage)
    
    # Forward to external system...
    await external_client.send(proto_msg)

await subscriber.listen(handle_ros2_message)
```

### Custom Converter

```python
from vyra_base.com.converters import ConverterInterface, ConverterFactory

class MessagePackConverter(ConverterInterface):
    @property
    def name(self) -> str:
        return "messagepack"
    
    def is_available(self) -> bool:
        try:
            import msgpack
            return True
        except ImportError:
            return False
    
    def convert_to(self, data, target_type=None):
        import msgpack
        return msgpack.packb(data)
    
    def convert_from(self, data, source_type=None):
        import msgpack
        return msgpack.unpackb(data)

# Register custom converter
ConverterFactory.register_custom_converter(MessagePackConverter())

# Use it
converter = ConverterFactory.get_converter("messagepack")
binary = converter.convert_to({"key": "value"})
```

## Available Converters

### Protobuf Converter

**Name**: `"protobuf"`

**Features**:
- Dict ↔ Protobuf message
- JSON ↔ Protobuf message
- Binary serialization/deserialization
- Type-safe conversion

**Methods**:
```python
dict_to_proto(data: dict, message_type: Type) -> Message
proto_to_dict(message: Message) -> dict
serialize(message: Message) -> bytes
deserialize(data: bytes, message_type: Type) -> Message
```

**Dependencies**: `protobuf`, `grpcio-tools`

### Future Converters

Additional converters can be added for:
- **MessagePack**: Efficient binary serialization
- **Avro**: Schema-based serialization (Apache Avro)
- **FlatBuffers**: Zero-copy buffer access
- **JSON Schema**: JSON with validation
- **CBOR**: Compact binary object representation

## Integration with Transport Layers

### Serialization in Zenoh

Zenoh's `ZenohSerializer` uses the converter system internally:

```python
# In ZenohSerializer
from vyra_base.com.converters import ConverterFactory

proto_converter = ConverterFactory.get_converter("protobuf")

def serialize_protobuf(data):
    if isinstance(data, ProtoMessage):
        return proto_converter.serialize(data)
    return proto_converter.dict_to_proto(data, MessageType)
```

### Type Conversion in ROS2

ROS2 typeconverter can leverage converters for external formats:

```python
from vyra_base.com.converters import ConverterFactory

def ros2_to_external(ros2_msg, external_format="protobuf"):
    converter = ConverterFactory.get_converter(external_format)
    data = ros2_msg_to_dict(ros2_msg)
    return converter.convert_to(data, ExternalType)
```

## API Reference

### ConverterInterface

Abstract base class for all converters.

```python
class ConverterInterface(ABC):
    @property
    @abstractmethod
    def name(self) -> str:
        """Converter name (e.g., 'protobuf', 'messagepack')"""
    
    @abstractmethod
    def convert_to(self, data: Any, target_type: Optional[Type] = None) -> Any:
        """Convert data to target format"""
    
    @abstractmethod
    def convert_from(self, data: Any, source_type: Optional[Type] = None) -> Any:
        """Convert data from source format"""
    
    @abstractmethod
    def is_available(self) -> bool:
        """Check if converter dependencies are installed"""
```

### ConverterRegistry

Singleton registry for converter management.

```python
ConverterRegistry.register(converter: ConverterInterface)
ConverterRegistry.get(name: str) -> Optional[ConverterInterface]
ConverterRegistry.list_converters() -> list[str]
ConverterRegistry.clear()
```

### ConverterFactory

Factory for accessing converters.

```python
ConverterFactory.get_converter(name: str) -> Optional[ConverterInterface]
ConverterFactory.list_available_converters() -> list[str]
ConverterFactory.register_custom_converter(converter: ConverterInterface)
```

## Best Practices

1. **Check Availability**: Always verify `converter.is_available()` before use
2. **Type Safety**: Provide explicit types for Protobuf conversions
3. **Error Handling**: Wrap conversions in try-except for production code
4. **Performance**: Consider caching converter instances
5. **Lazy Loading**: Converters are loaded only when needed

## Testing

```python
# Unit tests
pytest tests/test_converters.py -v

# Test specific converter
pytest tests/test_converters.py::test_protobuf_converter -v
```

## Dependencies

- **Core**: None (base interfaces)
- **Protobuf**: `protobuf>=4.0.0`, `grpcio-tools>=1.60.0`
- **MessagePack**: `msgpack>=1.0.0` (future)
- **Avro**: `avro-python3>=1.10.0` (future)

## See Also

- [Transport Layer README](../transport/README.md)
- [External Communication README](../external/README.md)
- [Zenoh Transport](../transport/zenoh/README.md)
- [Protobuf Examples](../../../../examples/protobuf_examples.py)
