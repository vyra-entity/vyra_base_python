# External Communication Protocols

Pure protocol implementations for external system integration.

## Architecture

The External Layer provides **pure protocol implementations** without VYRA abstractions:

| Protocol   | Type           | Client                    | Server                    |
|------------|----------------|---------------------------|---------------------------|
| Redis      | Pub/Sub + KV   | `RedisClient`            | Redis Server (external)   |
| gRPC       | RPC            | `GrpcClient`             | `GrpcServer`              |
| MQTT       | Pub/Sub        | `MqttClient`             | MQTT Broker (external)    |
| REST       | HTTP API       | `RestClient`             | FastAPI/aiohttp           |
| WebSocket  | Bidirectional  | `WebSocketClient`        | WebSocket server          |
| Shared Mem | Zero-copy IPC  | `SharedMemorySegment`    | `SharedMemorySegment`     |

**Note**: For VYRA abstractions (callable, speaker, job) use the **transport/** layer.

## Installation

```bash
# Redis
pip install redis

# gRPC  
pip install grpcio grpcio-tools

# MQTT
pip install paho-mqtt

# REST
pip install aiohttp requests

# WebSocket
pip install websockets

# Shared Memory
pip install posix-ipc msgpack
```

## gRPC Protocol

### GrpcClient

```python
from vyra_base.com.external.grpc import GrpcClient

client = GrpcClient(target="localhost:50051")
await client.connect()
response = await client.call_method("/service/Method", request_bytes)
await client.close()
```

### GrpcServer

```python
from vyra_base.com.external.grpc import GrpcServer
import my_service_pb2_grpc

class MyServicer(my_service_pb2_grpc.MyServiceServicer):
    async def UnaryMethod(self, request, context):
        return MyResponse(result="success")

server = GrpcServer("/tmp/my_service.sock")
server.add_service(
    my_service_pb2_grpc.add_MyServiceServicer_to_server,
    MyServicer()
)
await server.start()
```

**Legacy Aliases**: `GrpcUdsServer` → `GrpcServer`

## Using with VYRA Provider Model

For VYRA abstractions (callable, speaker, job), use the **transport/** layer:

```python
from vyra_base.com import InterfaceFactory, ProtocolType

provider = InterfaceFactory.create_provider(
    protocol=ProtocolType.REDIS,
    host="localhost",
    port=6379
)
await provider.initialize()

callable = await provider.create_callable(
    "service_name",
    callback=lambda req: {"result": req["x"] * 2}
)
```

See [transport/README.md](../transport/README.md) for details.

## Migration Notes

**Removed Files** (use transport/ layer instead):
- ❌ `external/*/provider.py` → Use `transport/*/provider.py`
- ❌ `external/*/callable.py` → Use `transport/*/callable.py`
- ❌ `external/*/speaker.py` → Use `transport/*/speaker.py`

**Renamed Files**:
- `uds_server.py` → `grpc_server.py`
- `uds_client.py` → Removed (use `grpc_client.py`)
