# Redis Interfaces System

## Overview

Redis interfaces provide a way to define and share `VyraCallable` definitions across modules using Protocol Buffers (.proto files). This system works alongside ROS2 interfaces but uses Redis as the transport layer.

## Architecture

Similar to ROS2 interfaces, Redis interfaces are:
- Defined in `.proto` files
- Compiled to Python code at build time
- Shared via NFS for cross-module communication
- Automatically loaded by modules

## Directory Structure

```
module/
├── storage/
│   └── redis_interfaces/          # Redis interface definitions
│       ├── *.proto                 # Protobuf definitions
│       ├── config/                 # Metadata configurations
│       │   └── redis_callables_meta.json
│       └── grpc_generated/         # Generated Python code (build artifact)
├── install/
│   └── {module}_redis_interfaces/  # Built Redis interfaces
└── /nfs/redis_interfaces/          # NFS shared Redis interfaces
    └── {module}_redis_interfaces/  # Accessible by all modules
```

## Workflow

### 1. Define Redis Interface (.proto)

Create a Protocol Buffers file in `storage/redis_interfaces/`:

```protobuf
// storage/redis_interfaces/sensor_service.proto
syntax = "proto3";

package module_sensors;

// Request message for sensor data
message GetSensorDataRequest {
    string sensor_id = 1;
    bool include_metadata = 2;
}

// Response message with sensor data
message GetSensorDataResponse {
    string sensor_id = 1;
    double value = 2;
    string unit = 3;
    int64 timestamp = 4;
}

// Sensor data service (used as VyraCallable via Redis)
service SensorService {
    rpc GetSensorData (GetSensorDataRequest) returns (GetSensorDataResponse);
}
```

### 2. Create Metadata Configuration

Create `storage/redis_interfaces/config/redis_callables_meta.json`:

```json
[
    {
        "name": "get_sensor_data",
        "type": "callable",
        "protocol": "redis",
        "proto_file": "sensor_service.proto",
        "service": "SensorService",
        "method": "GetSensorData",
        "request_type": "GetSensorDataRequest",
        "response_type": "GetSensorDataResponse",
        "description": "Retrieve sensor data via Redis",
        "displaystyle": {
            "visible": true,
            "category": "Sensors",
            "icon": "sensor"
        }
    }
]
```

### 3. Build Redis Interfaces

During Docker build, interfaces are automatically generated:

```dockerfile
# In module Dockerfile (builder stage)
RUN python3 tools/setup_redis_interfaces.py
```

This will:
- Find all `.proto` files in `storage/redis_interfaces/`
- Generate Python code using `grpc_tools.protoc`
- Copy to `install/{module}_redis_interfaces/`
- Stage for NFS deployment

### 4. Deploy to NFS

At container startup (`vyra_entrypoint.sh`):

```bash
# Copy Redis interfaces to NFS
REDIS_INTERFACE_SOURCE="/workspace/install/${MODULE_NAME}_redis_interfaces"
NFS_REDIS_PATH="/nfs/redis_interfaces/${MODULE_NAME}_${INSTANCE_ID}_redis_interfaces"

if [ -d "$REDIS_INTERFACE_SOURCE" ]; then
    mkdir -p "$NFS_REDIS_PATH"
    cp -r "$REDIS_INTERFACE_SOURCE"/* "$NFS_REDIS_PATH"/
    echo "✅ Redis interfaces deployed to NFS"
fi

# Load all Redis interfaces from NFS
for redis_if_dir in /nfs/redis_interfaces/*_redis_interfaces; do
    if [ -d "$redis_if_dir" ]; then
        export PYTHONPATH="$redis_if_dir:$PYTHONPATH"
        echo "   Loaded Redis interface: $(basename $redis_if_dir)"
    fi
done
```

### 5. Use Redis Interfaces in Code

```python
from vyra_base.com.transport.redis import RedisProvider
from vyra_base.com.core.types import ProtocolType

# Import generated protobuf messages
from module_sensors_redis_interfaces.sensor_service_pb2 import (
    GetSensorDataRequest,
    GetSensorDataResponse
)

# Initialize Redis provider
provider = RedisProvider(
    protocol=ProtocolType.REDIS,
    module_name="robot_controller"
)

await provider.initialize()

# Create callable (client-side)
sensor_callable = await provider.create_callable(
    name="module_sensors/get_sensor_data"
)

# Call the service
request = GetSensorDataRequest(
    sensor_id="temp_01",
    include_metadata=True
)

response_bytes = await sensor_callable.call(request.SerializeToString())
response = GetSensorDataResponse.FromString(response_bytes)

print(f"Sensor {response.sensor_id}: {response.value} {response.unit}")
```

## Comparison: ROS2 vs Redis Interfaces

| Feature | ROS2 Interfaces | Redis Interfaces |
|---------|----------------|------------------|
| **Definition** | `.msg`, `.srv`, `.action` files | `.proto` files |
| **Protocol** | DDS (ROS2) | Redis Pub/Sub + Key-Value |
| **Build Tool** | `colcon build` | `grpc_tools.protoc` |
| **Location** | `/nfs/ros_interfaces/` | `/nfs/redis_interfaces/` |
| **Use Case** | ROS2 Services/Topics | Redis VyraCallables |
| **Security** | SROS2 | Redis TLS + ACL |

## Benefits

1. **Protocol Flexibility**: Choose Redis or ROS2 based on use case
2. **Type Safety**: Protobuf provides strong typing
3. **Cross-Module**: Shared interfaces via NFS
4. **Backward Compatible**: Works alongside existing ROS2 interfaces
5. **Cloud-Ready**: Redis works without ROS2 dependency

## Tools

- `tools/setup_redis_interfaces.py` - Build Redis interfaces
- `tools/generate_redis_protos.py` - Generate Python from .proto
- `tools/nfs_redis_interface_helper.sh` - NFS management

## Environment Variables

```bash
# Redis interface paths
REDIS_INTERFACE_PATH=/workspace/storage/redis_interfaces
NFS_REDIS_INTERFACE_PATH=/nfs/redis_interfaces

# Enable Redis interface sharing
ENABLE_REDIS_INTERFACES=true
```

## Metadata Format

```json
{
    "name": "callable_name",
    "type": "callable",
    "protocol": "redis",
    "proto_file": "service.proto",
    "service": "ServiceName",
    "method": "MethodName",
    "request_type": "RequestMessage",
    "response_type": "ResponseMessage",
    "description": "Human-readable description",
    "timeout": 5.0,
    "retry_policy": {
        "max_retries": 3,
        "backoff_ms": 100
    }
}
```

## See Also

- [ROS2 Interface Sharing](/docs/NFS_INTERFACE_SHARING.md)
- [Redis Provider](/src/vyra_base/com/transport/redis/provider.py)
- [VyraCallable Documentation](/docs/VYRA_CALLABLE.md)
