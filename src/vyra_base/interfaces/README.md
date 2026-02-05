# VYRA Interfaces

This directory contains the core interface definitions for the VYRA framework. Interfaces define the communication contracts between modules and are available through multiple transport protocols.

## Directory Structure

```
interfaces/
├── speaker/        # .msg files - VYRA Speakers (ROS2 Messages)
├── callable/       # .srv files - VYRA Callables (ROS2 Services)
├── job/            # .action files - VYRA Jobs (ROS2 Actions)
├── proto/          # .proto files - Protocol Buffers for Redis/gRPC/external
├── config/         # Metadata JSON files describing interfaces
├── package.xml     # ROS2 package metadata
└── CMakeLists.template.txt  # ROS2 build configuration

```

## Interface Types

### Speaker (`.msg` files)
**Purpose**: Publish/Subscribe messaging for event streams  
**Transport**: ROS2 DDS (Data Distribution Service)  
**Location**: `speaker/` directory  
**Use Cases**: 
- Status updates (error feeds, state changes)
- Event notifications (news feed)
- Sensor data streams

**Example**: `VBASEErrorFeed.msg`
```
string error_id
string message
uint8 severity
time timestamp
```

### Callable (`.srv` files)
**Purpose**: Request/Response communication for synchronous operations  
**Transport**: ROS2 DDS + Protocol Buffers (Redis/gRPC)  
**Location**: `callable/` directory (ROS2), `proto/` directory (gRPC)  
**Use Cases**:
- Health checks
- Parameter get/set operations
- State transitions
- Data queries

**Example**: `VBASEHealthCheck.srv`
```
# Request
---
# Response
bool alive
string state
string health_status
string[] issues
```

**Equivalent Proto**: `HealthCheck.proto`
```protobuf
syntax = "proto3";
package vyra_base;

message HealthCheckRequest {
    // Empty request
}

message HealthCheckResponse {
    bool alive = 1;
    string state = 2;
    string health_status = 3;
    repeated string issues = 4;
}

service HealthCheckService {
    rpc HealthCheck (HealthCheckRequest) returns (HealthCheckResponse);
}
```

### Job (`.action` files)
**Purpose**: Long-running operations with progress feedback  
**Transport**: ROS2 Actions  
**Location**: `job/` directory  
**Use Cases**:
- Multi-step workflows
- Operations requiring progress updates
- Cancelable tasks

**Example**: `VBASELongTask.action`
```
# Goal
string task_id
---
# Result
bool success
string result_data
---
# Feedback
float32 progress
string current_step
```

### Proto (`.proto` files)
**Purpose**: Protocol Buffer definitions for non-ROS2 transports  
**Transport**: Redis Pub/Sub, gRPC, external systems  
**Location**: `proto/` directory  
**Use Cases**:
- Redis-based communication
- External system integration
- Language-agnostic interfaces (C++, Java, Go, etc.)
- High-performance serialization

**Key Benefits**:
- Transport-independent definitions
- Multi-language support
- Efficient binary serialization
- Strong typing with code generation

## Configuration Files (`config/` directory)

Metadata files describe how interfaces are exposed and used by the VYRA framework.

### File Format

Configuration files are JSON arrays containing interface metadata objects.

**Example**: `vyra_core_meta.json`
```json
[
    {
        "tags": ["ros2", "redis"],
        "type": "callable",
        "functionname": "health_check",
        "displayname": "Health Check",
        "description": "Check module health and operational status",
        "filetype": [
            "VBASEHealthCheck.srv",
            "HealthCheck.proto"
        ],
        "params": [],
        "returns": [
            {
                "name": "alive",
                "type": "bool",
                "description": "Module is alive and responding"
            },
            {
                "name": "state",
                "type": "string",
                "description": "Current operational state"
            },
            {
                "name": "health_status",
                "type": "string",
                "description": "Health status description"
            },
            {
                "name": "issues",
                "type": "string[]",
                "description": "List of current issues"
            }
        ],
        "displaystyle": {
            "visible": true,
            "category": "System",
            "icon": "health"
        }
    }
]
```

### Configuration Schema

#### Required Fields

| Field | Type | Description |
|-------|------|-------------|
| `tags` | string[] | Transport protocols: `["ros2"]`, `["redis"]`, or `["ros2", "redis"]` |
| `type` | string | Interface type: `"speaker"`, `"callable"`, or `"job"` |
| `functionname` | string | Internal identifier (snake_case) |
| `displayname` | string | Human-readable name |
| `description` | string | Purpose and behavior description |
| `filetype` | string[] | List of interface files (e.g., `["File.srv", "File.proto"]`) |

#### Optional Fields

| Field | Type | Description |
|-------|------|-------------|
| `params` | object[] | Input parameters (for callables/jobs) |
| `returns` | object[] | Output/result fields |
| `feedback` | object[] | Progress feedback fields (for jobs) |
| `displaystyle` | object | UI presentation configuration |
| `timeout` | number | Default timeout in seconds |
| `security` | object | Access control settings |

#### Parameter/Return Schema

```json
{
    "name": "parameter_name",
    "type": "bool|int|float|string|array",
    "description": "What this parameter represents",
    "default": "Optional default value",
    "required": true,
    "validation": {
        "min": 0,
        "max": 100,
        "pattern": "^[a-z]+$"
    }
}
```

#### Display Style Schema

```json
{
    "visible": true,
    "category": "Category Name",
    "icon": "icon_name",
    "color": "#hexcolor",
    "order": 10
}
```

### Tags Explanation

- **`["ros2"]`**: Available via ROS2 DDS transport only
- **`["redis"]`**: Available via Redis/gRPC transport only
- **`["ros2", "redis"]`**: Available via both transports (dual-transport)

**Typical Usage**:
- Speakers: `["ros2"]` (ROS2 pub/sub)
- Callables: `["ros2", "redis"]` (both transports for flexibility)
- Jobs: `["ros2"]` (ROS2 actions)

### Filetype Format

The `filetype` field lists all interface definition files for an interface:

**Single Transport**:
```json
"filetype": ["VBASEErrorFeed.msg"]
```

**Dual Transport** (ROS2 + Redis):
```json
"filetype": [
    "VBASEHealthCheck.srv",
    "HealthCheck.proto"
]
```

**Rules**:
- First entry: ROS2 format (`.msg`, `.srv`, or `.action`)
- Second entry: Proto format (`.proto`) for Redis/gRPC
- Use VBASE prefix for ROS2 files to avoid naming conflicts
- Proto files use clean names without prefix

## Transport Protocols

### ROS2 DDS Transport

**When to Use**:
- Module-to-module communication within the VYRA system
- Real-time data distribution
- QoS-managed communication
- Discovery-based networking

**Interface Files**: `.msg`, `.srv`, `.action`  
**Generated Code**: ROS2 Python/C++ packages  
**Deployment**: `/nfs/vyra_interfaces/{module}_interfaces/`

### Redis/gRPC Transport

**When to Use**:
- Cross-language communication
- External system integration
- Web service APIs
- Redis-based messaging
- Non-ROS2 environments

**Interface Files**: `.proto`  
**Generated Code**: Python (`*_pb2.py`, `*_pb2_grpc.py`), C++ (`.pb.h`, `.pb.cc`)  
**Deployment**: `/nfs/vyra_interfaces/{module}_proto_interfaces/`

## Build Process

### ROS2 Interfaces (colcon)

```bash
# Copy interfaces to module
python3 tools/setup_interfaces.py

# Build with colcon
colcon build --packages-select {module}_interfaces

# Result: install/{module}_interfaces/
#   ├── msg/ (compiled .msg files)
#   ├── srv/ (compiled .srv files)
#   ├── action/ (compiled .action files)
#   └── Python/C++ libraries
```

### Proto Interfaces (protoc)

```bash
# Generate Python + C++ from .proto files
python3 tools/setup_proto_interfaces.py

# Result: install/{module}_proto_interfaces/
#   ├── *_pb2.py (Python messages)
#   ├── *_pb2_grpc.py (Python services)
#   ├── *_pb2.pyi (Python type hints)
#   ├── *.pb.h (C++ headers)
#   └── *.pb.cc (C++ implementation)
```

### Unified Build

```bash
# Build everything (ROS2 + Proto)
python3 tools/setup_interfaces.py --all

# Build only ROS2 interfaces
python3 tools/setup_interfaces.py --ros2-only

# Build only Proto interfaces
python3 tools/setup_interfaces.py --proto-only
```

## NFS Deployment

All interfaces are deployed to NFS for cross-module access:

```
/nfs/vyra_interfaces/
├── {module1}_interfaces/           # ROS2 interfaces
│   ├── msg/
│   ├── srv/
│   ├── action/
│   └── config/
├── {module1}_proto_interfaces/     # Proto interfaces
│   ├── health_check_pb2.py
│   ├── health_check_pb2_grpc.py
│   ├── health_check.pb.h
│   └── health_check.pb.cc
├── {module2}_interfaces/
└── {module2}_proto_interfaces/
```

**Loading**:
- ROS2: Automatic via `ROS_INTERFACE_PATH`
- Proto: Added to `PYTHONPATH` and `LD_LIBRARY_PATH`

## Usage Examples

### Python - ROS2 Interface

```python
from v2_modulemanager_interfaces.msg import VBASEErrorFeed
from v2_modulemanager_interfaces.srv import VBASEHealthCheck

# Publish message
error_msg = VBASEErrorFeed()
error_msg.error_id = "ERR001"
error_msg.message = "System warning"
publisher.publish(error_msg)

# Call service
client = node.create_client(VBASEHealthCheck, '/v2_modulemanager/health_check')
request = VBASEHealthCheck.Request()
response = await client.call_async(request)
```

### Python - Proto Interface

```python
from v2_modulemanager_proto_interfaces.health_check_pb2 import (
    HealthCheckRequest,
    HealthCheckResponse
)
from v2_modulemanager_proto_interfaces.health_check_pb2_grpc import (
    HealthCheckServiceStub
)

# Call via gRPC
channel = grpc.insecure_channel('localhost:50051')
stub = HealthCheckServiceStub(channel)
request = HealthCheckRequest()
response = stub.HealthCheck(request)

# Or via Redis
import redis
r = redis.Redis()
request_data = HealthCheckRequest()
r.publish('health_check_request', request_data.SerializeToString())
```

### C++ - Proto Interface

```cpp
#include "health_check.pb.h"

vyra_base::HealthCheckRequest request;
vyra_base::HealthCheckResponse response;

// Use with gRPC or custom transport
channel->CallMethod(method, &request, &response);
```

## Adding New Interfaces

### 1. Create Interface File

**For ROS2 Transport**:
- Speaker: Create `{Name}.msg` in `speaker/`
- Callable: Create `VBASE{Name}.srv` in `callable/`
- Job: Create `VBASE{Name}.action` in `job/`

**For Redis/gRPC Transport**:
- Create `{Name}.proto` in `proto/`

### 2. Create Configuration Entry

Add to appropriate `config/*_meta.json`:

```json
{
    "tags": ["ros2", "redis"],
    "type": "callable",
    "functionname": "my_new_function",
    "displayname": "My New Function",
    "description": "Does something useful",
    "filetype": [
        "VBASEMyFunction.srv",
        "MyFunction.proto"
    ],
    "params": [...],
    "returns": [...]
}
```

### 3. Build and Deploy

```bash
# In vyra_base_python
cd src/vyra_base/interfaces

# If adding proto, convert from srv if needed
python3 ../../../tools/convert_srv_to_proto.py

# Commit changes
git add speaker/ callable/ job/ proto/ config/
git commit -m "Add MyFunction interface"
```

### 4. Update Modules

```bash
# Modules will automatically pick up new interfaces on next build
cd /workspace/module
python3 tools/setup_interfaces.py --all
colcon build
```

## Best Practices

1. **Use VBASE Prefix**: All ROS2 interfaces should use `VBASE` prefix to avoid conflicts
2. **Dual Transport for Callables**: Provide both `.srv` and `.proto` for maximum flexibility
3. **Comprehensive Metadata**: Fill out all config fields for better UI integration
4. **Semantic Versioning**: Update interface versions carefully to maintain compatibility
5. **Documentation**: Add clear descriptions in both interface files and config JSONs
6. **Type Safety**: Use specific types (bool, int32, string) rather than generic types
7. **Validation**: Define validation rules in config for input parameters

## Troubleshooting

**Issue**: Interfaces not found after build  
**Solution**: Check NFS mount and PYTHONPATH/ROS_INTERFACE_PATH

**Issue**: Proto generation fails  
**Solution**: Ensure `grpcio-tools` is installed: `pip install grpcio-tools`

**Issue**: ROS2 interfaces not compiling  
**Solution**: Verify `package.xml` and `CMakeLists.txt` are correctly configured

**Issue**: Config changes not reflected in UI  
**Solution**: Config files are cached - restart module or clear cache

## References

- [ROS2 Interface Documentation](https://docs.ros.org/en/rolling/Concepts/About-ROS-Interfaces.html)
- [Protocol Buffers Guide](https://protobuf.dev/)
- [gRPC Documentation](https://grpc.io/docs/)
- VYRA Framework Documentation: `/workspace/docs/`
