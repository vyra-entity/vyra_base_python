# VYRA Transport Layer

Complete implementation of pluggable communication protocols for the VYRA framework.

## Overview

The VYRA Transport Layer provides a unified abstraction for multiple communication protocols, implementing the **Provider Pattern** for seamless protocol switching.

### Architecture

```
vyra_base/com/
├── core/                    # Base types and exceptions
│   ├── types.py            # VyraCallable, VyraSpeaker, VyraJob
│   └── exceptions.py       # Transport exceptions
├── providers/              # Provider interface
│   └── protocol_provider.py
└── transport/              # Protocol implementations
    ├── shared_memory/      # ✅ POSIX shared memory
    ├── ros2/              # ✅ ROS2/DDS transport
    └── uds/               # ✅ Unix domain sockets
```

## Supported Protocols

### 1. Shared Memory Transport 🚀

**Zero-copy local IPC with deterministic latency (<500µs)**

#### Features
- POSIX shared memory segments
- PID-tracked discovery service
- Request-response (Callable)
- Publish-subscribe (Speaker)
- Automatic crash detection
- No network dependencies

#### Requirements
- `posix-ipc` package: `pip install posix-ipc`
- POSIX-compliant OS (Linux, macOS)

#### Usage

```python
from vyra_base.com.transport.shared_memory import SharedMemoryProvider
from vyra_base.com.core.types import ProtocolType

# Initialize provider
provider = SharedMemoryProvider(
    protocol=ProtocolType.SHARED_MEMORY,
    module_name="motor_control"
)

if await provider.check_availability():
    await provider.initialize(config={
        "segment_size": 4096,
        "serialization_format": "json"
    })
    
    # Create callable (server)
    async def handle_start(request):
        return {"status": "started", "speed": request["speed"]}
    
    callable = await provider.create_callable(
        "start",
        handle_start
    )
    
    # Create speaker (publisher)
    speaker = await provider.create_speaker(
        "sensor_data",
        is_publisher=True
    )
    
    await speaker.shout({"temperature": 23.5})
```

#### Components

- **`segment.py`**: Shared memory segment management with mutex synchronization
- **`serialization.py`**: JSON/MessagePack serialization
- **`discovery.py`**: PID-based filesystem discovery
- **`callable.py`**: Request-response interface
- **`speaker.py`**: Publish-subscribe interface
- **`provider.py`**: Protocol provider implementation

### 2. ROS2 Transport 🤖

**Distributed communication via DDS middleware**

#### Features
- ROS2 services, topics, and actions
- QoS policies
- Discovery and introspection
- SROS2 security support

#### Requirements
- ROS2 installation (Humble, Iron, Jazzy, or Kilted)
- Source `/opt/ros/<distro>/setup.bash`

#### Usage

```python
from vyra_base.com.transport.ros2 import ROS2Provider
from vyra_base.com.core.types import ProtocolType

# Initialize provider
provider = ROS2Provider(
    protocol=ProtocolType.ROS2,
    node_name="my_module"
)

if await provider.check_availability():
    await provider.initialize(config={
        "node_name": "vyra_node",
        "namespace": "/vyra"
    })
    
    # Create service (callable)
    async def handle_request(req):
        return {"result": req.value * 2}
    
    callable = await provider.create_callable(
        "calculate",
        handle_request,
        service_type="example_interfaces/srv/AddTwoInts"
    )
    
    # Create publisher (speaker)
    speaker = await provider.create_speaker(
        "sensor_data",
        message_type="std_msgs/msg/String"
    )
```

#### Components

All components from `datalayer/` are now available in `transport/ros2/`:

- **`node.py`**: VyraNode and CheckerNode
- **`publisher.py`**: ROS2 publisher
- **`subscriber.py`**: ROS2 subscriber
- **`service_server.py`**: ROS2 service server
- **`service_client.py`**: ROS2 service client
- **`action_server.py`**: ROS2 action server
- **`action_client.py`**: ROS2 action client
- **`provider.py`**: Protocol provider with optional import handling

### 3. UDS Transport 🔌

**Stream-based local IPC via Unix domain sockets**

#### Features
- Low-latency stream communication
- JSON serialization
- Automatic connection management
- Request-response pattern

#### Requirements
- Unix-like OS (Linux, macOS, BSD)

#### Usage

```python
from vyra_base.com.transport.uds import UDSProvider
from vyra_base.com.core.types import ProtocolType

# Initialize provider
provider = UDSProvider(
    protocol=ProtocolType.UDS,
    module_name="service_module"
)

if await provider.check_availability():
    await provider.initialize()
    
    # Create callable (server)
    async def handle_request(request):
        return {"result": request["value"] * 2}
    
    server = await provider.create_callable(
        "calculate",
        handle_request
    )
    
    # Create callable (client)
    client = await provider.create_callable(
        "calculate",
        None  # No callback for client
    )
    
    result = await client.call({"value": 21}, timeout=5.0)
    # result = {"result": 42}
```

#### Components

- **`socket.py`**: Unix socket management with length-prefixed framing
- **`callable.py`**: Request-response interface
- **`provider.py`**: Protocol provider implementation

#### Limitations

- **Local only**: Cannot communicate across network
- **No pub/sub**: Only request-response pattern (use Redis or ROS2 for pub/sub)

## Communication Patterns

### Callable (Request-Response)

Synchronous or asynchronous service calls:

```python
# Server side
async def handle_request(request):
    return {"result": request["value"] * 2}

callable = await provider.create_callable("service", handle_request)

# Client side
client = await provider.create_callable("service", None)
result = await client.call({"value": 21}, timeout=5.0)
```

**Supported by**: Shared Memory, ROS2, UDS

### Speaker (Publish-Subscribe)

One-way message broadcasting:

```python
# Publisher
speaker = await provider.create_speaker("topic", is_publisher=True)
await speaker.shout({"temperature": 23.5})

# Subscriber
subscriber = await provider.create_speaker("topic", is_publisher=False)
await subscriber.listen(lambda data: print(data))
```

**Supported by**: Shared Memory, ROS2

### Job (Long-Running Task)

Asynchronous tasks with progress feedback:

```python
async def execute_job(goal, feedback_callback):
    for i in range(100):
        await feedback_callback({"progress": i})
        await asyncio.sleep(0.1)
    return {"status": "complete"}

job = await provider.create_job("task", execute_job)
result = await job.execute({"target": "position_a"})
```

**Supported by**: ROS2 (Actions)

## Monitoring Integration

All transport operations are automatically monitored via Prometheus metrics:

```python
from vyra_base.com.monitoring import metrics

# Metrics recorded automatically:
# - vyra_com_calls_total
# - vyra_com_call_duration_seconds
# - vyra_com_errors_total
# - vyra_com_message_size_bytes
```

## Error Handling

Transport layer uses hierarchical exceptions:

```python
from vyra_base.com.core.exceptions import (
    CommunicationError,          # Base exception
    ProtocolUnavailableError,    # Protocol not installed
    ProtocolNotInitializedError, # Provider not initialized
    TransportError,              # Transport-level error
    ConnectionError,             # Connection failed
    TimeoutError,                # Operation timeout
    CallableError,               # Callable-specific error
    SpeakerError,                # Speaker-specific error
)

try:
    result = await callable.call(request, timeout=5.0)
except TimeoutError:
    logger.error("Service timeout")
except CallableError as e:
    logger.error(f"Service error: {e}")
```

## Testing

Run transport tests:

```bash
# Unit tests (no dependencies)
pytest tests/com/transport/shared_memory/test_segment.py -v

# Integration tests (requires posix_ipc)
pytest tests/com/transport/shared_memory/ -m integration

# Run examples
python examples/transport_examples.py
```

## Performance Characteristics

| Protocol       | Latency      | Throughput | Scope   | Dependencies |
|----------------|--------------|------------|---------|--------------|
| Shared Memory  | <500µs       | Very High  | Local   | posix-ipc    |
| ROS2           | 1-10ms       | High       | Network | ROS2         |
| UDS            | <1ms         | High       | Local   | None         |

## Migration from `datalayer`

All ROS2 components are now available in both locations:

```python
# Old (still works)
from vyra_base.com.datalayer.node import VyraNode

# New (recommended)
from vyra_base.com.transport.ros2 import VyraNode
```

The `datalayer` module remains for backward compatibility.

## Discovery

### Shared Memory Discovery

PID-tracked filesystem-based discovery:

```python
from vyra_base.com.transport.shared_memory import get_discovery

discovery = get_discovery()

# Register segment
discovery.register("motor_control", "start", "/vyra_motor_start", 4096)

# Discover segments
all_segments = discovery.discover_all()
motor_segments = discovery.discover_by_module("motor_control")

# Cleanup stale segments
cleaned = discovery.cleanup_stale_segments(max_age=300.0)

# Get statistics
stats = discovery.get_statistics()
```

Discovery files: `/tmp/vyra_sockets/<module>.<interface>.sm.pid`

## Best Practices

### 1. Protocol Selection

- **Shared Memory**: High-frequency local IPC (control loops, sensor fusion)
- **ROS2**: Distributed systems, multi-machine communication
- **UDS**: Simple local services, configuration management

### 2. Error Handling

Always check availability and handle initialization:

```python
provider = SharedMemoryProvider(ProtocolType.SHARED_MEMORY)

if not await provider.check_availability():
    logger.error("Shared memory not available")
    # Fallback to alternative protocol
    return

if not await provider.initialize():
    logger.error("Failed to initialize provider")
    return
```

### 3. Resource Cleanup

Always shutdown interfaces and providers:

```python
try:
    callable = await provider.create_callable("service", callback)
    # Use callable
finally:
    await callable.shutdown()
    await provider.shutdown()
```

### 4. Timeout Configuration

Set appropriate timeouts based on operation:

```python
# Fast local operation
result = await callable.call(request, timeout=1.0)

# Network operation
result = await callable.call(request, timeout=5.0)

# Long-running task
result = await job.execute(goal, timeout=30.0)
```

## Examples

See [`examples/transport_examples.py`](../../examples/transport_examples.py) for complete working examples of all three protocols.

## Troubleshooting

### Shared Memory Issues

**Error**: `posix_ipc not installed`
```bash
pip install posix-ipc
```

**Error**: Permission denied on `/tmp/vyra_sockets`
```bash
chmod 777 /tmp/vyra_sockets
```

**Issue**: Stale segments after crash
```python
from vyra_base.com.transport.shared_memory import get_discovery
discovery = get_discovery()
discovery.cleanup_stale_segments()
```

### ROS2 Issues

**Error**: `rclpy not available`
```bash
source /opt/ros/<distro>/setup.bash
```

**Error**: SROS2 permission denied
- Check keystore permissions in `sros2_keystore/`
- Disable security: `export ROS_SECURITY_ENABLE=false`

### UDS Issues

**Error**: Socket file not found
- Check `/tmp/vyra_sockets/` exists
- Verify server is running

**Error**: Connection refused
- Server not listening
- Socket path mismatch

## Contributing

When adding new transport protocols:

1. Implement `AbstractProtocolProvider`
2. Create protocol-specific callable/speaker/job classes
3. Add availability check with graceful fallback
4. Write unit and integration tests
5. Update documentation and examples

## License

Part of the VYRA framework - see main LICENSE file.
