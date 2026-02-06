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
    ├── zenoh/             # ✅ Eclipse Zenoh transport (DEFAULT)
    ├── ros2/              # ✅ ROS2/DDS transport
    ├── redis/             # ✅ Redis Pub/Sub
    └── uds/               # ✅ Unix domain sockets
```

## Supported Protocols

### 1. Zenoh Transport ⚡ (DEFAULT)

**Unified pub/sub and query/reply with zero-copy capabilities**

#### Features
- Efficient pub/sub with minimal overhead
- Query/reply for request-response patterns
- Router-based scalability
- Zero-copy shared memory transport
- Protocol agnostic (TCP, UDP, SHM)
- **Default protocol for all VYRA communication**

#### Requirements
- `eclipse-zenoh` package: `pip install eclipse-zenoh`
- Zenoh router (Docker service)

See [Zenoh Transport README](zenoh/README.md)

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

See [ROS2 Transport README](ros2/README.md)

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
from vyra_base.com.transport.t_uds import UDSProvider
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

All transport protocols implement three core communication patterns with consistent role terminology:

### 1. Speaker (Publish-Subscribe)

**One-to-many message broadcasting**

#### Roles
- **Shouter (Publisher)**: Broadcasts messages to a topic/channel
- **Listener (Subscriber)**: Receives messages from a topic/channel

#### Usage

```python
# Shouter (Publisher)
speaker = await provider.create_speaker("sensor/temperature", is_publisher=True)
await speaker.shout({"value": 23.5, "unit": "celsius"})

# Listener (Subscriber)
listener = await provider.create_speaker("sensor/temperature", is_publisher=False)

def handle_message(data):
    print(f"Temperature: {data['value']}°C")

await listener.listen(handle_message)
```

**Supported by**: Zenoh, ROS2, Redis

#### Methods
- `shout(data)`: Publish message (shouter/publisher only)
- `listen(callback)`: Subscribe to messages (listener/subscriber only)
- `shutdown()`: Cleanup resources

---

### 2. Callable (Request-Response)

**One-to-one bidirectional communication with response**

#### Roles
- **Responder (Server)**: Handles requests and returns responses
- **Requester (Client)**: Sends requests and waits for responses

#### Usage

```python
# Responder (Server)
async def handle_calculate(request):
    result = request["a"] + request["b"]
    return {"result": result}

responder = await provider.create_callable(
    "calculate",
    callback=handle_calculate,
    is_server=True
)

# Requester (Client)
requester = await provider.create_callable("calculate", is_server=False)
response = await requester.call({"a": 10, "b": 32}, timeout=5.0)
print(response["result"])  # 42
```

**Supported by**: Zenoh, ROS2, UDS

#### Methods
- `call(request, timeout)`: Send request and wait for response (requester/client only)
- Server automatically responds when callback returns

---

### 3. Job (Long-Running Task with Feedback)

**Asynchronous task execution with progress updates**

#### Roles
- **Greeter**: Starts the job and provides parameters
- **Talker**: Sends progress updates during execution
- **Finisher**: Completes job and sends final result

#### Workflow
```
Greeting → Talking* → Finishing
   ↓         ↓          ↓
  Start   Feedback   Complete
```

#### Usage

```python
# Server side (handles greeting, talking, finishing)
async def process_data(params, publish_feedback):
    """
    params: Job parameters (greeting phase)
    publish_feedback: Callback for progress updates (talking phase)
    returns: Final result (finishing phase)
    """
    total_steps = 100
    
    for i in range(total_steps):
        # Talking - send progress feedback
        await publish_feedback({
            "progress": (i + 1) / total_steps * 100,
            "current_step": i + 1,
            "message": f"Processing step {i + 1}/{total_steps}"
        })
        await asyncio.sleep(0.1)
    
    # Finishing - return final result
    return {
        "status": "complete",
        "result": params["input"] * 2,
        "steps_completed": total_steps
    }

job_server = await provider.create_job(
    "data_processing",
    result_callback=process_data,
    is_server=True
)

# Client side (greeting and receiving updates)
job_client = await provider.create_job("data_processing", is_server=False)

# Greeting - start the job
async def handle_feedback(feedback):
    """Called during talking phase"""
    print(f"Progress: {feedback['progress']}% - {feedback['message']}")

async def handle_result(result):
    """Called during finishing phase"""
    print(f"Job completed: {result['status']}, Result: {result['result']}")

# Listen for talking (feedback) and finishing (result)
await job_client.listen_feedback(handle_feedback)
await job_client.listen_result(handle_result)

# Send greeting (start job)
await job_client.start({"input": 42})
```

**Supported by**: Zenoh, ROS2 (Actions)

#### Methods
- **Server**:
  - `result_callback(params, publish_feedback)`: Job execution logic
  - Callback receives `publish_feedback` for progress updates
  
- **Client**:
  - `start(params)`: Greeting - initiate job with parameters
  - `listen_feedback(callback)`: Subscribe to talking - progress updates
  - `listen_result(callback)`: Subscribe to finishing - final result
  - `cancel()`: Cancel running job

#### Phases

1. **Greeting Phase**: Client sends job parameters via `start()`
2. **Talking Phase**: Server sends progress via `publish_feedback()`, client receives via `listen_feedback()`
3. **Finishing Phase**: Server returns final result, client receives via `listen_result()`

---

## Pattern Comparison

| Pattern | Direction | Response | Use Case | Roles |
|---------|-----------|----------|----------|-------|
| Speaker | One-to-many | None | Sensor data, events | Shouter → Listener |
| Callable | One-to-one | Immediate | Service calls, queries | Requester ↔ Responder |
| Job | One-to-one | Deferred + Progress | Long tasks, batch processing | Greeter → Talker → Finisher |

---

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

| Protocol | Latency   | Throughput | Scope   | Default |
|----------|-----------|------------|---------|---------|
| Zenoh    | <1ms      | Very High  | Network | ✅ Yes  |
| ROS2     | 1-10ms    | High       | Network | Fallback |
| Redis    | 2-5ms     | Medium     | Network | Fallback |
| UDS      | <1ms      | High       | Local   | Fallback |

## Migration from `datalayer`

All ROS2 components are now available in both locations:

```python
# Old (still works)
from vyra_base.com.datalayer.node import VyraNode

# New (recommended)
from vyra_base.com.transport.t_ros2 import VyraNode
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
 (Automatic Fallback)

The VYRA framework automatically selects the best available protocol:

**Default Fallback Order**:
1. **Zenoh** (primary) - Zero-copy, scalable, efficient
2. **ROS2** (fallback) - Distributed DDS-based communication
3. *3Redis** (fallback) - Pub/sub messaging
4. **UDS** (fallback) - Local-only request-response

```python
# Automatic protocol selection
@remote_callable
async def my_service(request, response=None):
    return {"result": request["value"] * 2}
# Uses Zenoh if available, falls back to ROS2, then Redis, then UDS

# Explicit protocol override
@remote_callable(protocols=[ProtocolType.ROS2])
async def ros2_only_service(request, response=None):
    return {"result": request["value"]}
```

### 2. Pattern Selection

- **Speaker (Shouter/Listener)**: Sensor data, events, state broadcasts
- **Callable (Requester/Responder)**: Configuration, queries, fast operations
- **Job (Greeter/Talker/Finisher)**: Data processing, long tasks, progress track
### 2. Error Handling

Always check availability and handle initialization:

```python
from vyra_base.com.transport.t_zenoh import ZenohProvider, ZENOH_AVAILABLE

if not ZENOH_AVAILABLE:
    logger.warningProvider not available")
    # Framework will auto-fallback to next protocol
    return

if not await provider.initialize():
    logger.error("Failed to initialize provider")
    return
```

### 4t await provider.initialize():
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

### 5. Timeout Configuration

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

### Zenoh Issues

**Error**: `eclipse-zenoh not installed`
```bash
pip install eclipse-zenoh
```

**Error**: Cannot connect to router
```bash
# Check router status
docker ps | grep zenoh-router

# View logs
docker logs zenoh-router -f

# Test connection
telnet zenoh-router 7447
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
