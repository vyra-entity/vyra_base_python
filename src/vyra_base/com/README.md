# VYRA Communication Architecture

Professional multi-protocol communication system for distributed applications.

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    Application Layer                        │
│  (@remote_callable, @remote_speaker, InterfaceFactory)      │
└────────────────────────┬────────────────────────────────────┘
                         │
┌────────────────────────▼────────────────────────────────────┐
│                   Core Layer                                │
│  (Types, Exceptions, Registry, Factory, Decorators)         │
└────────────────────────┬────────────────────────────────────┘
                         │
          ┌──────────────┼──────────────┬─────────────┐
          │              │              │             │
┌─────────▼────┐  ┌─────▼──────┐  ┌───▼──────────┐ ┌▼────────┐
│  Transport   │  │  External  │  │  Industrial  │ │Converter│
│   Layer      │  │   Layer    │  │    Layer     │ │  Layer  │
├──────────────┤  ├────────────┤  ├──────────────┤ ├─────────┤
│ • ROS2       │  │ • Redis    │  │ • Modbus     │ │ • Proto │
│ • Zenoh      │  │ • gRPC     │  │ • OPC UA     │ │         │
│ • Shared Mem │  │ • MQTT     │  │              │ └─────────┘
│ • UDS        │  │ • REST     │  │              │
│              │  │ • WebSocket│  │              │
└──────────────┘  └────────────┘  └──────────────┘
```

## Features

### 🚀 Multi-Protocol Support
- **Transport Layer**: ROS2, Zenoh, Redis, Unix Domain Sockets, Shared Memory
- **External Layer**: gRPC, MQTT, REST, WebSocket
- **Industrial Layer**: Modbus, OPC UA (northbound)

### 🔄 Automatic Fallback
```python
# Tries ROS2 → Zenoh → Redis → UDS
callable = await InterfaceFactory.create_callable(
    "my_service",
    callback=handle_request
)
```

### 🎯 Protocol-Agnostic Decorators
```python
from vyra_base.com import remote_service, remote_publisher, remote_actionServer
from vyra_base.com.core import IActionHandler, IGoalHandle, ActionStatus

class MyComponent(IActionHandler):
    @remote_service
    async def process(self, request, response=None):
        return {"result": request["value"] * 2}
    
    @remote_publisher(protocols=[ProtocolType.REDIS])
    async def publish_status(self, message):
        pass  # Automatically uses Redis Pub/Sub
    
    # NEW: Multi-callback ActionServer pattern (Blueprint System)
    @remote_actionServer.on_goal(name="long_task")
    async def accept_goal(self, goal_request) -> bool:
        return True  # Accept goal
    
    @remote_actionServer.on_cancel(name="long_task")
    async def handle_cancel(self, goal_handle: IGoalHandle) -> bool:
        return True  # Accept cancellation
    
    @remote_actionServer.execute(name="long_task")
    async def execute_task(self, goal_handle: IGoalHandle) -> dict:
        # Long-running execution with feedback
        for i in range(10):
            if goal_handle.is_cancel_requested():
                goal_handle.canceled()
                return {"status": ActionStatus.CANCELED}
            goal_handle.publish_feedback({"progress": (i+1)*10})
        goal_handle.succeed()
        return {"status": ActionStatus.SUCCEEDED, "result": "done"}
```

### 📊 Built-in Monitoring
- Prometheus metrics integration
- ROS2/DDS-specific metrics
- cAdvisor container metrics

## Quick Start

### Basic Usage

```python
from vyra_base.com import InterfaceFactory, ProtocolType

# Auto-select best protocol
callable = await InterfaceFactory.create_callable(
    "calculate",
    callback=lambda req: {"result": req["x"] + req["y"]}
)

# Explicit protocol with fallback
speaker = await InterfaceFactory.create_speaker(
    "updates",
    protocols=[ProtocolType.REDIS, ProtocolType.MQTT]
)

await speaker.shout({"event": "update", "data": 123})
```

### Provider-Based Usage

```python
from vyra_base.com.external import RedisProvider

# Initialize provider
provider = RedisProvider(
    host="localhost",
    port=6379,
    username="user",
    password="pass"
)
await provider.initialize()

# Create speaker
speaker = await provider.create_speaker(
    "temperature",
    callback=lambda msg: print(f"Temp: {msg}")
)

# Publish
await speaker.shout({"value": 23.5, "unit": "°C"})
```

## Protocol Details

### Transport Layer

#### ROS2 (Optional)
- **Type**: Service-oriented middleware
- **Use Case**: Robot systems, distributed computing
- **Features**: QoS, DDS, Security (SROS2)
- **Availability**: Requires ROS2 installation

#### Shared Memory
- **Type**: POSIX IPC
- **Use Case**: High-performance local IPC
- **Features**: Zero-copy, PID-based discovery
- **Availability**: Always (Linux/Unix)

#### Unix Domain Sockets
- **Type**: Local socket communication
- **Use Case**: Local service communication
- **Features**: Low latency, file-based addressing
- **Availability**: Always (Linux/Unix)

### External Layer

#### Redis
- **Type**: Pub/Sub + Key-Value Store
- **Use Case**: Distributed caching, messaging
- **Features**: TLS, ACL, Streams, Clustering
- **Install**: `pip install redis`

#### gRPC
- **Type**: RPC over Unix Sockets
- **Use Case**: High-performance RPC
- **Features**: Protobuf, Streaming, Load balancing
- **Install**: `pip install grpcio grpcio-tools`

#### MQTT
- **Type**: Lightweight pub/sub
- **Use Case**: IoT, constrained devices
- **Features**: QoS 0/1/2, LWT, Wildcards
- **Install**: `pip install paho-mqtt`

#### REST
- **Type**: HTTP API
- **Use Case**: Web services, microservices
- **Features**: JSON, CORS, TLS
- **Install**: `pip install aiohttp`

#### WebSocket
- **Type**: Bidirectional streaming
- **Use Case**: Real-time web apps
- **Features**: Low latency, Browser compatible
- **Install**: `pip install websockets`

### Industrial Layer

#### Modbus
- **Type**: Industrial protocol
- **Use Case**: PLC, SCADA integration
- **Features**: TCP/RTU, Register access
- **Install**: `pip install pymodbus`

#### OPC UA
- **Type**: Industrial automation
- **Use Case**: MES/SCADA northbound
- **Features**: Security, Discovery, Companion specs
- **Install**: `pip install asyncua`

## Configuration

### Protocol Priorities

Customize fallback order:

```python
from vyra_base.com import InterfaceFactory, ProtocolType

# Custom callable fallback
InterfaceFactory.set_fallback_chain(
    "callable",
    [ProtocolType.SHARED_MEMORY, ProtocolType.ROS2, ProtocolType.UDS]
)

# Custom speaker fallback
InterfaceFactory.set_fallback_chain(
    "speaker",
    [ProtocolType.REDIS, ProtocolType.MQTT]
)
```

### Provider Registration

```python
from vyra_base.com.providers import ProviderRegistry
from vyra_base.com.external import RedisProvider

registry = ProviderRegistry()

# Register custom provider
redis_provider = RedisProvider(host="redis.local")
await redis_provider.initialize()
registry.register_provider(ProtocolType.REDIS, redis_provider)
```

## Monitoring

### Prometheus Metrics

```python
from vyra_base.com.monitoring import CommunicationMonitor

monitor = CommunicationMonitor()

# Metrics available:
# - vyra_com_calls_total
# - vyra_com_call_duration_seconds
# - vyra_com_active_connections
# - vyra_com_errors_total
# - vyra_com_message_size_bytes
```

### ROS2-Specific Metrics

When ROS2 is available:
- `vyra_ros2_topic_throughput`
- `vyra_ros2_qos_violations`
- `vyra_ros2_discovery_nodes`
- `vyra_ros2_service_latency`

## Migration Guide

### From Old Datalayer

**Before:**
```python
from vyra_base.com.datalayer.interface_factory import remote_callable

class Component:
    @remote_callable
    async def my_method(self, request, response=None):
        return result
```

**After:**
```python
from vyra_base.com import remote_service  # Renamed from remote_callable

class Component:
    @remote_service  # Now supports multi-protocol
    async def my_method(self, request, response=None):
        return result
```

### ActionServer Migration

**Before (Legacy Single Callback):**
```python
@remote_actionServer(name="process")
async def execute_action(self, goal_handle):
    # Mixed concerns: validation, cancellation, execution
    ...
```

**After (Multi-Callback Blueprint Pattern):**
```python
from vyra_base.com.core import IActionHandler, IGoalHandle

class Component(IActionHandler):  # REQUIRED interface
    @remote_actionServer.on_goal(name="process")
    async def accept_goal(self, goal_request) -> bool:
        # Clean goal validation
        return goal_request.count <= 100
    
    @remote_actionServer.on_cancel(name="process")
    async def handle_cancel(self, goal_handle: IGoalHandle) -> bool:
        # Dedicated cancellation logic
        return True
    
    @remote_actionServer.execute(name="process")
    async def execute(self, goal_handle: IGoalHandle) -> dict:
        # Pure execution logic
        ...
```

**Benefits:**
- ✅ Separation of concerns
- ✅ Early goal rejection
- ✅ Better testability
- ✅ Aligns with IActionHandler interface (REQUIRED)

### Protocol-Specific Methods

**Before:**
```python
# ROS2-only
from vyra_base.com.datalayer.interface_factory import create_vyra_speaker

speaker = create_vyra_speaker(
    type=msg_type,
    node=ros2_node,
    ident_name="my_speaker"
)
```

**After:**
```python
# Multi-protocol
from vyra_base.com import InterfaceFactory, ProtocolType

speaker = await InterfaceFactory.create_speaker(
    "my_speaker",
    protocols=[ProtocolType.REDIS, ProtocolType.ROS2]
)
```

## Best Practices

### 1. Use InterfaceFactory for Flexibility
```python
# ✅ Good - Automatic protocol selection
callable = await InterfaceFactory.create_callable("service")

# ❌ Avoid - Hard-coded protocol
from vyra_base.com.transport.t_ros2 import ROS2Provider
provider = ROS2Provider()  # Only works with ROS2
```

### 2. Graceful Degradation
```python
# Check available protocols
from vyra_base.com import InterfaceFactory, ProtocolType

available = InterfaceFactory.get_available_protocols()
if ProtocolType.ROS2 not in available:
    logger.warning("ROS2 not available, using fallback")
```

### 3. Error Handling
```python
from vyra_base.com import InterfaceError, ProtocolUnavailableError

try:
    callable = await InterfaceFactory.create_callable("service")
except ProtocolUnavailableError:
    logger.error("No protocols available")
except InterfaceError as e:
    logger.error(f"Interface creation failed: {e}")
```

### 4. Resource Cleanup
```python
# Always cleanup
async with provider:
    speaker = await provider.create_speaker("topic")
    await speaker.shout(message)
# Auto-cleanup on exit

# Or manually:
try:
    speaker = await provider.create_speaker("topic")
    await speaker.shout(message)
finally:
    await speaker.shutdown()
    await provider.shutdown()
```

## Troubleshooting

### Protocol Not Available
```
ProtocolUnavailableError: ROS2 not available
```
**Solution**: Install ROS2 or configure fallback protocols

### Connection Timeout
```
TimeoutError: Call to 'service' timed out after 5.0s
```
**Solution**: Check network, increase timeout, verify service is running

### Shared Memory Permission Error
```
PermissionError: Cannot access /dev/shm/vyra_*
```
**Solution**: Check file permissions, run with proper user rights

## Architecture Decisions

### Why Multi-Protocol?
- **Flexibility**: Different use cases need different protocols
- **Resilience**: Automatic fallback ensures reliability
- **Performance**: Choose optimal protocol per scenario
- **Migration**: Gradual transition from ROS2-only

### Why Provider Pattern?
- **Abstraction**: Uniform interface across protocols
- **Testability**: Easy to mock providers
- **Extensibility**: Add new protocols without breaking changes

### Why Optional ROS2?
- **Slim Images**: Reduce container size by ~2GB
- **Embedded Systems**: Run on constrained devices
- **Cloud Deployments**: Use native cloud protocols

## Contributing

See [CONTRIBUTING.md](../../CONTRIBUTING.md) for development guidelines.

## License

VYRA Framework © 2026 Variobotic GmbH
