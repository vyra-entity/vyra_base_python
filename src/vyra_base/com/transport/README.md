# VYRA Transport Layer

Complete implementation of pluggable communication protocols for the VYRA framework.

## Overview

The VYRA Transport Layer provides a unified abstraction for multiple communication protocols, implementing the **Provider Pattern** for seamless protocol switching with automatic fallback.

### Architecture

```
vyra_base/com/
├── core/                    # Base types and exceptions
│   ├── types.py            # VyraPublisher, VyraSubscriber, VyraServer, VyraClient, VyraActionServer, VyraActionClient
│   ├── topic_builder.py    # Dynamic topic/service name generation
│   └── exceptions.py       # InterfaceError, ProtocolUnavailableError
├── providers/              # Provider interface
│   └── protocol_provider.py  # AbstractProtocolProvider
└── transport/              # Protocol implementations
    ├── t_zenoh/           # ✅ Eclipse Zenoh transport (DEFAULT)
    ├── t_ros2/            # ✅ ROS2/DDS transport
    ├── t_redis/           # ✅ Redis Pub/Sub
    └── t_uds/             # ✅ Unix domain sockets
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

See [Zenoh Transport README](t_zenoh/README.md)

### 2. ROS2 Transport 🤖

**Distributed communication via DDS middleware**

#### Features
- ROS2 services, topics, and actions
- QoS policies for reliability/latency tradeoffs
- Discovery and introspection
- SROS2 security support

#### Requirements
- ROS2 installation (Humble, Iron, Jazzy, or Kilted)
- Source `/opt/ros/<distro>/setup.bash`

### 3. Redis Transport 📮

**Message broker with pub/sub and state tracking**

#### Features
- Redis Pub/Sub for messaging
- Key-value storage for state
- Async client via redis.asyncio
- JSON serialization

#### Requirements
- Redis server running
- `redis` package: `pip install redis`

### 4. UDS Transport 🔌

**Stream-based local IPC via Unix domain sockets**

#### Features
- Low-latency stream communication
- JSON serialization with length-prefix framing
- Automatic connection management
- Request-response and action patterns

#### Requirements
- Unix-like OS (Linux, macOS, BSD)
- Local communication only

---

## Unified Interface Types

The VYRA Transport Layer implements **6 unified interface types** replacing the legacy 3-pattern system:

### 1. Publisher & Subscriber (Pub/Sub)

**One-to-many message broadcasting**

#### Publisher
Broadcasts messages to a topic without expecting responses.

```python
from vyra_base.com.transport.t_ros2 import ROS2Provider

provider = ROS2Provider(protocol=ProtocolType.ROS2, module_name="sensor_node")
await provider.initialize()

# Create publisher
publisher = await provider.create_publisher(
    name="temperature",
    msg_type=TemperatureMsg
)

# Publish messages
await publisher.publish(TemperatureMsg(value=23.5, unit="celsius"))
```

#### Subscriber
Receives messages from a topic via callback.

```python
# Create subscriber
async def handle_temperature(msg):
    print(f"Temperature: {msg.value}°{msg.unit}")

subscriber = await provider.create_subscriber(
    name="temperature",
    msg_type=TemperatureMsg,
    callback=handle_temperature
)
```

**Supported by**: Zenoh, ROS2, Redis

---

### 2. Server & Client (Request/Response)

**One-to-one bidirectional communication with immediate response**

#### Server
Handles requests and returns responses synchronously.

```python
# Create server
async def handle_calculate(request, response=None):
    result = request.a + request.b
    if response:
        response.result = result
        return response
    return CalculateResponse(result=result)

server = await provider.create_server(
    name="calculate",
    callback=handle_calculate,
    srv_type=CalculateService
)
```

#### Client
Sends requests and awaits responses.

```python
# Create client
client = await provider.create_client(
    name="calculate",
    srv_type=CalculateService
)

# Call service
request = CalculateRequest(a=10, b=32)
response = await client.call(request, timeout=5.0)
print(response.result)  # 42
```

**Supported by**: Zenoh, ROS2, Redis, UDS

---

### 3. ActionServer & ActionClient (Long-Running Tasks)

**Asynchronous task execution with progress feedback**

#### ActionServer
Executes goals, publishes feedback, and returns results.

**NEW**: Multi-callback pattern with decorators for better code organization. See [Blueprint System](../core/README.md#blueprint-system-blueprintspy).

**Recommended: Decorator Pattern**
```python
from vyra_base.com import remote_actionServer
from vyra_base.com.core import IActionHandler, IGoalHandle, ActionStatus

class MyComponent(IActionHandler):  # REQUIRED interface
    @remote_actionServer.on_goal(name="data_processing")
    async def accept_goal(self, goal_request) -> bool:
        """Validate and accept/reject goal"""
        return goal_request.input > 0
    
    @remote_actionServer.on_cancel(name="data_processing")
    async def handle_cancel(self, goal_handle: IGoalHandle) -> bool:
        """Handle cancellation request"""
        return True
    
    @remote_actionServer.execute(name="data_processing")
    async def execute(self, goal_handle: IGoalHandle) -> dict:
        """Execute with feedback"""
        for i in range(100):
            if goal_handle.is_cancel_requested():
                goal_handle.canceled()
                return {"status": ActionStatus.CANCELED}
            
            goal_handle.publish_feedback({"progress": i, "message": f"Step {i}/100"})
            await asyncio.sleep(0.1)
        
        goal_handle.succeed()
        return {"status": ActionStatus.SUCCEEDED, "data": goal_handle.goal.input * 2}
```

**Direct Provider Usage (Low-Level)**
```python
# Create action server
async def handle_goal_request(goal):
    # Accept or reject goal
    return True

async def handle_cancel_request(goal_handle):
    # Handle cancellation
    return True

async def execute_goal(goal_handle):
    # Execute task with feedback
    for i in range(100):
        if goal_handle.is_canceled:
            return ProcessResult(status="canceled")
        
        # Publish feedback
        await goal_handle.publish_feedback(
            ProcessFeedback(progress=i, message=f"Step {i}/100")
        )
        await asyncio.sleep(0.1)
    
    return ProcessResult(status="complete", data=goal_handle.goal.input * 2)

action_server = await provider.create_action_server(
    name="data_processing",
    handle_goal_request=handle_goal_request,
    handle_cancel_request=handle_cancel_request,
    execution_callback=execute_goal,
    action_type=ProcessAction
)
```

#### ActionClient
Sends goals, receives feedback, and gets results.

```python
# Create action client
async def handle_feedback(feedback):
    print(f"Progress: {feedback.progress}% - {feedback.message}")

async def handle_result(result):
    print(f"Completed: {result.status}, Data: {result.data}")

action_client = await provider.create_action_client(
    name="data_processing",
    feedback_callback=handle_feedback,
    goal_callback=handle_result,
    action_type=ProcessAction
)

# Send goal
goal_id = await action_client.send_goal(ProcessGoal(input=42))

# Cancel goal (optional)
await action_client.cancel_goal(goal_id)
```

**Supported by**: Zenoh, ROS2, Redis, UDS

---

## Interface Comparison

| Interface | Direction | Response | Feedback | Use Case |
|-----------|-----------|----------|----------|----------|
| Publisher | One-to-many | None | No | Sensor data, events, state broadcasts |
| Subscriber | Many-to-one | None | No | Event listening, data collection |
| Server | One-to-one | Immediate | No | Configuration, queries, fast operations |
| Client | One-to-one | Immediate | No | Service calls, RPC |
| ActionServer | One-to-one | Deferred | Yes | Data processing, long tasks |
| ActionClient | One-to-one | Deferred | Yes | Progress tracking, cancelable operations |

---

## Provider Pattern

All transports implement `AbstractProtocolProvider` with 6 create methods:

```python
from vyra_base.com.providers.protocol_provider import AbstractProtocolProvider

class CustomProvider(AbstractProtocolProvider):
    async def create_publisher(self, name: str, msg_type: type, **kwargs) -> VyraPublisher:
        """Create publisher for broadcasting messages"""
        
    async def create_subscriber(self, name: str, msg_type: type, callback: Callable, **kwargs) -> VyraSubscriber:
        """Create subscriber for receiving messages"""
        
    async def create_server(self, name: str, srv_type: type, callback: Callable, **kwargs) -> VyraServer:
        """Create server for handling requests"""
        
    async def create_client(self, name: str, srv_type: type, **kwargs) -> VyraClient:
        """Create client for sending requests"""
        
    async def create_action_server(self, name: str, action_type: type, 
                                   handle_goal_request: Callable,
                                   handle_cancel_request: Callable,
                                   execution_callback: Callable, **kwargs) -> VyraActionServer:
        """Create action server for long-running tasks"""
        
    async def create_action_client(self, name: str, action_type: type,
                                   feedback_callback: Callable = None,
                                   goal_callback: Callable = None, **kwargs) -> VyraActionClient:
        """Create action client for goal execution"""
```

---

## Automatic Protocol Fallback

The VYRA framework automatically selects the best available protocol:

**Default Fallback Order**:
1. **ROS2** (primary) - Well-tested, full feature support
2. **Zenoh** (fallback) - High performance when ROS2 unavailable
3. **Redis** (fallback) - Simple pub/sub when DDS unavailable
4. **UDS** (fallback) - Local-only fallback

```python
from vyra_base.com.core.decorators import remote_callable

# Automatic protocol selection with fallback
@remote_callable
async def my_service(request, response=None):
    response.result = request.value * 2
    return response
# Framework tries ROS2 → Zenoh → Redis → UDS

# Explicit protocol override
@remote_callable(protocols=[ProtocolType.ROS2])
async def ros2_only_service(request, response=None):
    response.result = request.value
    return response
```

---

## Error Handling

Transport layer uses hierarchical exceptions:

```python
from vyra_base.com.core.exceptions import (
    InterfaceError,              # Base interface exception
    ProtocolUnavailableError,    # Protocol not installed
    ProtocolNotInitializedError, # Provider not initialized
    TransportError,              # Transport-level error
    ConnectionError,             # Connection failed
    TimeoutError,                # Operation timeout
)

try:
    result = await client.call(request, timeout=5.0)
except TimeoutError:
    logger.error("Service timeout")
except InterfaceError as e:
    logger.error(f"Service error: {e}")
```

---

## Best Practices

### 1. Provider Initialization

Always check availability before initializing:

```python
from vyra_base.com.transport.t_ros2 import ROS2Provider, ROS2_AVAILABLE

if not ROS2_AVAILABLE:
    logger.warning("ROS2 not available, falling back")
    # Framework will auto-select next protocol
    provider = ZenohProvider(...)
else:
    provider = ROS2Provider(...)

if not await provider.initialize():
    logger.error("Failed to initialize provider")
    return
```

### 2. Resource Cleanup

Always shutdown interfaces and providers:

```python
try:
    server = await provider.create_server("service", callback, srv_type)
    # Use server
finally:
    await server.shutdown()
    await provider.shutdown()
```

### 3. Timeout Configuration

Set appropriate timeouts based on operation:

```python
# Fast local operation
result = await client.call(request, timeout=1.0)

# Network operation
result = await client.call(request, timeout=5.0)

# Long-running action
goal_id = await action_client.send_goal(goal, timeout=30.0)
```

### 4. Interface Selection Guidelines

- **Publisher**: Sensor data, periodic updates, events
- **Subscriber**: Data collection, event handling
- **Server/Client**: Configuration, queries, synchronous operations (<1 second)
- **ActionServer/ActionClient**: Data processing, tasks with progress (>1 second)

---

## Performance Characteristics

| Protocol | Latency   | Throughput | Scope   | Default Priority |
|----------|-----------|------------|---------|------------------|
| ROS2     | 1-10ms    | High       | Network | 1 (Primary)      |
| Zenoh    | <1ms      | Very High  | Network | 2 (Fallback)     |
| Redis    | 2-5ms     | Medium     | Network | 3 (Fallback)     |
| UDS      | <1ms      | High       | Local   | 4 (Fallback)     |

---

## Testing

Run transport tests:

```bash
# Unit tests (no external dependencies)
pytest tests/com/transport/ -m unit -v

# Integration tests (requires Redis, ROS2, Zenoh)
pytest tests/com/transport/ -m integration -v

# Specific transport
pytest tests/com/transport/t_ros2/ -v
pytest tests/com/transport/t_zenoh/ -v
pytest tests/com/transport/t_redis/ -v
pytest tests/com/transport/t_uds/ -v
```

---

## Migration from Legacy API

### Old 3-Pattern System → New 6-Interface System

| Legacy Pattern | Legacy Method | New Interface | New Method |
|----------------|---------------|---------------|------------|
| Speaker (Publisher) | `create_speaker(name, is_publisher=True)` | Publisher | `create_publisher(name, msg_type)` |
| Speaker (Subscriber) | `create_speaker(name, is_publisher=False)` | Subscriber | `create_subscriber(name, msg_type, callback)` |
| Callable (Server) | `create_callable(name, callback, is_server=True)` | Server | `create_server(name, srv_type, callback)` |
| Callable (Client) | `create_callable(name, is_server=False)` | Client | `create_client(name, srv_type)` |
| Job (Server) | `create_job(name, result_callback, is_server=True)` | ActionServer | `create_action_server(name, action_type, ...)` |
| Job (Client) | `create_job(name, is_server=False)` | ActionClient | `create_action_client(name, action_type, ...)` |

### Legacy Code Example

```python
# ❌ OLD (Deprecated)
speaker = await provider.create_speaker("temp", is_publisher=True)
await speaker.shout({"value": 23.5})

callable_server = await provider.create_callable("calc", callback, is_server=True)
callable_client = await provider.create_callable("calc", is_server=False)
result = await callable_client.call(request)

job = await provider.create_job("process", result_callback, is_server=True)
```

### New API Example

```python
# ✅ NEW (Current)
publisher = await provider.create_publisher("temp", TemperatureMsg)
await publisher.publish(TemperatureMsg(value=23.5))

server = await provider.create_server("calc", CalculateService, callback)
client = await provider.create_client("calc", CalculateService)
result = await client.call(request)

action_server = await provider.create_action_server(
    "process", ProcessAction, 
    handle_goal_request, handle_cancel_request, execution_callback
)
```

---

## Troubleshooting

### General Issues

**Error**: `No protocol available`
- Install at least one transport: `pip install rclpy` or `pip install eclipse-zenoh` or `pip install redis`
- Check availability: `await provider.check_availability()`

**Error**: `Provider not initialized`
- Call `await provider.initialize()` before creating interfaces
- Check initialization return value: `if not await provider.initialize(): ...`

### ROS2 Issues

**Error**: `rclpy not available`
```bash
# Install ROS2
sudo apt install ros-humble-desktop
source /opt/ros/humble/setup.bash
```

**Error**: SROS2 permission denied
```bash
# Check keystore permissions
ls -la sros2_keystore/

# Disable security (development only)
export ROS_SECURITY_ENABLE=false
```

**Error**: Service not found
- Verify service name matches: use `ros2 service list`
- Check node discovery: `ros2 node list`
- Ensure both nodes in same ROS_DOMAIN_ID

### Zenoh Issues

**Error**: `eclipse-zenoh not installed`
```bash
pip install eclipse-zenoh
```

**Error**: Cannot connect to Zenoh router
```bash
# Check router status
docker ps | grep zenoh

# Start router
docker run -d --name zenoh-router -p 7447:7447 eclipse/zenoh:latest

# Test connection
python -c "import zenoh; session = zenoh.open({}); print('Connected')"
```

**Error**: Zenoh session None
- Ensure Zenoh router is running
- Check network connectivity
- Verify Zenoh config in provider initialization

### Redis Issues

**Error**: `redis not installed`
```bash
pip install redis
```

**Error**: Connection refused
```bash
# Check Redis server
redis-cli ping  # Should return "PONG"

# Start Redis (Docker)
docker run -d --name redis -p 6379:6379 redis:latest

# Start Redis (Linux)
sudo systemctl start redis
```

**Error**: SSL/TLS errors
- Check certificate paths in RedisClient initialization
- Verify certificates exist and are readable
- For testing, disable TLS: `RedisClient(ssl=False)`

### UDS Issues

**Error**: Socket file not found
```bash
# Check socket directory exists
ls -la /tmp/vyra_sockets/

# Create if missing
mkdir -p /tmp/vyra_sockets/
```

**Error**: Permission denied
```bash
# Fix permissions
chmod 755 /tmp/vyra_sockets/
```

**Error**: Connection refused
- Ensure server is running before client connects
- Check socket path matches between server and client
- Verify module_name is consistent

---

## Examples

### Complete Publisher/Subscriber Example

```python
import asyncio
from vyra_base.com.transport.t_ros2 import ROS2Provider
from vyra_base.com.core.types import ProtocolType
from example_msgs.msg import Temperature  # Your message type

async def main():
    # Initialize provider
    provider = ROS2Provider(
        protocol=ProtocolType.ROS2,
        module_name="temperature_node"
    )
    
    if not await provider.initialize():
        print("Failed to initialize provider")
        return
    
    try:
        # Create publisher
        publisher = await provider.create_publisher(
            name="temperature",
            msg_type=Temperature
        )
        
        # Create subscriber
        async def handle_temp(msg):
            print(f"Received: {msg.value}°C")
        
        subscriber = await provider.create_subscriber(
            name="temperature",
            msg_type=Temperature,
            callback=handle_temp
        )
        
        # Publish messages
        for i in range(10):
            await publisher.publish(Temperature(value=20.0 + i))
            await asyncio.sleep(1.0)
            
    finally:
        await provider.shutdown()

if __name__ == "__main__":
    asyncio.run(main())
```

### Complete Server/Client Example

```python
import asyncio
from vyra_base.com.transport.t_uds import UDSProvider
from vyra_base.com.core.types import ProtocolType
from example_srvs.srv import Calculate  # Your service type

async def main():
    provider = UDSProvider(
        protocol=ProtocolType.UDS,
        module_name="calculator"
    )
    
    if not await provider.initialize():
        return
    
    try:
        # Create server
        async def calculate_handler(request, response=None):
            if response is None:
                response = Calculate.Response()
            response.result = request.a + request.b
            return response
        
        server = await provider.create_server(
            name="calculate",
            srv_type=Calculate,
            callback=calculate_handler
        )
        
        # Create client
        client = await provider.create_client(
            name="calculate",
            srv_type=Calculate
        )
        
        # Call service
        request = Calculate.Request(a=10, b=32)
        response = await client.call(request, timeout=5.0)
        print(f"Result: {response.result}")  # 42
        
    finally:
        await provider.shutdown()

if __name__ == "__main__":
    asyncio.run(main())
```

### Complete Action Example

```python
import asyncio
from vyra_base.com.transport.t_ros2 import ROS2Provider
from vyra_base.com.core.types import ProtocolType
from example_actions.action import Process  # Your action type

async def main():
    provider = ROS2Provider(
        protocol=ProtocolType.ROS2,
        module_name="processor"
    )
    
    if not await provider.initialize():
        return
    
    try:
        # Create action server
        async def handle_goal(goal):
            return True  # Accept goal
        
        async def handle_cancel(goal_handle):
            return True  # Allow cancellation
        
        async def execute_goal(goal_handle):
            for i in range(100):
                if goal_handle.is_canceled:
                    return Process.Result(status="canceled")
                
                # Publish feedback
                await goal_handle.publish_feedback(
                    Process.Feedback(progress=i, message=f"Step {i}/100")
                )
                await asyncio.sleep(0.1)
            
            return Process.Result(
                status="complete",
                data=goal_handle.goal.input * 2
            )
        
        action_server = await provider.create_action_server(
            name="process",
            action_type=Process,
            handle_goal_request=handle_goal,
            handle_cancel_request=handle_cancel,
            execution_callback=execute_goal
        )
        
        # Create action client
        async def handle_feedback(feedback):
            print(f"Progress: {feedback.progress}%")
        
        async def handle_result(result):
            print(f"Result: {result.status}, Data: {result.data}")
        
        action_client = await provider.create_action_client(
            name="process",
            action_type=Process,
            feedback_callback=handle_feedback,
            goal_callback=handle_result
        )
        
        # Send goal
        goal = Process.Goal(input=42)
        goal_id = await action_client.send_goal(goal)
        
        # Wait for completion
        await asyncio.sleep(15)
        
    finally:
        await provider.shutdown()

if __name__ == "__main__":
    asyncio.run(main())
```

---

## Contributing

When adding new transport protocols:

1. **Implement Provider**: Extend `AbstractProtocolProvider` with all 6 create methods
2. **Create vyra_models**: Implement VyraPublisher, VyraSubscriber, VyraServer, VyraClient, VyraActionServer, VyraActionClient
3. **Add Availability Check**: Implement `check_availability()` with graceful failure
4. **Write Tests**: Unit tests (no dependencies) + Integration tests (with protocol)
5. **Update Docs**: Add protocol-specific README.md with examples
6. **Register Provider**: Add to provider registry for auto-discovery

### Code Structure

```
t_myprotocol/
├── __init__.py                 # Exports + MYPROTOCOL_AVAILABLE flag
├── provider.py                 # MyProtocolProvider(AbstractProtocolProvider)
├── vyra_models/
│   ├── __init__.py
│   ├── publisher.py           # MyProtocolPublisherImpl(VyraPublisher)
│   ├── subscriber.py          # MyProtocolSubscriberImpl(VyraSubscriber)
│   ├── server.py              # MyProtocolServerImpl(VyraServer)
│   ├── client.py              # MyProtocolClientImpl(VyraClient)
│   ├── action_server.py       # MyProtocolActionServerImpl(VyraActionServer)
│   └── action_client.py       # MyProtocolActionClientImpl(VyraActionClient)
├── communication/             # Protocol-specific helpers
│   └── ...
└── README.md                  # Protocol documentation
```

---

## License

Part of the VYRA framework - see main repository LICENSE file.

---

## See Also

- [Core Types Documentation](../core/README.md)
- [Provider Pattern Documentation](../providers/README.md)
- [ROS2 Transport Details](t_ros2/README.md)
- [Zenoh Transport Details](t_zenoh/README.md)
- [Redis Transport Details](../../docs/com/transport/redis/README.md)
- [UDS Transport Details](../../docs/com/transport/uds/README.md)
