# Zenoh Transport Layer

Eclipse Zenoh-based transport implementation for VYRA framework.

## Overview

Zenoh provides a unified, scalable communication protocol combining pub/sub and query/reply patterns with zero-copy capabilities.

**Key Features:**
- ✅ Efficient Pub/Sub with minimal overhead
- ✅ Request-Response via Query/Reply
- ✅ Router-based scalability
- ✅ Zero-copy shared memory transport
- ✅ Multi-protocol (TCP, UDP, SHM)

## Installation

```bash
# Install Zenoh Python library
pip install eclipse-zenoh

# Deploy Zenoh Router (Docker)
docker run -d --name zenoh-router \
  -p 7447:7447 \
  eclipse/zenoh:latest

# Verify
docker logs zenoh-router
```

## Quick Start

```python
from vyra_base.com.transport.t_zenoh import ZenohProvider, ZENOH_AVAILABLE
from vyra_base.com.core.topic_builder import TopicBuilder

# Check availability
if not ZENOH_AVAILABLE:
    raise ImportError("Install eclipse-zenoh")

# Initialize provider
topic_builder = TopicBuilder(module_name="my_module", module_id="abc123")
provider = ZenohProvider(module_name="my_module", module_id="abc123")

await provider.initialize(config={
    "mode": "client",
    "connect": ["tcp/zenoh-router:7447"],
    "format": "json"
})
```

## API Reference

### 1. Publisher/Subscriber

```python
from example_msgs.msg import Temperature

# Publisher
publisher = await provider.create_publisher(
    name="temperature",
    topic_builder=topic_builder,
    message_type=Temperature
)
await publisher.publish(Temperature(value=23.5))

# Subscriber
async def on_temperature(msg):
    print(f"Received: {msg.value}°C")

subscriber = await provider.create_subscriber(
    name="temperature",
    topic_builder=topic_builder,
    subscriber_callback=on_temperature,
    message_type=Temperature
)
```

### 2. Server/Client

```python
from example_srvs.srv import Calculate

# Server
async def calculate(request, response=None):
    if not response:
        response = Calculate.Response()
    response.result = request.a + request.b
    return response

server = await provider.create_server(
    name="calculate",
    topic_builder=topic_builder,
    response_callback=calculate,
    service_type=Calculate
)

# Client
client = await provider.create_client(
    name="calculate",
    topic_builder=topic_builder,
    service_type=Calculate
)

request = Calculate.Request(a=10, b=32)
response = await client.call(request, timeout=5.0)
print(response.result)  # 42
```

### 3. Action Server/Client

```python
from example_actions.action import Process

# Action Server
async def handle_goal(goal):
    return True  # Accept

async def handle_cancel(goal_handle):
    return True  # Allow

async def execute(goal_handle):
    for i in range(100):
        # Publish feedback
        await action_server.publish_feedback(
            goal_handle['goal_id'],
            Process.Feedback(progress=i)
        )
        await asyncio.sleep(0.1)
    return Process.Result(status="done")

action_server = await provider.create_action_server(
    name="process",
    topic_builder=topic_builder,
    handle_goal_request=handle_goal,
    handle_cancel_request=handle_cancel,
    execution_callback=execute,
    action_type=Process
)

# Action Client
async def on_feedback(fb):
    print(f"Progress: {fb.progress}%")

async def on_result(result):
    print(f"Done: {result.status}")

action_client = await provider.create_action_client(
    name="process",
    topic_builder=topic_builder,
    action_type=Process,
    feedback_callback=on_feedback,
    goal_response_callback=on_result
)

goal_id = await action_client.send_goal(Process.Goal(input=42))
```

## Architecture

```
┌──────────────┐
│ VYRA Module  │
│  (Provider)  │
└──────┬───────┘
       │ eclipse-zenoh
       ▼
┌──────────────┐
│   Session    │
│ (TCP/UDP/SHM)│
└──────┬───────┘
       │ Network
       ▼
┌──────────────┐
│ Zenoh Router │
│   (Docker)   │
└──────┬───────┘
       │ Network
       ▼
┌──────────────┐
│ Other Modules│
└──────────────┘
```

**Internal Components:**

```
transport/t_zenoh/
├── provider.py          # ZenohProvider (6 create_* methods)
├── session.py           # Session lifecycle management
├── communication/       # Low-level Zenoh primitives
│   ├── publisher.py     # ZenohPublisher wrapper
│   ├── subscriber.py    # ZenohSubscriber wrapper
│   ├── queryable.py     # Service server
│   ├── query_client.py  # Service client
│   └── serializer.py    # JSON/MessagePack/Protobuf
└── vyra_models/         # 6 VYRA interfaces
    ├── publisher.py     # VyraPublisherImpl
    ├── subscriber.py    # VyraSubscriberImpl
    ├── server.py        # VyraServerImpl
    ├── client.py        # VyraClientImpl
    ├── action_server.py # VyraActionServerImpl
    └── action_client.py # VyraActionClientImpl
```

## Protocol Patterns

| Interface | Zenoh Primitives | Pattern |
|-----------|------------------|---------|
| Publisher | `declare_publisher()` + `put()` | Fire-and-forget |
| Subscriber | `declare_subscriber()` | Callback-based |
| Server | `declare_queryable()` + `reply()` | Request-response |
| Client | `get()` | Synchronous query |
| ActionServer | Queryable + Publisher | Hybrid (goal/cancel via query, feedback/result via pub) |
| ActionClient | Query + Subscriber | Hybrid (send_goal via query, listen via sub) |

## Configuration

```python
config = {
    "mode": "client",               # "client" | "peer" | "router"
    "connect": [                    # Router endpoints
        "tcp/zenoh-router:7447"
    ],
    "listen": [],                   # Listen endpoints (optional)
    "format": "json",               # "json" | "msgpack" | "protobuf" | "raw"
    "timeout_ms": 5000             # Query timeout
}

await provider.initialize(config)
```

## Performance

| Feature | Zenoh | ROS2 | Redis | UDS |
|---------|-------|------|-------|-----|
| Zero-Copy | ✅ Native | DDS-specific | ❌ | ✅ |
| Shared Memory | ✅ Auto | DDS-specific | ❌ | ✅ |
| Scalability | High | Medium | High | Low |
| Network | TCP/UDP/SHM | UDP (DDS) | TCP | Unix Socket |
| Discovery | Router | DDS | Pub/Sub | N/A |

**Best For:**
- Distributed multi-module communication
- High-throughput data streams
- Scalable microservices architecture
- Cross-network module communication

**Not Best For:**
- Single-machine local IPC (use UDS)
- Simple key-value caching (use Redis)
- ROS2 ecosystem integration (use ROS2 transport)

## Error Handling

```python
from vyra_base.com.core.exceptions import (
    ProtocolUnavailableError,
    ProviderError,
    InterfaceError
)

try:
    provider = ZenohProvider(module_name="test", module_id="123")
    
    if not await provider.check_availability():
        raise ProtocolUnavailableError("Zenoh not installed")
    
    await provider.initialize(config)
    publisher = await provider.create_publisher(...)
    
except ProtocolUnavailableError:
    logger.error("Zenoh library missing: pip install eclipse-zenoh")
except ProviderError as e:
    logger.error(f"Provider init failed: {e}")
except InterfaceError as e:
    logger.error(f"Interface creation failed: {e}")
finally:
    await provider.shutdown()
```

## Testing

```bash
# Unit tests (no router required)
pytest tests/transport/test_zenoh.py -v -m unit

# Integration tests (requires router)
docker run -d --name zenoh-router -p 7447:7447 eclipse/zenoh:latest
pytest tests/transport/test_zenoh.py -v -m integration
docker stop zenoh-router && docker rm zenoh-router
```

## Troubleshooting

### Connection Failed

**Symptom**: `Failed to initialize Zenoh provider`

**Solution**:
```bash
# 1. Check router is running
docker ps | grep zenoh-router

# 2. Test connectivity
telnet zenoh-router 7447

# 3. Check logs
docker logs zenoh-router -f

# 4. Verify Zenoh installation
python -c "import zenoh; print(zenoh.__version__)"
```

### Import Error

**Symptom**: `ModuleNotFoundError: No module named 'zenoh'`

**Solution**:
```bash
pip install eclipse-zenoh
```

### Timeout Issues

**Solution**: Increase timeout in config:
```python
config = {
    "mode": "client",
    "connect": ["tcp/zenoh-router:7447"],
    "timeout_ms": 30000  # 30 seconds
}
```

### Router Not Reachable

**Solution**: Check Docker networking:
```bash
# Verify network
docker network inspect vyra_network

# Ensure router on correct network
docker run -d --name zenoh-router \
  --network vyra_network \
  -p 7447:7447 \
  eclipse/zenoh:latest
```

## Best Practices

1. ✅ **Reuse Session**: Single provider session for all interfaces
2. ✅ **Use TopicBuilder**: Consistent topic naming across modules
3. ✅ **Check Availability**: Always verify `ZENOH_AVAILABLE` before use
4. ✅ **Proper Cleanup**: Call `await provider.shutdown()` on exit
5. ✅ **Router Placement**: Deploy on manager node in Docker Swarm
6. ✅ **Choose Format**: MessagePack for performance, JSON for debugging
7. ✅ **Set Timeouts**: Configure based on network latency

## Deployment (Docker Swarm)

```yaml
# docker-compose.yml
services:
  zenoh-router:
    image: eclipse/zenoh:latest
    ports:
      - "7447:7447"
    networks:
      - vyra_network
    deploy:
      replicas: 1
      placement:
        constraints:
          - node.role == manager
      restart_policy:
        condition: on-failure

networks:
  vyra_network:
    driver: overlay
```

## Dependencies

- **eclipse-zenoh**: `^1.0.0` (Python bindings)
- **Zenoh Router**: `eclipse/zenoh:latest` (Docker image)

## See Also

- [Transport Layer Overview](../README.md)
- [ROS2 Transport](../t_ros2/README.md)
- [Redis Transport](../../../docs/com/transport/redis/README.md)
- [UDS Transport](../../../docs/com/transport/uds/README.md)
- [Zenoh Documentation](https://zenoh.io/docs/)
- [Eclipse Zenoh GitHub](https://github.com/eclipse-zenoh/zenoh)
