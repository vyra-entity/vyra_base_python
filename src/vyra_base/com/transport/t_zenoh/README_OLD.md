# Zenoh Transport Layer

Eclipse Zenoh-based transport implementation for VYRA framework.

## Overview

Zenoh provides a unified, scalable communication protocol combining pub/sub and query/reply patterns with zero-copy capabilities. This transport layer offers:

- **Efficient Pub/Sub**: Topic-based messaging with minimal overhead
- **Query/Reply**: Request-response patterns for service calls  
- **Router Architecture**: Scalable via centralized Zenoh router
- **Zero-Copy**: Shared memory transport when available
- **Protocol Agnostic**: TCP, UDP, shared memory, or mixed

## Architecture

```
transport/t_zenoh/
├── __init__.py              # Module exports and availability check
├── session.py               # Zenoh session management
├── provider.py              # ZenohProvider (implements AbstractProtocolProvider)
├── communication/           # Core Zenoh primitives
│   ├── publisher.py         # Publishing functionality
│   ├── subscriber.py        # Subscription functionality
│   ├── queryable.py         # Service server (query handler)
│   ├── query_client.py      # Service client (query sender)
│   └── serializer.py        # Data serialization utilities
└── vyra_models/             # VYRA abstractions (6 interfaces)
    ├── publisher.py         # VyraPublisherImpl (VyraPublisher)
    ├── subscriber.py        # VyraSubscriberImpl (VyraSubscriber)
    ├── server.py            # VyraServerImpl (VyraServer)
    ├── client.py            # VyraClientImpl (VyraClient)
    ├── action_server.py     # VyraActionServerImpl (VyraActionServer)
    └── action_client.py     # VyraActionClientImpl (VyraActionClient)
```

## Installation

### Install Zenoh Python Library

```bash
pip install eclipse-zenoh
```

### Deploy Zenoh Router (Docker)

```bash
# Start Zenoh router
docker run -d --name zenoh-router \
  -p 7447:7447 \
  eclipse/zenoh:latest

# Verify router is running
docker logs zenoh-router
```

### Deploy Zenoh Router (Docker Swarm)

```yaml
# In docker-compose.yml
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
```

## Usage

### Provider Initialization

```python
from vyra_base.com.transport.t_zenoh import ZenohProvider, ZENOH_AVAILABLE
from vyra_base.com.core.topic_builder import TopicBuilder

if not ZENOH_AVAILABLE:
    print("Zenoh not available")
    exit(1)

# Create TopicBuilder
topic_builder = TopicBuilder(module_name="my_module", module_id="abc123")

# Create and initialize provider
provider = ZenohProvider(
    module_name="my_module",
    module_id="abc123"
)

await provider.initialize(config={
    "mode": "client",  # or "peer"
    "connect": ["tcp/zenoh-router:7447"],
    "format": "json"  # Serialization format
})
```
    "format": "json"  # or "msgpack"
})
```

### Publisher/Subscriber Pattern

```python
from example_msgs.msg import Temperature  # Your ROS2 message type

# Create Publisher
publisher = await provider.create_publisher(
    name="temperature",
    topic_builder=topic_builder,
    message_type=Temperature
)

# Publish messages
await publisher.publish(Temperature(value=23.5, unit="celsius"))

# Create Subscriber
async def handle_temperature(msg):
    print(f"Received: {msg.value}°C")

subscriber = await provider.create_subscriber(
    name="temperature",
    topic_builder=topic_builder,
    subscriber_callback=handle_temperature,
    message_type=Temperature
)
```

### Server/Client Pattern (Query/Reply)

```python
from example_srvs.srv import Calculate  # Your ROS2 service type

# Create Server
async def calculate_handler(request, response=None):
    if response is None:
        response = Calculate.Response()
    response.result = request.a + request.b
    return response

server = await provider.create_server(
    name="calculate",
    topic_builder=topic_builder,
    response_callback=calculate_handler,
    service_type=Calculate
)

# Create Client
client = await provider.create_client(
    name="calculate",
    topic_builder=topic_builder,
    service_type=Calculate
)

# Call service
request = Calculate.Request(a=10, b=32)
response = await client.call(request, timeout=5.0)
print(f"Result: {response.result}")  # 42
```

### Action Server/Client Pattern

```python
from example_actions.action import Process  # Your ROS2 action type

# Create Action Server
async def handle_goal(goal):
    return True  # Accept goal

async def handle_cancel(goal_handle):
    return True  # Allow cancellation

async def execute_goal(goal_handle):
    for i in range(100):
        if goal_handle['status'] == GoalStatus.CANCELED:
            return Process.Result(status="canceled")
        
        # Publish feedback
        await action_server.publish_feedback(
            goal_handle['goal_id'],
            Process.Feedback(progress=i)
        )
        await asyncio.sleep(0.1)
    
    return Process.Result(status="complete", data=goal_handle['goal'].input * 2)

action_server = await provider.create_action_server(
    name="process",
    topic_builder=topic_builder,
    handle_goal_request=handle_goal,
    handle_cancel_request=handle_cancel,
    execution_callback=execute_goal,
    action_type=Process
)

# Create Action Client
async def handle_feedback(feedback):
    print(f"Progress: {feedback.progress}%")

async def handle_result(result):
    print(f"Result: {result.status}")

action_client = await provider.create_action_client(
    name="process",
    topic_builder=topic_builder,
    action_type=Process,
    feedback_callback=handle_feedback,
    goal_response_callback=handle_result
)

# Send goal
goal = Process.Goal(input=42)
goal_id = await action_client.send_goal(goal)
```
```

### Job (Long-Running Task with Feedback)

Jobs implement the **Greeting → Talking → Finishing** pattern for long-running tasks with progress updates.

#### Server Side

```python
async def process_large_dataset(params, publish_feedback):
    """
    Job callback with three phases:
    
    1. Greeting: Receive job parameters
    2. Talking: Send progress feedback during execution
    3. Finishing: Return final result
    
    Args:
        params: Job parameters (greeting phase)
        publish_feedback: Async callback for progress updates (talking phase)
    
    Returns:
        Final result dictionary (finishing phase)
    """
    dataset_size = params["dataset_size"]
    batch_size = params.get("batch_size", 100)
    
    processed_items = 0
    total_batches = dataset_size // batch_size
    
    # Talking phase - send progress updates
    for batch_num in range(total_batches):
        # Process batch
        await process_batch(batch_num, batch_size)
        processed_items += batch_size
        
        # Publish feedback during execution
        await publish_feedback({
            "progress_percent": (processed_items / dataset_size) * 100,
            "processed_items": processed_items,
            "current_batch": batch_num + 1,
            "total_batches": total_batches,
            "status": "processing"
        })
        
        await asyncio.sleep(0.1)  # Simulate work
    
    # Finishing phase - return final result
    return {
        "status": "completed",
        "total_processed": processed_items,
        "execution_time": 42.5,
        "errors": 0
    }

# Create job server
job_server = await provider.create_job(
    "/data/process_dataset",
    result_callback=process_large_dataset,
    is_server=True
)
```

#### Client Side

```python
# Create job client
job_client = await provider.create_job(
    "/data/process_dataset",
    is_server=False
)

# Greeting phase - define job parameters
job_params = {
    "dataset_size": 10000,
    "batch_size": 100
}

# Talking phase - handle progress feedback
async def handle_feedback(feedback_data):
    """Called repeatedly during job execution"""
    print(f"Progress: {feedback_data['progress_percent']:.1f}%")
    print(f"Batch {feedback_data['current_batch']}/{feedback_data['total_batches']}")
    print(f"Processed: {feedback_data['processed_items']} items")
    print("---")

# Finishing phase - handle final result
async def handle_result(result_data):
    """Called once when job completes"""
    print(f"✅ Job {result_data['status']}")
    print(f"Total processed: {result_data['total_processed']}")
    print(f"Execution time: {result_data['execution_time']}s")
    if result_data['errors'] > 0:
        print(f"⚠️  Errors: {result_data['errors']}")

# Subscribe to feedback (talking) and result (finishing)
await job_client.listen_feedback(handle_feedback)
await job_client.listen_result(handle_result)

# Start job (greeting)
await job_client.start(job_params)

# Optional: Cancel job if needed
# await job_client.cancel()
```

#### Job Lifecycle

```
Client                           Server
  |                                |
  |  1. GREETING                   |
  |  start(params) --------------> |  Receives params
  |                                |  Starts execution
  |                                |
  |  2. TALKING                    |
  |                                |  publish_feedback()
  | <-------------- feedback(1)    |  (multiple times)
  | <-------------- feedback(2)    |
  | <-------------- feedback(3)    |
  |       ...                      |
  |                                |
  |  3. FINISHING                  |
  | <-------------- result         |  Returns final result
  |                                |  Job complete
  ✓
```

#### Job States

Jobs progress through the following states:

- `IDLE`: Job created but not started
- `RUNNING`: Job executing (talking phase)
- `SUCCEEDED`: Job completed successfully (finishing phase)
- `CANCELLED`: Job cancelled by client
- `FAILED`: Job execution failed with error

#### Advanced Usage

**Error Handling in Jobs**:

```python
async def robust_job(params, publish_feedback):
    try:
        for i in range(100):
            if some_condition:
                # Can throw exceptions - will be caught and reported
                raise ValueError("Invalid data encountered")
            
            await publish_feedback({"progress": i})
        
        return {"status": "success"}
    
    except Exception as e:
        # Server automatically handles exceptions
        # Client receives error in result
        raise

# Client receives error in result callback
async def handle_result(result):
    if "error" in result:
        print(f"Job failed: {result['error']}")
    else:
        print(f"Job succeeded: {result}")
```

**Job Cancellation**:

```python
# Client can cancel running job
await job_client.cancel()

# Server automatically stops execution
# Client receives cancelled status in result
```

**Multiple Concurrent Jobs**:

```python
# Each job instance is independent
job1 = await provider.create_job("/job1", callback1, is_server=True)
job2 = await provider.create_job("/job2", callback2, is_server=True)

# Clients can run jobs concurrently
client1 = await provider.create_job("/job1", is_server=False)
client2 = await provider.create_job("/job2", is_server=False)

await asyncio.gather(
    client1.start(params1),
    client2.start(params2)
)
```

#### Comparison with ROS2 Actions

Zenoh Jobs implement the same Goal/Feedback/Result pattern as ROS2 Actions:

| Phase | ROS2 Action | Zenoh Job |
|-------|-------------|-----------|
| Start | `send_goal()` | `start()` (Greeting) |
| Progress | Feedback callback | `listen_feedback()` (Talking) |
| Complete | Result callback | `listen_result()` (Finishing) |
| Cancel | `cancel_goal()` | `cancel()` |

**Migration from ROS2 Actions**:

```python
# ROS2 Action
action_client.send_goal_async(goal)
action_client.feedback_callback = handle_feedback
action_client.result_callback = handle_result

# Zenoh Job (equivalent)
await job_client.listen_feedback(handle_feedback)
await job_client.listen_result(handle_result)
await job_client.start(goal)
```

## Configuration

### Session Modes

- **client**: Connect to Zenoh router (recommended for modules)
- **peer**: Peer-to-peer mode (direct communication)
- **router**: Router mode (typically run as Docker service)

### Docker Compose

```yaml
services:
  zenoh-router:
    image: eclipse/zenoh:latest
    ports:
      - "7447:7447"
    command: >
      zenohd
      --config /etc/zenoh/zenoh.conf
    volumes:
      - ./config/zenoh.conf:/etc/zenoh/zenoh.conf

  my-module:
    image: my-module:latest
    environment:
      - ZENOH_MODE=client
      - ZENOH_CONNECT=tcp/zenoh-router:7447
    depends_on:
      - zenoh-router
```

### Entity Configuration

In module's `.env` or config:

```bash
# Zenoh Configuration
ZENOH_MODE=client
ZENOH_ROUTER_HOST=zenoh-router
ZENOH_ROUTER_PORT=7447
```

In Python code:

```python
zenoh_config = {
    "mode": os.getenv("ZENOH_MODE", "client"),
    "connect": [f"tcp/{os.getenv('ZENOH_ROUTER_HOST', 'zenoh-router')}:{os.getenv('ZENOH_ROUTER_PORT', '7447')}"]
}

await entity._register_transport_provider([ProtocolType.ZENOH])
```

## Serialization

Zenoh transport supports multiple serialization formats:

- **JSON**: Default, human-readable
- **MessagePack**: Efficient binary format
- **Protobuf**: Type-safe, language-agnostic
- **Raw**: Binary data as-is

```python
from vyra_base.com.transport.t_zenoh.communication.serializer import SerializationFormat

# Use MessagePack
speaker = await provider.create_speaker(
    "/data",
    format=SerializationFormat.MSGPACK
)

# Use Protobuf (with converter)
from vyra_base.com.converters import ConverterFactory

proto_converter = ConverterFactory.get_converter("protobuf")
binary_data = proto_converter.serialize(proto_message)

speaker = await provider.create_speaker(
    "/data",
    format=SerializationFormat.RAW
)
await speaker.shout(binary_data)
```

## Key Expressions

Zenoh uses key expressions (similar to topics) for routing:

```
/sensor/temperature      # Specific topic
/sensor/*                # Wildcard (all sensors)
/sensor/**               # Recursive wildcard (all sub-paths)
```

Examples:

```python
# Publish to specific sensor
await speaker1.shout(data)  # /sensor/temperature

# Subscribe to all sensors
subscriber = await provider.create_speaker("/sensor/*", is_publisher=False)

# Service for specific module
callable = await provider.create_callable("/v2_modulemanager/get_status")
```

## Performance Considerations

1. **Shared Memory**: Automatic when available (same host)
2. **Zero-Copy**: Used for large payloads
3. **QoS**: Reliable by default, can be configured
4. **Batching**: Combine multiple small messages
5. **Keep-Alive**: Router handles connection persistence

## Comparison with ROS2

| Feature | ROS2 | Zenoh |
|---------|------|-------|
| Discovery | DDS | Router-based |
| Zero-Copy | DDS-specific | Native |
| Transport | UDP (DDS) | TCP/UDP/SHM |
| QoS | Complex | Simplified |
| Scalability | Limited | High |
| Setup | Complex | Simple |

## Error Handling

```python
from vyra_base.com.core.exceptions import ProtocolUnavailableError, ProviderError

try:
    provider = ZenohProvider()
    await provider.initialize(config)
except ProtocolUnavailableError:
    logger.error("Zenoh not available")
except ProviderError as e:
    logger.error(f"Failed to initialize: {e}")
```

## Testing

```python
# Unit tests
pytest tests/transport/test_zenoh.py -v

# Integration tests (requires Zenoh router)
pytest tests/integration/test_zenoh_integration.py -m integration
```

## Dependencies

- **eclipse-zenoh**: `^1.0.0` (Python bindings)
- **Zenoh Router**: Docker image `eclipse/zenoh:latest`

Install:

```bash
pip install eclipse-zenoh
```

## Troubleshooting

### Connection Issues

```python
# Check if Zenoh is reachable
import zenoh

config = zenoh.Config()
config.insert_json5("connect/endpoints", '["tcp/zenoh-router:7447"]')

try:
    session = zenoh.open(config)
    print(f"Connected! Session ID: {session.info().zid()}")
    session.close()
except Exception as e:
    print(f"Connection failed: {e}")
```

### Router Not Found

```bash
# Check router status
docker ps | grep zenoh-router

# View router logs
docker logs zenoh-router -f

# Test connectivity
telnet zenoh-router 7447
```

### Session Timeout

Increase timeout in config:

```python
config = {
    "mode": "client",
    "connect": ["tcp/zenoh-router:7447"],
    "timeout_ms": 10000  # 10 seconds
}
```

## See Also

- [Transport Layer Overview](../README.md)
- [ROS2 Transport](../ros2/README.md)
- [Communication Converters](../../converters/README.md)
- [Zenoh Documentation](https://zenoh.io/docs/)
- [Eclipse Zenoh GitHub](https://github.com/eclipse-zenoh/zenoh)
