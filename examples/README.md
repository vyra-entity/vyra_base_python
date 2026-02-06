# VYRA Base Python Examples

This directory contains practical examples demonstrating VYRA's communication capabilities.

## Examples

### 1. External Communication
**File**: `external_communication.py`

Demonstrates integration with external (non-VYRA) systems:
- REST API communication
- MQTT broker integration  
- Bidirectional bridge between VYRA and external systems

```bash
pip install aiohttp paho-mqtt
python examples/external_communication.py
```

**Key Concepts**:
- External REST API integration
- MQTT pub/sub with external brokers
- Bridging external data to VYRA network
- Forwarding VYRA commands to external systems

---

### 2. Zenoh with Protobuf
**File**: `zenoh_protobuf_example.py`

Demonstrates using Zenoh transport with Protocol Buffers:
- Speaker (Pub/Sub) with Protobuf messages
- Callable (Request/Response) with Protobuf
- Job (Long-running task) with Protobuf feedback
- Binary Protobuf serialization

```bash
pip install eclipse-zenoh protobuf
python examples/zenoh_protobuf_example.py
```

**Key Concepts**:
- Type-safe communication with Protobuf
- Efficient binary serialization
- Cross-language message compatibility
- Schema evolution and versioning

---

### 3. Speaker Example (Publisher)
**File**: `example_speaker.py`

Demonstrates how to publish messages on ROS2 topics using VyraSpeaker.

```bash
python examples/example_speaker.py
```

**Key Concepts**:
- Creating a VyraSpeaker (Publisher)
- Publishing messages with `shout()`
- QoS configuration

---

### 4. Callable Example (Service Server)
**File**: `example_callable.py`

Demonstrates how to provide ROS2 services using VyraCallable.

```bash
# Run the service server
python examples/example_callable.py

# In another terminal, call the service
ros2 service call /callable_example_node/callable/add_service example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"
```

**Key Concepts**:
- Creating a VyraCallable (Service Server)
- Service callback functions
- Using `@remote_callable` decorator

---

### 5. Job Example (Action Server & Client)
**File**: `example_job.py`

Demonstrates long-running operations with feedback using VyraJob and VyraJobRunner.

```bash
# Terminal 1: Run action server
python examples/example_job.py server

# Terminal 2: Run action client
python examples/example_job.py client

# Or test with ROS2 CLI:
ros2 action send_goal /job_server_node/job/fibonacci_job example_interfaces/action/Fibonacci "{order: 10}"
```

**Key Concepts**:
- Creating a VyraJob (Action Server)
- Creating a VyraJobRunner (Action Client)
- Handling feedback during execution
- Processing final results
- Async goal execution

---

### 6. Listener Example (Subscriber)
**File**: `example_listener.py`

Subscribes to a ROS2 topic using VyraSpeaker with `is_publisher=False`. Pairs with `example_speaker.py`.

```bash
# Terminal 1: Start listener
python examples/example_listener.py

# Terminal 2: Publish (speaker or CLI)
python examples/example_speaker.py
# or: ros2 topic pub /vyra/example_topic std_msgs/msg/String "{data: 'Hello'}"
```

**Key Concepts**:
- VyraSpeaker as subscriber (`is_publisher=False`)
- `listen(callback)` for incoming messages
- Same topic name as publisher for pub/sub

---

### 7. Unified State Machine
**File**: `unified_state_machine_example.py`

Shows the 3-layer state machine: Lifecycle, Operational, and Health.

```bash
python examples/unified_state_machine_example.py
```

**Key Concepts**:
- `UnifiedStateMachine`: single API for all layers
- Lifecycle: start, complete_initialization, shutdown
- Operational: set_ready, start_task, pause, resume, stop, reset
- Health: report_warning, clear_warning, report_fault, recover

---

### 8. Redis Pub/Sub
**File**: `redis_pubsub_example.py`

Redis transport: publish and subscribe with `RedisProvider` and `create_speaker`. Requires Redis (e.g. `docker run -p 6379:6379 redis`).

```bash
# Terminal 1: Subscriber
python examples/redis_pubsub_example.py subscriber

# Terminal 2: Publisher
python examples/redis_pubsub_example.py publisher
```

**Key Concepts**:
- RedisProvider, create_speaker (channel)
- shout() to publish, listen(callback) to subscribe

---

### 9. Storage (Database)
**File**: `storage_example.py`

DbAccess (SQLite) and DbManipulator with a custom table (`tb_` prefix required).

```bash
python examples/storage_example.py
```

**Key Concepts**:
- DbAccess with db_config (SQLite)
- Custom table inheriting from Base (`tb_sensor_log`); table names must use `tb_` prefix
- DbManipulator: add(), get_all(), get_by_id()

Run from project root so that `vyra_base` and its logger are initialized correctly: `python examples/storage_example.py` or `pip install -e .` then run from any directory.

---

### 10. Communication Demo (Multi-Transport)
**File**: `communication_demo.py`

Runs several transport demos: Redis, UDS, Shared Memory, Modbus, External Registry. Good overview of the communication layer.

```bash
python examples/communication_demo.py
```

### 11. Operational State Machine (Lifecycle Methods)
**File**: `operational_state_machine_example.py`

OperationalStateMachine with lifecycle methods (initialize, start, pause, resume, stop, reset) and state validation.

```bash
python examples/operational_state_machine_example.py
```

---

## Prerequisites

Make sure you have:
1. ROS2 Kilted installed and sourced
2. vyra_base_python installed
3. example_interfaces package available

```bash
# Source ROS2
source /opt/ros/kilted/setup.bash

# Install vyra_base if needed
pip install -e .
```

## Understanding the Communication Patterns

| Component | ROS2 Equivalent | Use Case |
|-----------|----------------|----------|
| **VyraSpeaker** (is_publisher=True) | Publisher | One-way broadcast of data |
| **VyraSpeaker** (is_publisher=False) | Subscriber | Receive messages on a topic |
| **VyraCallable** | Service Server | Quick request/response |
| **VyraJob** | Action Server | Long-running tasks with feedback |

## Next Steps

- Read the full documentation: [docs/com.rst](../docs/com.rst)
- Explore the API reference: [docs/vyra_base.com.datalayer.rst](../docs/vyra_base.com.datalayer.rst)
- Check out module examples in the VYRA modules repository
