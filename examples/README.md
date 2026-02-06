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
| **VyraSpeaker** | Publisher | One-way broadcast of data |
| **VyraCallable** | Service Server | Quick request/response |
| **VyraJob** | Action Server | Long-running tasks with feedback |

## Next Steps

- Read the full documentation: [docs/com.rst](../docs/com.rst)
- Explore the API reference: [docs/vyra_base.com.datalayer.rst](../docs/vyra_base.com.datalayer.rst)
- Check out module examples in the VYRA modules repository
