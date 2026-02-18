# VYRA Communication Module (com)

Multi-protocol communication system for distributed VYRA modules.

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Your Module / Component                  │
│        @remote_service  @remote_publisher  @remote_subscriber│
│        @remote_actionServer    InterfaceFactory             │
└────────────────────────┬────────────────────────────────────┘
                         │
┌────────────────────────▼────────────────────────────────────┐
│                   Core Layer                                │
│    Types · Exceptions · Factory · Decorators · Blueprints  │
└────┬───────────────┬────────────────┬───────────────────────┘
     │               │                │
┌────▼────┐   ┌──────▼──────┐  ┌──────▼──────────┐
│Transport│   │  External   │  │   Industrial    │
├─────────┤   ├─────────────┤  ├─────────────────┤
│ ROS2    │   │ gRPC        │  │ Modbus          │
│ Zenoh   │   │ MQTT        │  │ OPC UA          │
│ Redis   │   │ REST        │  └─────────────────┘
│ UDS     │   │ WebSocket   │
└─────────┘   │ Shared Mem  │
              └─────────────┘
```

**Protocol fallback order (automatic):** `Zenoh → ROS2 → Redis → UDS`

---

## Layers

### Transport Layer (`transport/`)

Low-latency in-process / DDS-based protocols. All four are fully implemented.

| Module | Protocol | Use Case |
|---|---|---|
| `t_ros2/` | ROS2 / DDS | Distributed robot systems; requires `rclpy` |
| `t_zenoh/` | Zenoh | High-performance pub/sub (**default**); requires `eclipse-zenoh` |
| `t_redis/` | Redis | Pub/Sub + key-value store; requires `redis` |
| `t_uds/` | Unix Domain Sockets | Local IPC; no extra dependencies |

### External Layer (`external/`)

Out-of-process / cross-service protocols. All are optional imports.

| Module | Protocol | Use Case |
|---|---|---|
| `grpc/` | gRPC over UDS | High-performance RPC; requires `grpcio` |
| `mqtt/` | MQTT | IoT / constrained devices; requires `paho-mqtt` |
| `rest/` | HTTP REST | Web services; requires `aiohttp` |
| `websocket/` | WebSocket | Real-time browser clients; requires `websockets` |
| `shared_memory/` | POSIX Shared Memory | Zero-copy local IPC; Linux only |

### Industrial Layer (`industrial/`)

| Module | Protocol | Use Case |
|---|---|---|
| `modbus/` | Modbus TCP/RTU | PLC / SCADA integration; requires `pymodbus` |
| `opcua/` | OPC UA | MES northbound; requires `asyncua` |

---

## Quick Start

### Expose a Service (@remote_service)

```python
from vyra_base.com import remote_service, bind_decorated_callbacks, InterfaceFactory, ProtocolType

class CalculatorComponent:
    @remote_service(name="add", protocols=[ProtocolType.ZENOH], namespace="calculator")
    async def add(self, request, response=None):
        return {"result": request["x"] + request["y"]}

# Phase 2: bind callbacks and create interfaces
component = CalculatorComponent()
bind_decorated_callbacks(component, namespace="calculator")
```

### Publish Messages (@remote_publisher)

```python
from vyra_base.com import remote_publisher, ProtocolType

class SensorComponent:
    @remote_publisher(name="temperature", protocols=[ProtocolType.ZENOH])
    async def publish_temperature(self, value: float):
        return {"value": value, "unit": "°C"}
```

### Subscribe to Messages (@remote_subscriber)

```python
from vyra_base.com import remote_subscriber, ProtocolType

class DashboardComponent:
    @remote_subscriber(name="temperature", protocols=[ProtocolType.ZENOH])
    async def on_temperature(self, message):
        print(f"Received: {message['value']} {message['unit']}")
```

### Action Server (@remote_actionServer)

```python
from vyra_base.com import remote_actionServer, IActionHandler, IGoalHandle, ProtocolType

class ProcessingComponent(IActionHandler):
    @remote_actionServer.on_goal(name="process", protocols=[ProtocolType.ZENOH])
    async def on_goal(self, goal_request) -> bool:
        return goal_request.get("dataset") is not None  # accept/reject

    @remote_actionServer.execute(name="process")
    async def execute(self, goal_handle: IGoalHandle):
        for i in range(100):
            await goal_handle.publish_feedback({"progress": i})
        goal_handle.succeed()
        return {"status": "done"}

    @remote_actionServer.on_cancel(name="process")
    async def on_cancel(self, goal_handle: IGoalHandle) -> bool:
        return True  # accept cancellation
```

---

## Feeders (`feeder/`)

Feeders automatically publish structured data over the configured transport.

| Class | Purpose |
|---|---|
| `StateFeeder` | Publish module state changes |
| `NewsFeeder` | Publish informational messages |
| `ErrorFeeder` | Publish error reports |
| `AvailableModuleFeeder` | Publish module availability |

```python
from vyra_base.com import StateFeeder

feeder = StateFeeder(entity.node)
feeder.feed(state="RUNNING")
```

---

## Deprecated API (Removed)

The following names no longer exist in the public API. Use the replacements:

| Old Name | Replacement |
|---|---|
| `@remote_callable` | `@remote_service` |
| `@remote_speaker` | `@remote_publisher` |
| `@remote_listener` | `@remote_subscriber` |
| `@remote_job` | `@remote_actionServer` |
| `VyraCallable` | `VyraServer` |
| `VyraSpeaker` | `VyraPublisher` |
| `VyraJob` | `VyraActionServer` |
| `InterfaceType.CALLABLE` | `InterfaceType.SERVER` |
| `InterfaceType.SPEAKER` | `InterfaceType.PUBLISHER` |
| `InterfaceType.JOB` | `InterfaceType.ACTION_SERVER` |
| `create_vyra_callable()` | `InterfaceFactory.create_server()` |
| `create_vyra_speaker()` | `InterfaceFactory.create_publisher()` |
| `from vyra_base.com.datalayer...` | `from vyra_base.com...` |

The compatibility aliases (`remote_callable`, `remote_speaker`, `remote_job`) are still
available as private aliases for a transition period but are **not** exported in `__all__`.

---

## Further Documentation

- `core/README.md` — Core types, decorators, blueprints, factory
- `transport/README.md` — Transport layer details and provider setup
- `external/README.md` — External protocol configuration
- `feeder/README.md` — Feeder usage
