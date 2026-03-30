# vyra_base_python — VYRA Core Library

[![Documentation](https://img.shields.io/badge/docs-sphinx-blue)](https://vyra-entity.github.io/vyra_base_python/)
[![Python Version](https://img.shields.io/badge/python-3.11+-blue)](https://www.python.org/downloads/)

**Core Python library for the VYRA framework** — provides multi-protocol communication, state management, storage, security, and plugin primitives.

`vyra_base_python` can be used as a **standalone library** for any Python application. It is typically used as the foundation layer inside a **VYRA module**. A new module can be created from the official template:

```bash
# Create a new VYRA module from the template (requires copier)
copier copy gh:vyra-entity/vyra_module_template my_new_module
```

Alternatively, an existing module such as `v2_modulemanager` or `v2_dashboard` can be adapted as starting point. Both approaches provide the full Docker/Swarm infrastructure, frontend scaffolding and interface tooling on top of `vyra_base`.

---

## Overview

`vyra_base_python` provides:

- **Transport Protocols**: Zenoh (default, P2P/DDS), ROS2 (DDS middleware), Redis (Pub/Sub + key-store), UDS (Unix Domain Sockets)
- **External Protocols**: gRPC, MQTT, REST, WebSocket, TCP/UDP, Shared Memory
- **Industrial Protocols**: Modbus (TCP/RTU), OPC UA
- **Unified State Machine**: 3-layer state management (Lifecycle, Operational, Health)
- **Security Framework**: 5-level authentication model (NONE → DIGITAL_SIGNATURE)
- **Storage Layer**: SQLite (SQLAlchemy) + Redis client with TLS
- **Plugin System**: WASM runtime with Python stub fallback
- **Parameter System**: Runtime configuration and validation

---

## Quick Start

### Installation

```bash
# Install from source (development)
cd vyra_base_python
pip install -e .

# Install from wheel (production)
pip install vyra_base-<version>-py3-none-any.whl
```

### Communication with Zenoh (Blueprint Pattern)

Zenoh is the **default transport** in vyra_base. The decorator system uses a two-phase pattern: decorators define blueprints at class definition time; callbacks are bound during initialization.

```python
import asyncio
from vyra_base.com import (
    remote_service,
    remote_actionServer,
    bind_decorated_callbacks,
    InterfaceFactory,
    ProtocolType,
)
from vyra_base.com.core import IActionHandler, IGoalHandle


class SensorComponent(IActionHandler):
    """Component exposing services and actions via Zenoh."""

    # Phase 1: blueprint definition via decorators
    @remote_service(name="read_sensor", protocols=[ProtocolType.ZENOH])
    async def read_sensor(self, request, response=None):
        """Request/response service — returns current sensor value."""
        return {"value": 42.0, "unit": "°C"}

    @remote_actionServer.on_goal(name="calibrate")
    async def accept_calibration(self, goal_request) -> bool:
        """Validate calibration goal — accept only if mode is known."""
        return goal_request.get("mode") in ("auto", "manual")

    @remote_actionServer.on_cancel(name="calibrate")
    async def cancel_calibration(self, goal_handle: IGoalHandle) -> bool:
        return True  # allow cancel at any time

    @remote_actionServer.execute(name="calibrate")
    async def run_calibration(self, goal_handle: IGoalHandle) -> dict:
        """Long-running action with incremental feedback."""
        for step in range(10):
            if goal_handle.is_cancel_requested():
                goal_handle.canceled()
                return {"cancelled": True}
            goal_handle.publish_feedback({"progress": step * 10})
            await asyncio.sleep(0.1)
        goal_handle.succeed()
        return {"offset": 0.05}


async def main():
    factory = InterfaceFactory()
    component = SensorComponent()

    # Phase 2: bind callbacks and create interfaces
    bind_decorated_callbacks(component, factory)

    # Call the service from another component
    client = await factory.create_client(
        name="read_sensor",
        protocol=ProtocolType.ZENOH,
        namespace="sensor",
    )
    result = await client.call({"channel": 1})
    print(result)  # {'value': 42.0, 'unit': '°C'}


asyncio.run(main())
```

See [`examples/08_decorator_blueprints/`](examples/08_decorator_blueprints/) for more patterns including Redis and multi-transport fallback.

---

## Documentation

- **[Full API Documentation](https://vyra-entity.github.io/vyra_base_python/)** — complete Sphinx docs
- **[State Machine Guide](docs/state/README.md)** — 3-layer state management
- **[Security Framework](docs/security/SECURITY_FRAMEWORK.md)** — 5-level security model
- **[Transport Layer](src/vyra_base/com/transport/README.md)** — Zenoh, ROS2, Redis, UDS
- **[Translation Guide](docs/TRANSLATION_GUIDE.md)** — multilingual documentation

---

## Key Concepts

### 1. Transport Protocols

vyra_base supports four internal transport protocols. The protocol is selected per interface:

| Protocol | Use Case | Availability |
|---|---|---|
| **Zenoh** | Default, P2P, low-latency, distributed | `pip install eclipse-zenoh` |
| **ROS2** | DDS middleware, existing ROS2 ecosystems | ROS2 installation required |
| **Redis** | Shared state, pub/sub across services | Redis server required |
| **UDS** | Local IPC, same host, minimal overhead | POSIX only |

```python
from vyra_base.com import ProtocolType, remote_service

# Explicit protocol selection
@remote_service(name="my_service", protocols=[ProtocolType.ZENOH])
async def my_service(self, request, response=None):
    ...

# Multi-protocol with automatic fallback
@remote_service(name="my_service", protocols=[ProtocolType.ZENOH, ProtocolType.REDIS])
async def my_service(self, request, response=None):
    ...
```

See [`src/vyra_base/com/transport/README.md`](src/vyra_base/com/transport/README.md) for full protocol details.

### 2. State Management

The Unified State Machine coordinates three independent layers:

```python
from vyra_base.state import UnifiedStateMachine

usm = UnifiedStateMachine()
usm.start()                       # Lifecycle: Offline → Initializing
usm.complete_initialization()     # Lifecycle: Initializing → Active
usm.set_ready()                   # Operational: Idle → Ready
usm.start_task({"id": "job-1"})  # Operational: Ready → Running

states = usm.get_all_states()
# {'lifecycle': 'Active', 'operational': 'Running', 'health': 'Healthy'}

# Register transition callbacks
usm.on_any_change(lambda layer, old, new: print(f"{layer}: {old} → {new}"))
usm.on_health_change(lambda layer, old, new: print(f"Health changed: {new}"))
```

See [`docs/state/README.md`](docs/state/README.md) for the full state machine guide.

### 3. VyraEntity — Module Core

`VyraEntity` is the central object in every VYRA module. It wires together communication, parameters, feeders and storage:

```python
from vyra_base.core import VyraEntity

entity = await VyraEntity.create("my_module")

# Publish structured data to all registered transports
entity.publish_news("Module started")
entity.publish_state("RUNNING")

# Call a remote service on another module
result = await entity.call_service("other_module/service_name", request_data)
```

### 4. Security Framework

vyra_base implements a 5-level security model applied per-message:

| Level | Name | Protection |
|---|---|---|
| 1 | NONE | No checks |
| 2 | BASIC_AUTH | Module ID verification |
| 3 | EXTENDED_AUTH | ID + password authentication |
| 4 | HMAC | HMAC-SHA256 message integrity |
| 5 | DIGITAL_SIGNATURE | Certificate-based PKI |

```python
from vyra_base.security import SecurityManager, SecurityLevel

class MyModule(SecurityManager):
    def __init__(self):
        SecurityManager.__init__(self, max_security_level=SecurityLevel.HMAC)
```

See [`docs/security/SECURITY_FRAMEWORK.md`](docs/security/SECURITY_FRAMEWORK.md) for the complete guide.

### 5. Storage

```python
from vyra_base.storage import DbAccess, DbManipulator, DBTYPE
from vyra_base.com.transport.t_redis import RedisClient

# SQLite / PostgreSQL via SQLAlchemy
db = DbAccess(db_type=DBTYPE.SQLITE, db_path="/workspace/storage/data.db")
manipulator = DbManipulator(db)

# Redis (TLS)
redis = RedisClient(
    host="redis",
    port=6379,
    ssl=True,
    ssl_ca_certs="/workspace/storage/certificates/redis/ca-cert.pem",
)
await redis.set("key", "value")
value = await redis.get("key")
```

---

## Package Structure

```
src/vyra_base/
├── com/                        # Multi-protocol communication
│   ├── core/                   # Types, decorators, blueprints, factory
│   ├── transport/              # Protocol transports
│   │   ├── t_zenoh/            # Eclipse Zenoh (DEFAULT)
│   │   ├── t_ros2/             # ROS2 / DDS
│   │   ├── t_redis/            # Redis Pub/Sub + RedisClient
│   │   └── t_uds/              # Unix Domain Sockets
│   ├── external/               # External protocols
│   │   ├── grpc/               # gRPC client/server
│   │   ├── mqtt/               # MQTT client
│   │   ├── rest/               # REST client
│   │   ├── tcp/                # TCP client/server
│   │   ├── udp/                # UDP client/server
│   │   ├── websocket/          # WebSocket client
│   │   └── shared_memory/      # Shared memory IPC
│   ├── industrial/             # Industrial protocols
│   │   ├── modbus/             # Modbus TCP/RTU
│   │   └── opcua/              # OPC UA client/server
│   ├── feeder/                 # Multi-protocol data publishers
│   ├── handler/                # Protocol-specific feeder handlers
│   ├── converter/              # Protobuf and message converters
│   └── providers/              # Abstract provider pattern
├── core/                       # Entity, parameter, volatile storage
│   ├── entity.py               # VyraEntity — main module interface
│   ├── interface_builder.py    # Interface registration
│   ├── parameter.py            # Runtime parameters
│   └── volatile.py             # Volatile key-value storage
├── defaults/                   # Constants, entries, exceptions
├── helper/                     # Utilities (file I/O, logging, crypto)
├── interfaces/                 # ROS2/proto interface definitions + config
├── plugin/                     # WASM plugin runtime (WasmRuntime, StubRuntime)
├── security/                   # 5-level security framework
├── state/                      # 3-layer state machine
│   ├── unified.py              # UnifiedStateMachine
│   ├── lifecycle_layer.py
│   ├── operational_layer.py
│   └── health_layer.py
└── storage/                    # SQLite + Redis storage
```

---

## Development

### Setup

```bash
cd vyra_base_python
pip install -e ".[dev]"
```

### Tests

```bash
# Unit tests (no external dependencies)
pytest -m unit

# Integration tests (requires Redis, Zenoh)
pytest -m integration

# End-to-end tests (full stack)
pytest -m e2e

# With coverage
pytest --cov=vyra_base --cov-report=html
```

### Documentation

```bash
cd docs
./build_multilingual.sh
# or single language:
make html
```

---

## Multilingual Documentation

- **English** (default): https://vyra-entity.github.io/vyra_base_python/en/
- **Deutsch**: https://vyra-entity.github.io/vyra_base_python/de/

See [Translation Guide](docs/TRANSLATION_GUIDE.md).

---

## Internal Documentation

Each source module has its own `README.md` with detailed API documentation:

| Module | Description |
|--------|-------------|
| [`src/vyra_base/com/README.md`](src/vyra_base/com/README.md) | Multi-protocol communication layer |
| [`src/vyra_base/com/transport/README.md`](src/vyra_base/com/transport/README.md) | Transport protocols (Zenoh, ROS2, Redis, UDS) |
| [`src/vyra_base/core/README.md`](src/vyra_base/core/README.md) | VyraEntity, InterfaceBuilder, Parameter, Volatile |
| [`src/vyra_base/defaults/README.md`](src/vyra_base/defaults/README.md) | Constants, entry types, exceptions |
| [`src/vyra_base/helper/README.md`](src/vyra_base/helper/README.md) | Utilities (logging, file I/O, crypto, env) |
| [`src/vyra_base/interfaces/README.md`](src/vyra_base/interfaces/README.md) | Interface definitions and config format |
| [`src/vyra_base/plugin/README.md`](src/vyra_base/plugin/README.md) | WASM plugin runtime |
| [`src/vyra_base/security/README.md`](src/vyra_base/security/README.md) | 5-level security framework |
| [`src/vyra_base/state/README.md`](src/vyra_base/state/README.md) | Lifecycle, operational and health state machines |
| [`src/vyra_base/storage/README.md`](src/vyra_base/storage/README.md) | SQLite parameter storage and Redis volatile layer |

---

## Versioning

See [VERSIONING.md](VERSIONING.md) and [CHANGELOG.md](CHANGELOG.md).

## License

See [LICENSE](LICENSE).

## Maintainer

**Variobotic GmbH** — internal project for VYRA framework development.

