# VYRA Base Python — Examples

Practical examples showing how to use `vyra_base` to build VYRA modules.

## Structure

```
examples/
├── 01_service/                  ← Request/response services
│   ├── service_server.py        — @remote_service decorator, two-phase init
│   └── service_client.py        — InterfaceFactory.create_client()
│
├── 02_publisher_subscriber/     ← Pub/Sub messaging
│   ├── publisher.py             — @remote_publisher + shout()
│   └── subscriber.py            — @remote_subscriber callback
│
├── 03_action_server/            ← Long-running actions (goal/feedback/result)
│   ├── action_server.py         — IActionHandler + multi-callback decorator
│   └── action_client.py         — send_goal() + feedback callback
│
├── 04_protocols/                ← Protocol-specific examples
│   ├── zenoh_protobuf_example.py  — Zenoh + Protobuf serialization
│   ├── redis_pubsub_example.py    — Redis pub/sub
│   ├── transport_examples.py      — ROS2 + UDS provider setup
│   ├── external_communication.py  — gRPC, MQTT, REST
│   └── communication_demo.py      — Multi-protocol demo
│
├── 05_state_machine/            ← State machine patterns
│   ├── unified_state_machine_example.py   — UnifiedStateMachine usage
│   └── operational_state_machine_example.py — OperationalStateMachine metaclass
│
├── storage_example.py           ← SQLAlchemy database + Redis storage
├── DEPRECATED/                  ← Old ROS2-only examples (do not use)
└── README.md                    ← This file
```

---

## Quick Start

### Prerequisites

```bash
pip install vyra_base

# Optional — for Zenoh transport (recommended default):
pip install eclipse-zenoh

# Optional — for Redis transport:
pip install redis
```

### Run an example

```bash
cd /path/to/vyra_base_python

# Service server (Phase 2 binding demo, no network required)
python examples/01_service/service_server.py

# Publisher (requires Zenoh or Redis)
python examples/02_publisher_subscriber/publisher.py

# Subscriber (in a second terminal)
python examples/02_publisher_subscriber/subscriber.py

# Action server
python examples/03_action_server/action_server.py
```

---

## Key Concepts

### Two-Phase Initialization

All communication interfaces follow a **two-phase** pattern:

```python
# Phase 1 (automatic) — decorator creates a blueprint during class definition
class MyComponent:
    @remote_service(name="ping", protocols=[ProtocolType.ZENOH])
    async def ping(self, request, response=None):
        return {"pong": True}

# Phase 2 — bind the real callbacks to the blueprint
component = MyComponent()
bind_decorated_callbacks(component, namespace="my_module")

# The VYRA entity infrastructure then calls InterfaceFactory automatically.
# In tests or standalone scripts you can create interfaces manually:
server = await InterfaceFactory.create_from_blueprint(blueprint)
```

### Protocol Selection

Set `protocols` in order of preference. The first available protocol wins:

```python
@remote_service(
    name="my_service",
    protocols=[ProtocolType.ZENOH, ProtocolType.REDIS, ProtocolType.UDS],
    # Tries Zenoh first → Redis if Zenoh unavailable → UDS as last resort
)
```

Default fallback chain: `ZENOH → ROS2 → REDIS → UDS`

### Access Levels

Control who can call your service:

```python
from vyra_base.com import AccessLevel

@remote_service(name="admin_reset", access_level=AccessLevel.PRIVATE)
```

| Level | Description |
|---|---|
| `PUBLIC` | Accessible by all modules |
| `PROTECTED` | Authorized modules only |
| `PRIVATE` | Same module only |
| `INTERNAL` | Framework internal |

---

## DEPRECATED folder

The `DEPRECATED/` folder contains old ROS2-only API examples that use
`VyraCallable`, `VyraSpeaker`, `VyraJob`, `@remote_callable`, etc.
These APIs no longer exist in the public surface. Use the patterns
shown in `01_service/` to `03_action_server/` instead.
