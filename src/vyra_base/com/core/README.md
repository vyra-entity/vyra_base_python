# Core Communication Components (`com/core/`)

Foundation types, exceptions, factory, decorators and blueprints for the VYRA multi-protocol communication system.

## Overview

```
core/
├── types.py               ← ProtocolType, InterfaceType, AccessLevel, Vyra* type aliases
├── exceptions.py          ← All communication exceptions
├── decorators.py          ← @remote_service, @remote_publisher, @remote_subscriber, @remote_actionServer
├── blueprints.py          ← ServiceBlueprint, PublisherBlueprint, SubscriberBlueprint, ActionBlueprint
├── abstract_handlers.py   ← IServiceHandler, IActionHandler, IGoalHandle  (implement in your component)
├── factory.py             ← InterfaceFactory  (creates live communication objects)
├── callback_registry.py   ← CallbackRegistry  (stores and retrieves blueprints)
├── topic_builder.py       ← TopicBuilder, build_topic(), parse_topic()
├── interface_loader.py    ← InterfaceLoader  (dynamic .srv/.msg/.action loading)
└── interface_path_registry.py ← InterfacePathRegistry
```

---

## Types (`types.py`)

### ProtocolType

```python
from vyra_base.com.core.types import ProtocolType

class ProtocolType(str, Enum):
    # Transport Layer (in-process / low-latency)
    ROS2   = "ros2"          # ROS2 / DDS  — requires rclpy
    ZENOH  = "zenoh"         # Zenoh       — requires eclipse-zenoh  (DEFAULT)
    REDIS  = "redis"         # Redis       — requires redis
    UDS    = "uds"           # Unix Domain Sockets (no extra deps)

    # External Layer (cross-service)
    SHARED_MEMORY = "sharedmemory"  # POSIX SHM (Linux, no extra deps)
    MQTT          = "mqtt"          # requires paho-mqtt
    GRPC          = "grpc"          # requires grpcio
    REST          = "rest"          # requires aiohttp
    WEBSOCKET     = "websocket"     # requires websockets

    # Industrial Layer
    MODBUS = "modbus"   # requires pymodbus
    OPCUA  = "opcua"    # requires asyncua
```

### InterfaceType

```python
from vyra_base.com.core.types import InterfaceType

class InterfaceType(str, Enum):
    PUBLISHER     = "publisher"     # Publish-only (no callback)
    SUBSCRIBER    = "subscriber"    # Subscribe with callback
    SERVER        = "server"        # Request-Response server
    CLIENT        = "client"        # Request-Response client
    ACTION_SERVER = "actionServer"  # Long-running task server
    ACTION_CLIENT = "actionClient"  # Long-running task client
```

### AccessLevel

```python
from vyra_base.com.core.types import AccessLevel

class AccessLevel(str, Enum):
    PUBLIC    = "public"    # Accessible by all modules
    PROTECTED = "protected" # Accessible by authorized modules only
    PRIVATE   = "private"   # Accessible within the same module only
    INTERNAL  = "internal"  # Framework-internal use
```

### Protocol-agnostic type aliases

These are returned by `InterfaceFactory`:

| Type | Description |
|---|---|
| `VyraServer` | A running service server (request/response) |
| `VyraClient` | A service client |
| `VyraPublisher` | A publisher that can `.shout()` messages |
| `VyraSubscriber` | A subscriber that calls a callback on each message |
| `VyraActionServer` | An action server (goal / feedback / result) |
| `VyraActionClient` | An action client |

---

## Exceptions (`exceptions.py`)

```python
from vyra_base.com.core.exceptions import (
    CommunicationError,           # Base for all com errors
    ProtocolUnavailableError,     # Requested protocol not installed / not reachable
    ProtocolNotInitializedError,  # Protocol not set up yet
    TransportError,               # Low-level transport failure
    ProviderError,                # Provider setup / runtime error
    InterfaceError,               # Interface creation failed
    TServerError,                 # Server-specific error
    TSubscriberError,             # Subscriber-specific error
    ActionServerError,            # Action server error
)
```

---

## Decorators (`decorators.py`)

Decorators transform component methods into communication interface definitions.
They operate in a **two-phase** pattern: decorating creates a *blueprint*; calling
`bind_decorated_callbacks()` later attaches the callback to a live interface.

### @remote_service

Exposes a method as a request/response service.

```python
from vyra_base.com import remote_service, ProtocolType

class MyComponent:
    @remote_service(
        name="calculate",              # unique service name
        protocols=[ProtocolType.ZENOH], # preferred protocol(s)
        namespace="my_module",          # optional topic namespace
        access_level=AccessLevel.PUBLIC,
    )
    async def calculate(self, request, response=None):
        return {"result": request["x"] + request["y"]}
```

### @remote_publisher

Makes a method that publishes messages.

```python
from vyra_base.com import remote_publisher, ProtocolType

class SensorComponent:
    @remote_publisher(
        name="temperature",
        protocols=[ProtocolType.ZENOH],
        namespace="sensors",
    )
    async def publish_temperature(self, value: float):
        return {"value": value, "unit": "°C"}
```

### @remote_subscriber

Registers a method as a message subscriber.

```python
from vyra_base.com import remote_subscriber, ProtocolType

class DashboardComponent:
    @remote_subscriber(
        name="temperature",
        protocols=[ProtocolType.ZENOH],
        namespace="sensors",
    )
    async def on_temperature(self, message):
        print(f"Temperature: {message['value']} {message['unit']}")
```

### @remote_actionServer

Registers a group of methods as a multi-callback action server.
Your class **must** implement `IActionHandler`.

```python
from vyra_base.com import remote_actionServer, IActionHandler, IGoalHandle, ProtocolType

class ProcessingComponent(IActionHandler):
    @remote_actionServer.on_goal(name="process", protocols=[ProtocolType.ZENOH])
    async def on_goal(self, goal_request) -> bool:
        """Return True to accept, False to reject the goal."""
        return goal_request.get("dataset") is not None

    @remote_actionServer.execute(name="process")
    async def execute(self, goal_handle: IGoalHandle):
        """Main execution — publish feedback, then set result."""
        for i in range(100):
            await goal_handle.publish_feedback({"progress": i})
        goal_handle.succeed()
        return {"status": "done"}

    @remote_actionServer.on_cancel(name="process")
    async def on_cancel(self, goal_handle: IGoalHandle) -> bool:
        """Return True to accept cancellation."""
        return True
```

### Helper functions

```python
from vyra_base.com import get_decorated_methods, bind_decorated_callbacks

# Inspect which methods have been decorated on a component instance
methods = get_decorated_methods(component)

# Phase 2: attach callbacks to blueprints so InterfaceFactory can create live interfaces
bind_decorated_callbacks(component, namespace="my_module")
```

---

## Blueprints (`blueprints.py`)

Blueprints describe a communication interface *before* it is created.
They are produced automatically when you use the decorators above.

```python
from vyra_base.com import ServiceBlueprint, PublisherBlueprint, SubscriberBlueprint, ActionBlueprint

# Manually create a blueprint (rarely needed — decorators do this automatically)
bp = ServiceBlueprint(
    name="add",
    protocols=[ProtocolType.ZENOH],
    namespace="calculator",
)
bp.bind_callback(my_callback_fn)
service = await InterfaceFactory.create_from_blueprint(bp)
```

---

## InterfaceFactory (`factory.py`)

Creates live communication objects from blueprints or keyword arguments.

```python
from vyra_base.com import InterfaceFactory, ProtocolType

# Create a service server
server = await InterfaceFactory.create_server(
    "calculate",
    callback=handle_calculate,
    protocols=[ProtocolType.ZENOH],
)

# Create a service client
client = await InterfaceFactory.create_client(
    "calculate",
    protocols=[ProtocolType.ZENOH],
)
response = await client.call({"x": 5, "y": 3})

# Create a publisher
publisher = await InterfaceFactory.create_publisher(
    "events",
    protocols=[ProtocolType.ZENOH],
)
await publisher.shout({"event": "started"})

# Create a subscriber
subscriber = await InterfaceFactory.create_subscriber(
    "events",
    callback=handle_event,
    protocols=[ProtocolType.ZENOH],
)

# Create from a blueprint (recommended with decorators)
interface = await InterfaceFactory.create_from_blueprint(blueprint)
```

### Automatic fallback

If the first protocol in `protocols` is unavailable, the factory tries the next one:

```python
# Tries Zenoh → falls back to Redis → falls back to UDS
server = await InterfaceFactory.create_server(
    "my_service",
    callback=handler,
    protocols=[ProtocolType.ZENOH, ProtocolType.REDIS, ProtocolType.UDS],
)

# Check which protocols are currently available
available = InterfaceFactory.get_available_protocols()
```

---

## Abstract Handlers (`abstract_handlers.py`)

Your component must implement these interfaces when using the corresponding decorators.

### IServiceHandler

```python
from vyra_base.com import IServiceHandler

class MyService(IServiceHandler):
    # No abstract methods — IServiceHandler is a marker interface.
    # Use @remote_service decorator on your methods.
    pass
```

### IActionHandler + IGoalHandle

```python
from vyra_base.com import IActionHandler, IGoalHandle

class MyAction(IActionHandler):
    # Use @remote_actionServer.on_goal / .execute / .on_cancel decorators.
    async def execute(self, goal_handle: IGoalHandle):
        await goal_handle.publish_feedback({"progress": 50})
        goal_handle.succeed()
        return {"result": "done"}
```

`IGoalHandle` methods:

| Method | Description |
|---|---|
| `goal_handle.succeed()` | Mark the action as succeeded |
| `goal_handle.abort()` | Mark the action as aborted |
| `goal_handle.canceled()` | Mark the action as canceled |
| `await goal_handle.publish_feedback(data)` | Send intermediate progress |
| `goal_handle.is_cancel_requested` | Check if client requested cancellation |

---

## Topic Builder (`topic_builder.py`)

Generates consistent topic/service names following VYRA naming conventions.

```python
from vyra_base.com import TopicBuilder, build_topic, parse_topic, InterfaceType

# Simple helper
topic = build_topic(
    namespace="my_module",
    name="temperature",
    interface_type=InterfaceType.PUBLISHER,
)
# → "my_module/temperature"

# Full builder
builder = TopicBuilder(namespace="my_module", module_id="sensor_01")
name = builder.build(name="status", interface_type=InterfaceType.SERVER)

# Parse an existing topic string back to components
components = parse_topic("my_module/temperature")
print(components.namespace, components.name)
```

---

## CallbackRegistry (`callback_registry.py`)

Central registry for all blueprints. Used internally by decorators and `InterfaceFactory`.

```python
from vyra_base.com import CallbackRegistry

# List all registered blueprints
blueprints = CallbackRegistry.get_all_blueprints()

# Get a specific blueprint by name and namespace
bp = CallbackRegistry.get_blueprint(name="calculate", namespace="calculator")
```

---

## Migration from Legacy API

| Old | New |
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
