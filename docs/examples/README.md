# VYRA Communication Decorator Examples

This folder contains examples demonstrating the **two-phase initialization decorator system** for VYRA communication interfaces.

## Overview

The refactored decorator system separates interface **definition** (blueprints) from **implementation** (callbacks), enabling:

- ✅ **Late Binding**: Define interfaces before components exist
- ✅ **Dynamic Registration**: Add/remove interfaces at runtime
- ✅ **Better Testing**: Mock callbacks without full components
- ✅ **Metadata-Driven**: Load interface definitions from JSON, bind later

## Two-Phase Pattern

```python
# Phase 1: Blueprint Creation (during decoration/JSON loading)
blueprint = ServiceBlueprint(name="calculate", protocols=[ProtocolType.ROS2])
CallbackRegistry.register_blueprint(blueprint)

# Phase 2: Callback Binding (during component initialization)
async def my_callback(request, response=None):
    return {"result": 42}

blueprint.bind_callback(my_callback)

# Phase 3: Interface Creation
service = await InterfaceFactory.create_from_blueprint(blueprint)
```

## Examples

### 1. Basic Service (`example_basic_service.py`)

Demonstrates the core two-phase pattern with services:

```python
class CalculatorComponent:
    @remote_service(name="add", protocols=[ProtocolType.REDIS])
    async def add_numbers(self, request, response=None):
        return {"result": request["x"] + request["y"]}

# Blueprints created automatically during decoration
# Later: bind callbacks and create interfaces
bind_decorated_callbacks(component, namespace="calculator")
```

**Run:**
```bash
python -m vyra_base.com.core.examples.example_basic_service
```

**Shows:**
- Old vs. new decorator approach
- Blueprint registration
- Callback binding
- Late binding capabilities

### 2. Publisher with Property Setters (`example_publisher_property.py`)

Demonstrates auto-publishing property pattern (inspired by Untitled-2 sketch):

```python
class RobotController:
    @property
    def status(self):
        return self._status
    
    @status.setter
    def status(self, value):
        self._status = value
        asyncio.create_task(self._publisher.shout(value))  # Auto-publish!

# Usage: robot.status = {"state": "running"}  # Publishes automatically!
```

**Run:**
```bash
python -m vyra_base.com.core.examples.example_publisher_property
```

**Shows:**
- Property setter publishing pattern
- Comparison with explicit publishing
- Mission simulation with auto-publishing

## Core Concepts

### HandlerBlueprint

Represents an interface **definition** (what it should be):

```python
from vyra_base.com.core.blueprints import ServiceBlueprint

blueprint = ServiceBlueprint(
    name="my_service",
    protocols=[ProtocolType.ROS2, ProtocolType.ZENOH],
    metadata={"qos": 10, "description": "My service"},
    service_type=MyServiceType
)
```

### CallbackRegistry

Global registry managing blueprints and their lifecycle:

```python
from vyra_base.com.core.callback_registry import CallbackRegistry

# Register blueprint
CallbackRegistry.register_blueprint(blueprint, namespace="module_name")

# Bind callback later
CallbackRegistry.bind_callback("my_service", my_callback, namespace="module_name")

# Debug
CallbackRegistry.debug_print()
```

### InterfaceFactory

Creates actual transport interfaces from blueprints:

```python
from vyra_base.com.core.factory import InterfaceFactory

# From blueprint
service = await InterfaceFactory.create_from_blueprint(blueprint)

# Or directly
service = await InterfaceFactory.create_server(
    name="my_service",
    response_callback=my_callback,
    protocols=[ProtocolType.ROS2]
)
```

### Decorators

Mark methods as communication interfaces:

```python
from vyra_base.com.core.decorators import (
    remote_service,      # Request/response (ROS2 Service, gRPC)
    remote_publisher,    # Pub/sub sender (ROS2 Topic, Redis)
    remote_subscriber,   # Pub/sub receiver
    remote_actionServer, # Long-running tasks (ROS2 Action)
    bind_decorated_callbacks
)

class Component:
    @remote_service(name="process", protocols=[ProtocolType.ROS2])
    async def process_data(self, request, response=None):
        return {"status": "ok"}

# Bind all decorated methods
bind_decorated_callbacks(component, namespace="my_module")
```

## Integration with VYRA Modules

In a real VYRA module, the pattern looks like:

```python
# === In _base_.py (Phase 1) ===
async def _create_base_interfaces():
    """Load metadata from JSON, create blueprints (no callbacks yet)."""
    interfaces = []
    for metadata in load_json("config/interfaces.json"):
        blueprint = ServiceBlueprint(
            name=metadata["functionname"],
            protocols=[ProtocolType.ROS2],
            metadata=metadata
        )
        CallbackRegistry.register_blueprint(blueprint, namespace="my_module")
        interfaces.append(blueprint)
    return interfaces

# === In application.py (Phase 2) ===
class Component:
    @remote_service(name="initialize")
    async def initialize(self, request, response=None):
        # Implementation
        return {"success": True}
    
    async def set_interfaces(self):
        """Bind callbacks to blueprints, create interfaces."""
        bind_decorated_callbacks(self, namespace="my_module")
        
        # Create interfaces from bound blueprints
        for name in CallbackRegistry.list_bound(namespace="my_module"):
            blueprint = CallbackRegistry.get_blueprint(name)
            await InterfaceFactory.create_from_blueprint(blueprint)
```

## Requirements

Examples require:
- ✅ `vyra_base_python` installed
- ⚠️  Redis running (for publisher/subscriber examples)
  - Start with: `docker run -d -p 6379:6379 redis:latest`
- ⚠️  ROS2 environment (for ROS2 protocol examples)

If Redis/ROS2 not available, examples will demonstrate the pattern without actual transport.

## More Examples (Coming Soon)

- `example_action.py` - Long-running action with feedback
- `example_late_binding.py` - Advanced late binding scenarios
- `example_handler_interfaces.py` - Using optional abstract base classes
- `example_testing.py` - Unit testing with mock callbacks

## See Also

- [../DECORATOR_GUIDE.md](../DECORATOR_GUIDE.md) - Full decorator API documentation
- [../MIGRATION_GUIDE.md](../MIGRATION_GUIDE.md) - Migrating from old decorators
- [../blueprints.py](../blueprints.py) - Blueprint implementation
- [../callback_registry.py](../callback_registry.py) - Registry implementation
- [../decorators.py](../decorators.py) - Decorator implementation
