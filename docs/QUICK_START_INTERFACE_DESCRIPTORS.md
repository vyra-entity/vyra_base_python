# Interface Descriptor System - Quick Start Guide

**Target Audience:** VYRA Module Developers  
**Last Updated:** February 17, 2026  
**Related Docs:** 
- [Full Architecture](./INTERFACE_DESCRIPTOR_ARCHITECTURE.md)
- [Implementation Plan](../BLUEPRINT_REFACTORING_PLAN.md)

---

## 🚀 TL;DR - 30 Second Overview

**Old Way (Tight Coupling):**
```python
# ❌ Callback must exist NOW when creating interface
interface = await InterfaceFactory.create_server(
    name="calculate",
    response_callback=my_function,  # ← Must be available
    protocols=[ProtocolType.ROS2],
    node=node
)
```

**New Way (Two-Phase Late Binding):**
```python
# ✅ PHASE 1: Define interface early (from JSON)
descriptor = ServiceDescriptor(name="calculate", protocols=[ProtocolType.ROS2])
CallbackRegistry.register_descriptor(descriptor, namespace="my_module")

# ✅ PHASE 2: Bind callback later (from decorator)
@remote_service(name="calculate")
async def calculate(self, request, response=None):
    return {"result": 42}

bind_decorated_callbacks(component, namespace="my_module")

# ✅ PHASE 3: Create active interface (automatic)
interface = await InterfaceFactory.create_from_descriptor(descriptor, node=node)
```

**Why?** Interfaces can be defined before their implementations exist → better testability, cleaner architecture.

---

## 📚 Core Concepts

### 1. Descriptor vs Interface

| Concept | What It Is | When Created |
|---------|-----------|--------------|
| **InterfaceDescriptor** | Definition/Blueprint of interface (metadata, protocols, etc.) | Early (module init, from JSON) |
| **VyraServer/Publisher/etc.** | Active running interface (network connection, ROS2 node, etc.) | Late (after callbacks bound) |

**Analogy:** 
- **Descriptor** = Recipe card (ingredients, steps, but no food yet)
- **Interface** = Cooked meal (actual food you can eat)

### 2. Three-Phase Pattern

```
┌─────────────────┐      ┌─────────────────┐      ┌─────────────────┐
│  PHASE 1        │      │  PHASE 2        │      │  PHASE 3        │
│  Register       │  →   │  Bind           │  →   │  Create         │
│  Descriptor     │      │  Callback       │      │  Interface      │
└─────────────────┘      └─────────────────┘      └─────────────────┘
   From JSON              From @decorator         Via Factory
   In _base_.py           In application.py       In interface.py
   Early                  When component ready    When callback bound
```

### 3. Key Classes

```python
# Descriptors (definitions)
from vyra_base.com import (
    ServiceDescriptor,       # For request-response
    PublisherDescriptor,     # For one-way pub
    SubscriberDescriptor,    # For message receive
    ActionDescriptor         # For long-running tasks (multi-callback!)
)

# Registry (storage)
from vyra_base.com import CallbackRegistry

# Factory (creation)
from vyra_base.com import InterfaceFactory

# Decorators (marking callbacks)
from vyra_base.com import (
    remote_service,
    remote_publisher,
    remote_subscriber,
    remote_actionServer
)

# Helpers
from vyra_base.com import (
    get_decorated_methods,
    bind_decorated_callbacks
)
```

---

## 🛠️ How To Use - Step by Step

### Scenario: Add New Service "calculate"

#### Step 1: Define Interface Metadata (JSON)

**File:** `src/my_module_interfaces/config/interface_metadata.json`

```json
{
  "functionname": "calculate",
  "type": "service",
  "filetype": "my_module_interfaces.srv.Calculate",
  "protocols": ["ros2"],
  "qos": 10
}
```

#### Step 2: Create Descriptor in _base_.py

**File:** `src/my_module/my_module/_base_.py`

```python
from vyra_base.com import ServiceDescriptor, CallbackRegistry

async def _create_base_interfaces():
    """
    PHASE 1: Create descriptors from JSON metadata.
    Called during entity initialization.
    """
    blueprints = []
    metadata_list = await _load_interface_metadata()  # Load JSON
    
    for meta in metadata_list:
        if meta["type"] == "service":
            descriptor = ServiceDescriptor(
                name=meta["functionname"],
                protocols=[ProtocolType.ROS2],
                metadata=meta,
                service_type=_load_service_type(meta["filetype"])
            )
            
            # Register in global registry
            CallbackRegistry.register_descriptor(
                descriptor,
                namespace="my_module"  # ← Your module name
            )
            
            blueprints.append(descriptor)
    
    return blueprints
```

#### Step 3: Define Callback in application.py

**File:** `src/my_module/my_module/application/application.py`

```python
from vyra_base.com import remote_service

class Application:
    """
    PHASE 2: Define callbacks with decorators.
    Decorators mark methods for later binding.
    """
    
    @remote_service(name="calculate")  # ← Must match descriptor name!
    async def calculate(self, request, response=None):
        """
        Calculate service handler.
        
        Args:
            request: Calculate.Request with fields x, y
            response: Calculate.Response (optional, unused)
        
        Returns:
            dict or Response object with result
        """
        result = request.x + request.y
        return {"result": result}
```

#### Step 4: Bind Callbacks in interface.py

**File:** `src/my_module/my_module/interface.py`

```python
from vyra_base.com import (
    bind_decorated_callbacks,
    CallbackRegistry,
    InterfaceFactory
)

async def auto_register_callable_interfaces(entity, component):
    """
    PHASE 2 + 3: Bind callbacks and create interfaces.
    Called after component initialization.
    """
    
    # PHASE 2: Discover decorated methods and bind to descriptors
    results = bind_decorated_callbacks(
        component,
        namespace="my_module"
    )
    
    logger.info(f"✅ Bound {len(results)} callbacks")
    
    # PHASE 3: Create interfaces from bound descriptors
    for name in CallbackRegistry.list_bound(namespace="my_module"):
        descriptor = CallbackRegistry.get_descriptor(name, namespace="my_module")
        
        if descriptor:
            interface = await InterfaceFactory.create_from_descriptor(
                descriptor,
                node=entity.node  # ROS2 node for protocol
            )
            
            if interface:
                logger.info(f"✅ Created interface: {name}")
            else:
                logger.warning(f"⚠️ Interface '{name}' pending (awaiting callback)")
```

#### Step 5: Test

```bash
# Start module
cd /home/holgder/VOS2_WORKSPACE
./tools/vyra_up.sh

# In another terminal, test service
ros2 service list | grep calculate
ros2 service call /my_module/calculate my_module_interfaces/srv/Calculate "{x: 5, y: 3}"
# Should return: result: 8
```

---

## 🎯 Common Patterns

### Pattern 1: Simple Service

```python
# In _base_.py
descriptor = ServiceDescriptor(name="my_service", protocols=[ProtocolType.ROS2])
CallbackRegistry.register_descriptor(descriptor, namespace="my_module")

# In application.py
@remote_service(name="my_service")
async def my_service(self, request, response=None):
    return {"result": process(request.data)}
```

### Pattern 2: Publisher (No Callback Needed)

```python
# In _base_.py
descriptor = PublisherDescriptor(name="status", protocols=[ProtocolType.ROS2])
CallbackRegistry.register_descriptor(descriptor, namespace="my_module")

# In application.py
@remote_publisher(name="status")
async def publish_status(self, message):
    pass  # Decorator handles publishing automatically

# Usage in code
await self.publish_status({"state": "running"})
```

### Pattern 3: Subscriber

```python
# In _base_.py
descriptor = SubscriberDescriptor(name="updates", protocols=[ProtocolType.ROS2])
CallbackRegistry.register_descriptor(descriptor, namespace="my_module")

# In application.py
@remote_subscriber(name="updates")
async def on_update(self, message):
    logger.info(f"Received update: {message}")
    # Process message
```

### Pattern 4: ActionServer (Multi-Callback) ⭐ NEW!

```python
# In _base_.py
descriptor = ActionDescriptor(
    name="process_batch",
    protocols=[ProtocolType.ROS2],
    action_type=ProcessBatchAction
)
CallbackRegistry.register_descriptor(descriptor, namespace="my_module")

# In application.py
@remote_actionServer.on_goal(name="process_batch")
async def accept_goal(self, goal_request):
    """Decide whether to accept the goal."""
    if goal_request.count > 1000:
        logger.warning("Rejecting large batch")
        return False
    return True

@remote_actionServer.on_cancel(name="process_batch")
async def cancel_batch(self, goal_handle):
    """Decide whether to accept cancel request."""
    logger.info("Cancel requested, accepting")
    return True

@remote_actionServer.execute(name="process_batch")
async def execute_batch(self, goal_handle):
    """Main execution logic."""
    count = goal_handle.goal.count
    
    for i in range(count):
        # Check for cancellation
        if goal_handle.is_cancel_requested():
            goal_handle.canceled()
            return {"processed": i, "status": ActionStatus.CANCELED}
        
        # Do work
        process_item(i)
        
        # Send feedback
        goal_handle.publish_feedback({
            "progress": i + 1,
            "total": count
        })
    
    goal_handle.succeed()
    return {"processed": count, "status": ActionStatus.SUCCEEDED}
```

### Pattern 5: Property Setter Auto-Publishing

```python
class SensorComponent:
    def __init__(self):
        self._temperature = 0.0
    
    @property
    def temperature(self) -> float:
        return self._temperature
    
    @temperature.setter
    @remote_publisher(name="sensor/temperature")
    async def temperature(self, value: float):
        self._temperature = value
        # Decorator automatically publishes to topic

# Usage
sensor.temperature = 25.5  # Publishes {"temperature": 25.5}
```

---

## 🐛 Debugging & Troubleshooting

### Problem: "Descriptor not found"

```python
# Check what's registered
CallbackRegistry.debug_print()
```

**Output:**
```
============================================================
CALLBACK REGISTRY STATE
============================================================
[✓] my_module/calculate
[✗] my_module/process_data  ← Not bound yet!
[✓] my_module/status
============================================================
```

**Solution:** Check that decorator name matches descriptor name exactly.

### Problem: "Interface not created"

```python
# List unbound descriptors
unbound = CallbackRegistry.list_unbound(namespace="my_module")
print(f"Unbound interfaces: {unbound}")
```

**Solution:** 
1. Check decorator name matches descriptor name
2. Verify `bind_decorated_callbacks()` was called
3. Check callback signature (request, response=None for services)

### Problem: "ActionServer missing callback"

```python
descriptor = CallbackRegistry.get_descriptor("my_action", namespace="my_module")

if isinstance(descriptor, ActionDescriptor):
    print(f"on_goal bound: {descriptor.is_bound('on_goal')}")
    print(f"on_cancel bound: {descriptor.is_bound('on_cancel')}")
    print(f"execute bound: {descriptor.is_bound('execute')}")
    print(f"Fully bound: {descriptor.is_fully_bound()}")
```

**Solution:** Ensure all three decorators are present:
- `@remote_actionServer.on_goal(name="...")`
- `@remote_actionServer.on_cancel(name="...")`
- `@remote_actionServer.execute(name="...")`

### Problem: "Callback signature validation fails"

```python
# Example error
ValueError: Service callback 'calculate' must accept at least 1 parameter (request)
```

**Solution:** Check your callback signature:

```python
# ✅ CORRECT
async def calculate(self, request, response=None):
    ...

# ❌ WRONG - missing request parameter
async def calculate(self):
    ...

# ❌ WRONG - missing response parameter
async def calculate(self, request):
    ...  # Should have response=None as second param
```

---

## 📋 Checklist for Adding New Interface

- [ ] **Step 1:** Add metadata to `interface_metadata.json`
- [ ] **Step 2:** Create descriptor in `_base_.py` → `_create_base_interfaces()`
- [ ] **Step 3:** Add `@remote_*` decorator to method in `application.py`
- [ ] **Step 4:** Method name matches descriptor name exactly
- [ ] **Step 5:** Callback signature matches interface type:
  - Service: `async def name(self, request, response=None)`
  - Publisher: `async def name(self, message)`
  - Subscriber: `async def name(self, message)`
  - Action (3 methods):
    - `async def on_goal(self, goal_request) -> bool`
    - `async def on_cancel(self, goal_handle) -> bool`
    - `async def execute(self, goal_handle) -> dict`
- [ ] **Step 6:** Call `bind_decorated_callbacks()` in `interface.py`
- [ ] **Step 7:** Test with `ros2 service list` / `ros2 topic list` / `ros2 action list`

---

## 🎓 Advanced Topics

### Custom Descriptor Validation

```python
class CustomServiceDescriptor(ServiceDescriptor):
    def _validate_callback(self, callback: Callable) -> None:
        super()._validate_callback(callback)
        
        # Additional validation
        sig = inspect.signature(callback)
        return_annotation = sig.return_annotation
        
        if return_annotation is inspect.Signature.empty:
            logger.warning(
                f"Service '{self.name}' callback missing return type annotation"
            )
```

### Manual Descriptor Creation (Without JSON)

```python
# Useful for dynamic interfaces
descriptor = ServiceDescriptor(
    name=f"dynamic_service_{i}",
    protocols=[ProtocolType.ROS2],
    metadata={"dynamic": True},
    service_type=MyServiceType
)

CallbackRegistry.register_descriptor(descriptor, namespace="my_module")
```

### Pending Interface Background Processing

```python
# In entity.py - automatically processes pending interfaces
async def _process_pending_interfaces_loop(self):
    """Background task to create interfaces as callbacks become bound."""
    while True:
        try:
            await asyncio.sleep(1.0)
            results = await InterfaceFactory.process_pending_interfaces()
            
            if results:
                logger.info(f"✅ Processed {len(results)} pending interfaces")
        
        except asyncio.CancelledError:
            break
        except Exception as e:
            logger.error(f"Error processing pending: {e}")
```

---

## 📚 Related Documentation

- **Full Architecture:** [INTERFACE_DESCRIPTOR_ARCHITECTURE.md](./INTERFACE_DESCRIPTOR_ARCHITECTURE.md)
- **Implementation Plan:** [BLUEPRINT_REFACTORING_PLAN.md](../BLUEPRINT_REFACTORING_PLAN.md)
- **Migration Guide:** [MIGRATION_GUIDE.md](./MIGRATION_GUIDE.md) *(TODO)*
- **Decorator Guide:** [DECORATOR_GUIDE.md](./DECORATOR_GUIDE.md) *(TODO)*
- **API Reference:** [com/README.md](../src/vyra_base/com/README.md)

---

## 🎯 Summary

### Remember

1. **Descriptors = Definitions** (what should exist)
2. **Interfaces = Implementations** (actively running)
3. **Three Phases:** Register → Bind → Create
4. **Namespace Matters:** Always use your module name
5. **Names Must Match:** Decorator name = Descriptor name

### Quick Commands

```python
# Debug registry state
CallbackRegistry.debug_print()

# List unbound interfaces
CallbackRegistry.list_unbound(namespace="my_module")

# Check specific descriptor
descriptor = CallbackRegistry.get_descriptor("my_service", namespace="my_module")
print(f"Bound: {descriptor.is_bound()}")
```

### When Things Go Wrong

1. Check registry state: `CallbackRegistry.debug_print()`
2. Verify names match exactly
3. Check callback signature
4. Look at logs for binding errors
5. Ask in #vyra-dev Discord channel

---

**Happy Coding!** 🚀
