# VYRA Interface Descriptor Architecture

**Version:** 2.0  
**Date:** February 17, 2026  
**Status:** Design Complete, Implementation Pending

---

## 🎯 Overview

Das **Interface Descriptor System** ermöglicht **Two-Phase Late Binding** für Kommunikationsinterfaces in VYRA. Interfaces werden als **Descriptors** (Beschreibungen) registriert, bevor ihre Implementierungen (Callbacks) verfügbar sind.

### Key Benefits

- ✅ **Late Binding:** Callbacks können nach Interface-Registration gebunden werden
- ✅ **Testability:** Mock-Callbacks ohne vollständige Component-Struktur
- ✅ **Separation of Concerns:** Definition (was) vs. Implementation (wie)
- ✅ **Protocol-Agnostic:** Descriptors funktionieren für ROS2, Zenoh, Redis, etc.
- ✅ **Multi-Callback Support:** ActionServer mit on_goal/on_cancel/execute

---

## 🏗️ Architecture Principles

### 1. Three-Phase Pattern

```
┌─────────────────────────────────────────────────────────────┐
│ PHASE 1: DESCRIPTOR REGISTRATION                           │
│ (At Entity/Module Init, from JSON metadata or decorators)  │
└─────────────────────────────────────────────────────────────┘
                           │
          ┌────────────────┼────────────────┐
          │                │                │
          v                v                v
    JSON Metadata    @remote_service    Manual Creation
          │                │                │
          └────────────────┼────────────────┘
                           │
                           v
              ┌────────────────────────────┐
              │ ServiceDescriptor          │
              │ - name: "calculate"        │
              │ - protocols: [ROS2]        │
              │ - callback: None ← UNBOUND │
              └────────────────────────────┘
                           │
                           v
              ┌────────────────────────────┐
              │ CallbackRegistry           │
              │ .register_descriptor()     │
              │ namespace="v2_modulemanager"│
              └────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│ PHASE 2: CALLBACK BINDING                                  │
│ (At Component Init, from decorated methods)                │
└─────────────────────────────────────────────────────────────┘
                           │
                Component Instance Created
                with @remote_service methods
                           │
                           v
              ┌────────────────────────────┐
              │ get_decorated_methods()    │
              │ Returns: {servers: [...]}  │
              └────────────────────────────┘
                           │
                           v
              ┌────────────────────────────┐
              │ bind_decorated_callbacks() │
              │ Matches by name + namespace│
              └────────────────────────────┘
                           │
                           v
              descriptor.bind_callback(method)
              descriptor.is_bound() == True ✅

┌─────────────────────────────────────────────────────────────┐
│ PHASE 3: INTERFACE CREATION                                │
│ (Immediate or deferred via pending queue)                  │
└─────────────────────────────────────────────────────────────┘
                           │
                           v
              ┌────────────────────────────┐
              │ InterfaceFactory           │
              │ .create_from_descriptor()  │
              └────────────────────────────┘
                           │
          ┌────────────────┴───────────────┐
          │                                │
          v                                v
    Callback Bound?                  No Callback?
          │                                │
          v                                v
┌─────────────────────┐         ┌─────────────────────┐
│ Create Interface    │         │ Add to Pending      │
│ - VyraServer        │         │ Background task     │
│ - VyraPublisher     │         │ retries when bound  │
│ - VyraActionServer  │         │                     │
└─────────────────────┘         └─────────────────────┘
          │
          v
  ✅ Interface Active (in InterfaceRegistry)
```

### 2. Class Hierarchy

```
InterfaceDescriptor (ABC)
  ├── name: str
  ├── interface_type: InterfaceType
  ├── protocols: List[ProtocolType]
  ├── metadata: Dict[str, Any]
  └── _callback: Optional[Callable]
       │
       ├── bind_callback(callback)
       ├── unbind_callback()
       ├── is_bound() → bool
       └── _validate_callback(callback)  [abstract]
          │
          ├─── ServiceDescriptor
          │    ├── service_type: Type
          │    └── _validate_callback() → checks (request, response=None)
          │
          ├─── PublisherDescriptor
          │    ├── message_type: Type
          │    └── _validate_callback() → checks (message)
          │
          ├─── SubscriberDescriptor
          │    ├── message_type: Type
          │    └── _validate_callback() → checks (message)
          │
          └─── ActionDescriptor ⭐ Multi-Callback
               ├── action_type: Type
               ├── _callbacks: Dict[str, Callable]
               │    ├── 'on_goal': Optional[Callable]
               │    ├── 'on_cancel': Optional[Callable]
               │    └── 'execute': Optional[Callable]
               │
               ├── bind_callback(callback, callback_type='execute')
               ├── bind_callbacks(**callbacks)
               ├── get_callback(callback_type) → Optional[Callable]
               ├── is_bound(callback_type=None) → bool
               ├── is_fully_bound() → bool
               └── _validate_callback(callback, callback_type)
```

### 3. Descriptor vs Transport Classes

**Key Concept:** Descriptors beschreiben, Transport-Klassen implementieren.

| Aspect | InterfaceDescriptor | VyraTransport (types.py) |
|--------|---------------------|--------------------------|
| **Purpose** | Interface-**Definition** (was) | Interface-**Implementation** (wie) |
| **Location** | `com/core/` | `com/core/types.py` + `com/transport/` |
| **State** | Metadata, Callbacks (optional) | Active connection, protocol-specific handle |
| **Lifecycle** | Created early, bound later | Created when fully configured |
| **Protocol** | Protocol-agnostic | Protocol-specific (VyraServerImpl, etc.) |
| **Callbacks** | Stores references | Executes callbacks |

**Example:**

```python
# PHASE 1: Create Descriptor (protocol-agnostic)
descriptor = ServiceDescriptor(
    name="calculate",
    protocols=[ProtocolType.ROS2, ProtocolType.ZENOH],
    metadata={"qos": 10}
)
# At this point: No callback, no connection, just metadata

# PHASE 2: Bind Callback
async def calculate(request, response=None):
    return {"result": request["x"] + request["y"]}

descriptor.bind_callback(calculate)

# PHASE 3: Create Transport Interface (protocol-specific)
interface: VyraServer = await InterfaceFactory.create_from_descriptor(
    descriptor,
    node=ros2_node  # Protocol-specific dependency
)
# Now: ROS2 Service is running, connected, ready to handle requests
```

---

## 📦 Core Components

### 1. InterfaceDescriptor (ABC)

**File:** `com/core/interface_descriptors.py`

```python
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Callable, Optional, List, Dict, Any

@dataclass
class InterfaceDescriptor(ABC):
    """
    Abstract base descriptor for all communication interfaces.
    
    Represents the *definition* of an interface separate from its implementation.
    """
    
    name: str
    interface_type: InterfaceType
    protocols: List[ProtocolType] = field(default_factory=list)
    metadata: Dict[str, Any] = field(default_factory=dict)
    _callback: Optional[Callable] = field(default=None, repr=False)
    
    @property
    def callback(self) -> Optional[Callable]:
        return self._callback
    
    def is_bound(self) -> bool:
        """Check if callback has been bound."""
        return self._callback is not None
    
    def bind_callback(self, callback: Callable) -> None:
        """Bind a callback implementation."""
        if self.is_bound():
            raise RuntimeError(f"Descriptor '{self.name}' already bound")
        self._validate_callback(callback)
        self._callback = callback
    
    def unbind_callback(self) -> Optional[Callable]:
        """Remove callback binding."""
        old = self._callback
        self._callback = None
        return old
    
    @abstractmethod
    def _validate_callback(self, callback: Callable) -> None:
        """Validate callback signature (implemented by subclasses)."""
        pass
```

**Design Principles:**
- **Immutable Metadata:** name, type, protocols set at creation
- **Late Binding:** _callback starts as None
- **Validation:** Each subclass validates its callback signature
- **Single Responsibility:** Only stores interface definition

### 2. ServiceDescriptor

**For:** Request-Response communication (ROS2 Service, gRPC, etc.)

```python
class ServiceDescriptor(InterfaceDescriptor):
    """
    Descriptor for service (request-response) interfaces.
    
    Expected callback:
        async def handler(request, response=None) -> dict | Response
    """
    
    def __init__(
        self,
        name: str,
        protocols: Optional[List[ProtocolType]] = None,
        metadata: Optional[Dict[str, Any]] = None,
        service_type: Optional[Any] = None
    ):
        super().__init__(
            name=name,
            interface_type=InterfaceType.SERVER,
            protocols=protocols or [],
            metadata=metadata or {}
        )
        self.service_type = service_type  # ROS2 service type or schema
    
    def _validate_callback(self, callback: Callable) -> None:
        sig = inspect.signature(callback)
        params = list(sig.parameters.keys())
        if params and params[0] == 'self':
            params = params[1:]
        
        if len(params) < 1:
            raise ValueError(
                f"Service callback must accept at least 1 parameter (request)"
            )
```

**Usage:**

```python
# Create descriptor
descriptor = ServiceDescriptor(
    name="calculate",
    protocols=[ProtocolType.ROS2],
    service_type=CalculateService
)

# Register in registry
CallbackRegistry.register_descriptor(descriptor, namespace="my_module")

# Later: Bind callback
@remote_service(name="calculate")
async def calculate(self, request, response=None):
    return {"result": request.x + request.y}

# Automatic binding via decorator system
bind_decorated_callbacks(component, namespace="my_module")
```

### 3. PublisherDescriptor

**For:** Publish-only communication (no callbacks)

```python
class PublisherDescriptor(InterfaceDescriptor):
    """
    Descriptor for publisher (one-way) interfaces.
    
    Note: Publishers don't have callbacks - they send data only.
    The callback here is for the publish() wrapper method.
    """
    
    def __init__(
        self,
        name: str,
        protocols: Optional[List[ProtocolType]] = None,
        metadata: Optional[Dict[str, Any]] = None,
        message_type: Optional[Any] = None
    ):
        super().__init__(
            name=name,
            interface_type=InterfaceType.PUBLISHER,
            protocols=protocols or [],
            metadata=metadata or {}
        )
        self.message_type = message_type
    
    def _validate_callback(self, callback: Callable) -> None:
        # Publisher callbacks (if any) should accept message parameter
        sig = inspect.signature(callback)
        params = list(sig.parameters.keys())
        if params and params[0] == 'self':
            params = params[1:]
        
        if len(params) < 1:
            raise ValueError(
                f"Publisher callback must accept at least 1 parameter (message)"
            )
```

### 4. SubscriberDescriptor

**For:** Subscribe with callback

```python
class SubscriberDescriptor(InterfaceDescriptor):
    """
    Descriptor for subscriber interfaces.
    
    Expected callback:
        async def handler(message) -> None
    """
    
    def __init__(
        self,
        name: str,
        protocols: Optional[List[ProtocolType]] = None,
        metadata: Optional[Dict[str, Any]] = None,
        message_type: Optional[Any] = None
    ):
        super().__init__(
            name=name,
            interface_type=InterfaceType.SUBSCRIBER,
            protocols=protocols or [],
            metadata=metadata or {}
        )
        self.message_type = message_type
    
    def _validate_callback(self, callback: Callable) -> None:
        sig = inspect.signature(callback)
        params = list(sig.parameters.keys())
        if params and params[0] == 'self':
            params = params[1:]
        
        if len(params) < 1:
            raise ValueError(
                f"Subscriber callback must accept at least 1 parameter (message)"
            )
```

### 5. ActionDescriptor ⭐ Multi-Callback

**For:** Long-running tasks with goal/cancel/execute lifecycle

**Key Feature:** Multiple callbacks für komplexe Action-Lifecycle.

```python
class ActionDescriptor(InterfaceDescriptor):
    """
    Descriptor for ActionServer with multiple lifecycle callbacks.
    
    Required callbacks:
    - execute: Main execution (required)
    - on_goal: Accept/reject goal (optional, default: accept)
    - on_cancel: Accept/reject cancel (optional, default: accept)
    """
    
    def __init__(
        self,
        name: str,
        protocols: Optional[List[ProtocolType]] = None,
        metadata: Optional[Dict[str, Any]] = None,
        action_type: Optional[Any] = None
    ):
        super().__init__(
            name=name,
            interface_type=InterfaceType.ACTION_SERVER,
            protocols=protocols or [],
            metadata=metadata or {}
        )
        self.action_type = action_type
        
        # Multi-callback storage
        self._callbacks: Dict[str, Optional[Callable]] = {
            'on_goal': None,
            'on_cancel': None,
            'execute': None
        }
    
    # Override parent's callback property
    @property
    def callback(self) -> Optional[Callable]:
        """Returns 'execute' callback for backward compatibility."""
        return self._callbacks.get('execute')
    
    def bind_callback(
        self, 
        callback: Callable, 
        callback_type: str = 'execute'
    ) -> None:
        """
        Bind a specific callback type.
        
        Args:
            callback: Function to bind
            callback_type: One of 'on_goal', 'on_cancel', 'execute'
        """
        if callback_type not in self._callbacks:
            raise ValueError(
                f"Invalid callback_type '{callback_type}'. "
                f"Must be one of: {list(self._callbacks.keys())}"
            )
        
        if self._callbacks[callback_type] is not None:
            raise RuntimeError(
                f"Callback '{callback_type}' already bound for '{self.name}'"
            )
        
        self._validate_callback(callback, callback_type)
        self._callbacks[callback_type] = callback
        logger.debug(
            f"✅ Bound '{callback_type}' to ActionDescriptor '{self.name}'"
        )
    
    def bind_callbacks(self, **callbacks) -> None:
        """
        Bind multiple callbacks at once.
        
        Example:
            descriptor.bind_callbacks(
                on_goal=handle_goal,
                on_cancel=handle_cancel,
                execute=execute_task
            )
        """
        for callback_type, callback in callbacks.items():
            if callback is not None:
                self.bind_callback(callback, callback_type)
    
    def get_callback(self, callback_type: str) -> Optional[Callable]:
        """Get specific callback by type."""
        return self._callbacks.get(callback_type)
    
    def is_bound(self, callback_type: Optional[str] = None) -> bool:
        """
        Check if callbacks are bound.
        
        Args:
            callback_type: Check specific callback, or None for "required bound"
        
        Returns:
            bool: If callback_type given, checks that one.
                  If None, checks if 'execute' (required) is bound.
        """
        if callback_type:
            return self._callbacks.get(callback_type) is not None
        
        # 'execute' is required
        return self._callbacks['execute'] is not None
    
    def is_fully_bound(self) -> bool:
        """Check if ALL callbacks are bound."""
        return all(cb is not None for cb in self._callbacks.values())
    
    def unbind_callback(self, callback_type: str = 'execute') -> Optional[Callable]:
        """Remove specific callback binding."""
        old_callback = self._callbacks.get(callback_type)
        if callback_type in self._callbacks:
            self._callbacks[callback_type] = None
        return old_callback
    
    def _validate_callback(
        self, 
        callback: Callable, 
        callback_type: str
    ) -> None:
        """Validate callback signature based on type."""
        sig = inspect.signature(callback)
        params = list(sig.parameters.keys())
        
        if params and params[0] == 'self':
            params = params[1:]
        
        # All action callbacks require at least 1 parameter
        if len(params) < 1:
            raise ValueError(
                f"ActionServer '{callback_type}' callback must accept "
                f"at least 1 parameter. Got: {params}"
            )
        
        # Type-specific validation
        if callback_type == 'on_goal':
            # Should accept goal_request
            pass
        elif callback_type == 'on_cancel':
            # Should accept goal_handle
            pass
        elif callback_type == 'execute':
            # Should accept goal_handle
            pass
```

**Usage Examples:**

```python
# METHOD 1: Separate decorators (recommended)
@remote_actionServer.on_goal(name="process_batch")
async def accept_goal(self, goal_request):
    return goal_request.count <= 100

@remote_actionServer.on_cancel(name="process_batch")
async def cancel_goal(self, goal_handle):
    return True

@remote_actionServer.execute(name="process_batch")
async def execute_batch(self, goal_handle):
    for i in range(10):
        goal_handle.publish_feedback({"progress": i})
    return {"processed": 10}

# METHOD 2: Manual binding
descriptor = ActionDescriptor(name="process_batch")
descriptor.bind_callbacks(
    on_goal=accept_goal,
    on_cancel=cancel_goal,
    execute=execute_batch
)

# METHOD 3: Class-based handler (optional)
class BatchHandler(IActionGoalHandler):
    async def on_goal(self, goal_request):
        return True
    
    async def on_cancel(self, goal_handle):
        return True
    
    async def execute(self, goal_handle):
        return {"result": "success"}

handler = BatchHandler()
descriptor.bind_callbacks(
    on_goal=handler.on_goal,
    on_cancel=handler.on_cancel,
    execute=handler.execute
)
```

---

## 🔧 Supporting Components

### CallbackRegistry

**File:** `com/core/callback_registry.py`

Thread-safe singleton for managing descriptors across modules.

```python
class CallbackRegistry:
    """
    Global registry for interface descriptors.
    
    Features:
    - Per-module namespacing
    - Thread-safe operations
    - Late binding support
    - Query & debugging tools
    """
    
    _instance = None
    _lock = RLock()
    
    def __init__(self):
        self._descriptors: Dict[str, InterfaceDescriptor] = {}
    
    @classmethod
    def register_descriptor(
        cls,
        descriptor: InterfaceDescriptor,
        namespace: str = "global"
    ) -> None:
        """Register a descriptor in a namespace."""
        full_name = f"{namespace}/{descriptor.name}"
        registry = cls()
        
        with cls._lock:
            registry._descriptors[full_name] = descriptor
    
    @classmethod
    def get_descriptor(
        cls,
        name: str,
        namespace: str = "global"
    ) -> Optional[InterfaceDescriptor]:
        """Retrieve descriptor by name."""
        full_name = f"{namespace}/{name}"
        registry = cls()
        with cls._lock:
            return registry._descriptors.get(full_name)
    
    @classmethod
    def bind_callback(
        cls,
        name: str,
        callback: Callable,
        callback_type: str = 'default',  # For ActionDescriptor
        namespace: str = "global"
    ) -> bool:
        """
        Bind callback to registered descriptor.
        
        Args:
            name: Interface name
            callback: Function to bind
            callback_type: For ActionDescriptor multi-callback
            namespace: Module namespace
        
        Returns:
            bool: True if binding successful
        """
        descriptor = cls.get_descriptor(name, namespace)
        if not descriptor:
            logger.warning(f"Descriptor '{name}' not found in '{namespace}'")
            return False
        
        try:
            if isinstance(descriptor, ActionDescriptor) and callback_type != 'default':
                descriptor.bind_callback(callback, callback_type)
            else:
                descriptor.bind_callback(callback)
            return True
        except Exception as e:
            logger.error(f"Failed to bind callback: {e}")
            return False
    
    @classmethod
    def list_unbound(cls, namespace: str = "global") -> List[str]:
        """List all descriptors without bound callbacks."""
        registry = cls()
        unbound = []
        
        with cls._lock:
            for full_name, descriptor in registry._descriptors.items():
                if full_name.startswith(f"{namespace}/"):
                    if not descriptor.is_bound():
                        unbound.append(descriptor.name)
        
        return unbound
    
    @classmethod
    def list_bound(cls, namespace: str = "global") -> List[str]:
        """List all descriptors with bound callbacks."""
        registry = cls()
        bound = []
        
        with cls._lock:
            for full_name, descriptor in registry._descriptors.items():
                if full_name.startswith(f"{namespace}/"):
                    if descriptor.is_bound():
                        bound.append(descriptor.name)
        
        return bound
    
    @classmethod
    def debug_print(cls) -> None:
        """Print registry state for debugging."""
        registry = cls()
        print("\n" + "="*60)
        print("CALLBACK REGISTRY STATE")
        print("="*60)
        
        with cls._lock:
            for full_name, descriptor in sorted(registry._descriptors.items()):
                bound_status = "✓" if descriptor.is_bound() else "✗"
                
                # For ActionDescriptor, show multi-callback status
                if isinstance(descriptor, ActionDescriptor):
                    callbacks_status = {
                        k: ("✓" if v else "✗")
                        for k, v in descriptor._callbacks.items()
                    }
                    print(
                        f"[{bound_status}] {full_name} "
                        f"(Action: {callbacks_status})"
                    )
                else:
                    print(f"[{bound_status}] {full_name}")
        
        print("="*60 + "\n")
```

### InterfaceFactory

**File:** `com/core/factory.py`

Creates active interfaces from descriptors with protocol fallback.

```python
class InterfaceFactory:
    """
    Factory for creating interfaces from descriptors.
    
    Handles:
    - Protocol fallback chain
    - Pending interface queue
    - Protocol availability checking
    """
    
    _pending_interfaces: Dict[str, InterfaceDescriptor] = {}
    
    @classmethod
    async def create_from_descriptor(
        cls,
        descriptor: InterfaceDescriptor,
        **kwargs
    ) -> Optional[Union[VyraServer, VyraPublisher, VyraActionServer]]:
        """
        Create active interface from descriptor.
        
        Args:
            descriptor: InterfaceDescriptor to realize
            **kwargs: Protocol-specific parameters (node, provider, etc.)
        
        Returns:
            Active interface or None if callback not bound yet
        """
        # Check if callback is bound
        if not descriptor.is_bound():
            logger.info(
                f"Descriptor '{descriptor.name}' not bound, adding to pending"
            )
            cls._pending_interfaces[descriptor.name] = descriptor
            return None
        
        # Determine interface type
        if descriptor.interface_type == InterfaceType.SERVER:
            return await cls._create_server_from_descriptor(descriptor, **kwargs)
        
        elif descriptor.interface_type == InterfaceType.PUBLISHER:
            return await cls._create_publisher_from_descriptor(descriptor, **kwargs)
        
        elif descriptor.interface_type == InterfaceType.SUBSCRIBER:
            return await cls._create_subscriber_from_descriptor(descriptor, **kwargs)
        
        elif descriptor.interface_type == InterfaceType.ACTION_SERVER:
            return await cls._create_action_server_from_descriptor(descriptor, **kwargs)
        
        else:
            raise InterfaceError(
                f"Unsupported interface type: {descriptor.interface_type}"
            )
    
    @classmethod
    async def _create_action_server_from_descriptor(
        cls,
        descriptor: ActionDescriptor,
        **kwargs
    ) -> Optional[VyraActionServer]:
        """
        Create ActionServer from multi-callback descriptor.
        
        Extracts on_goal, on_cancel, execute callbacks and passes to factory.
        """
        if not isinstance(descriptor, ActionDescriptor):
            raise TypeError("Expected ActionDescriptor for ACTION_SERVER")
        
        # Extract callbacks
        on_goal_cb = descriptor.get_callback('on_goal')
        on_cancel_cb = descriptor.get_callback('on_cancel')
        execute_cb = descriptor.get_callback('execute')
        
        if not execute_cb:
            logger.warning(
                f"ActionDescriptor '{descriptor.name}' missing 'execute' callback"
            )
            return None
        
        # Protocol fallback
        for protocol in descriptor.protocols or [ProtocolType.ROS2]:
            try:
                provider = ProviderRegistry.get_provider(protocol)
                
                action_server = await provider.create_action_server(
                    name=descriptor.name,
                    action_type=descriptor.action_type,
                    handle_goal_request=on_goal_cb,
                    handle_cancel_request=on_cancel_cb,
                    execution_callback=execute_cb,
                    **kwargs
                )
                
                if action_server:
                    logger.info(
                        f"✅ Created ActionServer '{descriptor.name}' "
                        f"via {protocol.value}"
                    )
                    return action_server
            
            except Exception as e:
                logger.debug(
                    f"Protocol {protocol.value} unavailable: {e}, trying next..."
                )
        
        raise ProtocolUnavailableError(
            f"No protocol available for ActionServer '{descriptor.name}'"
        )
    
    @classmethod
    async def process_pending_interfaces(cls) -> Dict[str, bool]:
        """
        Attempt to create interfaces from pending queue.
        
        Called by background task in Entity.
        
        Returns:
            Dict mapping interface names to creation success
        """
        results = {}
        pending_copy = list(cls._pending_interfaces.items())
        
        for name, descriptor in pending_copy:
            if descriptor.is_bound():
                try:
                    interface = await cls.create_from_descriptor(descriptor)
                    if interface:
                        del cls._pending_interfaces[name]
                        results[name] = True
                        logger.info(f"✅ Created pending interface '{name}'")
                    else:
                        results[name] = False
                except Exception as e:
                    logger.error(f"Failed to create pending interface '{name}': {e}")
                    results[name] = False
        
        return results
```

---

## 🎨 Decorator System

### Current Decorators

**File:** `com/core/decorators.py`

```python
# SERVICE (Request-Response)
@remote_service(name="calculate", namespace="my_module")
async def calculate(self, request, response=None):
    return {"result": request.x + request.y}

# PUBLISHER (One-way)
@remote_publisher(name="status", namespace="my_module")
async def publish_status(self, message):
    pass  # Publishing handled by decorator

# SUBSCRIBER (Receive messages)
@remote_subscriber(name="updates", namespace="my_module")
async def on_update(self, message):
    logger.info(f"Received: {message}")

# ACTION SERVER (Multi-callback) - NEW!
@remote_actionServer.on_goal(name="process_batch", namespace="my_module")
async def accept_goal(self, goal_request):
    return goal_request.count <= 100

@remote_actionServer.on_cancel(name="process_batch", namespace="my_module")
async def cancel_batch(self, goal_handle):
    return True

@remote_actionServer.execute(name="process_batch", namespace="my_module")
async def execute_batch(self, goal_handle):
    for i in range(goal_request.count):
        goal_handle.publish_feedback({"progress": i})
    return {"processed": goal_request.count}
```

### Helper Functions

```python
def get_decorated_methods(obj: Any) -> Dict[str, List[Callable]]:
    """
    Extract all decorated methods from object.
    
    Returns:
        {
            "servers": [method1, method2, ...],
            "publishers": [...],
            "subscribers": [...],
            "actions": [...]  # ActionDescriptor info with callback_types
        }
    """
    ...

def bind_decorated_callbacks(
    obj: Any,
    namespace: str = "global"
) -> Dict[str, bool]:
    """
    Bind all decorated methods to registered descriptors.
    
    For ActionServer methods, binds by callback_type.
    
    Returns:
        Dict mapping interface names to binding success
    """
    ...
```

---

## 📖 Usage Examples

### Example 1: Simple Service

```python
# PHASE 1: Register descriptor (in _base_.py)
async def _create_base_interfaces():
    descriptor = ServiceDescriptor(
        name="calculate",
        protocols=[ProtocolType.ROS2],
        service_type=CalculateService
    )
    
    CallbackRegistry.register_descriptor(
        descriptor,
        namespace="v2_modulemanager"
    )
    
    return [descriptor]

# PHASE 2: Define callback (in application.py)
class Application:
    @remote_service(name="calculate")
    async def calculate(self, request, response=None):
        return {"result": request.x + request.y}

# PHASE 3: Bind & create (in interface.py)
async def auto_register_interfaces(entity, component):
    # Bind decorated methods to descriptors
    bind_decorated_callbacks(component, namespace="v2_modulemanager")
    
    # Create interfaces from bound descriptors
    for name in CallbackRegistry.list_bound(namespace="v2_modulemanager"):
        descriptor = CallbackRegistry.get_descriptor(name, namespace="v2_modulemanager")
        interface = await InterfaceFactory.create_from_descriptor(
            descriptor,
            node=entity.node
        )
```

### Example 2: ActionServer with Multi-Callback

```python
# PHASE 1: Register descriptor
descriptor = ActionDescriptor(
    name="process_batch",
    protocols=[ProtocolType.ROS2],
    action_type=ProcessBatchAction
)

CallbackRegistry.register_descriptor(descriptor, namespace="my_module")

# PHASE 2: Define callbacks
class BatchProcessor:
    @remote_actionServer.on_goal(name="process_batch")
    async def on_goal(self, goal_request):
        if goal_request.count > 1000:
            logger.warning("Rejecting large batch")
            return False
        return True
    
    @remote_actionServer.on_cancel(name="process_batch")
    async def on_cancel(self, goal_handle):
        logger.info("Cancel requested")
        return True
    
    @remote_actionServer.execute(name="process_batch")
    async def execute(self, goal_handle):
        count = goal_handle.goal.count
        
        for i in range(count):
            if goal_handle.is_cancel_requested():
                goal_handle.canceled()
                return {"processed": i, "status": "canceled"}
            
            # Do work
            process_item(i)
            
            # Send feedback
            goal_handle.publish_feedback({"progress": i + 1, "total": count})
        
        goal_handle.succeed()
        return {"processed": count, "status": "success"}

# PHASE 3: Bind & create
processor = BatchProcessor()
bind_decorated_callbacks(processor, namespace="my_module")

descriptor = CallbackRegistry.get_descriptor("process_batch", namespace="my_module")
action_server = await InterfaceFactory.create_from_descriptor(
    descriptor,
    node=ros2_node
)
```

### Example 3: Property Setter Auto-Publishing

```python
class SensorComponent:
    def __init__(self):
        self._temperature = 0.0
        
        # Descriptor registered in registry
        # Publisher interface created at entity init
    
    @property
    def temperature(self) -> float:
        return self._temperature
    
    @temperature.setter
    @remote_publisher(name="sensor/temperature")
    async def temperature(self, value: float):
        self._temperature = value
        # Decorator automatically publishes to topic

# Usage:
sensor = SensorComponent()
sensor.temperature = 25.5  # Automatically published to "sensor/temperature"
```

---

## 🔄 Migration from Old System

### Old System (DEPRECATED)

```python
# OLD: Direct interface creation with callback
from vyra_base.com import InterfaceFactory

interface = await InterfaceFactory.create_server(
    name="calculate",
    response_callback=my_calculate_function,  # ← Must be available NOW
    protocols=[ProtocolType.ROS2],
    service_type=CalculateService,
    node=ros2_node
)
```

**Problems:**
- ❌ Callback must exist at interface creation
- ❌ No separation of definition and implementation
- ❌ Hard to test without full component
- ❌ No late binding support

### New System (CURRENT)

```python
# NEW: Two-phase with descriptor
from vyra_base.com import ServiceDescriptor, CallbackRegistry, InterfaceFactory

# PHASE 1: Define interface (early, from JSON)
descriptor = ServiceDescriptor(
    name="calculate",
    protocols=[ProtocolType.ROS2],
    service_type=CalculateService
)
CallbackRegistry.register_descriptor(descriptor, namespace="my_module")

# PHASE 2: Bind callback (later, when component ready)
@remote_service(name="calculate")
async def calculate(self, request, response=None):
    return {"result": request.x + request.y}

bind_decorated_callbacks(component, namespace="my_module")

# PHASE 3: Create interface (automatic or manual)
interface = await InterfaceFactory.create_from_descriptor(
    descriptor,
    node=ros2_node
)
```

**Benefits:**
- ✅ Callbacks can be bound late
- ✅ Clear separation of concerns
- ✅ Easy to mock for testing
- ✅ Pending queue for delayed binding

---

## 🧪 Testing Strategy

### Unit Tests

```python
# test_interface_descriptors.py
def test_service_descriptor_creation():
    descriptor = ServiceDescriptor(name="test", protocols=[])
    assert descriptor.name == "test"
    assert not descriptor.is_bound()

def test_service_descriptor_binding():
    descriptor = ServiceDescriptor(name="test", protocols=[])
    
    async def callback(request, response=None):
        return {"result": 42}
    
    descriptor.bind_callback(callback)
    assert descriptor.is_bound()
    assert descriptor.callback == callback

def test_action_descriptor_multi_callback():
    descriptor = ActionDescriptor(name="test", protocols=[])
    
    async def on_goal(goal_request):
        return True
    
    async def execute(goal_handle):
        return {"result": "success"}
    
    descriptor.bind_callbacks(on_goal=on_goal, execute=execute)
    
    assert descriptor.is_bound()  # execute is bound
    assert descriptor.is_bound('on_goal')
    assert not descriptor.is_fully_bound()  # on_cancel missing
```

### Integration Tests

```python
# test_late_binding_integration.py
async def test_two_phase_late_binding():
    # PHASE 1: Register descriptor
    descriptor = ServiceDescriptor(name="test_service", protocols=[])
    CallbackRegistry.register_descriptor(descriptor, namespace="test")
    
    # Verify not bound yet
    assert not descriptor.is_bound()
    
    # PHASE 2: Bind callback
    async def callback(request, response=None):
        return {"result": request["x"] * 2}
    
    descriptor.bind_callback(callback)
    assert descriptor.is_bound()
    
    # PHASE 3: Create interface
    interface = await InterfaceFactory.create_from_descriptor(
        descriptor,
        node=mock_ros2_node
    )
    
    assert interface is not None
    assert interface.name == "test_service"
```

---

## 📊 Performance Considerations

### Memory Overhead

- **Descriptor Size:** ~200-300 bytes per descriptor (metadata + callback ref)
- **Registry Size:** O(n) where n = number of interfaces across all modules
- **Typical Module:** 10-50 descriptors = ~5-15 KB overhead

### Time Complexity

| Operation | Complexity | Notes |
|-----------|-----------|-------|
| Register descriptor | O(1) | Dict insertion |
| Get descriptor | O(1) | Dict lookup |
| Bind callback | O(1) | Direct assignment |
| List unbound | O(n) | Linear scan, but filtered by namespace |
| Create from descriptor | O(p) | p = number of protocols in fallback chain |

### Best Practices

1. **Pre-register descriptors** during module initialization
2. **Batch bind callbacks** after component creation (not one-by-one)
3. **Use namespaces** to scope descriptors per module (improves list_unbound performance)
4. **Background processing** for pending interfaces (non-blocking)

---

## 🎯 Summary

### Key Principles

1. **Descriptors = Definitions** (what interfaces should exist)
2. **Transport = Implementations** (how interfaces work)
3. **Three-Phase Pattern** (register → bind → create)
4. **Late Binding** (callbacks can be added after registration)
5. **Multi-Callback Support** (ActionServer with on_goal/on_cancel/execute)

### When to Use What

| Use Case | Use This |
|----------|---------|
| Define interface from JSON | ServiceDescriptor, ActionDescriptor, etc. |
| Bind callback from decorator | @remote_service, @remote_actionServer.execute |
| Create active interface | InterfaceFactory.create_from_descriptor() |
| Query unbound interfaces | CallbackRegistry.list_unbound(namespace) |
| Debug binding issues | CallbackRegistry.debug_print() |

### Migration Checklist

- [ ] Replace `create_server()` with `ServiceDescriptor` + `create_from_descriptor()`
- [ ] Replace `create_publisher()` with `PublisherDescriptor` + `create_from_descriptor()`
- [ ] Update ActionServer to use multi-callback pattern
- [ ] Add ActionStatus to action implementations
- [ ] Move examples to `docs/examples/`
- [ ] Update module `_base_.py` to create descriptors
- [ ] Update module `interface.py` to bind callbacks
- [ ] Add unit tests for descriptors
- [ ] Add integration tests for late binding

---

**Next Steps:** See [BLUEPRINT_REFACTORING_PLAN.md](../BLUEPRINT_REFACTORING_PLAN.md) for implementation roadmap.
