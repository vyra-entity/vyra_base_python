# VYRA Interface Descriptor System - Comprehensive Improvement Plan

**Datum:** 17. Februar 2026  
**Status:** 🎯 Planning Phase  
**Context:** Based on analysis of REFACTORING_STATUS.md and current implementation

---

## 🔍 Executive Summary

Nach Analyse der aktuellen Blueprint-Architektur wurden folgende Hauptprobleme identifiziert:

| Problem | Current State | Proposed Solution | Priority |
|---------|--------------|-------------------|----------|
| **Naming** | "Blueprint" ist missverständlich | Rename zu "InterfaceDescriptor" | 🔴 HIGH |
| **ActionServer Callbacks** | Nur single callback support | Multi-callback pattern mit on_goal/on_cancel/execute | 🔴 HIGH |
| **ActionStatus** | Fehlt komplett | Enum in types.py hinzufügen | 🔴 HIGH |
| **Folder Structure** | Korrekt (core/ für definitions) | Behalten | ✅ OK |
| **Examples Location** | In core/examples/ | Nach docs/examples/ verschieben | 🟡 MEDIUM |
| **Abstract Interfaces** | Fehlen für strukturiertes Handling | Optional: IGoalHandler, IServiceHandler ABCs | 🟢 LOW |

---

## 🏗️ Architecture Analysis

### Current System Structure

```
vyra_base/com/
├── core/                           # Protocol-agnostic definitions ✅ CORRECT
│   ├── blueprints.py              # Interface descriptors (RENAME NEEDED)
│   ├── callback_registry.py       # Registriert blueprints
│   ├── decorators.py              # @remote_service, @remote_publisher, etc.
│   ├── factory.py                 # Creates interfaces from blueprints
│   ├── types.py                   # VyraTransport, VyraServer, etc.
│   └── examples/                  # ❌ MOVE to docs/examples/
│
├── transport/                      # Protocol-specific implementations ✅ CORRECT
│   ├── registry.py                # InterfaceRegistry (active interfaces)
│   ├── t_ros2/                    # ROS2 implementations (VyraServerImpl)
│   ├── t_zenoh/                   # Zenoh implementations
│   ├── t_uds/                     # Unix Domain Socket implementations
│   └── t_redis/                   # (Future) Redis implementations
│
├── external/                       # External protocol integrations
├── industrial/                     # Industrial protocols (Modbus, OPC UA)
└── ...
```

### Two-Phase Late Binding Pattern

```
┌──────────────────────────────────────────────────────────────────┐
│ PHASE 1: DESCRIPTOR REGISTRATION (At Module Init, from JSON)    │
└──────────────────────────────────────────────────────────────────┘
                              │
                              v
                ┌─────────────────────────────┐
                │ InterfaceDescriptor         │
                │ - name: "calculate"         │
                │ - protocols: [ROS2, Zenoh]  │
                │ - callbacks: {}  ← EMPTY    │
                └─────────────────────────────┘
                              │
                              v
                ┌─────────────────────────────┐
                │ CallbackRegistry            │
                │ .register_descriptor()      │
                └─────────────────────────────┘

┌──────────────────────────────────────────────────────────────────┐
│ PHASE 2: CALLBACK BINDING (At Component Init, from Decorators)  │
└──────────────────────────────────────────────────────────────────┘
                              │
                              v
                Component with @remote_service methods
                              │
                              v
                ┌─────────────────────────────┐
                │ bind_decorated_callbacks()  │
                │ descriptor.bind_callback()  │
                └─────────────────────────────┘
                              │
                              v
                Descriptor with callbacks bound ✅

┌──────────────────────────────────────────────────────────────────┐
│ PHASE 3: INTERFACE CREATION (Immediate or Late)                 │
└──────────────────────────────────────────────────────────────────┘
                              │
                              v
                ┌─────────────────────────────┐
                │ InterfaceFactory            │
                │ .create_from_descriptor()   │
                └─────────────────────────────┘
                              │
                              v
                VyraServer / VyraPublisher / VyraActionServer (active)
```

### Callback Types Mapping

**Current Transport Types (types.py):**

| Transport Class | Callbacks | Count |
|----------------|-----------|-------|
| VyraPublisher | *(none)* | 0 |
| VyraSubscriber | subscriber_callback | 1 |
| VyraServer | response_callback | 1 |
| VyraClient | request_callback | 1 |
| VyraActionServer | handle_goal_request, handle_cancel_request, execution_callback | **3** |
| VyraActionClient | direct_response_callback, feedback_callback, goal_callback | **3** |

**Problem:** Blueprint system nur für single callback designed!

---

## 🎯 Improvement Plan

### 1. Rename: Blueprint → InterfaceDescriptor

**Rationale:**
- "Blueprint" suggeriert Gang-of-Four Design Pattern (Builder/Prototype)
- System erstellt keine Kopien von Objekten
- Es geht um **Interface-Beschreibungen/Definitionen**
- Bessere Namen: `InterfaceDescriptor`, `InterfaceDefinition`, `InterfaceSpec`
- **Entscheidung: `InterfaceDescriptor`** (klarer, professioneller)

**Changes:**
```python
# OLD (blueprints.py)
HandlerBlueprint
ServiceBlueprint
PublisherBlueprint
SubscriberBlueprint
ActionBlueprint

# NEW (interface_descriptors.py)
InterfaceDescriptor (ABC)
ServiceDescriptor
PublisherDescriptor
SubscriberDescriptor
ActionDescriptor
```

**Files to change:**
- Rename: `core/blueprints.py` → `core/interface_descriptors.py`
- Update: `core/callback_registry.py` (imports & methods)
- Update: `core/decorators.py` (imports & docstrings)
- Update: `core/factory.py` (imports & methods)
- Update: `core/__init__.py` (exports)
- Update: `com/__init__.py` (exports)

### 2. Multi-Callback Support for ActionServer/Client

**Problem:**
```python
# Current - nur single callback
class ActionBlueprint(HandlerBlueprint):
    def __init__(self, ...):
        self._callback = None  # ❌ Kann nur 1 Callback speichern
```

**Solution: Callback Dictionary Pattern**

```python
# NEW - multiple callbacks
class ActionDescriptor(InterfaceDescriptor):
    """
    Descriptor for ActionServer with multiple lifecycle callbacks.
    
    Required callbacks:
    - on_goal: async def(goal_request) -> bool (accept/reject)
    - on_cancel: async def(goal_handle) -> bool (accept/reject)
    - execute: async def(goal_handle) -> result (main execution)
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
    
    def bind_callback(self, callback: Callable, callback_type: str = 'execute') -> None:
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
        
        self._validate_callback(callback, callback_type)
        self._callbacks[callback_type] = callback
        logger.debug(f"✅ Bound '{callback_type}' callback to ActionDescriptor '{self.name}'")
    
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
            callback_type: Check specific callback, or None for "all required"
        """
        if callback_type:
            return self._callbacks.get(callback_type) is not None
        
        # For ActionServer, 'execute' is required, others optional
        return self._callbacks['execute'] is not None
    
    def is_fully_bound(self) -> bool:
        """Check if ALL callbacks are bound."""
        return all(cb is not None for cb in self._callbacks.values())
    
    def unbind_callback(self, callback_type: str) -> Optional[Callable]:
        """Remove specific callback binding."""
        old_callback = self._callbacks.get(callback_type)
        if callback_type in self._callbacks:
            self._callbacks[callback_type] = None
        return old_callback
    
    def _validate_callback(self, callback: Callable, callback_type: str) -> None:
        """Validate callback signature based on type."""
        sig = inspect.signature(callback)
        params = list(sig.parameters.keys())
        
        # Remove 'self' if present
        if params and params[0] == 'self':
            params = params[1:]
        
        if callback_type in ['on_goal', 'on_cancel', 'execute']:
            if len(params) < 1:
                raise ValueError(
                    f"ActionServer '{callback_type}' callback must accept "
                    f"at least 1 parameter. Got: {params}"
                )
```

**Decorator Updates:**

```python
# NEW decorator patterns
@remote_actionServer.on_goal
async def handle_goal(self, goal_request):
    return True  # Accept

@remote_actionServer.on_cancel
async def handle_cancel(self, goal_handle):
    return True  # Accept cancel

@remote_actionServer.execute(name="process_batch")
async def execute_batch(self, goal_handle):
    # Main execution
    for i in range(10):
        goal_handle.publish_feedback({"progress": i})
    return {"result": "success"}

# OR combined:
@remote_actionServer(name="process_batch")
class BatchProcessor:
    async def on_goal(self, goal_request):
        return True
    
    async def on_cancel(self, goal_handle):
        return True
    
    async def execute(self, goal_handle):
        return {"result": "success"}
```

**Alternative: Explicit Multi-Decorator Pattern**

```python
# Pattern 1: Separate decorators (cleaner for different functions)
@remote_actionServer.on_goal(name="process_batch")
async def accept_batch_goal(self, goal_request):
    return goal_request.items_count <= 100

@remote_actionServer.on_cancel(name="process_batch")
async def cancel_batch(self, goal_handle):
    return True

@remote_actionServer.execute(name="process_batch")
async def execute_batch(self, goal_handle):
    # Main work
    return {"processed": 10}

# Pattern 2: Convention-based naming (discover by name)
@remote_actionServer(name="process_batch")
async def process_batch_execute(self, goal_handle):
    return {"result": "success"}

@remote_actionServer(name="process_batch", callback_type="on_goal")
async def process_batch_on_goal(self, goal_request):
    return True

# Pattern 3: Single decorator with sub-decorators (complex but explicit)
@remote_actionServer(name="process_batch")
class ProcessBatch:
    @on_goal
    async def handle_goal(self, goal_request): ...
    
    @on_cancel
    async def handle_cancel(self, goal_handle): ...
    
    @execute
    async def main_execution(self, goal_handle): ...
```

**RECOMMENDATION: Pattern 1 (Separate Decorators)**
- Most flexible
- Clear separation of concerns
- Easy to test individually
- Consistent with single-callback pattern

### 3. Add ActionStatus Enum

**Add to `core/types.py`:**

```python
class ActionStatus(Enum):
    """Status values for action goals (ROS2 Action compatible)."""
    # Goal states
    ACCEPTED = 1      # Goal accepted by server
    EXECUTING = 2     # Goal is executing
    CANCELING = 3     # Goal is being canceled
    
    # Terminal states
    SUCCEEDED = 4     # Goal completed successfully
    CANCELED = 5      # Goal was canceled
    ABORTED = 6       # Goal aborted due to error
    
    # ROS2 compatibility
    UNKNOWN = 0       # Unknown state
```

**Usage in ActionServer:**

```python
async def execute_batch(self, goal_handle):
    try:
        # Work...
        goal_handle.succeed()
        return ActionStatus.SUCCEEDED, result
    except CancelledError:
        return ActionStatus.CANCELED, {}
    except Exception as e:
        return ActionStatus.ABORTED, {"error": str(e)}
```

### 4. Abstract Interface Classes (Optional)

**Purpose:** Provide structured callback handlers for complex logic.

**Add to `core/abstract_handlers.py`:**

```python
"""
Abstract Handler Interfaces

Optional base classes for structured callback implementations.
Use when you have complex handler logic that benefits from inheritance.
"""
from abc import ABC, abstractmethod
from typing import Any

class IServiceHandler(ABC):
    """Abstract interface for service request handlers."""
    
    @abstractmethod
    async def handle_request(self, request: Any) -> Any:
        """
        Handle service request.
        
        Args:
            request: Incoming request data
            
        Returns:
            Response data
        """
        pass

class ISubscriberHandler(ABC):
    """Abstract interface for subscription handlers."""
    
    @abstractmethod
    async def handle_message(self, message: Any) -> None:
        """Handle incoming message."""
        pass

class IActionGoalHandler(ABC):
    """Abstract interface for action goal handlers."""
    
    @abstractmethod
    async def on_goal(self, goal_request: Any) -> bool:
        """Accept or reject goal."""
        pass
    
    @abstractmethod
    async def on_cancel(self, goal_handle: Any) -> bool:
        """Accept or reject cancel request."""
        pass
    
    @abstractmethod
    async def execute(self, goal_handle: Any) -> Any:
        """Execute the action and return result."""
        pass

# Usage example:
class BatchProcessorHandler(IActionGoalHandler):
    async def on_goal(self, goal_request):
        return goal_request.items_count <= 100
    
    async def on_cancel(self, goal_handle):
        return True
    
    async def execute(self, goal_handle):
        for i in range(10):
            goal_handle.publish_feedback({"progress": i})
        return {"processed": 10}

# In component:
handler = BatchProcessorHandler()

descriptor = ActionDescriptor(name="process_batch")
descriptor.bind_callbacks(
    on_goal=handler.on_goal,
    on_cancel=handler.on_cancel,
    execute=handler.execute
)
```

**NOTE:** This is OPTIONAL. Decorators work fine without inheritance.

### 5. Reorganize Examples Folder

**Current:**
```
vyra_base/com/core/examples/
├── example_basic_service.py
├── example_publisher_property.py
└── README.md
```

**Proposed:**
```
vyra_base/docs/examples/
├── 01_basic_service/
│   ├── example.py
│   ├── README.md
│   └── expected_output.txt
├── 02_publisher_patterns/
│   ├── property_setter.py
│   ├── batch_publishing.py
│   └── README.md
├── 03_action_server/
│   ├── simple_action.py
│   ├── multi_callback_action.py
│   └── README.md
└── README.md (index of all examples)
```

**Rationale:**
- Examples sind Dokumentation, nicht Core-Code
- Besser organisiert mit separaten Ordnern
- Easier to add more examples
- Matches industry standard (docs/examples/)

### 6. Update REFACTORING_STATUS.md Alignment

**Current Open Tasks from REFACTORING_STATUS.md:**

| Task | Status in Plan | Notes |
|------|---------------|-------|
| 8. Entity Integration | ✅ Included in implementation | entity.py changes |
| 9. Module Integration | ✅ Included | v2_modulemanager changes |
| 10. Tests | ✅ Will add | Unit + Integration tests |
| 11. Documentation | ✅ Will update | DECORATOR_GUIDE.md etc. |

**Additions:**
- ActionStatus Enum (not in original plan)
- Multi-callback pattern (partially mentioned)
- Abstract handlers (not in original plan)
- Examples reorganization (not in original plan)

---

## 📋 Implementation Checklist

### Phase 1: Core Renaming & Multi-Callback (HIGH Priority)

- [ ] **1.1 Rename Files**
  - [ ] `core/blueprints.py` → `core/interface_descriptors.py`
  - [ ] Update all imports in:
    - [ ] `core/callback_registry.py`
    - [ ] `core/decorators.py`
    - [ ] `core/factory.py`
    - [ ] `core/__init__.py`
    - [ ] `com/__init__.py`

- [ ] **1.2 Rename Classes**
  - [ ] `HandlerBlueprint` → `InterfaceDescriptor`
  - [ ] `ServiceBlueprint` → `ServiceDescriptor`
  - [ ] `PublisherBlueprint` → `PublisherDescriptor`
  - [ ] `SubscriberBlueprint` → `SubscriberDescriptor`
  - [ ] `ActionBlueprint` → `ActionDescriptor`

- [ ] **1.3 Implement Multi-Callback Pattern**
  - [ ] Update `ActionDescriptor` with `_callbacks` dict
  - [ ] Implement `bind_callback(callback, callback_type)`
  - [ ] Implement `bind_callbacks(**callbacks)`
  - [ ] Implement `get_callback(callback_type)`
  - [ ] Implement `is_bound(callback_type)` & `is_fully_bound()`
  - [ ] Update validation per callback type

- [ ] **1.4 Add ActionStatus Enum**
  - [ ] Add `ActionStatus` to `core/types.py`
  - [ ] Export in `core/__init__.py`
  - [ ] Update documentation

- [ ] **1.5 Update CallbackRegistry**
  - [ ] Support multi-callback binding
  - [ ] Add `bind_callback(name, callback, callback_type, namespace)`
  - [ ] Update `list_bound()` to show callback_types
  - [ ] Update `debug_print()` format

### Phase 2: Decorator Multi-Callback Support (HIGH Priority)

- [ ] **2.1 Implement Action Decorators**
  - [ ] `@remote_actionServer.on_goal(name)`
  - [ ] `@remote_actionServer.on_cancel(name)`
  - [ ] `@remote_actionServer.execute(name)`
  - [ ] Update `get_decorated_methods()` for multi-callback
  - [ ] Update `bind_decorated_callbacks()` for ActionDescriptor

- [ ] **2.2 Update Factory**
  - [ ] `create_from_descriptor()` supports ActionDescriptor multi-callback
  - [ ] Map callbacks to VyraActionServer constructor
  - [ ] Handle partial binding (execute required, others optional)

### Phase 3: Entity & Module Integration (From REFACTORING_STATUS.md)

- [ ] **3.1 Entity Integration**
  - [ ] Update `entity.py` `set_interfaces()` for descriptors
  - [ ] Add `bind_interface_callbacks()` method
  - [ ] Add background task for pending processing
  - [ ] Update startup/shutdown lifecycle

- [ ] **3.2 Module Integration**
  - [ ] Update `v2_modulemanager/_base_.py` → descriptors
  - [ ] Update `v2_modulemanager/interface.py` → binding
  - [ ] Update `v2_modulemanager/application.py` decorators
  - [ ] Test full module startup

### Phase 4: Examples & Documentation (MEDIUM Priority)

- [ ] **4.1 Reorganize Examples**
  - [ ] Create `docs/examples/` structure
  - [ ] Move existing examples
  - [ ] Add action server examples
  - [ ] Add multi-callback examples
  - [ ] Create index README.md

- [ ] **4.2 Abstract Handlers (Optional)**
  - [ ] Create `core/abstract_handlers.py`
  - [ ] Implement `IServiceHandler`
  - [ ] Implement `ISubscriberHandler`
  - [ ] Implement `IActionGoalHandler`
  - [ ] Add example usage in docs

- [ ] **4.3 Update Documentation**
  - [ ] Create `INTERFACE_DESCRIPTOR_GUIDE.md`
  - [ ] Update `DECORATOR_GUIDE.md` with multi-callback
  - [ ] Update `MIGRATION_GUIDE.md`
  - [ ] Update `com/README.md` with new architecture
  - [ ] Update `.github/copilot-instructions.md`

### Phase 5: Testing (HIGH Priority)

- [ ] **5.1 Unit Tests**
  - [ ] `test_interface_descriptors.py`
    - [ ] ServiceDescriptor creation & binding
    - [ ] ActionDescriptor multi-callback
    - [ ] Validation tests
  - [ ] `test_callback_registry_multi.py`
    - [ ] Multi-callback registration
    - [ ] Namespace isolation
    - [ ] Statistics

- [ ] **5.2 Integration Tests**
  - [ ] `test_action_server_integration.py`
    - [ ] Full action lifecycle
    - [ ] Goal acceptance/rejection
    - [ ] Cancellation
    - [ ] Feedback publishing
  - [ ] `test_late_binding_multi_callback.py`
    - [ ] Register descriptor → bind multiple callbacks → create interface

- [ ] **5.3 E2E Tests**
  - [ ] Module startup with ActionServer
  - [ ] ROS2 action call from client
  - [ ] Verify all callbacks invoked

---

## 🔄 Backward Compatibility Strategy

### Breaking Changes

1. **Class Name Changes**
   - Old: `from vyra_base.com import ServiceBlueprint`
   - New: `from vyra_base.com import ServiceDescriptor`

2. **ActionBlueprint API**
   - Old: `blueprint.bind_callback(fn)`
   - New: `descriptor.bind_callback(fn, callback_type='execute')`
   - New: `descriptor.bind_callbacks(on_goal=..., execute=...)`

### Migration Path

**OPTION A: Hard Break (Recommended)**
- Remove old names completely
- Force migration in all modules
- Clean codebase, no legacy baggage

**OPTION B: Deprecated Aliases**
```python
# In core/__init__.py
from vyra_base.com.core.interface_descriptors import (
    InterfaceDescriptor,
    ServiceDescriptor,
    PublisherDescriptor,
    SubscriberDescriptor,
    ActionDescriptor
)

# DEPRECATED aliases
HandlerBlueprint = InterfaceDescriptor  # DEPRECATED
ServiceBlueprint = ServiceDescriptor    # DEPRECATED
# ... etc
```

**RECOMMENDATION:** OPTION A (Hard Break)
- System ist neu, wenig external usage
- Cleaner codebase
- Easier to maintain

---

## 📊 Success Criteria

### Minimum Viable Product (MVP)

- [ ] All classes renamed to *Descriptor
- [ ] ActionDescriptor supports 3 callbacks
- [ ] ActionStatus enum added
- [ ] Decorators support multi-callback for actions
- [ ] v2_modulemanager starts successfully
- [ ] ROS2 ActionServer with multi-callback works

### Full Success

- [ ] All modules migrated
- [ ] Examples reorganized in docs/
- [ ] Tests > 80% coverage
- [ ] Documentation complete
- [ ] No regressions in existing functionality
- [ ] Performance equal or better

---

## 🎯 Next Steps

### Immediate (Today/Tomorrow)

1. ✅ **Review & Approve Plan** - Get stakeholder buy-in
2. 📝 **Update REFACTORING_STATUS.md** - Incorporate this plan
3. 🔧 **Start Phase 1.1** - Rename files & classes
4. 🔧 **Implement Phase 1.3** - Multi-callback pattern

### This Week

- Complete Phase 1 (Core changes)
- Complete Phase 2 (Decorator updates)
- Start Phase 3 (Entity integration)

### Next Week

- Complete Phase 3 (Module integration)
- Complete Phase 4 (Examples & docs)
- Complete Phase 5 (Testing)

---

## 📞 Questions & Decisions

### Open Questions

1. **Naming:** `InterfaceDescriptor` vs `InterfaceDefinition` vs `InterfaceSpec`?
   - **DECISION:** `InterfaceDescriptor` (most descriptive)

2. **Action Decorator Pattern:** Separate decorators vs combined?
   - **DECISION:** Separate decorators (Pattern 1 in section 2)

3. **Abstract Handlers:** Required or optional?
   - **DECISION:** Optional (nice to have, not required)

4. **Backward Compatibility:** Hard break or deprecated aliases?
   - **DECISION:** Hard break (system is new enough)

5. **Examples Location:** `docs/examples/` vs `examples/` at root?
   - **DECISION:** `docs/examples/` (keeps docs together)

### Known Risks

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|------------|
| Breaking existing modules | HIGH | HIGH | Comprehensive testing, staged rollout |
| ActionServer multi-callback complexity | MEDIUM | MEDIUM | Good documentation, examples |
| Decorator syntax confusion | LOW | MEDIUM | Clear error messages, examples |
| Migration effort underestimated | MEDIUM | MEDIUM | Parallel work on multiple modules |

---

## 📚 References

- **Original Plan:** `REFACTORING_STATUS.md`
- **Current Code:**
  - `src/vyra_base/com/core/blueprints.py` (to be renamed)
  - `src/vyra_base/com/core/types.py` (VyraActionServer)
  - `src/vyra_base/com/transport/t_ros2/vyra_models/action_server.py`
- **Architecture Docs:**
  - `src/vyra_base/com/README.md`
  - `.github/copilot-instructions.md`

---

**STATUS:** ✅ Plan Complete - Ready for Implementation  
**Next Action:** Review and approve plan → Start Phase 1.1 (Renaming)
