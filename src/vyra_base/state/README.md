# vyra_base.state

3-layer hierarchical state machine for VYRA modules, conforming to industrial standards
(IEC 61508, IEC 61131-3, ISO 13849).

## Public API

```python
from vyra_base.state import (
    # Unified interface
    UnifiedStateMachine,
    OperationalStateMachine,
    # Individual layers
    LifecycleLayer,
    OperationalLayer,
    HealthLayer,
    # State types
    LifecycleState,
    OperationalState,
    HealthState,
    # Events
    StateEvent,
    EventType,
    # State machine primitives
    StateMachine,
    StateMachineConfig,
    StateMachineError,
    InvalidTransitionError,
    LayerViolationError,
)
```

---

## Layer Architecture

```
┌─────────────────────────────────────┐
│  Lifecycle Layer                    │  Controls Operational + Health
│  Offline → Initializing → Active    │
│                → Shutting Down      │
└─────────────────────────────────────┘
       ↓ enables / disables
┌─────────────────────────────────────┐
│  Operational Layer                  │  Runtime activity states
│  Idle → Ready → Running → Paused    │
└─────────────────────────────────────┘
       ↓ can escalate
┌─────────────────────────────────────┐
│  Health Layer                       │  Diagnostics and fault management
│  Healthy → Warning → Error → Fatal  │
└─────────────────────────────────────┘
```

**Layer rules:**
- **Lifecycle** controls when Operational transitions are allowed
- **Health** can override Operational state and escalate Lifecycle
- **Operational** cannot directly modify Lifecycle or Health — it can only report events

---

## UnifiedStateMachine

`UnifiedStateMachine` is the primary interface coordinating all three layers:

```python
from vyra_base.state import UnifiedStateMachine

usm = UnifiedStateMachine()

# Lifecycle transitions
usm.start()                        # Offline → Initializing
usm.complete_initialization()      # Initializing → Active

# Operational transitions
usm.set_ready()                    # Idle → Ready
usm.start_task({"id": "job-1"})   # Ready → Running
usm.pause("operator requested")   # Running → Paused
usm.resume()                       # Paused → Running
usm.stop({"result": "ok"})        # Running → Idle

# Health transitions
usm.report_warning({"temp": "high"})   # Healthy → Warning
usm.report_error({"code": 42})         # Warning → Error
usm.recover({"action": "reset"})       # Error → Healthy

# Query
states = usm.get_all_states()
# {"lifecycle": "Active", "operational": "Running", "health": "Healthy"}

history = usm.get_history()   # list of StateTransition objects

# Callbacks
usm.on_any_change(lambda layer, old, new: print(f"{layer}: {old} → {new}"))
usm.on_lifecycle_change(lambda layer, old, new: ...)
usm.on_operational_change(lambda layer, old, new: ...)
usm.on_health_change(lambda layer, old, new: ...)
```

---

## OperationalStateMachine

`OperationalStateMachine` is a mixin class that wraps `UnifiedStateMachine` and integrates
it with the `@operation` decorator for automatic state transitions:

```python
from vyra_base.state import OperationalStateMachine, operation

class MyApplication(OperationalStateMachine):

    @operation(requires="Ready", transitions_to="Running")
    async def start_process(self):
        # State automatically transitions to Running before this runs
        # and back to Idle/Ready on completion
        return {"status": "done"}
```

---

## State Types

### LifecycleState

```
Offline → Initializing → Active → ShuttingDown → Offline
```

### OperationalState

```
Idle → Ready → Running ⇌ Paused
                ↓
              Stopping → Idle
```

### HealthState

```
Healthy → Warning → Error → CriticalError
                     ↑
                  (recover)
```

---

## Events

`StateEvent` carries metadata about what triggered a transition:

```python
from vyra_base.state import StateEvent, EventType

event = StateEvent(
    event_type=EventType.COMMAND,
    source="operator",
    data={"reason": "manual start"},
)
```

---

## Individual Layers

Each layer can also be used independently:

```python
from vyra_base.state import LifecycleLayer, OperationalLayer, HealthLayer, StateMachineConfig

lifecycle = LifecycleLayer(StateMachineConfig())
lifecycle.transition_to(LifecycleState.INITIALIZING)
```

---

## Documentation

Full state machine documentation (diagrams, transition tables, event reference):

- [`docs/state/README.md`](../../../../docs/state/README.md)
- [`docs/state/STATE_MACHINE.md`](../../../../docs/state/STATE_MACHINE.md)
- [`docs/state/3_Layer_Statemachine.md`](../../../../docs/state/3_Layer_Statemachine.md)

---

## Files

| File | Description |
|---|---|
| `unified.py` | `UnifiedStateMachine` — coordinates all three layers |
| `state_machine.py` | `StateMachine` base + `StateMachineConfig` |
| `lifecycle_layer.py` | `LifecycleLayer` |
| `operational_layer.py` | `OperationalLayer` |
| `health_layer.py` | `HealthLayer` |
| `state_types.py` | `LifecycleState`, `OperationalState`, `HealthState` enums + validators |
| `state_events.py` | `StateEvent`, `EventType` |
| `operational_state_machine.py` | `OperationalStateMachine` mixin |
| `operational_metaclass.py` | `MetaOperationalState` metaclass |
| `operation_decorator.py` | `@operation` decorator + `OperationConfig` |
