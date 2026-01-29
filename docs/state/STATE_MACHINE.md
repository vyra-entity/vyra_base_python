# 3-Layer State Machine System

Professional industrial-grade state machine for automation and robotics applications.

## Overview

The 3-Layer State Machine implements a hierarchical finite state machine (FSM) with three independent but interacting layers:

1. **Lifecycle Layer** - Controls module existence (startup, shutdown, recovery)
2. **Operational Layer** - Manages runtime activity (tasks, processing, delegation)
3. **Health Layer** - Monitors system integrity (warnings, faults, critical states)

## Key Features

- ✅ **Industrial Standards Compliance**
  - IEC 61508 (Functional Safety)
  - IEC 61131-3 (PLC Programming)
  - ISO 13849 (Safety of Machinery)
  - ROS2 Lifecycle compatible

- ✅ **Thread-Safe Operation**
  - Reentrant locks (RLock)
  - Concurrent state queries
  - Safe event processing

- ✅ **Event-Driven Architecture**
  - 30+ predefined event types
  - Immutable event objects
  - Complete audit trail

- ✅ **Validated Transitions**
  - Type-safe state enums
  - Transition validation
  - Layer interaction rules

- ✅ **Production Ready**
  - Comprehensive error handling
  - Detailed logging
  - Diagnostic API

## Architecture

### Layer Hierarchy

```
┌─────────────────────────────────────────────────────────┐
│  Lifecycle Layer (Module Existence)                    │
│  Controls: Operational Layer                           │
│  Affected by: Health Layer (escalation)                │
├─────────────────────────────────────────────────────────┤
│  States:                                                │
│  • Uninitialized → Initializing → Active                │
│  • Active → Recovering → Active/Deactivated             │
│  • Active → ShuttingDown → Deactivated                  │
└─────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────┐
│  Operational Layer (Runtime Activity)                   │
│  Controlled by: Lifecycle Layer                         │
│  Reports to: Health Layer (via events)                  │
├─────────────────────────────────────────────────────────┤
│  States:                                                │
│  • Idle → Ready → Running → Completed                   │
│  • Running ↔ Paused (interrupts)                        │
│  • Running → Processing/Delegating → Running            │
│  • Running → Blocked → Running                          │
└─────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────┐
│  Health Layer (System Integrity)                        │
│  Regulates: Operational Layer                           │
│  Escalates: Lifecycle Layer (on critical faults)        │
├─────────────────────────────────────────────────────────┤
│  States:                                                │
│  • OK → Warning → OK                                    │
│  • Warning → Overloaded → Warning                       │
│  • Warning/Overloaded → Faulted → OK                    │
│  • Faulted → Critical (forces shutdown)                 │
└─────────────────────────────────────────────────────────┘
```

### Layer Interaction Rules

| Rule | Description | Example |
|------|-------------|---------|
| **Lifecycle → Operational** | Lifecycle controls which operational states are allowed | Active: all states allowed<br>Recovering: only Paused/Blocked<br>ShuttingDown: only Paused/Idle |
| **Health → Operational** | Health can pause/block operational activity | Overloaded: pause running tasks<br>Critical: block all operations |
| **Health → Lifecycle** | Health can escalate lifecycle state | Faulted: trigger Recovering<br>Critical: force ShuttingDown |
| **Operational → Lifecycle** | Operational CANNOT directly affect Lifecycle | Reports events only |

## Quick Start

### Basic Usage

```python
from vyra_base.state import UnifiedStateMachine

# Create state machine
usm = UnifiedStateMachine()

# Standard startup sequence
usm.start()                      # Lifecycle: Initializing
usm.complete_initialization()    # Lifecycle: Active
usm.ready()                       # Operational: Ready

# Task execution
usm.start_task({'task_id': '123'})   # Operational: Running
usm.complete({'result': 'success'})   # Operational: Completed

# Health monitoring
usm.report_warning({'cpu': '85%'})    # Health: Warning
usm.clear_warning()                   # Health: OK

# Shutdown
usm.shutdown()                   # Lifecycle: ShuttingDown
usm.complete_shutdown()          # Lifecycle: Deactivated
```

### Layer-Specific API

```python
from vyra_base.state import StateMachine, LifecycleLayer, OperationalLayer, HealthLayer

# Create core FSM
fsm = StateMachine()

# Create layer interfaces
lifecycle = LifecycleLayer(fsm)
operational = OperationalLayer(fsm)
health = HealthLayer(fsm)

# Use layer-specific methods
lifecycle.start()
lifecycle.complete_initialization()

operational.ready()
operational.start_task({'data': 'value'})

health.report_warning({'metric': 'value'})
```

### Callbacks and Monitoring

```python
# Subscribe to state changes
def on_lifecycle_change(layer, old_state, new_state):
    print(f"Lifecycle: {old_state} → {new_state}")

usm.on_lifecycle_change(on_lifecycle_change)

# Get current states
states = usm.get_all_states()
print(states)  # {'lifecycle': 'Active', 'operational': 'Running', 'health': 'OK'}

# Get diagnostic information
diagnostics = usm.get_diagnostic_info()
print(diagnostics['fsm_diagnostics'])

# Check state history
history = usm.get_history(limit=10)
for transition in history:
    print(f"{transition.layer}: {transition.from_state} → {transition.to_state}")
```

## State Definitions

### Lifecycle States

| State | Description | Allowed Operational States |
|-------|-------------|----------------------------|
| **Uninitialized** | Initial state, not yet started | Idle only |
| **Initializing** | Performing initialization | Idle only |
| **Active** | Fully operational | All states |
| **Recovering** | Recovering from fault | Idle, Paused, Blocked |
| **ShuttingDown** | Graceful shutdown in progress | Paused, Idle |
| **Deactivated** | Shut down, no activity | Idle only |

### Operational States

| State | Description | Entry Conditions |
|-------|-------------|------------------|
| **Idle** | No activity | Default state |
| **Ready** | Ready to accept tasks | From Idle |
| **Running** | Executing task | From Ready |
| **Processing** | Background processing | From Running |
| **Delegating** | Task delegated to another module | From Running |
| **Paused** | Task paused (resumable) | From Running |
| **Blocked** | Blocked by external condition | From Running |
| **Completed** | Task finished | From Running |

### Health States

| State | Description | Severity Level |
|-------|-------------|----------------|
| **OK** | Normal operation | 0 |
| **Warning** | Non-critical issues | 1 |
| **Overloaded** | Resource constraints | 2 |
| **Faulted** | Critical fault detected | 3 |
| **Critical** | System failure (forces shutdown) | 4 |

## Events

### Lifecycle Events

| Event | Transition | Description |
|-------|------------|-------------|
| `START` | Uninitialized → Initializing | Begin initialization |
| `INIT_SUCCESS` | Initializing → Active | Initialization complete |
| `INIT_FAILURE` | Initializing → Recovering | Initialization failed |
| `SHUTDOWN` | Active → ShuttingDown | Begin shutdown |
| `FINISHED` | ShuttingDown → Deactivated | Shutdown complete |
| `FAULT_DETECTED` | Active → Recovering | Fault requires recovery |
| `RECOVERY_SUCCESS` | Recovering → Active | Recovery complete |
| `RECOVERY_FAILED` | Recovering → Deactivated | Recovery failed |

### Operational Events

| Event | Transition | Description |
|-------|------------|-------------|
| `READY` | Idle → Ready | Ready for tasks |
| `TASK_START` | Ready → Running | Start task |
| `TASK_PAUSE` | Running → Paused | Pause task |
| `TASK_RESUME` | Paused → Running | Resume task |
| `TASK_COMPLETE` | Running → Completed | Task done |
| `AUTO_RESET` | Completed → Ready | Reset to ready |
| `BACKGROUND_PROCESSING` | Running → Processing | Enter processing |
| `PROCESSING_DONE` | Processing → Running | Exit processing |
| `DELEGATE_TO_OTHER` | Running → Delegating | Delegate task |
| `DELEGATE_DONE` | Delegating → Running | Delegation complete |
| `BLOCK_DETECTED` | Running → Blocked | Blockage detected |
| `UNBLOCK` | Blocked → Running | Unblock |

### Health Events

| Event | Transition | Description |
|-------|------------|-------------|
| `WARN` | OK → Warning | Warning detected |
| `CLEAR_WARNING` | Warning → OK | Warning cleared |
| `OVERLOAD` | Warning → Overloaded | Resource overload |
| `LOAD_REDUCED` | Overloaded → Warning | Load reduced |
| `FAULT` | Warning/Overloaded → Faulted | Fault detected |
| `RECOVER` | Faulted → OK | Recovery complete |
| `ESCALATE` | Faulted → Critical | Escalate to critical |
| `RESET` | Critical → Faulted | Reset from critical |

### Interrupt Events

| Event | Effect | Description |
|-------|--------|-------------|
| `INTERRUPT` | Pauses operational | Regular interrupt |
| `EMERGENCY_STOP` | All layers to safe state | Emergency stop |
| `PRIORITY_OVERRIDE` | Override current state | Priority event |

## Configuration

```python
from vyra_base.state import StateMachine, StateMachineConfig

config = StateMachineConfig(
    # Initial states
    initial_lifecycle=LifecycleState.UNINITIALIZED,
    initial_operational=OperationalState.IDLE,
    initial_health=HealthState.OK,
    
    # Operational state when entering recovery
    operational_on_recovery=OperationalState.PAUSED,
    
    # Operational state when shutting down
    operational_on_shutdown=OperationalState.PAUSED,
    
    # Enable detailed transition logging
    enable_transition_log=True,
    
    # Maximum history size (0 = unlimited)
    max_history_size=1000,
    
    # Strict mode (raises exceptions on invalid transitions)
    strict_mode=True,
)

fsm = StateMachine(config)
```

## Error Handling

### Exceptions

```python
from vyra_base.state import (
    StateMachineError,          # Base exception
    InvalidTransitionError,     # Invalid state transition
    LayerViolationError,        # Layer rule violation
)

try:
    fsm.send_event(StateEvent(EventType.SHUTDOWN))
except InvalidTransitionError as e:
    logger.error(f"Invalid transition: {e}")
except LayerViolationError as e:
    logger.error(f"Layer rule violation: {e}")
```

### Strict vs Lenient Mode

**Strict Mode (Production):**
```python
config = StateMachineConfig(strict_mode=True)
fsm = StateMachine(config)
# Raises exceptions on invalid transitions
```

**Lenient Mode (Development):**
```python
config = StateMachineConfig(strict_mode=False)
fsm = StateMachine(config)
# Logs warnings, continues execution
```

## Industrial Use Cases

### Example: Automated Manufacturing Cell

```python
# Setup
cell = UnifiedStateMachine()
cell.start()
cell.complete_initialization()
cell.ready()

# Production cycle
while production_active:
    # Start task
    cell.start_task({'part_id': get_next_part()})
    
    # Monitor health
    if temperature > WARNING_THRESHOLD:
        cell.report_warning({'temperature': temperature})
    
    if temperature > CRITICAL_THRESHOLD:
        cell.report_fault({'temperature': temperature})
        # Health escalation triggers recovery automatically
    
    # Complete task
    cell.complete({'parts_produced': 1})
    cell.reset()

# Shutdown
cell.shutdown()
cell.complete_shutdown()
```

### Example: Robot Control System

```python
robot = UnifiedStateMachine()

# Startup with error handling
if not robot.standard_startup():
    logger.error("Robot startup failed")
    sys.exit(1)

# Task execution with delegation
robot.start_task({'action': 'pick', 'target': 'A1'})

if complex_operation_required:
    robot.delegate_to_other({'delegate_to': 'robot_2'})
    # Wait for delegation
    robot.complete_delegation({'status': 'success'})

robot.complete()

# Emergency stop
if emergency_detected:
    robot.emergency_stop("Obstacle detected")
```

## Testing

```bash
# Run all state machine tests
pytest tests/test_state_types.py tests/test_state_events.py tests/test_state_machine.py -v

# Run specific test class
pytest tests/test_state_machine.py::TestLifecycleTransitions -v

# Run with coverage
pytest tests/test_state_machine.py --cov=vyra_base.state --cov-report=html
```

## API Reference

### StateMachine

Core state machine engine.

**Methods:**
- `get_current_state()` - Get all layer states
- `send_event(event)` - Process state transition event
- `subscribe(layer, callback, priority)` - Register callback
- `get_history(limit)` - Get transition history
- `get_diagnostic_info()` - Get full diagnostics

### UnifiedStateMachine

Unified high-level interface combining all layers.

**Methods:**
- Lifecycle: `start()`, `complete_initialization()`, `shutdown()`
- Operational: `ready()`, `start_task()`, `pause()`, `complete()`
- Health: `report_warning()`, `report_fault()`, `recover()`
- Monitoring: `get_all_states()`, `get_diagnostic_info()`

### Layer Classes

**LifecycleLayer:**
- `start()`, `complete_initialization()`, `shutdown()`
- `is_active()`, `is_recovering()`, `can_accept_tasks()`

**OperationalLayer:**
- `ready()`, `start_task()`, `pause()`, `resume()`, `complete()`
- `is_running()`, `is_busy()`, `can_start_task()`

**HealthLayer:**
- `report_warning()`, `report_fault()`, `recover()`
- `is_ok()`, `is_degraded()`, `get_severity_level()`

## Migration from Old State Machine

If migrating from the old state machine:

1. Old files backed up in `src/vyra_base/state/OLD/`
2. Import paths changed:
   ```python
   # Old
   from vyra_base.state.state_machine import StateMachine
   
   # New
   from vyra_base.state import UnifiedStateMachine
   usm = UnifiedStateMachine()
   ```
3. New layer-based API
4. Event-driven transitions instead of direct state changes

## Contributing

When extending the state machine:

1. Add new states to state_types.py
2. Define valid transitions in TRANSITIONS sets
3. Add events to state_events.py
4. Update layer interaction rules
5. Add comprehensive tests
6. Update documentation

## License

See LICENSE file.

## Authors

VYRA Development Team

---

**Version:** 3.0.0  
**Last Updated:** 2025-12-05  
**Standards:** IEC 61508, IEC 61131-3, ISO 13849
