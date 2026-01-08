# Operational State Machine - Complete API Documentation

## Overview

The VYRA Operational State Machine provides comprehensive automatic state management for modules following industrial automation best practices. It combines two complementary approaches:

1. **Static Lifecycle Methods**: Fixed state transitions through explicit method calls (`initialize()`, `pause()`, etc.)
2. **Dynamic Operations**: Flexible runtime state management using the `@operation` decorator with automatic reference counting

This architecture enables both predictable lifecycle control and flexible concurrent operation tracking.

## Architecture

### Three-Layer State System

The state machine operates on three independent but coordinated layers:

1. **Lifecycle Layer**: Module existence states (OFFLINE, INITIALIZING, ACTIVE, RECOVERING, SHUTTING_DOWN)
2. **Operational Layer**: Runtime activity states (IDLE, READY, RUNNING, PAUSED, STOPPED, ERROR)
3. **Health Layer**: Diagnostic states (HEALTHY, WARNING, CRITICAL)

### Operational State Flow

```
IDLE → initialize() → READY → @operation → RUNNING → @operation completes → READY
                         ↓                     ↑
                         └─────────────────────┘
                       (multiple operations via reference counting)

Error Handling:
  ANY STATE → (failure) → ERROR → reset() → IDLE
  
Pause/Resume:
  RUNNING → pause() → PAUSED → resume() → READY
  
Shutdown:
  RUNNING/PAUSED → stop() → STOPPED → reset() → IDLE
```

## Operational States

| State | Description | Entry | Exit |
|-------|-------------|-------|------|
| `IDLE` | Initial state, not operational | System start, after reset | `initialize()` |
| `READY` | Initialized, ready for operations | After initialize/resume | Operations start, pause, stop |
| `RUNNING` | One or more operations active | First operation starts (counter 0→1) | Last operation ends (counter 1→0) |
| `PAUSED` | Temporarily suspended | `pause()` from RUNNING | `resume()` or `stop()` |
| `STOPPED` | Clean shutdown completed | `stop()` from RUNNING/PAUSED | `reset()` |
| `ERROR` | Failure occurred, needs recovery | Any lifecycle method failure | `reset()` only |

## Static Lifecycle Methods

Static methods provide deterministic state transitions with well-defined pre/post conditions.

### Method Reference

#### initialize()

Performs one-time module initialization and transitions to READY state.

**Signature:**
```python
def initialize(self) -> bool:
    """
    User implementation: Initialize module resources.
    
    Returns:
        True on success, False on failure
    """
```

**State Transition:**
- Pre-condition: `IDLE`
- Success: `IDLE → READY`
- Failure: `IDLE → ERROR`
- Post-action: Operation counter reset to 0

**Use Cases:**
- Hardware initialization
- Configuration loading
- Resource allocation
- Connection establishment

**Example:**
```python
from vyra_base import state

class MyModule(state.OperationalStateMachine):
    def initialize(self):
        """Initialize hardware and configuration."""
        try:
            self.config = self._load_config()
            self.hardware = self._init_hardware()
            return self._verify_ready()
        except Exception as e:
            self.logger.error(f"Initialization failed: {e}")
            return False
```

---

#### pause()

Temporarily suspends ongoing operations.

**Signature:**
```python
def pause(self) -> bool:
    """
    User implementation: Pause module operations.
    
    Returns:
        True on success, False on failure
    """
```

**State Transition:**
- Pre-condition: `RUNNING`
- Success: `RUNNING → PAUSED`
- Failure: `RUNNING → ERROR`

**Use Cases:**
- Temporary operation suspension
- Resource yielding
- Coordinated multi-module pause

**Example:**
```python
def pause(self):
    """Pause ongoing operations."""
    if self.hardware:
        self.hardware.pause()
    self._save_state()
    return True
```

---

#### resume()

Resumes from paused state, returns to READY (not RUNNING).

**Signature:**
```python
def resume(self) -> bool:
    """
    User implementation: Resume module operations.
    
    Returns:
        True on success, False on failure
    """
```

**State Transition:**
- Pre-condition: `PAUSED`
- Success: `PAUSED → READY`
- Failure: `PAUSED → ERROR`
- Post-action: Operation counter reset to 0

**Use Cases:**
- Resume after pause
- Recover saved state
- Re-establish resource connections

**Example:**
```python
def resume(self):
    """Resume from paused state."""
    if self.hardware:
        self.hardware.resume()
    self._restore_state()
    return True
```

---

#### stop()

Performs clean shutdown of operations.

**Signature:**
```python
def stop(self) -> bool:
    """
    User implementation: Stop module operations.
    
    Returns:
        True on success, False on failure
    """
```

**State Transition:**
- Pre-condition: `RUNNING` or `PAUSED`
- Success: `→ STOPPED`
- Failure: `→ ERROR`

**Use Cases:**
- Normal shutdown
- Release resources
- Finalize processing

**Example:**
```python
def stop(self):
    """Stop module operations cleanly."""
    if self.hardware:
        self.hardware.stop()
    self._finalize()
    return True
```

---

#### reset()

Resets module to initial IDLE state, used for recovery from ERROR or STOPPED states.

**Signature:**
```python
def reset(self) -> bool:
    """
    User implementation: Reset module to initial state.
    
    Returns:
        True on success, False on failure
    """
```

**State Transition:**
- Pre-condition: `STOPPED` or `ERROR`
- Success: `→ IDLE`
- No failure transition (already in terminal state)

**Use Cases:**
- Error recovery
- Complete reset
- Prepare for re-initialization

**Example:**
```python
def reset(self):
    """Reset to initial state."""
    if self.hardware:
        self.hardware.release()
        self.hardware = None
    self.config = None
    return True
```

## Dynamic Operations with @operation Decorator

The `@operation` decorator provides automatic state management for runtime operations with built-in reference counting.

### Key Concepts

#### Reference Counting

The decorator maintains an internal counter tracking active operations:

- **Counter increment**: When operation starts
- **Counter decrement**: When operation completes (success or failure)
- **State transition to RUNNING**: When counter goes 0 → 1
- **State transition to READY**: When counter goes 1 → 0
- **Multiple concurrent operations**: Counter > 1, state remains RUNNING

#### Automatic State Management

```python
# State before: READY, counter: 0
result = module.process_data("item1")  
# During: RUNNING, counter: 1
# After: READY, counter: 0

# Nested operations
result = module.outer_operation()
# outer starts: READY → RUNNING, counter: 1
#   inner starts: counter: 2
#   inner ends: counter: 1
# outer ends: counter: 0, RUNNING → READY
```

### @operation Decorator Reference

#### Basic Usage

```python
from vyra_base import state

class DataProcessor(state.OperationalStateMachine):
    @state.operation
    def process_item(self, item):
        """Process item with automatic state management."""
        result = self._do_work(item)
        return result
```

#### Parameters

```python
@operation(
    required_states: Optional[Set[OperationalState]] = None,
    pre_transition: Optional[OperationalState] = None,
    success_transition: Optional[OperationalState] = None,
    failure_transition: Optional[OperationalState] = None,
    use_reference_counting: bool = True
)
```

**Parameter Details:**

- **required_states**: States allowed before operation executes
  - Default: `{OperationalState.READY, OperationalState.RUNNING}`
  - Raises `OperationalStateError` if not in required state
  
- **pre_transition**: Manual state transition before execution
  - Default: `None`
  - Only used when `use_reference_counting=False`
  
- **success_transition**: State to transition to on success
  - Default: `None`
  - Only used when `use_reference_counting=False`
  
- **failure_transition**: State to transition to on failure
  - Default: `None`
  - Only used when `use_reference_counting=False`
  
- **use_reference_counting**: Enable automatic READY ↔ RUNNING transitions
  - Default: `True`
  - When `True`: Uses reference counting for state management
  - When `False`: Uses manual transitions specified above

#### Return Value Interpretation

The decorator interprets return values to determine success/failure:

- **bool return**: `True` = success, `False` = failure
- **non-bool return**: Always treated as success
- **exception raised**: Always treated as failure

#### Exception Safety

The decorator ensures counter consistency even with exceptions:

```python
@state.operation
def risky_operation(self):
    # Counter increments, state → RUNNING
    result = self._might_fail()  # May raise exception
    # Counter ALWAYS decrements (handled by decorator)
    return result
```

### Usage Examples

#### Basic Operation

```python
class DataProcessor(state.OperationalStateMachine):
    def __init__(self, state_machine):
        super().__init__(state_machine)
        self.items_processed = 0
    
    def initialize(self):
        """Initialize processor."""
        self.items_processed = 0
        return True
    
    @state.operation
    def process_item(self, item):
        """Process single item."""
        # Automatic: READY → RUNNING (if counter was 0)
        result = self._do_processing(item)
        self.items_processed += 1
        # Automatic: RUNNING → READY (if counter becomes 0)
        return result

# Usage
processor = DataProcessor(state_machine)
processor.initialize()  # IDLE → READY

# Each call automatically manages state
result1 = processor.process_item("data1")  # READY → RUNNING → READY
result2 = processor.process_item("data2")  # READY → RUNNING → READY
```

#### Nested Operations

Multiple operations can be active simultaneously:

```python
class BatchProcessor(state.OperationalStateMachine):
    @state.operation
    def process_batch(self, items):
        """Process multiple items."""
        # Counter: 0 → 1, State: READY → RUNNING
        results = []
        for item in items:
            result = self.process_single(item)
            # During inner: counter 1 → 2 → 1, stays RUNNING
            results.append(result)
        # Counter: 1 → 0, State: RUNNING → READY
        return results
    
    @state.operation
    def process_single(self, item):
        """Process one item."""
        # Counter: n → n+1 (increments during outer operation)
        result = self._transform(item)
        # Counter: n+1 → n (decrements but may stay > 0)
        return result
```

State flow:
1. `process_batch` starts: counter 0→1, READY→RUNNING
2. First `process_single`: counter 1→2, stays RUNNING
3. First `process_single` ends: counter 2→1, stays RUNNING
4. Last `process_single` ends: counter 2→1, stays RUNNING
5. `process_batch` ends: counter 1→0, RUNNING→READY

#### Custom State Requirements

Restrict operations to specific states:

```python
class RestrictedProcessor(state.OperationalStateMachine):
    @state.operation(required_states={state.OperationalState.RUNNING})
    def critical_task(self):
        """Can only run when already in RUNNING state."""
        # Ensures another operation is already active
        return self._do_critical_work()
    
    @state.operation(required_states={state.OperationalState.READY})
    def initialization_task(self):
        """Must start from READY (not from RUNNING)."""
        return self._setup_fresh()
```

#### Manual State Transitions

Disable reference counting for custom control:

```python
class ManualProcessor(state.OperationalStateMachine):
    @state.operation(
        required_states={state.OperationalState.READY},
        pre_transition=state.OperationalState.RUNNING,
        success_transition=state.OperationalState.READY,
        failure_transition=state.OperationalState.ERROR,
        use_reference_counting=False
    )
    def manual_operation(self, data):
        """Operation with explicit state transitions."""
        # Before: READY → RUNNING (no counter)
        result = self._process(data)
        # Success: RUNNING → READY (no counter)
        # Failure: RUNNING → ERROR (no counter)
        return result
```

#### Long-Running Async Operations

For truly asynchronous operations, manually manage the counter:

```python
class AsyncProcessor(state.OperationalStateMachine):
    def __init__(self, state_machine):
        super().__init__(state_machine)
        self.active_tasks = {}
    
    def start_async_task(self, task_id, data):
        """Start background task without blocking."""
        # Validate state
        if not (self.is_ready() or self.is_running()):
            raise state.OperationalStateError("Cannot start task")
        
        # Manually increment counter
        self._increment_operation_counter()
        
        # Schedule async work
        task = self._schedule_background(data, 
            callback=lambda r: self._on_task_complete(task_id, r))
        
        self.active_tasks[task_id] = task
        return task_id
    
    def _on_task_complete(self, task_id, result):
        """Called when async task finishes."""
        del self.active_tasks[task_id]
        
        # Manually decrement counter
        self._decrement_operation_counter()
        
        return result
```

## Mixing Static and Dynamic Methods

Combine static lifecycle control with dynamic operations:

```python
class HybridModule(state.OperationalStateMachine):
    def initialize(self):
        """Static: IDLE → READY"""
        self.resources = self._allocate()
        return True
    
    @state.operation
    def process_data(self, data):
        """Dynamic: READY ↔ RUNNING"""
        return self._transform(data)
    
    def pause(self):
        """Static: RUNNING → PAUSED"""
        self._save_checkpoint()
        return True
    
    def resume(self):
        """Static: PAUSED → READY (counter reset)"""
        self._restore_checkpoint()
        return True
    
    def stop(self):
        """Static: RUNNING/PAUSED → STOPPED"""
        self._finalize()
        return True
    
    def reset(self):
        """Static: STOPPED/ERROR → IDLE"""
        self.resources = None
        return True

# Usage
module = HybridModule(state_machine)

# Static lifecycle
module.initialize()          # IDLE → READY

# Dynamic operations
module.process_data("d1")    # READY → RUNNING → READY
module.process_data("d2")    # READY → RUNNING → READY

# Start operation to enter RUNNING
module._increment_operation_counter()  # READY → RUNNING

# Static pause while in RUNNING
module.pause()               # RUNNING → PAUSED

# Static resume
module.resume()              # PAUSED → READY (counter reset to 0)

# Static shutdown
module._increment_operation_counter()  # Enter RUNNING for stop
module.stop()                # RUNNING → STOPPED
module.reset()               # STOPPED → IDLE
```

## State Query Methods

Methods to inspect current state:

```python
# Operational state queries
def is_idle(self) -> bool
def is_ready(self) -> bool
def is_running(self) -> bool
def is_paused(self) -> bool
def is_stopped(self) -> bool
def is_error(self) -> bool

# Get current state
def get_operational_state(self) -> OperationalState

# Get all states (lifecycle, operational, health)
def get_all_states(self) -> Dict[str, str]

# Operation counter (for debugging/monitoring)
def get_operation_counter(self) -> int
```

**Example:**
```python
if module.is_ready():
    result = module.process(data)
elif module.is_error():
    module.reset()
    module.initialize()

# Monitoring
counter = module.get_operation_counter()
print(f"Active operations: {counter}")

states = module.get_all_states()
print(f"Lifecycle: {states['lifecycle']}, "
      f"Operational: {states['operational']}, "
      f"Health: {states['health']}")
```

## Error Handling

### ERROR State

When any lifecycle method (`on_initialize`, `on_pause`, `on_resume`, `on_stop`) returns `False`, the module transitions to ERROR state.

**Recovery from ERROR:**
```python
# Method 1: Reset and reinitialize
if module.is_error():
    module.reset()      # ERROR → IDLE
    module.initialize() # IDLE → READY

# Method 2: Check before operations
@state.operation
def safe_operation(self, data):
    """Operation with error handling."""
    try:
        return self._risky_work(data)
    except Exception as e:
        self.logger.error(f"Operation failed: {e}")
        raise  # Decorator handles counter cleanup

# Usage
try:
    module.safe_operation("data")
except Exception:
    # Counter properly decremented, state consistent
    if module.is_error():
        module.reset()
        module.initialize()
```

### Exception Handling in Operations

The decorator ensures counter consistency even with exceptions:

```python
@state.operation
def processing_with_errors(self, data):
    """Proper exception handling."""
    # Counter incremented, state → RUNNING
    
    if not self._validate(data):
        # Return False signals failure
        return False
    
    try:
        result = self._might_fail(data)
    except ValueError:
        # Re-raise after cleanup
        raise
    finally:
        # Counter ALWAYS decremented by decorator
        pass
    
    return result

# Usage
try:
    result = module.processing_with_errors(bad_data)
except ValueError:
    # Counter properly cleaned up
    # State returned to READY if counter reached 0
    pass
```

## Best Practices

### 1. Counter Reset on Initialize/Resume

The operation counter is automatically reset when:
- `initialize()` succeeds: Ensures clean state after startup
- `resume()` succeeds: Clears any stale counter state after pause

### 2. Always Reset from ERROR

Never try to continue from ERROR state - always reset first:

```python
# ✅ CORRECT
if module.is_error():
    module.reset()
    module.initialize()

# ❌ WRONG - don't try to operate in ERROR state
if module.is_error():
    module.process_data(data)  # Will raise OperationalStateError
```

### 3. Check State Before Operations

While the decorator validates state, explicit checks improve clarity:

```python
# Explicit check
if module.is_ready() or module.is_running():
    result = module.process(data)
else:
    logger.warning(f"Cannot process in state {module.get_operational_state()}")
```

### 4. Monitor Operation Counter

For debugging or monitoring active operations:

```python
counter = module.get_operation_counter()
if counter > 0:
    logger.info(f"{counter} operations in progress")
    
# Warning: state/counter mismatch
if module.is_running() and counter == 0:
    logger.warning("RUNNING state but no active operations")
```

### 5. Return Bool from Lifecycle Methods

Always return `True` (success) or `False` (failure):

```python
def initialize(self):
    try:
        self._setup()
        return True  # Success
    except Exception as e:
        logger.error(f"Failed: {e}")
        return False  # Failure → ERROR state
```

### 6. Use Lifecycle Methods for State Control

Use static methods for deterministic state changes:
- `initialize()` for startup
- `pause()`/`resume()` for suspension
- `stop()` for shutdown
- `reset()` for recovery

Use `@operation` for runtime work:
- Data processing
- Request handling
- Task execution

### 7. Handle Nested Operations Carefully

Understand counter behavior with nested calls:

```python
@state.operation
def outer(self):
    # Counter: 0 → 1, READY → RUNNING
    self.inner()  # Counter: 1 → 2 → 1, stays RUNNING
    # Counter: 1 → 0, RUNNING → READY

@state.operation  
def inner(self):
    # Nested call increments/decrements counter
    pass
```

## Industrial Compliance

The state machine follows industrial automation standards:

- **IEC 61508**: Functional safety
- **IEC 61131-3**: Programmable controller programming  
- **ISO 13849**: Safety-related control systems

**Key compliance features:**
- Deterministic state transitions
- Clear error states and recovery paths
- Comprehensive state validation
- Operation tracking and monitoring

## Troubleshooting

### "Cannot call operation in state X"

**Cause**: Operation called from invalid state (usually IDLE or PAUSED)

**Solution**: Initialize first or check state
```python
if module.is_idle():
    module.initialize()
module.process(data)
```

### State Stuck in RUNNING

**Cause**: Operation counter not decremented (rare with decorator)

**Solution**: Reset counter via resume() or check for exceptions
```python
# Emergency counter reset
module.resume()  # Resets counter to 0 if in PAUSED
# OR
logger.warning(f"Counter stuck at {module.get_operation_counter()}")
```

### "Cannot call on_stop in state Ready"

**Cause**: Trying to stop when no operations running

**Solution**: Only stop from RUNNING or PAUSED
```python
if module.is_running() or module.is_paused():
    module.stop()
```

### Module Stuck in ERROR

**Cause**: Lifecycle method failure

**Solution**: Always reset from ERROR
```python
module.reset()      # ERROR → IDLE
module.initialize() # IDLE → READY
```

## Migration from on_start()

For legacy code using `on_start()` method:

### Old Pattern
```python
class OldModule(OperationalStateMachine):
    def initialize(self):
        return True
    
    def on_start(self):
        # Ran on every transition to RUNNING
        return True
    
    def process(self):
        # Manual state management
        pass
```

### New Pattern
```python
class NewModule(OperationalStateMachine):
    def initialize(self):
        # Goes directly to READY (not RUNNING)
        return True
    
    @state.operation
    def process(self):
        # Automatically manages READY ↔ RUNNING
        return True
```

**Key Differences:**
1. No `start()` method - use `@operation` instead
2. `initialize()` goes to READY (not RUNNING)
3. Reference counting manages transitions automatically
4. Multiple concurrent operations supported

## Complete Example

```python
from vyra_base import state

class ProductionLine(state.OperationalStateMachine):
    """Complete example showing all features."""
    
    def __init__(self, state_machine):
        super().__init__(state_machine)
        self.hardware = None
        self.items_processed = 0
        self.checkpoint = None
    
    # Static Lifecycle Methods
    
    def initialize(self):
        """IDLE → READY: Initialize production line."""
        try:
            self.hardware = self._connect_hardware()
            self.items_processed = 0
            return self.hardware.test_connection()
        except Exception as e:
            self.logger.error(f"Init failed: {e}")
            return False  # Goes to ERROR
    
    def pause(self):
        """RUNNING → PAUSED: Pause production."""
        self.checkpoint = self._save_state()
        self.hardware.pause()
        return True
    
    def resume(self):
        """PAUSED → READY: Resume production."""
        self.hardware.resume()
        if self.checkpoint:
            self._restore_state(self.checkpoint)
        return True  # Counter reset to 0
    
    def stop(self):
        """RUNNING/PAUSED → STOPPED: Stop production."""
        self.hardware.stop()
        self._save_statistics()
        return True
    
    def reset(self):
        """STOPPED/ERROR → IDLE: Reset line."""
        if self.hardware:
            self.hardware.disconnect()
            self.hardware = None
        self.items_processed = 0
        self.checkpoint = None
        return True
    
    # Dynamic Operations
    
    @state.operation
    def process_item(self, item):
        """Process single item with automatic state management."""
        # READY → RUNNING (if counter was 0)
        result = self.hardware.process(item)
        self.items_processed += 1
        # RUNNING → READY (if counter becomes 0)
        return result
    
    @state.operation
    def process_batch(self, items):
        """Process multiple items (nested operations)."""
        # Counter: 0 → 1, READY → RUNNING
        results = []
        for item in items:
            # Nested: counter temporarily goes higher
            result = self.process_item(item)
            results.append(result)
        # Counter: 1 → 0, RUNNING → READY
        return results
    
    @state.operation(required_states={state.OperationalState.RUNNING})
    def emergency_override(self):
        """Special operation requiring RUNNING state."""
        return self.hardware.emergency_action()

# Complete Usage Scenario
line = ProductionLine(state_machine)

# Startup
line.initialize()                        # IDLE → READY

# Normal operation
result1 = line.process_item("item1")     # READY → RUNNING → READY
result2 = line.process_item("item2")     # READY → RUNNING → READY

# Batch processing
batch = line.process_batch(["i1", "i2"]) # Nested operations

# Pause/Resume
line._increment_operation_counter()      # READY → RUNNING
line.pause()                             # RUNNING → PAUSED
line.resume()                            # PAUSED → READY (counter reset)

# Error recovery
try:
    line.process_item("bad_item")
except Exception:
    if line.is_error():
        line.reset()                     # ERROR → IDLE
        line.initialize()                # IDLE → READY

# Shutdown
line._increment_operation_counter()      # READY → RUNNING
line.stop()                              # RUNNING → STOPPED
line.reset()                             # STOPPED → IDLE
```

## See Also

- **state_machine.py**: 3-layer state machine core implementation
- **operational_metaclass.py**: Automatic method wrapping mechanism
- **operation_decorator.py**: @operation decorator implementation
- **state_types.py**: State definitions and valid transitions
- **state_events.py**: Event types and event-to-state mappings

## Version

This documentation corresponds to vyra_base_python with:
- Operation decorator with reference counting
- ERROR operational state  
- Automatic counter reset on initialize/resume
- Static and dynamic method support
