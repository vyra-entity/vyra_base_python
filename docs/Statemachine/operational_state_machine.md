# Operational State Machine with Metaclass

## Overview

The `OperationalStateMachine` provides automatic operational state management through a metaclass-based system. This enables clean, declarative code where lifecycle methods automatically handle state transitions, validation, and error handling.

## Architecture

### Components

1. **`MetaOperationalState`** (Metaclass)
   - Automatically wraps lifecycle methods (prefixed with `on_`)
   - Validates pre-conditions before execution
   - Manages state transitions on success/failure
   - Handles exceptions and logging

2. **`OperationalStateMachine`** (Base Class)
   - Inherits from `MetaOperationalState` metaclass
   - Integrates with 3-layer `StateMachine`
   - Provides public API methods (initialize, start, pause, etc.)
   - Routes method calls to user-defined `on_*` methods

3. **User Module Classes**
   - Inherit from `OperationalStateMachine`
   - Implement `on_*` methods for custom logic
   - Get automatic state management for free

## Lifecycle Methods and State Transitions

### `on_initialize()`
**Purpose**: Setup and initialization logic

**State Transitions**:
- **Pre-condition**: Must be in `IDLE` state
- **Pre-transition**: `IDLE` → `READY` (automatic)
- **On success**: `READY` → `RUNNING`
- **On failure**: `READY` → `STOPPED`

**Example**:
```python
def on_initialize(self):
    """Initialize hardware and load configuration."""
    self.config = load_config()
    self.hardware = setup_hardware()
    return True  # Success
```

### `on_start()`
**Purpose**: Start main processing

**State Transitions**:
- **Pre-condition**: Must be in `READY` state
- **On success**: `READY` → `RUNNING`
- **On failure**: `READY` → `STOPPED`

**Example**:
```python
def on_start(self):
    """Start task processing."""
    self.start_background_workers()
    return True  # Success
```

### `on_pause()`
**Purpose**: Pause current operation

**State Transitions**:
- **Pre-condition**: Must be in `RUNNING` state
- **On success**: Current state → `PAUSED`
- **On failure**: No state change

**Example**:
```python
def on_pause(self):
    """Pause task processing."""
    self.save_checkpoint()
    return True  # Success
```

### `on_resume()`
**Purpose**: Resume paused operation

**State Transitions**:
- **Pre-condition**: Must be in `PAUSED` state
- **On success**: `PAUSED` → `READY`
- **On failure**: `PAUSED` → `STOPPED`

**Example**:
```python
def on_resume(self):
    """Resume from paused state."""
    self.restore_checkpoint()
    return True  # Success
```

### `on_stop()`
**Purpose**: Stop current operation

**State Transitions**:
- **Pre-condition**: Must be in `RUNNING` or `PAUSED` state
- **On success**: Current state → `STOPPED`
- **On failure**: No state change

**Example**:
```python
def on_stop(self):
    """Stop processing and cleanup."""
    self.cleanup_resources()
    return True  # Success
```

### `on_reset()`
**Purpose**: Reset to initial state

**State Transitions**:
- **Pre-condition**: Must be in `STOPPED` state
- **On success**: `STOPPED` → `IDLE`
- **On failure**: No state change

**Example**:
```python
def on_reset(self):
    """Reset to initial state."""
    self.clear_all_data()
    return True  # Success
```


## Usage Examples

### Basic Usage

```python
from vyra_base.state import (
    StateMachine,
    OperationalStateMachine,
)

class MyModule(OperationalStateMachine):
    """Custom module with automatic state management."""
    
    def __init__(self, state_machine):
        super().__init__(state_machine)
        self.data = None
    
    def on_initialize(self):
        """Initialize module."""
        print("Initializing...")
        self.data = []
        return True  # Success
    
    def on_start(self):
        """Start processing."""
        print("Starting...")
        return True  # Success

# Create state machine and module
fsm = StateMachine()
module = MyModule(fsm)

# Use the module - automatic state management!
module.initialize()  # IDLE -> READY -> RUNNING
print(f"State: {module.get_operational_state().value}")  # "Running"
```

### Error Handling

Methods can return `False` to indicate failure, or raise exceptions:

```python
class RobustModule(OperationalStateMachine):
    
    def on_initialize(self):
        """Initialize with validation."""
        if not self.validate_config():
            return False  # Failure: READY -> STOPPED
        
        if not self.connect_to_hardware():
            raise Exception("Hardware connection failed")  # Also triggers failure
        
        return True  # Success: READY -> RUNNING
```

### Full Lifecycle Example

```python
class ProductionModule(OperationalStateMachine):
    
    def on_initialize(self):
        """Setup phase."""
        self.load_configuration()
        self.initialize_hardware()
        return True
    
    def on_start(self):
        """Production start."""
        self.begin_processing()
        return True
    
    def on_pause(self):
        """Temporary pause."""
        self.save_state()
        return True
    
    def on_resume(self):
        """Resume production."""
        self.restore_state()
        return True
    
    def on_stop(self):
        """Clean shutdown."""
        self.finalize_processing()
        return True
    
    def on_reset(self):
        """Reset for next run."""
        self.clear_all_state()
        return True

# Usage
fsm = StateMachine()
module = ProductionModule(fsm)

module.initialize()  # Setup and start
module.pause()       # Temporary pause
module.resume()      # Continue
module.stop()        # Clean stop
module.reset()       # Back to IDLE
```

### Integration with UnifiedStateMachine

```python
from vyra_base.state import UnifiedStateMachine

# Use unified state machine for full 3-layer control
usm = UnifiedStateMachine()

# Lifecycle layer: Start the module
usm.start()  # OFFLINE -> INITIALIZING
usm.complete_initialization()  # INITIALIZING -> ACTIVE

# Operational layer: Use OperationalStateMachine
module = ProductionModule(usm.fsm)
module.initialize()  # Automatic operational state management

# Access all layers
print(usm.get_all_states())
# {'lifecycle': 'Active', 'operational': 'Running', 'health': 'Healthy'}
```

## State Validation

The metaclass automatically validates that methods are called in the correct state:

```python
module = MyModule(fsm)

# Current state: IDLE
module.start()  # ❌ Raises OperationalStateError
# "Cannot call on_start in state Idle. Required states: ['Ready']"

module.initialize()  # ✓ Correct: IDLE -> READY -> RUNNING
module.start()  # ✓ Now would work if state was READY
```

## Return Values

Methods should return:
- `True`: Success (triggers success transition)
- `False`: Failure (triggers failure transition)
- `None`: Treated as success
- Exception: Treated as failure

```python
def on_initialize(self):
    # Explicit success
    return True
    
def on_start(self):
    # Implicit success (no return)
    self.do_something()
    
def on_stop(self):
    # Explicit failure
    if not self.can_stop():
        return False
    self.stop()
    return True
```

## Benefits

1. **Automatic State Management**: No manual state transition code
2. **Validation**: Pre-conditions checked automatically
3. **Error Handling**: Exceptions caught and handled gracefully
4. **Logging**: Built-in logging for debugging
5. **Clean Code**: Focus on business logic, not state management
6. **Type Safety**: Enforced through enum types
7. **Industrial Standards**: Follows IEC 61508, ISO 13849

## Advanced Features

### Custom Validation

```python
class ValidatedModule(OperationalStateMachine):
    
    def on_initialize(self):
        """Initialize with custom validation."""
        # Check prerequisites
        if not self.check_prerequisites():
            logger.error("Prerequisites not met")
            return False
        
        # Perform initialization
        self.setup()
        
        # Validate results
        if not self.validate_setup():
            logger.error("Setup validation failed")
            return False
        
        return True  # All checks passed
```

### State Queries

```python
# Query current state
current = module.get_operational_state()  # Returns OperationalState enum

# Convenience methods
if module.is_running():
    print("Module is running")

if module.is_idle():
    print("Module is idle")

# Get all states (lifecycle, operational, health)
all_states = module.get_all_states()
```

### Direct State Machine Access

```python
# Access underlying state machine for advanced use
fsm = module._state_machine

# Send custom events
from vyra_base.state import StateEvent, EventType
event = StateEvent(EventType.WARN, metadata={"cpu": "85%"})
fsm.send_event(event)
```

## Best Practices

1. **Return Early**: Return `False` immediately on validation failure
2. **Log Errors**: Use logging to document why operations failed
3. **Validate First**: Check prerequisites before doing work
4. **Clean Failure**: Always leave system in consistent state on failure
5. **Document States**: Document which states your methods expect
6. **Test Transitions**: Test both success and failure paths

## Comparison: Before and After

### Before (Manual State Management)

```python
class OldModule:
    def initialize(self):
        # Check state
        if self.state != State.IDLE:
            raise Exception("Wrong state")
        
        # Transition to READY
        self.state = State.READY
        
        try:
            # Do work
            self.setup()
            
            # Success transition
            self.state = State.RUNNING
            return True
        except Exception as e:
            # Failure transition
            self.state = State.STOPPED
            return False
```

### After (Metaclass-Based)

```python
class NewModule(OperationalStateMachine):
    def on_initialize(self):
        """Initialize module."""
        self.setup()
        return True  # All state management automatic!
```

## See Also

- `examples/operational_state_machine_example.py` - Complete examples
- `state_machine.py` - Core 3-layer state machine
- `unified.py` - Unified state machine interface
- `state_types.py` - State definitions and transition rules
