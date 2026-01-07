# Quick Reference: OperationalStateMachine

## Basic Template

```python
from vyra_base.state import (
    StateMachine,
    OperationalStateMachine,
    StateEvent,
    EventType,
)

class MyModule(OperationalStateMachine):
    def __init__(self, state_machine):
        super().__init__(state_machine)
        self.data = None
    
    def on_initialize(self):
        """Setup and initialization."""
        self.data = []
        return True  # Success
    
    def on_start(self):
        """Start main processing."""
        return True

# Usage
fsm = StateMachine()
fsm.send_event(StateEvent(EventType.START))
fsm.send_event(StateEvent(EventType.INIT_SUCCESS))

module = MyModule(fsm)
module.initialize()  # Automatic: IDLE→READY→RUNNING
```

## Lifecycle Methods Quick Reference

| Method | Call When | Pre-State | Result State |
|--------|-----------|-----------|--------------|
| `initialize()` | Module startup | IDLE | RUNNING (success)<br>STOPPED (failure) |
| `start()` | Begin processing | READY | RUNNING (success)<br>STOPPED (failure) |
| `pause()` | Temporary pause | RUNNING | PAUSED |
| `resume()` | Resume after pause | PAUSED | READY (success)<br>STOPPED (failure) |
| `stop()` | End processing | RUNNING/PAUSED | STOPPED |
| `reset()` | Return to initial | STOPPED | IDLE |

## Return Values

```python
def on_initialize(self):
    return True    # ✓ Success - trigger success transition
    return False   # ✗ Failure - trigger failure transition
    return None    # ✓ Treated as success
    # Exception    # ✗ Treated as failure
```

## State Queries

```python
# Get current state
state = module.get_operational_state()  # Returns OperationalState enum

# Convenience checks
if module.is_idle():
    module.initialize()

if module.is_ready():
    module.start()

if module.is_running():
    module.pause()

# Get all states (3 layers)
all_states = module.get_all_states()
# {'lifecycle': 'Active', 'operational': 'Running', 'health': 'Healthy'}
```

## Error Handling

```python
from vyra_base.state import OperationalStateError

try:
    module.start()  # Fails if not in READY state
except OperationalStateError as e:
    print(f"Invalid state: {e}")
```

## State Transition Diagram

```
IDLE ───initialize()──→ READY ───start()──→ RUNNING
 ↑                        ↑                     ↓
 │                        │                  pause()
 │                        │                     ↓
 │                     resume()              PAUSED
 │                        ↓                     ↓
 └─────reset()────── STOPPED ←────stop()───────┘
```

## Common Patterns

### Pattern 1: Simple Start/Stop

```python
class SimpleModule(OperationalStateMachine):
    def on_initialize(self):
        self.setup()
        return True
    
    def on_stop(self):
        self.cleanup()
        return True

module.initialize()  # Setup and start
module.stop()        # Stop
module.reset()       # Reset to IDLE
```

### Pattern 2: With Pause/Resume

```python
class ProcessingModule(OperationalStateMachine):
    def on_initialize(self):
        self.load_data()
        return True
    
    def on_pause(self):
        self.save_checkpoint()
        return True
    
    def on_resume(self):
        self.restore_checkpoint()
        return True

module.initialize()  # Start processing
module.pause()       # Pause
module.resume()      # Resume
module.start()       # Continue processing
```

### Pattern 3: With Validation

```python
class ValidatedModule(OperationalStateMachine):
    def on_initialize(self):
        if not self.check_prerequisites():
            return False  # Failure → STOPPED
        
        self.setup()
        
        if not self.validate_setup():
            return False  # Failure → STOPPED
        
        return True  # Success → RUNNING
```

### Pattern 4: With Error Handling

```python
class RobustModule(OperationalStateMachine):
    def on_initialize(self):
        try:
            self.connect_hardware()
            self.load_config()
            return True
        except Exception as e:
            self.logger.error(f"Init failed: {e}")
            return False  # Automatic transition to STOPPED
```

## Integration with UnifiedStateMachine

```python
from vyra_base.state import UnifiedStateMachine

# Use unified state machine for full 3-layer control
usm = UnifiedStateMachine()

# Lifecycle layer
usm.start()
usm.complete_initialization()

# Operational layer with metaclass
module = MyModule(usm.fsm)
module.initialize()

# Access all layers
print(usm.get_all_states())
```

## Debugging Tips

### Enable Logging

```python
import logging
logging.basicConfig(level=logging.DEBUG)

# You'll see automatic logs:
# INFO: Executing on_initialize from state Idle
# DEBUG: Pre-transition: Idle -> Ready
# INFO: Success transition: Ready -> Running
# INFO: on_initialize completed. Current state: Running
```

### Check State Before Operations

```python
print(f"Current state: {module.get_operational_state().value}")
print(f"Can initialize: {module.is_idle()}")
print(f"Can start: {module.is_ready()}")
```

### Handle State Errors Gracefully

```python
from vyra_base.state import OperationalStateError

try:
    module.start()
except OperationalStateError as e:
    print(f"Cannot start: {e}")
    # Get module back to correct state
    if module.is_idle():
        module.initialize()
```

## Do's and Don'ts

### ✅ Do

- Return `True` for success, `False` for failure
- Validate prerequisites before doing work
- Use appropriate lifecycle methods for operations
- Check state with `is_*()` methods before operations
- Log errors when returning `False`

### ❌ Don't

- Don't manually change `_state_machine` state
- Don't call `_set_operational_state()` directly
- Don't mix manual state management with metaclass methods
- Don't assume success - always handle failure paths
- Don't forget to start lifecycle layer before using operational methods

## Performance Notes

- State validation: O(1) - set membership check
- Transition execution: O(1) - dictionary lookup
- Callback notifications: O(n) where n = number of callbacks
- Negligible overhead for most applications

## See Also

- Full documentation: `docs/operational_state_machine.md`
- Examples: `examples/operational_state_machine_example.py`
- API reference: Docstrings in source code
- 3-layer state machine: `docs/state_machine_guide.md`
