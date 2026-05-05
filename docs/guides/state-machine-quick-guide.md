# State Machine Quick Guide

## Basic Pattern

```python
class Module(OperationalStateMachine):
    def initialize(self):
        # Setup
        return True
    
    @operation
    def work(self):
        # Process
        return result
    
    def stop(self):
        # Cleanup
        return True
```

## State Flow

```
IDLE --initialize()--> READY --work()--> RUNNING
                         ↑                   ↓
                         +---pause()/stop()--+
```

## Common Operations

| Need | Method | Pre-State | Post-State |
|------|--------|-----------|-----------|
| Setup | initialize() | IDLE | READY |
| Start work | @operation | READY | RUNNING |
| Pause | pause() | RUNNING | PAUSED |
| Resume | resume() | PAUSED | READY |
| Stop | stop() | RUNNING | STOPPED |
| Reset | reset() | STOPPED | IDLE |

## See Also

- Full reference: [State Machine Full API](../components/state-machine/state-machine_FULL.rst)
