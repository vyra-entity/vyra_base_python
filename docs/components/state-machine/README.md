# Operational State Machine Documentation

This directory contains the complete documentation for VYRA's Operational State Machine.

## Documentation Files

- **state-machine_FULL.rst** — Complete API Reference
  - Comprehensive overview of all states, transitions, methods
  - Detailed @operation decorator documentation
  - Advanced usage patterns and examples
  - **Target**: Developers implementing complex state behavior

- **state-machine_QUICK.rst** — Quick Reference
  - Quick lookup tables for common methods
  - State transition diagram
  - Common patterns and templates
  - **Target**: Developers needing quick answers

## Key Concepts

The Operational State Machine provides:

1. **Static Lifecycle Methods**: `initialize()`, `pause()`, `resume()`, `stop()`, `reset()`
2. **Dynamic Operations**: `@operation` decorator with automatic reference counting
3. **Three-Layer Architecture**: Lifecycle, Operational, and Health layers
4. **Automatic State Management**: States transition based on reference counting

## Quick Start

```python
from vyra_base import state

class MyModule(state.OperationalStateMachine):
    def initialize(self):
        # Your initialization code
        return True
    
    @state.operation
    def process_data(self, data):
        # Automatic state management
        return self._do_work(data)

# Usage
module = MyModule(state_machine)
module.initialize()  # IDLE → READY
result = module.process_data("test")  # READY → RUNNING → READY
```

## States Overview

| State | Purpose |
|-------|---------|
| IDLE | Initial state, not operational |
| READY | Initialized, ready for operations |
| RUNNING | One or more operations active |
| PAUSED | Temporarily suspended |
| STOPPED | Clean shutdown completed |
| ERROR | Failure occurred, needs recovery |

## See Also

- Parent component: [Communication Layer](../communication/)
- Related: [Interfaces](../interfaces/)
- Examples: [State Machine Examples](../../guides/examples/basic-state-machine.md)

---

**Note**: This documentation consolidated from previous scattered .md and .rst files.
All state definitions are now unified in these two files:
- `states.rst`, `events.rst`, `transitions.rst` (deprecated, content merged into FULL)
- `STATE_MACHINE.md`, `3_Layer_Statemachine.md` (deprecated, outdated states)
- `operational_state_machine.md` (deprecated, content merged into FULL and QUICK)
