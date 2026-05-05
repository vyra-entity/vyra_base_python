# Quick Start Guide

Get up and running with VYRA in 10 minutes.

## Installation

```bash
pip install vyra-base
```

## Basic Setup

```python
from vyra_base import VyraEntity, ROS2Transports

# Create entity
entity = VyraEntity("my_module")

# Set up interfaces
await entity.set_interfaces(ROS2Transports())

# Initialize
await entity.initialize()
```

## State Machine

```python
from vyra_base.state import OperationalStateMachine

class MyModule(OperationalStateMachine):
    def initialize(self):
        return True
    
    @operation
    def process(self, data):
        return self._work(data)
```

## Documentation

- [State Machine Guide](../guides/state-machine-quick-guide.md)
- [Interfaces How-To](../guides/interfaces-how-to.md)
- [Logging Setup](../guides/logging-setup.md)

## Next Steps

1. Read [State Machine Quick Guide](../guides/state-machine-quick-guide.md)
2. Explore [Examples](../guides/examples/)
3. Check [API Reference](../api-reference/)
