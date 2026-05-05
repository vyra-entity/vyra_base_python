# Example: Basic State Machine

A simple example showing how to use VYRA's Operational State Machine.

## Setup

```python
from vyra_base.state import OperationalStateMachine, operation
import asyncio
```

## Simple Module

```python
class DataProcessor(OperationalStateMachine):
    """Example module with basic state management."""
    
    def __init__(self, state_machine):
        super().__init__(state_machine)
        self.data = []
        self.processed_count = 0
    
    def initialize(self):
        """Initialize module resources."""
        print("🔧 Initializing DataProcessor...")
        self.data = []
        self.processed_count = 0
        return True
    
    @operation
    def process_item(self, item):
        """Process a single item."""
        print(f"⚙️  Processing: {item}")
        # Simulate work
        result = str(item).upper()
        self.data.append(result)
        self.processed_count += 1
        return result
    
    @operation
    def batch_process(self, items):
        """Process multiple items."""
        print(f"📦 Batch processing {len(items)} items...")
        results = []
        for item in items:
            result = self.process_item(item)
            results.append(result)
        return results
    
    def pause(self):
        """Pause processing."""
        print("⏸️  Pausing...")
        return True
    
    def resume(self):
        """Resume processing."""
        print("▶️  Resuming...")
        return True
    
    def stop(self):
        """Stop processing cleanly."""
        print(f"🛑 Stopping... (processed {self.processed_count} items)")
        return True
    
    def reset(self):
        """Reset to initial state."""
        print("🔄 Resetting...")
        self.data = []
        self.processed_count = 0
        return True
```

## Usage Example

```python
async def main():
    from vyra_base.state import StateMachine
    
    # Create state machine
    fsm = StateMachine()
    
    # Create processor
    processor = DataProcessor(fsm)
    
    # Initialize
    print("State:", processor.get_operational_state())
    # Output: State: <OperationalState.IDLE: 'IDLE'>
    
    processor.initialize()
    print("State:", processor.get_operational_state())
    # Output: State: <OperationalState.READY: 'READY'>
    
    # Process single items
    result1 = processor.process_item("hello")
    print(f"Result: {result1}")
    # Output: Result: HELLO
    # State: RUNNING -> READY (automatic)
    
    # Process batch
    results = processor.batch_process(["world", "vyra", "rocks"])
    print(f"Batch results: {results}")
    # Output: Batch results: ['WORLD', 'VYRA', 'ROCKS']
    
    # Pause/Resume
    processor.pause()
    print("State:", processor.get_operational_state())
    # Output: State: <OperationalState.PAUSED: 'PAUSED'>
    
    processor.resume()
    print("State:", processor.get_operational_state())
    # Output: State: <OperationalState.READY: 'READY'>
    
    # Stop
    processor.stop()
    print("State:", processor.get_operational_state())
    # Output: State: <OperationalState.STOPPED: 'STOPPED'>
    
    # Reset
    processor.reset()
    print("State:", processor.get_operational_state())
    # Output: State: <OperationalState.IDLE: 'IDLE'>

# Run
if __name__ == "__main__":
    asyncio.run(main())
```

## Output

```
🔧 Initializing DataProcessor...
State: <OperationalState.IDLE: 'IDLE'>
State: <OperationalState.READY: 'READY'>
⚙️  Processing: hello
Result: HELLO
State: <OperationalState.RUNNING: 'RUNNING'>
📦 Batch processing 3 items...
⚙️  Processing: world
⚙️  Processing: vyra
⚙️  Processing: rocks
Batch results: ['WORLD', 'VYRA', 'ROCKS']
⏸️  Pausing...
State: <OperationalState.PAUSED: 'PAUSED'>
▶️  Resuming...
State: <OperationalState.READY: 'READY'>
🛑 Stopping... (processed 4 items)
State: <OperationalState.STOPPED: 'STOPPED'>
🔄 Resetting...
State: <OperationalState.IDLE: 'IDLE'>
```

## Key Points

1. **Inherit from OperationalStateMachine** - Base class provides state management
2. **Implement lifecycle methods** - `initialize()`, `pause()`, `resume()`, `stop()`, `reset()`
3. **Use @operation decorator** - Automatic state transitions for operations
4. **Check current state** - Use `get_operational_state()` to inspect state

## Reference Counting

Notice how multiple `process_item()` calls within `batch_process()` work:

```
batch_process starts: READY -> RUNNING (counter: 0->1)
  process_item #1: counter: 1->2 (stays RUNNING)
  process_item #1 ends: counter: 2->1 (stays RUNNING)
  process_item #2: counter: 1->2 (stays RUNNING)
  process_item #2 ends: counter: 2->1 (stays RUNNING)
  process_item #3: counter: 1->2 (stays RUNNING)
  process_item #3 ends: counter: 2->1 (stays RUNNING)
batch_process ends: counter: 1->0, RUNNING -> READY
```

The reference counting ensures the state only transitions back to READY when all operations complete.

## Next Steps

- See [State Machine Full API](../../components/state-machine/state-machine_FULL.rst) for advanced features
- See [State Machine Quick Reference](../state-machine-quick-guide.md) for common patterns
- Combine with [Custom Handlers](./custom-handler.md) for communication
