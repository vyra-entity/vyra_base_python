# Example: Custom Handler Implementation

Learn how to create a custom communication handler in VYRA.

## Handler Types

VYRA provides several built-in handlers:
- **ROS2Handler** - ROS2 DDS communication
- **ZenohHandler** - Zenoh pub/sub queries
- **RedisHandler** - Redis pub/sub
- **UDSHandler** - Unix Domain Socket
- **DatabaseHandler** - Database persistence

## Custom Handler Template

```python
from vyra_base.com.handler import CommunicationHandler
from vyra_base.com.handler.interfaces import PublisherInterface
from typing import Any, Dict
import logging

logger = logging.getLogger(__name__)

class CustomHandler(CommunicationHandler):
    """
    Example custom communication handler.
    Implements a simple in-memory message queue.
    """
    
    def __init__(self, config: Dict[str, Any] = None):
        super().__init__(config or {})
        self.message_queue = {}
        self.subscriptions = {}
        logger.info("✅ CustomHandler initialized")
    
    async def setup(self) -> bool:
        """Setup handler resources."""
        logger.info("🔧 Setting up CustomHandler...")
        # Validate configuration
        if not self.config:
            self.config = {}
        return True
    
    async def shutdown(self) -> bool:
        """Clean shutdown."""
        logger.info("🛑 Shutting down CustomHandler...")
        self.message_queue.clear()
        self.subscriptions.clear()
        return True
    
    async def publish(self, topic: str, message: Any) -> bool:
        """Publish a message to a topic."""
        logger.info(f"📤 Publishing to {topic}: {message}")
        
        # Store in queue
        if topic not in self.message_queue:
            self.message_queue[topic] = []
        self.message_queue[topic].append(message)
        
        # Notify subscribers
        if topic in self.subscriptions:
            for callback in self.subscriptions[topic]:
                try:
                    await callback(message)
                except Exception as e:
                    logger.error(f"❌ Callback error: {e}")
        
        return True
    
    async def subscribe(self, topic: str, callback) -> bool:
        """Subscribe to topic updates."""
        logger.info(f"📥 Subscribing to {topic}")
        
        if topic not in self.subscriptions:
            self.subscriptions[topic] = []
        self.subscriptions[topic].append(callback)
        
        return True
    
    async def unsubscribe(self, topic: str, callback) -> bool:
        """Unsubscribe from topic."""
        logger.info(f"🚫 Unsubscribing from {topic}")
        
        if topic in self.subscriptions:
            self.subscriptions[topic].remove(callback)
        
        return True
    
    async def get_topics(self) -> list:
        """List all available topics."""
        return list(self.message_queue.keys())
    
    async def get_status(self) -> Dict[str, Any]:
        """Get handler status."""
        return {
            "topics": len(self.message_queue),
            "subscribers": sum(len(v) for v in self.subscriptions.values()),
            "messages": sum(len(v) for v in self.message_queue.values()),
        }
```

## Usage Example

```python
import asyncio

async def main():
    # Create handler instance
    handler = CustomHandler()
    
    # Setup
    await handler.setup()
    
    # Define callback
    async def on_message(message):
        print(f"📨 Received: {message}")
    
    # Subscribe
    await handler.subscribe("events", on_message)
    
    # Publish messages
    await handler.publish("events", {"event": "start", "time": 123456})
    await handler.publish("events", {"event": "stop", "time": 123789})
    
    # Check status
    status = await handler.get_status()
    print(f"Status: {status}")
    # Output: Status: {'topics': 1, 'subscribers': 1, 'messages': 2}
    
    # List topics
    topics = await handler.get_topics()
    print(f"Topics: {topics}")
    # Output: Topics: ['events']
    
    # Cleanup
    await handler.shutdown()

# Run
if __name__ == "__main__":
    asyncio.run(main())
```

## Integrating with VyraEntity

```python
from vyra_base import VyraEntity

async def entity_example():
    # Create entity
    entity = VyraEntity("my_module")
    
    # Create handler
    handler = CustomHandler({
        "buffer_size": 1000,
        "timeout": 5.0
    })
    
    # Register handler (would need to wire into entity interfaces)
    # entity.set_handler(handler)
    
    # Setup
    await entity.initialize()
    
    # Use handler
    await handler.publish("status", {"state": "running"})
    
    # Cleanup
    await entity.shutdown()
```

## Key Methods to Implement

| Method | Purpose | Returns |
|--------|---------|---------|
| `setup()` | Initialize resources | bool |
| `shutdown()` | Clean shutdown | bool |
| `publish(topic, message)` | Send message | bool |
| `subscribe(topic, callback)` | Register listener | bool |
| `unsubscribe(topic, callback)` | Unregister listener | bool |
| `get_topics()` | List all topics | list |
| `get_status()` | Handler status | dict |

## Best Practices

1. **Error Handling**: Wrap callbacks in try/except
2. **Logging**: Use structured logging for debugging
3. **Cleanup**: Properly shutdown resources
4. **Async**: Use async/await for I/O operations
5. **Configuration**: Accept config dict for flexibility
6. **Testing**: Create mock handlers for testing

## Testing Your Handler

```python
import pytest

@pytest.mark.asyncio
async def test_custom_handler():
    handler = CustomHandler()
    
    # Setup
    assert await handler.setup() == True
    
    # Subscribe
    received = []
    async def callback(msg):
        received.append(msg)
    
    await handler.subscribe("test", callback)
    
    # Publish
    test_msg = {"type": "test"}
    await handler.publish("test", test_msg)
    
    # Verify
    assert len(received) == 1
    assert received[0] == test_msg
    
    # Cleanup
    assert await handler.shutdown() == True
```

## Reference

- [Communication Layer Overview](../../components/communication/overview.rst)
- [Handler Interfaces](../../components/communication/handler/interfaces.rst)
- [Built-in Handlers](../../components/communication/handler/)
