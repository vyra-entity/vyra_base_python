# Feeders - Multi-Protocol Data Publishers

Automatic data distribution system with ROS2-optional support and protocol fallback.

## Overview

Feeders automatically publish data (state, news, errors) to multiple subscribers using the best available protocol. They integrate with VyraEntity for seamless module state management.

## Architecture

```
┌──────────────────────────────────────────────────┐
│               VyraEntity                         │
│  (publish_state, publish_news, publish_error)    │
└─────────────────┬────────────────────────────────┘
                  │
      ┌───────────┼────────────┐
      │           │            │
┌─────▼──────┐ ┌──▼────┐ ┌─────▼─────┐
│StateFeeder │ │News   │ │Error      │
│            │ │Feeder │ │Feeder     │
└────────────┘ └───────┘ └───────────┘
      │             │          │
      └─────────────┴──────────┘
                    │
         ┌──────────▼──────────┐
         │  InterfaceFactory   │
         │  (Protocol Fallback)│
         └─────────────────────┘
                    │
    ┌───────────────┼────────────┐
    │               │            │
┌───▼────┐    ┌─────▼────┐  ┌────▼───┐
│ Redis  │    │  ROS2    │  │  MQTT  │
│ Pub/Sub│    │ Topics   │  │ Pub/Sub│
└────────┘    └──────────┘  └────────┘
```

## Features

- **ROS2-Optional**: Works with or without ROS2 installed
- **Automatic Fallback**: Redis → ROS2 → MQTT → SharedMemory
- **Config-based Protocol Selection**: Protocol resolved automatically from interface config JSON
- **Actionable Error Messages**: Fuzzy-match suggestions when a feeder name is not found
- **Type Safety**: Native Python types or ROS2 messages
- **Async Support**: Full async/await integration
- **Entity Integration**: Seamless VyraEntity integration

---

## Protocol Resolution via Interface Config (FeederConfigResolver)

Jeder Feeder wählt sein Transportprotokoll **automatisch** aus den
Interface-Config-JSON-Dateien der Module.  Das vermeidet hartcodierte
Protokoll-Parameter und stellt sicher, dass Feeder immer zum konfigurierten
Transport des jeweiligen Moduls passen.

### Funktionsweise

1. Der Feeder übergibt seinen `feeder_name` (= `functionname` im JSON) an den
   `FeederConfigResolver.resolve()`.
2. Der Resolver durchsucht rekursiv alle JSON-Dateien in den `interface_paths`.
3. Er filtert Einträge mit `"type": "publisher"` und sucht einen mit passendem
   `functionname`.
4. Aus dem `tags`-Feld (`["zenoh"]`, `["ros2"]`, …) wird das Transportprotokoll
   abgeleitet.
5. Das Ergebnis enthält Protokoll, Interface-Datei-Typen (`.proto`) und Metadaten.

### Interface-Config-Schema (Beispiel)

```json
[
  {
    "type":         "publisher",
    "functionname": "StateFeed",
    "tags":         ["zenoh"],
    "filetype":     ["VBASEStateFeed.proto"],
    "displayname":  "State Feed",
    "description":  "Überträgt Zustandsänderungen."
  }
]
```

### Usage

```python
from vyra_base.com.feeder.config_resolver import FeederConfigResolver

result = FeederConfigResolver.resolve(
    feeder_name="StateFeed",
    interface_paths=["/workspace/install/my_interfaces/share/my_interfaces/config"],
)

if result is None:
    # Error wurde bereits geloggt, inkl. Fuzzy-Match-Vorschläge
    raise RuntimeError("StateFeed nicht in Interface-Config gefunden")

print(result.protocol)    # "zenoh"
print(result.file_types)  # ["VBASEStateFeed.proto"]
print(result.tags)        # ["zenoh"]
```

### Fehlerfall: Feeder-Name nicht gefunden

Wenn der `feeder_name` in keiner Interface-Config-Datei vorhanden ist, wird ein
detailliertes Fehlermeldung mit Vorschlägen ähnlicher Publisher-Namen geloggt:

```
❌ Feeder 'StaetFeed' not found as a publisher in any interface config.
   Searched 2 JSON file(s) across paths: ["/workspace/.../config"]
   All known publishers: ['ErrorFeed', 'NewsFeed', 'StateFeed']
   Did you mean one of: ['StateFeed']?
```

Die Vorschläge werden mit `fuzzy_match()` aus `vyra_base.helper.func` erzeugt.

---

## BaseFeeder

Base class for all feeders with multi-protocol support.

### Usage

```python
from vyra_base.com.feeder import BaseFeeder
from vyra_base.com.core.types import ProtocolType

class CustomFeeder(BaseFeeder):
    async def start(self):
        """Initialize feeder"""
        # Create speaker with fallback
        self._speaker = await self._create_speaker(
            topic_name="my_custom_topic",
            protocols=[ProtocolType.REDIS, ProtocolType.MQTT]
        )
        await self._speaker.initialize()
    
    async def publish_data(self, data: dict):
        """Publish custom data"""
        if self._speaker:
            await self._speaker.shout(data)
```

### Methods

**`_create_speaker(topic_name, protocols=None)`**
- Creates speaker with protocol fallback
- **Default protocols**: Redis → ROS2 → MQTT → SharedMemory
- Returns `VyraSpeaker` instance

**`start()`**
- Initialize feeder and create speakers
- Must be overridden in subclasses

**`stop()`**
- Shutdown feeder and cleanup resources
- Calls `shutdown()` on all speakers

## StateFeeder

Publishes module state changes.

### States

```python
from vyra_base.state import State

State.UNDEFINED    # Initial state
State.IDLE         # Ready but not active
State.READY        # Initialized and ready
State.RUNNING      # Active processing
State.PAUSED       # Temporarily suspended
State.STOPPED      # Gracefully stopped
State.ERROR        # Error occurred
```

### Usage

```python
from vyra_base.com.feeder import StateFeeder

# With ROS2
feeder = StateFeeder(entity, node=ros2_node)
await feeder.start()

# Without ROS2 (uses Redis fallback)
feeder = StateFeeder(entity, node=None)
await feeder.start()

# Publish state
await feeder.push_state("RUNNING")

# Stop feeder
await feeder.stop()
```

### Message Format

**ROS2 Mode** (when available):
```python
# Uses vyra_interfaces/msg/VyraState.msg
{
    "state": "RUNNING",
    "timestamp": 1234567890.123,
    "module_name": "v2_modulemanager"
}
```

**Native Mode** (without ROS2):
```python
{
    "state": "RUNNING",
    "timestamp": 1234567890.123,
    "module_name": "v2_modulemanager"
}
```

### Topic Name

- ROS2: `/vyra/state/{module_name}`
- Redis: `vyra:state:{module_name}`
- MQTT: `vyra/state/{module_name}`

## NewsFeeder

Publishes informational messages and events.

### Usage

```python
from vyra_base.com.feeder import NewsFeeder

# With ROS2
feeder = NewsFeeder(entity, node=ros2_node)
await feeder.start()

# Without ROS2
feeder = NewsFeeder(entity, node=None)
await feeder.start()

# Publish news
await feeder.push_news({
    "type": "info",
    "message": "Processing started",
    "details": {"items": 100}
})

# Publish via entity
entity.publish_news("System ready")

# Stop feeder
await feeder.stop()
```

### Message Format

**ROS2 Mode**:
```python
# Uses vyra_interfaces/msg/VyraNews.msg
{
    "title": "info",
    "news": "Processing started",
    "timestamp": 1234567890.123,
    "module_name": "v2_modulemanager",
    "details": "{\"items\": 100}"  # JSON string
}
```

**Native Mode**:
```python
{
    "type": "info",
    "message": "Processing started",
    "timestamp": 1234567890.123,
    "module_name": "v2_modulemanager",
    "details": {"items": 100}
}
```

### Topic Name

- ROS2: `/vyra/news/{module_name}`
- Redis: `vyra:news:{module_name}`
- MQTT: `vyra/news/{module_name}`

## ErrorFeeder

Publishes error messages and exceptions.

### Usage

```python
from vyra_base.com.feeder import ErrorFeeder

# With ROS2
feeder = ErrorFeeder(entity, node=ros2_node)
await feeder.start()

# Without ROS2
feeder = ErrorFeeder(entity, node=None)
await feeder.start()

# Publish error
await feeder.push_error({
    "error_type": "ValueError",
    "message": "Invalid input",
    "traceback": "...",
    "severity": "high"
})

# Publish via entity
entity.publish_error("Critical error occurred")

# Stop feeder
await feeder.stop()
```

### Message Format

**ROS2 Mode**:
```python
# Uses vyra_interfaces/msg/VyraError.msg
{
    "error_type": "ValueError",
    "message": "Invalid input",
    "timestamp": 1234567890.123,
    "module_name": "v2_modulemanager",
    "traceback": "...",
    "severity": "high"
}
```

**Native Mode**:
```python
{
    "error_type": "ValueError",
    "message": "Invalid input",
    "timestamp": 1234567890.123,
    "module_name": "v2_modulemanager",
    "traceback": "...",
    "severity": "high"
}
```

### Topic Name

- ROS2: `/vyra/error/{module_name}`
- Redis: `vyra:error:{module_name}`
- MQTT: `vyra/error/{module_name}`

## VyraEntity Integration

Feeders are automatically managed by VyraEntity:

```python
from vyra_base.core.entity import VyraEntity

# Create entity (auto-detects ROS2)
entity = VyraEntity(
    module_name="my_module",
    project_settings=settings
)

# Feeders created automatically
# - StateFeeder
# - NewsFeeder  
# - ErrorFeeder

# Use convenience methods
entity.publish_state("RUNNING")
entity.publish_news("Process started")
entity.publish_error("Error occurred")

# Or access feeders directly
await entity.state_feeder.push_state("PAUSED")
```

## Protocol Selection

### Default Fallback Chain

Feeders try protocols in this order:
1. **Redis** - Best for distributed systems
2. **ROS2** - If available (robot systems)
3. **MQTT** - IoT and constrained devices
4. **SharedMemory** - Local high-performance

### Custom Protocol Selection

```python
from vyra_base.com.core.types import ProtocolType

# Redis only
feeder = StateFeeder(
    entity,
    node=None,
    protocols=[ProtocolType.REDIS]
)

# MQTT with fallback
feeder = NewsFeeder(
    entity,
    node=None,
    protocols=[ProtocolType.MQTT, ProtocolType.REDIS]
)
```

## ROS2 Compatibility

### With ROS2 Installed

```python
from vyra_base.com.datalayer.node import VyraNode

# Create ROS2 node
node = VyraNode("my_module")

# Feeders use ROS2 messages
state_feeder = StateFeeder(entity, node=node)
await state_feeder.start()

# Publishes vyra_interfaces/msg/VyraState
await state_feeder.push_state("RUNNING")
```

### Without ROS2

```python
# No ROS2 node needed
state_feeder = StateFeeder(entity, node=None)
await state_feeder.start()

# Automatically uses Redis/MQTT/SharedMemory
await state_feeder.push_state("RUNNING")
```

### Type Conversion

Feeders automatically handle type conversion:

**ROS2 Mode**:
- Uses `ROS2TypeConverter` for message conversion
- Converts Python dicts to ROS2 messages
- Preserves ROS2 message structure

**Native Mode**:
- Uses Python dicts directly
- No conversion overhead
- JSON-serializable data

## Subscribing to Feeder Data

### Redis Subscription

```python
from vyra_base.storage.redis_client import RedisClient

redis = RedisClient(host="redis", port=6379)

# Subscribe to state updates
pubsub = redis.pubsub()
await pubsub.subscribe("vyra:state:my_module")

async for message in pubsub.listen():
    if message["type"] == "message":
        state_data = json.loads(message["data"])
        print(f"State: {state_data['state']}")
```

### ROS2 Subscription

```python
from vyra_base.com.datalayer.node import VyraNode
from vyra_interfaces.msg import VyraState

node = VyraNode("listener")

def state_callback(msg: VyraState):
    print(f"State: {msg.state}")

node.create_subscription(
    VyraState,
    "/vyra/state/my_module",
    state_callback,
    10
)
```

### MQTT Subscription

```python
import paho.mqtt.client as mqtt

def on_message(client, userdata, msg):
    data = json.loads(msg.payload)
    print(f"State: {data['state']}")

client = mqtt.Client()
client.on_message = on_message
client.connect("mqtt.example.com", 1883)
client.subscribe("vyra/state/my_module")
client.loop_forever()
```

## Performance Considerations

### Protocol Latency

| Protocol      | Avg Latency | Throughput   | Use Case              |
|---------------|-------------|--------------|----------------------|
| SharedMemory  | < 1 ms      | Very High    | Local high-freq      |
| ROS2          | 1-5 ms      | High         | Robot systems        |
| Redis         | 2-10 ms     | High         | Distributed          |
| MQTT          | 10-50 ms    | Medium       | IoT, WAN             |

### Optimization Tips

```python
# 1. Use SharedMemory for local high-frequency
feeder = StateFeeder(
    entity,
    protocols=[ProtocolType.SHARED_MEMORY]
)

# 2. Batch messages when possible
messages = [{"state": "RUNNING"}, {"state": "PAUSED"}]
for msg in messages:
    await feeder.push_state(msg["state"])

# 3. Reduce message size
await feeder.push_news({
    "message": "Short message",  # ✅ Compact
    # "details": large_data_blob  # ❌ Avoid large payloads
})
```

## Error Handling

```python
from vyra_base.com.core.exceptions import SpeakerError

try:
    feeder = StateFeeder(entity, node=None)
    await feeder.start()
    await feeder.push_state("RUNNING")
except SpeakerError as e:
    logger.error(f"Failed to publish state: {e}")
    # Fallback to logging
    logger.info("State: RUNNING")
```

## Testing

### Mock Feeder

```python
class MockFeeder:
    def __init__(self):
        self.states = []
        self.news = []
        self.errors = []
    
    async def start(self):
        pass
    
    async def stop(self):
        pass
    
    async def push_state(self, state):
        self.states.append(state)
    
    async def push_news(self, message):
        self.news.append(message)
    
    async def push_error(self, error):
        self.errors.append(error)
```

### Test Example

```python
import pytest

@pytest.mark.asyncio
async def test_state_feeder():
    entity = MockEntity()
    feeder = StateFeeder(entity, node=None)
    
    await feeder.start()
    await feeder.push_state("RUNNING")
    
    # Verify published
    assert len(entity.state_feeder.states) == 1
    assert entity.state_feeder.states[0] == "RUNNING"
    
    await feeder.stop()
```

## Migration Guide

### From ROS2-Only Feeders

**Before**:
```python
from vyra_base.com.feeder import StateFeeder

# Required ROS2 node
feeder = StateFeeder(entity, node=ros2_node)
await feeder.start()
```

**After**:
```python
from vyra_base.com.feeder import StateFeeder

# ROS2 optional
feeder = StateFeeder(entity, node=None)  # Auto-fallback
await feeder.start()
```

## Best Practices

### 1. Always Call start()/stop()
```python
# ✅ Good
feeder = StateFeeder(entity)
await feeder.start()
try:
    await feeder.push_state("RUNNING")
finally:
    await feeder.stop()
```

### 2. Use Entity Convenience Methods
```python
# ✅ Good - Cleaner
entity.publish_state("RUNNING")

# ❌ Avoid - More verbose
await entity.state_feeder.push_state("RUNNING")
```

### 3. Keep Messages Small
```python
# ✅ Good - Compact
entity.publish_news("Process complete")

# ❌ Avoid - Large payloads
entity.publish_news({
    "message": "Complete",
    "details": large_data_blob  # Use separate storage
})
```

### 4. Handle Protocol Unavailability
```python
from vyra_base.com.core.exceptions import ProtocolUnavailableError

try:
    feeder = StateFeeder(entity, protocols=[ProtocolType.REDIS])
    await feeder.start()
except ProtocolUnavailableError:
    logger.warning("Redis unavailable, using fallback")
    feeder = StateFeeder(entity)  # Use default fallback
    await feeder.start()
```

## See Also

- [Core Components](../core/README.md) - InterfaceFactory, decorators
- [External Layer](../external/README.md) - Redis, MQTT protocols
- [VyraEntity](../../core/entity.py) - Entity integration
- [State Machine](../../state/README.md) - State management
