# Topic Builder

Der Topic Builder stellt eine einheitliche Namenskonvention für alle Transportprotokolle (ROS2, Zenoh, Redis, UDS) bereit.

## Namenskonvention

```
<module_name>_<module_id>/<function_name>{/<optional_subaction>}
```

### Beispiele

- `v2_modulemanager_abc123/get_modules`
- `v2_dashboard_xyz789/set_config/theme`
- `sensor_node_def456/data/temperature`

## Verwendung

### 1. TopicBuilder erstellen

```python
from vyra_base.com.core import TopicBuilder

# Erstelle Builder für ein Modul
builder = TopicBuilder(
    module_name="v2_modulemanager",
    module_id="733256b82d6b48a48bc52b5ec73ebfff"
)
```

### 2. Topic-Namen generieren

```python
# Einfacher Service/Topic
service_name = builder.build("get_modules")
# → "v2_modulemanager_733256b82d6b48a48bc52b5ec73ebfff/get_modules"

# Mit Subaction
config_topic = builder.build("set_config", subaction="theme")
# → "v2_modulemanager_733256b82d6b48a48bc52b5ec73ebfff/set_config/theme"

# Mit Prefix
state_topic = builder.build_with_prefix("state", "update")
# → "v2_modulemanager_733256b82d6b48a48bc52b5ec73ebfff/state/update"
```

### 3. Integration in Transportprotokolle

Der TopicBuilder wird automatisch in allen Vyra Models unterstützt:

#### ROS2

```python
from vyra_base.com.transport.t_ros2.vyra_models import ROS2Callable, ROS2Speaker
from vyra_base.com.core import TopicBuilder

builder = TopicBuilder("v2_modulemanager", "abc123")

# ROS2 Service mit TopicBuilder
callable = ROS2Callable(
    name="get_modules",  # Wird zu "v2_modulemanager_abc123/get_modules"
    callback=handle_request,
    node=node,
    service_type=service_type,
    topic_builder=builder
)

# ROS2 Topic mit TopicBuilder
speaker = ROS2Speaker(
    name="sensor_data",  # Wird zu "v2_modulemanager_abc123/sensor_data"
    node=node,
    message_type=message_type,
    topic_builder=builder
)
```

#### Zenoh

```python
from vyra_base.com.transport.t_zenoh.vyra_models import ZenohCallable, ZenohSpeaker

# Zenoh Query/Reply mit TopicBuilder
callable = ZenohCallable(
    name="get_modules",
    session=session,
    callback=handle_query,
    topic_builder=builder
)

# Zenoh Pub/Sub mit TopicBuilder
speaker = ZenohSpeaker(
    name="sensor_data",
    session=session,
    topic_builder=builder
)
```

#### Redis

```python
from vyra_base.com.transport.t_redis.vyra_models import RedisCallable, RedisSpeaker

# Redis Request-Response mit TopicBuilder
callable = RedisCallable(
    name="calculate",
    callback=handle_request,
    redis_client=client,
    topic_builder=builder
)

# Redis Pub/Sub mit TopicBuilder
speaker = RedisSpeaker(
    name="events",
    redis_client=client,
    topic_builder=builder
)
```

#### Unix Domain Sockets (UDS)

```python
from vyra_base.com.transport.t_uds.vyra_models import UDSCallable

# UDS mit TopicBuilder (Name wird für Socket-Datei verwendet)
callable = UDSCallable(
    name="ipc_service",
    callback=handle_request,
    topic_builder=builder
)
# Socket-Pfad: /tmp/vyra_uds/v2_modulemanager_abc123_ipc_service.sock
```

## Vorteile

1. **Einheitlichkeit**: Alle Protokolle verwenden dieselbe Namenskonvention
2. **Klarheit**: Module-Namen und IDs sind sofort erkennbar
3. **Namensraum-Isolation**: Keine Konflikte zwischen Modulen
4. **Debugging**: Einfaches Identifizieren von Kommunikationsendpunkten
5. **Discovery**: Tools können Modul-Zugehörigkeit automatisch erkennen

## Topic-Namen validieren und parsen

```python
# Validieren
is_valid = builder.validate("v2_modulemanager_abc123/get_modules")
# → True

# Parsen
components = builder.parse("v2_modulemanager_abc123/get_modules/subaction")
# → TopicComponents(
#     module_name='v2_modulemanager',
#     module_id='abc123',
#     function_name='get_modules',
#     subaction='subaction'
# )
```

## Convenience-Funktionen

Für einmalige Verwendung ohne Builder-Instanz:

```python
from vyra_base.com import build_topic, parse_topic

# Topic generieren
topic = build_topic("v2_modulemanager", "abc123", "get_modules")

# Topic parsen
components = parse_topic("v2_modulemanager_abc123/get_modules")
```

## Integration in VyraEntity

Der TopicBuilder sollte in der VyraEntity-Initialisierung erstellt und an alle Interfaces übergeben werden:

```python
from vyra_base.com.core import TopicBuilder

class MyEntity:
    def __init__(self, module_name: str, module_id: str):
        self.topic_builder = TopicBuilder(module_name, module_id)
    
    async def create_interfaces(self):
        # Interfaces mit TopicBuilder erstellen
        callable = await provider.create_callable(
            name="my_service",
            callback=self.handle_service,
            topic_builder=self.topic_builder
        )
```

## Testing

Der TopicBuilder ist vollständig in die bestehende Test-Suite integriert. Tests befinden sich in `tests/com/test_topic_builder.py`.

## Migration von bestehendem Code

Bestehender Code funktioniert weiterhin ohne TopicBuilder. Die Integration ist optional und abwärtskompatibel:

```python
# Alt (funktioniert weiterhin)
callable = ROS2Callable(
    name="my_service",
    callback=handle_request,
    node=node,
    service_type=service_type
)

# Neu (mit TopicBuilder)
callable = ROS2Callable(
    name="my_service",
    callback=handle_request,
    node=node,
    service_type=service_type,
    topic_builder=builder  # Optional!
)
```

## Siehe auch

- [Communication Module Documentation](./README.md)
- [ROS2 Transport](./transport/t_ros2/README.md)
- [Zenoh Transport](./transport/t_zenoh/README.md)
