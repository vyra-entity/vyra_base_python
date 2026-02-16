# Transport vyra_models Parameter-Korrekturen - Checkliste

## Problem
Alle Transport vyra_models Klassen müssen `topic_builder: TopicBuilder` verwenden statt direkter Parameter (`topic`, `service_name`, `action_name`).

## Status

### ✅ Zenoh Transport (100% komplett)
- [x] publisher.py - `topic_builder` + `message_type`, `initialize()` nutzt `build_topic()`
- [x] subscriber.py - `topic_builder` + `message_type` + `subscriber_callback`
- [x] server.py - `topic_builder` + `response_callback` + `service_type`
- [x] client.py - `topic_builder` + `request_callback` + `service_type`
- [x] action_server.py - `topic_builder` + 3 callbacks + `action_type`
- [x] action_client.py - `topic_builder` + 3 callbacks + `action_type`

### ⚠️ Redis Transport (2/6 komplett)
- [x] publisher.py - Komplett korrigiert
- [x] subscriber.py - Komplett korrigiert
- [ ] server.py - **__init__ korrigiert**, aber noch TODO:
  - [ ] `initialize()`: `service_name = self.topic_builder.build_topic(self.name)` hinzufügen
  - [ ] `self._request_channel = f"srv:{service_name}:requests"` in `initialize()` verschieben
  - [ ] `cleanup()`: `self.service_name` → `self.name`
  - [ ] `shutdown()` Methode hinzufügen
  - [ ] `@property interface_type` entfernen
- [ ] client.py - **TODO:** Vollständige Korrektur
  - Ändern: `__init__(name, srv_type, service_name, ...)` → ` __init__(name, topic_builder, request_callback, redis_client, service_type, ...)`
  - `super().__init__(name, topic_builder, request_callback, ProtocolType.REDIS, **kwargs)`
  - `initialize()`: `service_name = self.topic_builder.build_topic(self.name)` + `self._service_name = service_name`
  - `_response_channel = f"srv:{service_name}:response:{self._client_id}"` in initialize
  - Alle Referenzen `self.service_name` → `self._service_name`
  - `cleanup()` + `shutdown()` korrigieren
- [  ] action_server.py - **TODO:** Vollständige Korrektur
  - Ändern: `__init__(name, action_type, action_name, ...)` → `__init__(name, topic_builder, handle_goal_request, handle_cancel_request, execution_callback, ...)`
  - `super().__init__(name, topic_builder, handle_goal_request, handle_cancel_request, execution_callback, ProtocolType.REDIS, **kwargs)`
  - `initialize()`: `action_name = self.topic_builder.build_topic(self.name)` + `self._action_name = action_name`
  - Update alle channel Namen: `f"action:{self._action_name}:..."`
  - `cleanup()` + `shutdown()` korrigieren
- [ ] action_client.py - **TODO:** Vollständige Korrektur
  - Analog zu action_server.py
  - `super().__init__(name, topic_builder, direct_response, feedback_callback, goal_response_callback, ProtocolType.REDIS, **kwargs)`
  - `initialize()`: `action_name = self.topic_builder.build_topic(self.name)`
  - Update alle channel Namen

### ❌ UDS Transport (0/6 begonnen)
- [ ] publisher.py - **TODO:** Vollständige Korrektur
  - Ändern: `__init__(name, msg_type, topic, ...)` → `__init__(name, topic_builder, message_type, module_name, ...)`
  - `super().__init__(name, topic_builder, ProtocolType.UDS, **kwargs)`
  - `initialize()`: `topic_name = self.topic_builder.build_topic(self.name)`
  - `_socket_path = self._socket_dir / f"pub_{module_name}_{topic_name}.sock"`
  - `cleanup()` + `shutdown()` hinzufügen, `@property interface_type` entfernen
- [ ] subscriber.py - **TODO:** Vollständige Korrektur
  - Analog zu publisher
  - `initialize()`: `topic_name = self.topic_builder.build_topic(self.name)`
  - `_socket_path = self._socket_dir / f"sub_{module_name}_{topic_name}.sock"`
- [ ] server.py - **TODO:** Vollständige Korrektur
  - `__init__(name, topic_builder, response_callback, service_type, module_name, ...)`
  - `super().__init__(name, topic_builder, response_callback, ProtocolType.UDS, **kwargs)`
  - `initialize()`: `service_name = self.topic_builder.build_topic(self.name)`
  - `_socket_path = self._socket_dir / f"srv_{module_name}_{service_name}.sock"`
- [ ] client.py - **TODO:** Vollständige Korrektur
  - Analog zu server
  - `initialize()`: `service_name = self.topic_builder.build_topic(self.name)`
- [ ] action_server.py - **TODO:** Vollständige Korrektur
  - `__init__(name, topic_builder, 3 callbacks, action_type, module_name, ...)`
  - `super().__init__(name, topic_builder, handle_goal_request, handle_cancel_request, execution_callback, ProtocolType.UDS, **kwargs)`
  - `initialize()`: `action_name = self.topic_builder.build_topic(self.name)`
  - `_socket_path = self._socket_dir / f"act_{module_name}_{action_name}.sock"`
- [ ] action_client.py - **TODO:**, Vollständige Korrektur
  - Analog zu action_server

---

## Automatisierte Korrektur-Patterns

Für jede Datei:

### 1. Imports aktualisieren
```python
# ALT
from vyra_base.com.core.types import VyraPublisher, InterfaceType

# NEU
from vyra_base.com.core.types import VyraPublisher, ProtocolType
from vyra_base.com.core.topic_builder import TopicBuilder
```

### 2. __init__ Parameter anpassen

#### Publisher/Subscriber:
```python
# ALT
def __init__(self, name, msg_type, topic, transport_handle, **kwargs):
    super().__init__(name, msg_type, topic, **kwargs)

# NEU
def __init__(self, name, topic_builder, transport_handle, message_type, **kwargs):
    super().__init__(name, topic_builder, ProtocolType.XXX, **kwargs)
    self.message_type = message_type
```

#### Server/Client:
```python
# ALT
def __init__(self, name, srv_type, service_name, callback, transport_handle, **kwargs):
    super().__init__(name, srv_type, service_name, callback, **kwargs)

# NEU
def __init__(self, name, topic_builder, callback, transport_handle, service_type, **kwargs):
    super().__init__(name, topic_builder, callback, ProtocolType.XXX, **kwargs)
    self.service_type = service_type
```

#### ActionServer/ActionClient:
```python
# ALT
def __init__(self, name, action_type, action_name, cb1, cb2, cb3, transport_handle, **kwargs):
    super().__init__(name, action_type, action_name, cb1, cb2, cb3, **kwargs)

# NEU
def __init__(self, name, topic_builder, cb1, cb2, cb3, transport_handle, action_type, **kwargs):
    super().__init__(name, topic_builder, cb1, cb2, cb3, ProtocolType.XXX, **kwargs)
    self.action_type = action_type
```

### 3. initialize() anpassen
```python
async def initialize(self) -> bool:
    try:
        # NEU: Topic/Service/Action Name via TopicBuilder generieren
        topic_name = self.topic_builder.build_topic(self.name)  # oder service_name, action_name
        self._topic_name = topic_name  # Speichern für spätere Nutzung
        
        # Rest der Initialisierung...
        logger.info(f"✅ {self.__class__.__name__} initialized: {topic_name}")
        self._initialized = True
        return True
```

### 4. cleanup() anpassen
```python
async def cleanup(self):
    # Cleanup logic...
    logger.info(f"🔄 {self.__class__.__name__} cleaned up: {self.name}")  # NICHT self.topic/service_name!
```

### 5. shutdown() hinzufügen (wenn fehlt)
```python
async def shutdown(self) -> None:
    """Shutdown interface."""
    await self.cleanup()
    self._initialized = False
```

### 6. @property interface_type entfernen
```python
# ALT - KOMPLETT LÖSCHEN:
@property
def interface_type(self) -> InterfaceType:
    return InterfaceType.PUBLISHER
```

---

## Nächste Schritte

1. **Redis fertigstellen** (4 Files):
   - server.py: initialize() + cleanup() anpassen
   - client.py: Komplette Korrektur
   - action_server.py: Komplette Korrektur
   - action_client.py: Komplette Korrektur

2. **UDS alle Files** (6 Files):  
   - publisher.py → subscriber.py → server.py → client.py → action_server.py → action_client.py
   - Alle folgen den gleichen Patterns wie oben

3. **Provider Updates**:
   - `t_zenoh/provider.py`: `create_*()` methods implementieren
   - `t_redis/provider.py`: `create_*()` methods implementieren
   - `t_uds/provider.py`: `create_*()` methods implementieren

4. **__init__.py Updates**:
   - Exports in jedem vyra_models/__init__.py ergänzen

---

## Schnell-Fix Kommandos

Für globale Replacements (VORSICHT - nur wenn klar!):

```bash
cd /home/holgder/VYRA/vyra_base_python/src/vyra_base/com/transport/t_redis/vyra_models
# InterfaceType → ProtocolType in allen Files
sed -i 's/from vyra_base\.com\.core\.types import \(.*\), InterfaceType/from vyra_base.com.core.types import \1, ProtocolType\nfrom vyra_base.com.core.topic_builder import TopicBuilder/' *.py

# @property interface_type entfernen
sed -i '/@property/,/return InterfaceType\./d' *.py
```

⚠️ **ACHTUNG**: Sed-Kommandos nur nach Backup verwenden!
