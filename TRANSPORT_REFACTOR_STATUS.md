# VYRA Transport Layer Refaktoring - Status & Migration Guide

## ✅ Abgeschlossen (Stand: 2026-02-16)

### Phase 1: Interface-Struktur
- ✓ JSON Metadata aktualisiert (`type: speaker→publisher, callable→server, job→actionServer`)
- ✓ Interface-Generator Tool (`tools/generate_interfaces.py`)
- ✓ .msg/.srv/.action Files generiert (14 Interfaces)
- ✓ .proto Files pro Interface-Typ generiert
- ✓ Deprecate unreferenzierte Proto-Files (12 Stück markiert)

### Phase 2: Core Type System
- ✓ 6 neue Klassen in `types.py`:
  - `VyraPublisher` (publish-only, no callback)
  - `VyraSubscriber` (mit `async def subscriber_callback(msg)`)
  - `VyraServer` (mit `async def response_callback(request)`)
  - `VyraClient` (mit optional `async def request_callback(response)`)
  - `VyraActionServer` (mit 3 callbacks: handle_goal_request, handle_cancel, execution)
  - `VyraActionClient` (mit 3 callbacks: direct_response, feedback, goal)
- ✓ Legacy-Klassen deprecatet (VyraCallable, VyraSpeaker, VyraJob)
- ✓ Factory erweitert (6 neue Fallback-Chains + create_* Methoden)
- ✓ Provider Interface erweitert (6 neue abstract methods)

### Phase 3: Transport Parameter Corrections
- ✓ **ROS2**: Alle 6 vyra_models Dateien vollständig implementiert
- ✓ **Zenoh**: Alle 6 vyra_models Parameter korrigiert (topic_builder, ProtocolType)
- ✓ **Redis**: Alle 6 vyra_models Parameter korrigiert via auto_fix script
- ⚠️ **UDS**: Nur callable.py existiert, 6 neue Dateien fehlen noch

### Phase 4: Provider Layer (KOMPLETT ✅)
- ✓ **Zenoh Provider**: Alle 6 create_* Methoden implementiert (~300 LOC)
- ✓ **Redis Provider**: Alle 6 create_* Methoden implementiert (~290 LOC)
- ✓ **UDS Provider**: Alle 6 create_* Methoden implementiert (~250 LOC)
- ✓ **Factory**: Alle 6 create_* Methoden mit Fallback-Chains implementiert

### Phase 5: Module Exports
- ✓ **t_zenoh/vyra_models/__init__.py**: Neue Klassen exportiert
- ✓ **t_redis/vyra_models/__init__.py**: Neue Klassen exportiert
- ✓ **t_uds/vyra_models/__init__.py**: Neue Klassen exportiert

---

## 📝 TODO: Verbleibende Aufgaben

### UDS Transport Implementation
**Status**: ⚠️ vyra_models Dateien fehlen (Provider ist bereit)

**Files zu erstellen** (`transport/t_uds/vyra_models/`):
- [ ] publisher.py (Datagram sockets)
- [ ] subscriber.py (Datagram sockets)
- [ ] server.py (Stream sockets)
- [ ] client.py (Stream sockets)
- [ ] action_server.py (Stream sockets + state messages)
- [ ] action_client.py (Stream sockets + state messages)

**Provider Status**: ✅ Bereit (create_* Methoden vorhanden)

**Provider Update** (`transport/t_zenoh/provider.py`):
- [ ] `create_publisher()`
- [ ] `create_subscriber()`
- [ ] `create_server()`
- [ ] `create_client()`
- [ ] `create_action_server()`
- [ ] `create_action_client()`

**Hinweis**: Zenoh ist async-native, keine Callback-Adapter nötig!

---

### Phase 3: Redis Transport
**Status**: ⚠️ Teilweise vorhanden, muss erweitert werden

**NEU zu implementieren** (`transport/t_redis/vyra_models/`):
- [ ] publisher.py (Redis Pub/Sub Channel)
- [ ] subscriber.py (Redis Pub/Sub mit callback)
- [ ] server.py (Request/Response Pattern mit Keys)
- [ ] client.py (Request/Response Pattern)
- [ ] action_server.py (State-Tracking via Redis Keys + Pub/Sub)
- [ ] action_client.py (Goal send + State subscribe)

**Action Server Pattern**:
```
Keys:
  action:{name}:{id}:state    → running|succeeded|aborted|canceled
  action:{name}:{id}:feedback → JSON feedback data
  action:{name}:{id}:result   → JSON result data
  
Channels:
  action:{name}:control       → goal_request | cancel_request
  action:{name}:{id}:updates  → feedback | result notifications
```

**Provider Update** (`transport/t_redis/provider.py`):
- [ ] 6 neue `create_*` Methoden

---

### Phase 3: UDS Transport
**Status**: ⚠️ Nur Callable vorhanden, Rest fehlt

**NEU zu implementieren** (`transport/t_uds/vyra_models/`):
- [ ] publisher.py (Unix Datagram Socket Sender)
- [ ] subscriber.py (Unix Datagram Socket Receiver)
- [ ] server.py (Unix Stream Socket RPC Server)
- [ ] client.py (Unix Stream Socket RPC Client)
- [ ] action_server.py (Stream-basiert mit State Messages)
- [ ] action_client.py (Stream-basiert mit State Messages)

**Socket Paths**:
```
/tmp/vyra_sockets/
  ├── pub_{module}_{name}.sock       (Datagram for pub/sub)
  ├── srv_{module}_{name}.sock       (Stream for request/response)
  └── act_{module}_{name}.sock       (Stream for actions)
```

**Provider Update** (`transport/t_uds/provider.py`):
- [ ] 6 neue `create_*` Methoden

---

## Phase 4: Feeder Refaktorierung
**Status**: ⚠️ Noch nicht begonnen

**Files zu aktualisieren** (`com/feeder/`):
- [ ] feeder.py: `self._speaker` → `self._publisher`, `shout()` → `publish()`
- [ ] news_feeder.py
- [ ] state_feeder.py
- [ ] error_feeder.py

**Änderungen**:
```python
# Alt
self._speaker = await InterfaceFactory.create_speaker(...)
await self._speaker.shout(message)

# Neu
self._publisher = await InterfaceFactory.create_publisher(...)
await self._publisher.publish(message)
```

---

## Phase 5: v2_modulemanager Integration
**Status**: ⚠️ Noch nicht begonnen

**Files zu aktualisieren**:
- [ ] `tools/setup_interfaces.py`: Proto-Ordner Handling entfernen
- [ ] `interface.py`: `auto_register_*` Funktionen erweitern
- [ ] Metadata-Handling: type-Namen anpassen

---

## Phase 6: Documentation
**Status**: ⚠️ Noch nicht begonnen

**READMEs zu aktualisieren**:
- [ ] `com/README.md`: Neue Architektur dokumentieren
- [ ] `com/transport/README.md`: 6x4 Matrix (Patterns × Transporte)
- [ ] `com/transport/t_*/README.md`: Transport-spezifische Details
- [ ] `interfaces/README.md`: Generator-Tool Usage

**Neues Dokument**:
- [ ] `docs/TRANSPORT_MIGRATION_GUIDE.md`: Migration von Legacy zu neuen Types

---

## Migration Checkliste für bestehenden Code

### 1. Imports aktualisieren
```python
# Alt
from vyra_base.com import VyraSpeaker, VyraCallable, VyraJob

# Neu
from vyra_base.com import (
    VyraPublisher, VyraSubscriber,  # statt VyraSpeaker
    VyraServer, VyraClient,          # statt VyraCallable
    VyraActionServer, VyraActionClient  # statt VyraJob
)
```

### 2. Factory Calls anpassen
```python
# Alt: Speaker (pub + sub in einem)
speaker = await InterfaceFactory.create_speaker("topic", callback=on_msg)
await speaker.shout(msg)

# Neu: Getrennt
publisher = await InterfaceFactory.create_publisher("topic")
await publisher.publish(msg)

subscriber = await InterfaceFactory.create_subscriber("topic", subscriber_callback=on_msg)
await subscriber.subscribe()
```

### 3. Callbacks zu async konvertieren
```python
# Alt (sync oder async)
def callback(msg):
    process(msg)

# Neu (immer async)
async def subscriber_callback(msg):
    await async_process(msg)
```

### 4. Provider Implementierungen
```python
# Alt
class MyProvider(AbstractProtocolProvider):
    async def create_speaker(self, name, **kwargs) -> VyraSpeaker:
        ...

# Neu
class MyProvider(AbstractProtocolProvider):
    async def create_publisher(self, name, **kwargs) -> VyraPublisher:
        ...
    
    async def create_subscriber(self, name, subscriber_callback, **kwargs) -> VyraSubscriber:
        ...
```

---

## Testing Strategy

### Unit Tests
- [ ] `tests/com/core/test_new_types.py`: VyraPublisher, VyraSubscriber, etc.
- [ ] `tests/com/transport/t_ros2/test_*`: Alle 6 neuen Klassen
- [ ] `tests/com/transport/t_zenoh/test_*`: Alle 6 neuen Klassen
- [ ] `tests/com/transport/t_redis/test_*`: Alle 6 neuen Klassen
- [ ] `tests/com/transport/t_uds/test_*`: Alle 6 neuen Klassen

### Integration Tests
- [ ] `tests/integration/test_transport_parity.py`: 4×6 Matrix (alle Transporte × alle Patterns)
- [ ] `tests/integration/test_async_callbacks.py`: Callback-Execution über alle Transporte
- [ ] `tests/integration/test_redis_complete.py`: Neue Redis Implementierungen
- [ ] `tests/integration/test_uds_complete.py`: Neue UDS Implementierungen

### E2E Tests
- [ ] Module-Start mit neuen Interfaces
- [ ] Cross-Module Communication
- [ ] Feeder-Flow

---

## Nächste Schritte (Empfohlene Reihenfolge)

1. **Zenoh Transport** fertigstellen (async-native, einfacher)
2. **Redis Transport** erweitern (ActionServer/Client neu)
3. **UDS Transport** erweitern (Publisher/Subscriber/Action neu)
4. **Feeder** refaktorieren (quick win)
5. **v2_modulemanager** anpassen
6. **Tests** schreiben
7. **Documentation** aktualisieren

---

## Bekannte Issues & Limitationen

### Callback-Adapter Performance
ROS2 Callback-Adapter nutzt `run_until_complete()` in sync context - kann bei hoher Frequenz Performance-Impact haben. Alternative: Dedicated async executor thread.

### Protobuf Type Mapping
`builtin_interfaces/Time` wird zu `int64` vereinfacht für cross-transport compatibility. Bei Bedarf custom nested message types unterstützen.

### UDS Socket Cleanup
Unix Domain Sockets müssen bei shutdown explizit gelöscht werden (`unlink()`). Aktuell nicht in allen vyra_models implementiert.

---

## Kontakt & Fragen

Bei Fragen zur Migration oder Implementierung:
- Siehe detaillierte READMEs in `com/*/README.md`
- Referenz-Implementation: `transport/t_ros2/vyra_models/`
- Core Types Doku: `com/core/types.py` docstrings
