# com/handler — VYRA Feeder Handlers

Handlers sind die **Transport-Brücke** zwischen einem Feeder und einem konkreten
Kommunikationsprotokoll.  Jeder Handler implementiert die abstrakte Basisklasse
`IFeederHandler` und kann wahlweise als Python `logging.Handler` in einem Logger
registriert oder direkt per `dispatch()` aufgerufen werden.

---

## Handler-Hierarchie

```
IFeederHandler   (interfaces.py)          ← ABC: dispatch() + get_protocol() + logging.Handler
    └── CommunicationHandler  (communication.py)    ← konkreter Basis-Handler
            ├── ROS2Handler    (ros2.py)             → t_ros2 Provider (via InterfaceFactory)
            ├── ZenohHandler   (zenoh.py)            → t_zenoh Provider (via InterfaceFactory)
            ├── RedisHandler   (redis.py)            → t_redis Provider (via InterfaceFactory)
            ├── UDSHandler     (uds.py)              → t_uds  Provider (via InterfaceFactory)
            └── DBCommunicationHandler (database.py) → async DB-Writer (DatabaseWriter Protocol)
```

### Design-Entscheidung: Hybrid-Ansatz

Transport-Handler erben von `logging.Handler` (**Backward-Kompatibilität**) *und* implementieren
das `IFeederHandler`-Interface mit `async dispatch()`.  So können Handler:

1. In einen Python-Logger eingehängt werden (`logger.addHandler(handler)`)
2. Direkt vom Feeder aufgerufen werden (`await handler.dispatch(msg)`)

Die `emit()` Methode des `logging.Handler` delegiert an `dispatch()`, sodass
beide Wege konsistent bleiben.

---

## Transport-Handler

> **Wichtig:** Transport-Handler erstellen ihren internen `VyraPublisher` **immer**
> über `InterfaceFactory.create_publisher()` — niemals direkt über Provider-Klassen.
> Das hält die CAL-Schicht (`t_ros2`, `t_zenoh`, etc.) vollständig gekapselt.

### Erstellen über HandlerFactory (empfohlen)

```python
from vyra_base.com.handler.factory import HandlerFactory
from vyra_base.com.core.types import ProtocolType

handler = await HandlerFactory.create(
    protocol=ProtocolType.ZENOH,
    initiator="StateFeeder",
    feeder_name="StateFeed",
    message_type=my_proto_type,
)
```

### Manuell (wenn Publisher bereits vorhanden)

```python
from vyra_base.com.handler.zenoh import ZenohHandler
from vyra_base.com.core.factory import InterfaceFactory
from vyra_base.com.core.types import ProtocolType

publisher = await InterfaceFactory.create_publisher(
    name="StateFeed",
    protocols=[ProtocolType.ZENOH],
    message_type=my_proto_type,
)
handler = ZenohHandler(initiator="StateFeeder", publisher=publisher, type=my_proto_type)
```

---

## Protokoll-Übersicht

| Handler            | Protokoll | Anwendungsfall                                    |
|--------------------|-----------|---------------------------------------------------|
| `ROS2Handler`      | `ros2`    | ROS2-Netz (typsichere Msgs, QoS-Konfiguration)    |
| `ZenohHandler`     | `zenoh`   | Verteilte Systeme, hoher Durchsatz, P2P           |
| `RedisHandler`     | `redis`   | Fan-out Broadcasting, mehrere Konsumenten         |
| `UDSHandler`       | `uds`     | Intra-Host, latenzoptimiert, kein Netzwerk nötig  |
| `DBCommunicationHandler` | `database` | Persistenz / Audit-Trail                    |

---

## Datenbank-Handler

`DBCommunicationHandler` akzeptiert **jedes** Objekt, das das
`DatabaseWriter`-Protocol implementiert:

```python
from vyra_base.com.handler.database import DBCommunicationHandler, DatabaseWriter
from typing import runtime_checkable, Protocol

# Eigener DB-Writer (beliebige Technologie)
class MyRedisWriter:
    async def write(self, record: dict) -> None:
        await redis.hset(f"feed:{record['timestamp']}", mapping=record)

handler = DBCommunicationHandler(database=MyRedisWriter(), source="StateFeeder")
```

**Persistiertes Record-Schema:**

```json
{
    "timestamp": "2026-01-01T12:00:00.000Z",
    "level":     "INFO",
    "source":    "StateFeeder",
    "message":   "<str(domain_object)>",
    "metadata":  { "key": "value" }
}
```

---

## IFeederHandler — Eigene Handler implementieren

```python
from vyra_base.com.handler.interfaces import IFeederHandler
from typing import Any

class MyCustomHandler(IFeederHandler):
    __handlerName__ = "MyCustomHandler"

    async def dispatch(self, message: Any) -> None:
        # Eigene Transport-Logik
        await my_transport.send(message)

    def get_protocol(self) -> str:
        return "my_protocol"

    def is_available(self) -> bool:
        return my_transport.is_connected()
```

---

## HandlerFactory

Die `HandlerFactory` ist der empfohlene Einstiegspunkt für das Erstellen von Handlern,
da sie die Publisher-Erstellung über `InterfaceFactory` kapselt:

```python
from vyra_base.com.handler.factory import HandlerFactory
from vyra_base.com.core.types import ProtocolType

# Transport-Handler
handler = await HandlerFactory.create(
    protocol=ProtocolType.REDIS,
    initiator="ErrorFeeder",
    feeder_name="ErrorFeed",
    message_type=error_proto_type,
)

# Datenbank-Handler
db_handler = await HandlerFactory.create(
    protocol="database",
    initiator="StateFeeder",
    feeder_name="StateFeed",
    message_type=None,
    database=my_db_writer,
)
```

---

## Tests

```bash
pytest tests/com/handler/ -v
```
