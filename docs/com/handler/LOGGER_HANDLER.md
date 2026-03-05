# VyraLogHandler

Ein in-Memory-Logging-Handler für VYRA-Module, der Log-Einträge gepuffert hält und per Zenoh-Service abrufbar macht.

## Übersicht

`VyraLogHandler` ist ein Standard-`logging.Handler`, der die letzten N Log-Nachrichten eines Moduls in einem Ringpuffer (deque) speichert. Er wird automatisch von `VyraEntity` registriert und über den Zenoh-Service `get_log_history` nach außen exponiert.

## Klasse

```python
from vyra_base.com.handler.logger import VyraLogHandler
```

### Konstruktor

```python
VyraLogHandler(capacity: int = 1000)
```

| Parameter | Typ | Standard | Beschreibung |
|-----------|-----|----------|--------------|
| `capacity` | int | 1000 | Maximale Anzahl gespeicherter Einträge (Ringpuffer) |

### Methoden

#### `emit(record: logging.LogRecord) -> None`

Wird vom Python-Logging-Framework automatisch aufgerufen. Speichert den Eintrag als Dict im Ringpuffer.

Gespeichertes Format:
```python
{
    "level":       "INFO",                    # Levelname
    "message":     "...",                     # Formatierte Nachricht
    "logger_name": "v2_modulemanager.main",   # Logger-Name
    "timestamp":   "2025-01-01T12:00:00.000", # ISO-8601
    "seq":         1735732800000              # Unix-Millisekunden (zur Deduplikation)
}
```

#### `get_recent(limit: int = 100) -> list[dict]`

Gibt die neuesten `limit` Einträge zurück (chronologisch, älteste zuerst).

```python
handler = VyraLogHandler(capacity=500)
entries = handler.get_recent(limit=50)
# -> [{"level": "INFO", "message": "...", ...}, ...]
```

---

## Integration in VyraEntity

`VyraEntity` instanziiert `VyraLogHandler` automatisch und hängt ihn an den Root-Logger:

```python
# entity.py (intern)
self._log_handler = VyraLogHandler(capacity=1000)
logging.getLogger().addHandler(self._log_handler)
```

---

## Zenoh-Service `get_log_history`

Module, die `modulemanager_state.meta.json` (oder `dashboard_state.meta.json`) laden, stellen automatisch einen Zenoh-Service bereit:

**Service-Key**: `{module_key}/get_log_history`  
**Parameter**: `limit` (int32, optional, default 100)  
**Rückgabe**: `logs_json` (String, JSON-Array), `success` (bool)

Beispiel-Aufruf (Python):
```python
from vyra_base.com.handler.interfaces import InterfaceFactory
from vyra_base.com.handler.factory import ProtocolType

client = await InterfaceFactory.create_client(
    entity,
    "v2_modulemanager_<hash>/get_log_history",
    protocol=ProtocolType.ZENOH,
)
response = await client.call(limit=50)
import json
logs = json.loads(response.logs_json)
```

### SSE-Polling via REST-API (v2_modulemanager)

Der v2_modulemanager-Backend-Webserver bietet einen Server-Sent-Events-Endpoint, der `get_log_history` alle 2 Sekunden pollt:

```
GET /api/v2_modulemanager/instances/{instance_id}/logs
Accept: text/event-stream
```

Events werden nur gesendet, wenn neue Einträge seit dem letzten Poll vorliegen (Deduplizierung per `seq`-Feld).

---

## Export

```python
from vyra_base.com.handler import VyraLogHandler
```

Oder direkt:
```python
from vyra_base.com.handler.logger import VyraLogHandler
```

---

## Datei-Pfad

`src/vyra_base/com/handler/logger.py`
