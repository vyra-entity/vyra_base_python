# Redis-Verbindung und Operationen

Dieser Guide zeigt die Verwendung des `RedisClient` für Key-Value-Storage und Pub/Sub mit TLS-Unterstützung.

## 1. Redis-Client erstellen

### Einfache Initialisierung (mit Umgebungsvariablen)

```python
from vyra_base.storage.redis_client import RedisClient

# Verwendet automatisch Umgebungsvariablen:
# - REDIS_HOST (default: "redis")
# - REDIS_PORT (default: 6379)
# - REDIS_USERNAME (default: "modulemanager")
# - REDIS_PASSWORD
# - REDIS_TLS_ENABLED (default: "true")
redis = RedisClient(module_name="my_module")

# Daten speichern
await redis.set("config:param1", "value123")

# Daten lesen
value = await redis.get("config:param1")
print(f"Wert: {value}")
```

### Manuelle Konfiguration

```python
redis = RedisClient(
    module_name="my_module",
    host="redis",
    port=6379,
    username="modulemanager",
    password="secure_password",
    db=0,
    use_tls=True
)
```

### TLS-Konfiguration

```python
# TLS mit Standard-Zertifikatspfaden
redis = RedisClient(
    module_name="my_module",
    use_tls=True
)
# Verwendet automatisch:
# - /workspace/storage/certificates/redis/ca-cert.pem
# - /workspace/storage/certificates/redis/client-cert.pem
# - /workspace/storage/certificates/redis/client-key.pem

# TLS deaktivieren (nur Development!)
redis = RedisClient(
    module_name="my_module",
    use_tls=False
)
```

## 2. Key-Value Operationen

### String-Werte

```python
# Speichern
await redis.set("config:timeout", "30")
await redis.set("config:enabled", "true")

# Lesen
timeout = await redis.get("config:timeout")
print(f"Timeout: {timeout}")  # "30"

# Löschen
await redis.delete("config:timeout")

# Existenz prüfen
exists = await redis.exists("config:timeout")
print(f"Existiert: {exists}")  # False
```

### Mit Expiration (TTL)

```python
# Wert mit Ablaufzeit (in Sekunden)
await redis.set("session:user123", "active", ex=3600)  # 1 Stunde

# TTL prüfen
ttl = await redis.ttl("session:user123")
print(f"Ablauf in {ttl} Sekunden")

# TTL setzen für existierenden Key
await redis.expire("session:user123", 7200)  # 2 Stunden
```

### JSON-Daten

```python
import json

# Objekt speichern
config = {
    "timeout": 30,
    "retry_count": 3,
    "enabled": True
}
await redis.set("config:main", json.dumps(config))

# Objekt laden
data = await redis.get("config:main")
if data:
    config = json.loads(data)
    print(f"Timeout: {config['timeout']}")
```

## 3. Hash-Operationen

Hashes sind ideal für strukturierte Daten:

```python
# Hash setzen (alle Felder)
await redis.hset("user:1", mapping={
    "username": "max.mustermann",
    "email": "max@example.com",
    "role": "ADMIN"
})

# Einzelnes Feld setzen
await redis.hset("user:1", "last_login", "2026-01-29 14:30:00")

# Hash komplett lesen
user_data = await redis.hgetall("user:1")
print(user_data)
# {'username': 'max.mustermann', 'email': 'max@example.com', ...}

# Einzelnes Feld lesen
username = await redis.hget("user:1", "username")
print(f"Username: {username}")

# Mehrere Felder lesen
values = await redis.hmget("user:1", ["username", "email"])
print(f"Username: {values[0]}, Email: {values[1]}")

# Feld löschen
await redis.hdel("user:1", "last_login")
```

## 4. List-Operationen

Listen für Queues, Logs, History:

```python
# Elemente hinzufügen (rechts)
await redis.rpush("logs:system", "Log entry 1")
await redis.rpush("logs:system", "Log entry 2")

# Elemente hinzufügen (links)
await redis.lpush("queue:tasks", "Task 1")

# Länge ermitteln
length = await redis.llen("logs:system")
print(f"{length} Log-Einträge")

# Bereich lesen
logs = await redis.lrange("logs:system", 0, 9)  # Erste 10 Einträge
for log in logs:
    print(log)

# Element entfernen (von rechts)
last_log = await redis.rpop("logs:system")

# Element entfernen (von links)
first_task = await redis.lpop("queue:tasks")
```

## 5. Set-Operationen

Sets für eindeutige Werte:

```python
# Mitglieder hinzufügen
await redis.sadd("users:online", "user1", "user2", "user3")

# Mitglied prüfen
is_online = await redis.sismember("users:online", "user1")
print(f"User1 online: {is_online}")

# Alle Mitglieder
online_users = await redis.smembers("users:online")
print(f"Online: {online_users}")

# Anzahl Mitglieder
count = await redis.scard("users:online")
print(f"{count} Benutzer online")

# Mitglied entfernen
await redis.srem("users:online", "user2")
```

## 6. Pub/Sub - Publish/Subscribe

### Publisher

```python
# Nachricht veröffentlichen
await redis.publish("events:system", "System started")
await redis.publish("events:user", "User logged in")

# JSON-Nachricht
import json
event = {
    "type": "user_login",
    "username": "max",
    "timestamp": "2026-01-29 14:30:00"
}
await redis.publish("events:user", json.dumps(event))
```

### Subscriber (Einzelner Channel)

```python
import asyncio
import json

async def handle_messages(redis: RedisClient):
    """Empfängt Nachrichten von einem Channel."""
    async for message in redis.subscribe("events:system"):
        if message["type"] == "message":
            data = message["data"]
            print(f"Nachricht empfangen: {data}")
            
            # JSON parsen
            try:
                event = json.loads(data)
                print(f"Event: {event['type']}")
            except:
                pass

# Task starten
asyncio.create_task(handle_messages(redis))
```

### Subscriber (Pattern Matching)

```python
async def handle_pattern_messages(redis: RedisClient):
    """Empfängt Nachrichten von mehreren Channels via Pattern."""
    # Abonniert: events:system, events:user, events:error, ...
    async for message in redis.psubscribe("events:*"):
        if message["type"] == "pmessage":
            channel = message["channel"]
            data = message["data"]
            print(f"[{channel}] {data}")

asyncio.create_task(handle_pattern_messages(redis))
```

### Multi-Listener (Mehrere Channels gleichzeitig)

```python
async def multi_listener(redis: RedisClient):
    """Hört auf mehrere Channels und Patterns."""
    channels = ["events:system", "events:user"]
    patterns = ["logs:*", "alerts:*"]
    
    async for message in redis.multi_listener(channels=channels, patterns=patterns):
        msg_type = message["type"]
        
        if msg_type == "message":
            # Normale Channel-Nachricht
            channel = message["channel"]
            data = message["data"]
            print(f"[{channel}] {data}")
            
        elif msg_type == "pmessage":
            # Pattern-Nachricht
            pattern = message["pattern"]
            channel = message["channel"]
            data = message["data"]
            print(f"[{pattern}→{channel}] {data}")

asyncio.create_task(multi_listener(redis))
```

## 7. Batch-Operationen

### Pipeline (für atomare Operationen)

```python
# Pipeline erstellen
pipe = redis.pipeline()

# Mehrere Befehle hinzufügen
pipe.set("counter:a", 1)
pipe.set("counter:b", 2)
pipe.incr("counter:a")
pipe.incr("counter:b")

# Ausführen
results = await pipe.execute()
print(results)  # [True, True, 2, 3]
```

### Multi-Get

```python
# Mehrere Keys gleichzeitig lesen
keys = ["config:param1", "config:param2", "config:param3"]
values = await redis.mget(*keys)

for key, value in zip(keys, values):
    print(f"{key}: {value}")
```

### Multi-Set

```python
# Mehrere Keys gleichzeitig setzen
await redis.mset({
    "config:param1": "value1",
    "config:param2": "value2",
    "config:param3": "value3"
})
```

## 8. Key-Management

### Keys suchen (Pattern)

```python
# Alle Keys mit Prefix
keys = await redis.keys("config:*")
print(f"Config-Keys: {keys}")

# Alle User-Sessions
sessions = await redis.keys("session:user*")
```

**⚠️ Warnung:** `keys()` ist langsam bei vielen Keys. In Produktion `scan()` verwenden!

### Scan (Iterator)

```python
# Sicherer Iterator für große Keyspaces
cursor = 0
all_keys = []

while True:
    cursor, keys = await redis.scan(cursor, match="config:*", count=100)
    all_keys.extend(keys)
    if cursor == 0:
        break

print(f"Gefunden: {len(all_keys)} Keys")
```

### Key-Typ prüfen

```python
key_type = await redis.type("config:main")
print(f"Typ: {key_type}")  # "string", "hash", "list", "set", ...
```

## 9. Integration in Component

```python
from vyra_base.state import OperationalStateMachine
from vyra_base.storage.redis_client import RedisClient

class Component(OperationalStateMachine):
    
    def __init__(self):
        super().__init__()
        self.redis: RedisClient = None
    
    async def initialize(self, request=None, response=None) -> bool:
        """Initialisiert Redis-Verbindung."""
        self.redis = RedisClient(
            module_name="my_module",
            host="redis",
            port=6379,
            use_tls=True
        )
        
        # Konfiguration aus Redis laden
        config_json = await self.redis.get("config:my_module")
        if config_json:
            import json
            self.config = json.loads(config_json)
        
        # Event-Listener starten
        asyncio.create_task(self._listen_events())
        
        return True
    
    async def _listen_events(self):
        """Hört auf System-Events."""
        async for message in self.redis.subscribe("events:system"):
            if message["type"] == "message":
                await self._handle_event(message["data"])
    
    async def _handle_event(self, event: str):
        """Verarbeitet eingehende Events."""
        print(f"Event received: {event}")
    
    async def stop(self, request=None, response=None) -> bool:
        """Schließt Redis-Verbindung."""
        if self.redis:
            await self.redis.close()
        return True
```

## 10. Best Practices

### ✅ Empfohlen

```python
# Strukturierte Key-Namen (Namespace)
await redis.set("module:config:timeout", "30")
await redis.set("module:state:current", "RUNNING")

# JSON für komplexe Daten
data = {"a": 1, "b": 2}
await redis.set("data:obj", json.dumps(data))

# Expiration für Sessions/Caches
await redis.set("session:user123", token, ex=3600)

# Hashes für Objekte
await redis.hset("user:123", mapping={
    "username": "test",
    "email": "test@example.com"
})
```

### ❌ Vermeiden

```python
# NICHT: Lange Werte ohne Struktur
await redis.set("data", "very_long_unstructured_string...")

# NICHT: keys() bei vielen Einträgen
keys = await redis.keys("*")  # Blockiert Redis!

# NICHT: Keine Expiration für temporäre Daten
await redis.set("temp:cache", data)  # Wird nie gelöscht!

# NICHT: Einzelne Operationen in Schleife
for i in range(1000):
    await redis.set(f"key:{i}", f"value{i}")
# BESSER: Pipeline verwenden
```

## 11. Fehlerbehandlung

```python
from vyra_base.helper.logger import Logger, LogEntry

try:
    value = await redis.get("config:param")
    if value is None:
        Logger.log(LogEntry("Key not found").warn())
except redis.ConnectionError:
    Logger.log(LogEntry("Redis connection failed").error())
except redis.TimeoutError:
    Logger.log(LogEntry("Redis timeout").error())
except Exception as e:
    Logger.log(LogEntry(f"Redis error: {e}").error())
```

## Nächste Schritte

- [Tabellen und Modelle erstellen](./table_creation.md)
- [API-Referenz](./api_reference.md)
- [Zurück zur Übersicht](./README.md)
