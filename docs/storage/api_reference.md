# Storage API - Referenz

Vollständige API-Referenz für alle Storage-Klassen und -Methoden.

## DbManipulator

CRUD-Operationen für SQLAlchemy-Modelle.

### Konstruktor

```python
DbManipulator(db_access: DbAccess, model: Type[Base])
```

**Parameter:**
- `db_access`: DbAccess-Instanz (geöffnete Datenbankverbindung)
- `model`: SQLAlchemy-Modell (Subclass von Base)

**Beispiel:**
```python
user_db = DbManipulator(db_access=db, model=User)
```

### Methoden

#### create()

Erstellt einen oder mehrere Datensätze.

```python
async def create(data: Union[dict, list[dict]]) -> DBReturnValue
```

**Parameter:**
- `data`: Dictionary mit Feldwerten oder Liste von Dictionaries

**Rückgabe:**
- `DBReturnValue`:
  - `status`: `DBSTATUS.SUCCESS` oder `DBSTATUS.ERROR`
  - `value`: Erstellte(s) Model-Objekt(e)
  - `details`: Fehlermeldung bei Fehler

**Beispiel:**
```python
result = await user_db.create(data={
    "username": "test",
    "email": "test@example.com"
})

if result.status == DBSTATUS.SUCCESS:
    user = result.value
    print(f"Erstellt: {user.id}")
```

---

#### get_all()

Liest alle Datensätze (optional mit Filtern).

```python
async def get_all(
    filters: Optional[dict] = None,
    order_by: Optional[str] = None,
    limit: Optional[int] = None
) -> DBReturnValue
```

**Parameter:**
- `filters`: Dictionary mit Filterbedingungen
- `order_by`: Spaltenname für Sortierung
- `limit`: Maximale Anzahl Ergebnisse

**Rückgabe:**
- `DBReturnValue`:
  - `status`: `DBSTATUS.SUCCESS` oder `DBSTATUS.NOT_FOUND`
  - `value`: Liste von Model-Objekten
  - `details`: Fehlermeldung bei Fehler

**Beispiel:**
```python
# Alle aktiven Benutzer
result = await user_db.get_all(
    filters={"enabled": True},
    order_by="username"
)

if result.status == DBSTATUS.SUCCESS:
    users = result.value
    for user in users:
        print(user.username)
```

---

#### get_by_id()

Liest einen Datensatz nach Primary Key.

```python
async def get_by_id(id: Union[uuid.UUID, int]) -> DBReturnValue
```

**Parameter:**
- `id`: Primary Key-Wert (oder -1 für letzten Eintrag)

**Rückgabe:**
- `DBReturnValue`:
  - `status`: `DBSTATUS.SUCCESS` oder `DBSTATUS.NOT_FOUND`
  - `value`: Model-Objekt
  - `details`: Fehlermeldung bei Fehler

**Beispiel:**
```python
result = await user_db.get_by_id(id=5)

if result.status == DBSTATUS.SUCCESS:
    user = result.value
    print(f"User: {user.username}")
```

---

#### update()

Aktualisiert einen oder mehrere Datensätze.

```python
async def update(
    data: dict,
    id: Optional[Union[uuid.UUID, int]] = None,
    filters: Optional[dict] = None
) -> DBReturnValue
```

**Parameter:**
- `data`: Dictionary mit zu aktualisierenden Feldern
- `id`: Primary Key (exklusiv mit filters)
- `filters`: Filterbedingungen (exklusiv mit id)

**Rückgabe:**
- `DBReturnValue`:
  - `status`: `DBSTATUS.SUCCESS` oder `DBSTATUS.NOT_FOUND`
  - `value`: Aktualisiertes Model-Objekt (mit id) oder Anzahl (mit filters)
  - `details`: Fehlermeldung bei Fehler

**Beispiel:**
```python
# Nach ID
result = await user_db.update(
    id=5,
    data={"email": "new@example.com"}
)

# Mit Filter
result = await user_db.update(
    filters={"enabled": False},
    data={"enabled": True}
)
```

---

#### delete()

Löscht einen oder mehrere Datensätze.

```python
async def delete(
    id: Optional[Union[uuid.UUID, int]] = None,
    filters: Optional[dict] = None
) -> DBReturnValue
```

**Parameter:**
- `id`: Primary Key (exklusiv mit filters)
- `filters`: Filterbedingungen (exklusiv mit id)

**Rückgabe:**
- `DBReturnValue`:
  - `status`: `DBSTATUS.SUCCESS` oder `DBSTATUS.NOT_FOUND`
  - `value`: Anzahl gelöschter Datensätze
  - `details`: Fehlermeldung bei Fehler

**Beispiel:**
```python
result = await user_db.delete(id=5)

if result.status == DBSTATUS.SUCCESS:
    print(f"{result.value} Datensätze gelöscht")
```

---

#### count()

Zählt Datensätze (optional mit Filtern).

```python
async def count(filters: Optional[dict] = None) -> DBReturnValue
```

**Parameter:**
- `filters`: Filterbedingungen

**Rückgabe:**
- `DBReturnValue`:
  - `status`: `DBSTATUS.SUCCESS`
  - `value`: Anzahl Datensätze (int)

**Beispiel:**
```python
result = await user_db.count(filters={"role": "ADMIN"})
print(f"Admins: {result.value}")
```

---

#### get_table_structure()

Gibt Tabellenspalten zurück.

```python
def get_table_structure() -> DBReturnValue
```

**Rückgabe:**
- `DBReturnValue`:
  - `status`: `DBSTATUS.SUCCESS`
  - `value`: Liste von Spaltennamen

**Beispiel:**
```python
result = user_db.get_table_structure()
columns = result.value
print(f"Spalten: {columns}")
# ['id', 'username', 'email', ...]
```

---

## RedisClient

Asynchroner Redis-Client mit TLS und Pub/Sub.

### Konstruktor

```python
RedisClient(
    module_name: str,
    host: str = None,
    port: int = None,
    username: str = None,
    password: str = None,
    db: int = 0,
    use_tls: bool = None
)
```

**Parameter:**
- `module_name`: Modul-Name (für Logging)
- `host`: Redis-Host (default: REDIS_HOST env oder "redis")
- `port`: Redis-Port (default: REDIS_PORT env oder 6379)
- `username`: ACL-Username (default: REDIS_USERNAME env)
- `password`: ACL-Password (default: REDIS_PASSWORD env)
- `db`: Datenbank-Nummer (default: 0)
- `use_tls`: TLS aktivieren (default: REDIS_TLS_ENABLED env oder True)

### Key-Value Methoden

#### set()

```python
async def set(
    key: str,
    value: str,
    ex: Optional[int] = None,
    px: Optional[int] = None,
    nx: bool = False,
    xx: bool = False
) -> bool
```

**Parameter:**
- `key`: Schlüssel
- `value`: Wert
- `ex`: Ablaufzeit in Sekunden
- `px`: Ablaufzeit in Millisekunden
- `nx`: Nur setzen wenn nicht existiert
- `xx`: Nur setzen wenn existiert

---

#### get()

```python
async def get(key: str) -> Optional[str]
```

**Rückgabe:** Wert oder None

---

#### delete()

```python
async def delete(*keys: str) -> int
```

**Rückgabe:** Anzahl gelöschter Keys

---

#### exists()

```python
async def exists(*keys: str) -> int
```

**Rückgabe:** Anzahl existierender Keys

---

### Hash-Methoden

#### hset()

```python
async def hset(
    name: str,
    key: Optional[str] = None,
    value: Optional[str] = None,
    mapping: Optional[dict] = None
) -> int
```

---

#### hget()

```python
async def hget(name: str, key: str) -> Optional[str]
```

---

#### hgetall()

```python
async def hgetall(name: str) -> dict
```

---

#### hdel()

```python
async def hdel(name: str, *keys: str) -> int
```

---

### List-Methoden

#### lpush() / rpush()

```python
async def lpush(name: str, *values: str) -> int
async def rpush(name: str, *values: str) -> int
```

---

#### lpop() / rpop()

```python
async def lpop(name: str) -> Optional[str]
async def rpop(name: str) -> Optional[str]
```

---

#### lrange()

```python
async def lrange(name: str, start: int, end: int) -> list[str]
```

---

#### llen()

```python
async def llen(name: str) -> int
```

---

### Set-Methoden

#### sadd()

```python
async def sadd(name: str, *values: str) -> int
```

---

#### srem()

```python
async def srem(name: str, *values: str) -> int
```

---

#### smembers()

```python
async def smembers(name: str) -> set[str]
```

---

#### sismember()

```python
async def sismember(name: str, value: str) -> bool
```

---

#### scard()

```python
async def scard(name: str) -> int
```

---

### Pub/Sub-Methoden

#### publish()

```python
async def publish(channel: str, message: str) -> int
```

**Rückgabe:** Anzahl Empfänger

---

#### subscribe()

```python
async def subscribe(*channels: str) -> AsyncIterator[dict]
```

**Rückgabe:** Async-Iterator für Nachrichten

---

#### psubscribe()

```python
async def psubscribe(*patterns: str) -> AsyncIterator[dict]
```

**Rückgabe:** Async-Iterator für Pattern-Nachrichten

---

#### multi_listener()

```python
async def multi_listener(
    channels: Optional[list[str]] = None,
    patterns: Optional[list[str]] = None
) -> AsyncIterator[dict]
```

---

### Utility-Methoden

#### keys()

```python
async def keys(pattern: str = "*") -> list[str]
```

**⚠️ Warnung:** Langsam bei vielen Keys!

---

#### scan()

```python
async def scan(
    cursor: int = 0,
    match: Optional[str] = None,
    count: Optional[int] = None
) -> tuple[int, list[str]]
```

---

#### ttl()

```python
async def ttl(key: str) -> int
```

**Rückgabe:** Verbleibende Sekunden (-1 = kein TTL, -2 = nicht existent)

---

#### expire()

```python
async def expire(key: str, seconds: int) -> bool
```

---

#### type()

```python
async def type(key: str) -> str
```

**Rückgabe:** "string", "hash", "list", "set", "zset", "none"

---

## DbAccess

Low-Level Datenbankzugriff (normalerweise über DbManipulator verwendet).

### Konstruktor

```python
DbAccess(
    db_path: str,
    echo: bool = False
)
```

**Parameter:**
- `db_path`: Pfad zur SQLite-Datenbank
- `echo`: SQL-Logging aktivieren

### Methoden

#### open()

```python
async def open() -> None
```

Öffnet Datenbankverbindung und erstellt Tabellen.

---

#### close()

```python
async def close() -> None
```

Schließt Datenbankverbindung.

---

#### create_tables()

```python
async def create_tables() -> None
```

Erstellt alle registrierten Tabellen.

---

## DBReturnValue

Standardisierte Rückgabewerte für DB-Operationen.

### Attribute

- `status`: Status-Code (siehe DBSTATUS)
- `value`: Hauptrückgabewert (Daten oder Fehlermeldung)
- `details`: Zusätzliche Details

### Methoden

#### success_return()

```python
def success_return() -> DBReturnValue
```

Setzt Status auf SUCCESS.

---

#### error_return()

```python
def error_return(details: str = "") -> DBReturnValue
```

Setzt Status auf ERROR mit Details.

---

## DBSTATUS (Enum)

Status-Codes für Datenbank-Operationen:

- `DBSTATUS.SUCCESS`: Operation erfolgreich
- `DBSTATUS.ERROR`: Allgemeiner Fehler
- `DBSTATUS.NOT_FOUND`: Datensatz nicht gefunden
- `DBSTATUS.DUPLICATE`: Duplikat (Primary Key Verletzung)
- `DBSTATUS.INVALID`: Ungültige Parameter

---

## Siehe auch

- [Datenbankzugriff einrichten](./database_setup.md)
- [Datenbank-Operationen](./database_operations.md)
- [Redis-Verbindung](./redis_connection.md)
- [Tabellen anlegen](./table_creation.md)
