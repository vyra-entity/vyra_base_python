# Storage API - Database & Redis

Die VYRA Storage API bietet a einheitliche Schnittstelle für Database Access (SQLAlchemy) und Redis-Caching mit TLS-Unterstützung.

## Overview

**Main Components:**
- **DbManipulator**: SQLAlchemy-basierte Database-CRUD-Operationen
- **RedisClient**: Asynchroner Redis-Client mit TLS und Pub/Sub
- **Base**: SQLAlchemy Base-Klasse für Data Models
- **DBReturnValue**: Standardisierte Rückgabewerte mit Status-Tracking

## Table of Contents

1. [Database Access setup](./database_setup.md)
2. [Database-Operationen](./database_operations.md)
3. [Redis-Connection](./redis_connection.md)
4. [Tables anlegen](./table_creation.md)
6. [VYRA Base Blueprint](./vyra_base_blueprint.md)

## Quick Start

### Database (SQLAlchemy)

```python
from vyra_base.storage.db_access import DbAccess
from vyra_base.storage.db_manipulator import DbManipulator
from your_module.models import User

# 1. Database Connection create
db = DbAccess(db_path="/workspace/storage/database/module.db")
await db.open()

# 2. Manipulator für Table create
user_db = DbManipulator(db_access=db, model=User)

# 3. Daten read
result = await user_db.get_all()
if result.status == DBSTATUS.SUCCESS:
    users = result.value
    print(f"Found: {len(users)} User")
```

### Redis

```python
from vyra_base.storage.redis_client import RedisClient

# 1. Redis-Client create
redis = RedisClient(
    module_name="my_module",
    host="redis",
    port=6379,
    use_tls=True
)

# 2. Daten speichern
await redis.set("config:param1", "value123")

# 3. Daten read
value = await redis.get("config:param1")
print(f"Wert: {value}")
```

## Status-Codes

Alle Database-Operationen geben `DBReturnValue` zurück mit folgenden Status-Codes:

```python
DBSTATUS.SUCCESS      # Operation erfolgreich
DBSTATUS.ERROR        # Allgemeiner Error
DBSTATUS.NOT_FOUND    # Datensatz nicht Found
DBSTATUS.DUPLICATE    # Duplikat (Primary Key Verletzung)
DBSTATUS.INVALID      # Ungültige Parameter
```

## Best Practices

1. **Immer async/await verwenden** - Alle DB/Redis-Operationen sind asynchron
2. **Status prüfen** - Vor Zugriff auf `result.value` immer `result.status` prüfen
3. **Ressourcen schließen** - `await db.close()` nach Verwendung aufrufen
4. **TLS aktivieren** - In Produktion immer Redis TLS verwenden
5. **Error Handling** - `@ErrorTraceback.w_check_error_exist` Decorator verwenden

## Architektur

```
┌─────────────────────────────────────┐
│         Your Module Code            │
│  (application/component classes)    │
└──────────────┬──────────────────────┘
               │
               ├──────────────┐
               ▼              ▼
    ┌──────────────┐   ┌──────────────┐
    │DbManipulator │   │ RedisClient  │
    │  (CRUD API)  │   │ (KV + PubSub)│
    └──────┬───────┘   └──────┬───────┘
           │                  │
           ▼                  ▼
    ┌──────────────┐   ┌──────────────┐
    │  DbAccess    │   │   Redis      │
    │ (SQLAlchemy) │   │  (TLS/ACL)   │
    └──────┬───────┘   └──────┬───────┘
           │                  │
           ▼                  ▼
    ┌──────────────────────────────────┐
    │   SQLite DB           Redis      │
    │ /workspace/storage/  redis:6379  │
    └──────────────────────────────────┘
```

## Siehe auch

- **State Machine**: [../state/README.md](../state/README.md)
- **Helper Functions**: [../helper/README.md](../helper/README.md)
- **Communication Layer**: [../com/README.md](../com/README.md)
