# vyra_base.storage

SQLite/PostgreSQL database storage and Redis volatile storage for VYRA modules.

## Public API

```python
from vyra_base.storage import (
    # Database
    DbAccess,
    DBTYPE,
    DBSTATUS,
    DBMESSAGE,
    DbManipulator,
    DBReturnValue,
    # Base classes for table definition
    Base,
    IntEnum,
    # Storage base
    Storage,
    # Pre-built tables
    tb_parameters,    # alias for Parameter table
    tb_error_logs,    # alias for ErrorLog table
    ERROR_LOG_MAX_ROWS,
    # Redis (backward-compat re-export from com/transport/t_redis)
    RedisClient,
    REDIS_TYPE,
)
```

> **Note:** `RedisClient` has been moved to `vyra_base.com.transport.t_redis`.
> The import above is kept for backward compatibility.

---

## Database (SQLite / PostgreSQL)

### DbAccess

Manages the SQLAlchemy async engine and session factory:

```python
from vyra_base.storage import DbAccess, DBTYPE

# SQLite (default for modules)
db = DbAccess(db_type=DBTYPE.SQLITE, db_path="/workspace/storage/data.db")
await db.connect()

# PostgreSQL
db = DbAccess(db_type=DBTYPE.POSTGRES, db_url="postgresql+asyncpg://user:pw@host/dbname")
await db.connect()
```

### DbManipulator

CRUD operations on SQLAlchemy ORM models:

```python
from vyra_base.storage import DbAccess, DbManipulator, DBTYPE
from vyra_base.storage.tb_params import Parameter

db = DbAccess(db_type=DBTYPE.SQLITE, db_path="/workspace/storage/params.db")
manipulator = DbManipulator(db, Parameter)

# Create
result = await manipulator.create({"name": "max_speed", "value": "5.0", "type": "float"})

# Read
rows = await manipulator.read(filters={"name": "max_speed"})

# Update
result = await manipulator.update({"name": "max_speed"}, {"value": "10.0"})

# Delete
result = await manipulator.delete({"name": "max_speed"})
```

`DBReturnValue` is a typed result object containing `status`, `data`, and `message`.

### Defining Custom Tables

All tables **must** use the `tb_` prefix by convention:

```python
from sqlalchemy import Column, String, Float
from vyra_base.storage import Base

class tb_sensor_data(Base):
    __tablename__ = "tb_sensor_data"

    id     = Column(String, primary_key=True)
    value  = Column(Float, nullable=False)
    unit   = Column(String, default="°C")
```

---

## Redis

`RedisClient` is the recommended way to interact with Redis in VYRA modules.
It supports TLS, async operations, and all standard Redis data types.

```python
from vyra_base.com.transport.t_redis import RedisClient, REDIS_TYPE

redis = RedisClient(
    host="redis",
    port=6379,
    ssl=True,
    ssl_ca_certs="/workspace/storage/certificates/redis/ca-cert.pem",
)

# String
await redis.set("key", "value")
value = await redis.get("key")

# Hash
await redis.hset("my_hash", "field", "value")
val = await redis.hget("my_hash", "field")

# Pub/Sub
await redis.publish("channel", "message")
await redis.subscribe("channel", callback=my_handler)
```

---

## Storage Base Class

`Storage` is the base class for per-module storage objects that bundle `DbAccess`
and optionally `RedisClient`:

```python
from vyra_base.storage import Storage

storage = Storage(db_access=db, redis_client=redis)
await storage.initialize()
```

---

## Pre-built Tables

| Export | Table Name | Description |
|---|---|---|
| `tb_parameters` | `tb_params` | Module parameter store |
| `tb_error_logs` | `tb_error_log` | Error log ring buffer |

`ERROR_LOG_MAX_ROWS` defines the maximum rows before the error log table is trimmed.

---

## Files

| File | Description |
|---|---|
| `db_access.py` | `DbAccess`, `DBTYPE`, `DBSTATUS`, `DBMESSAGE` |
| `db_manipulator.py` | `DbManipulator`, `DBReturnValue` |
| `storage.py` | `Storage` base class |
| `tb_base.py` | `Base` (SQLAlchemy declarative base), `IntEnum` |
| `tb_params.py` | `Parameter` table for module parameters |
| `tb_error_log.py` | `ErrorLog` table + `ERROR_LOG_MAX_ROWS` |
