# vyra_base.core

Central module for VYRA entity management, parameter handling, and volatile storage.

## Public API

```python
from vyra_base.core import VyraEntity, InterfaceBuilder, Parameter, Volatile
```

---

## VyraEntity

`VyraEntity` is the main entry point for every VYRA module. It wires together communication
(multi-protocol transport), state machines, feeders, storage access, security, and parameters
into a single cohesive object.

```python
from vyra_base.core import VyraEntity

entity = await VyraEntity.create("my_module")

# Publish structured data to all registered transports
entity.publish_news("Module started")
entity.publish_state("RUNNING")
entity.publish_error("Sensor unreachable", severity=2)

# Call a remote service on another module
result = await entity.call_service("other_module/read_sensor", {"channel": 1})

# Register interfaces (activates all decorated services/actions)
await entity.set_interfaces(interface_list)

# Access sub-systems
entity.state_machine    # UnifiedStateMachine
entity.parameter        # Parameter instance
entity.volatile         # Volatile instance
entity.storage          # Storage instance (SQLite)
entity.redis_client     # RedisClient instance
```

The entity class provides the internal log history endpoint and manages ring-buffer
log storage (`VyraLogHandler`, capacity 10 000, since 0.1.8+build.128).

---

## InterfaceBuilder

`InterfaceBuilder` loads interface descriptors from the module's `interfaces/config/`
directory and creates the corresponding communication objects via `InterfaceFactory`.

```python
from vyra_base.core import InterfaceBuilder

builder = InterfaceBuilder(module_name="my_module", module_id="abc123")
interfaces = await builder.build()
await entity.set_interfaces(interfaces)
```

See `src/vyra_base/interfaces/README.md` for the config format.

---

## Parameter

`Parameter` manages runtime-configurable module parameters persisted in SQLite.
Other modules can subscribe to a parameter-change event via Zenoh.

```python
from vyra_base.core import Parameter
from vyra_base.storage import DbAccess, DBTYPE

db = DbAccess(db_type=DBTYPE.SQLITE, db_path="/workspace/storage/params.db")
param = Parameter(
    parameter_base_types={"get_param": GetParamSrv, "set_param": SetParamSrv},
    node=None,
    storage_access_persistant=db,
)

await param.set("max_speed", 5.0)
value = await param.get("max_speed")
```

Parameters are validated against `parameter_rules.yaml` by `ParameterValidator`.

---

## Volatile

`Volatile` stores temporary, in-memory key-value data in Redis. It is not persisted
across restarts. Useful for current IO states, runtime status, and temporary buffers.
Other modules can subscribe to volatile change events.

```python
from vyra_base.core import Volatile
from vyra_base.com.transport.t_redis import RedisClient

redis = RedisClient(host="redis", port=6379)
volatile = Volatile(
    storage_access_transient=redis,
    module_name="my_module",
    module_id="abc123",
    node=None,
    transient_base_types={},
)

await volatile.write("current_state", "RUNNING")
value = await volatile.read("current_state")
```

All volatile keys are namespaced: `{module_name}_{module_id}/volatile/{key}`.

---

## Files

| File | Description |
|---|---|
| `entity.py` | `VyraEntity` — main module object |
| `interface_builder.py` | `InterfaceBuilder` — loads and builds interfaces from config |
| `parameter.py` | `Parameter` — persistent, DB-backed module parameters |
| `parameter_rules.yaml` | Validation rules for parameter types and ranges |
| `parameter_validator.py` | `ParameterValidator` — runtime validation |
| `volatile.py` | `Volatile` — Redis-backed temporary storage |

