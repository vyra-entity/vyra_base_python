# VYRA Interface Architecture

Functional description of how interface definitions are loaded, validated,
resolved, and activated as live transport endpoints.

---

## 1. Three-Phase Pipeline Overview

Every VYRA interface follows a deterministic three-phase lifecycle before
network traffic flows:

```mermaid
flowchart TD
    subgraph phase1 [Phase 1 — Definition]
        MF["*.meta.json"]
        MR["ManifestResolver\nadd_manifest_paths()"]
        MF -->|"glob + validate"| MR
    end

    subgraph phase2 [Phase 2 — Runtime]
        ER["EndpointRegistry\nInterfaceEndpoint store"]
        CB["Callbacks\n@remote_service / @remote_actionServer"]
        SR["SchemaResolver\nload_interface(name, SchemaType)"]
        MR -->|"bind_manifest()"| ER
        CB -->|"bind_callback()"| ER
        SR -->|"bind_schema()"| ER
    end

    subgraph phase3 [Phase 3 — Transport]
        TPF["TransportProviderFactory\ncreate_server / create_publisher …"]
        TPR["TransportProviderRegistry\nlive VyraServer / VyraPublisher …"]
        ER -->|"list_ready() → _try_activate"| TPF
        TPF -->|"transport_key"| TPR
        TPF -->|"set_transport_key()"| ER
    end

    ORC["EndpointOrchestrator\nasync event loop"]
    MR -.->|change_event| ORC
    SR -.->|change_event| ORC
    ER -.->|change_event| ORC
    ORC --> phase2
    ORC --> phase3
```

---

## 2. Phase 1 — Definition (ManifestResolver)

### Role
`ManifestResolver` owns the filesystem paths that hold `*.meta.json`
metadata and is responsible for loading and validating those files.

### Key rules
- Single addition point: `add_manifest_paths(paths)` — silently
  deduplicates.
- JSON Schema validation uses `self.manifest_schema_path` which defaults to
  `vyra_base/assets/schemas/interface_config.json`.
- Metadata is cached; cache is invalidated whenever new paths are added.
- Every mutation emits an asyncio `change_event` consumed by the
  `EndpointOrchestrator`.

### Data flow

```mermaid
sequenceDiagram
    participant M as Module (_base_.py)
    participant E as VyraEntity
    participant MR as ManifestResolver
    participant ORC as EndpointOrchestrator
    participant ER as EndpointRegister

    M->>E: add_manifest_paths([/workspace/src/v2_modulemanager_interfaces])
    E->>MR: add_manifest_paths(paths)
    MR->>MR: validate + cache *.meta.json
    MR-->>ORC: change_event.set()
    ORC->>MR: load_interface_metadata()
    ORC->>ER: bind_manifest(fn_name, meta_dict)
```

### `*.meta.json` schema excerpt
```json
{
  "tags": ["zenoh"],
  "type": "service",
  "functionname": "get_interface_list",
  "displayname": "Get Interface List",
  "filetype": ["VBASEGetInterfaceList.proto"],
  "params": [],
  "returns": [{ "name": "interface_list", "datatype": "string[]" }],
  "access_level": 1,
  "displaystyle": { "visible": true, "published": false }
}
```

---

## 3. Phase 2 — Runtime (EndpointRegistry)

### InterfaceEndpoint state machine

```mermaid
stateDiagram-v2
    [*] --> UNREGISTERED : dangling stub created\nby bind_callback()
    UNREGISTERED --> DEFINITION_LOADED : bind_manifest()
    DEFINITION_LOADED --> SCHEMA_RESOLVED : bind_schema()
    DEFINITION_LOADED --> CALLBACK_BOUND : is_bound() = True\n(manifest already set)
    SCHEMA_RESOLVED --> CALLBACK_BOUND : all required\ncallbacks bound
    CALLBACK_BOUND --> ACTIVE : EndpointOrchestrator\nactivates transport
    ACTIVE --> [*] : shutdown / unregister
```

### EndpointRegistry API
```
register_endpoint(endpoint)        → str (key)
bind_callback(name, fn, type)      → bool  ← dangling stub if no endpoint yet
bind_manifest(name, meta_dict)     → bool  ← merges into dangling stub if exists
bind_schema(name, schema_ref)      → bool
set_transport_key(name, key)       → bool  (called by orchestrator)
list_unbound()  / list_incomplete() / list_ready() / list_active()
get_statistics()
```

### Dangling stub pattern
When a callback is registered (e.g. at class decoration time) **before** the
manifest is available, `EndpointRegistry.bind_callback` creates a
`ServiceEndpoint` stub with empty metadata.  When `bind_manifest` later
arrives it checks whether a stub with a matching `functionname` exists and
**merges** the manifest into it, preserving the already-bound callback.

```mermaid
sequenceDiagram
    participant D as @remote_service decorator
    participant ER as EndpointRegistry
    participant ORC as EndpointOrchestrator
    participant MR as ManifestResolver
    participant TPF as TransportProviderFactory

    D->>ER: bind_callback("get_interface_list", fn)
    Note over ER: stub created — manifest_key=None
    MR-->>ORC: change_event
    ORC->>MR: load_interface_metadata()
    ORC->>ER: bind_manifest("get_interface_list", {...})
    Note over ER: stub found by fn_name → merged
    ORC->>ER: bind_schema("get_interface_list", pb2_mod)
    Note over ER: state = CALLBACK_BOUND → ready
    ORC->>TPF: create_server("get_interface_list", callback, protocols)
    ORC->>ER: set_transport_key("get_interface_list", key)
```

---

## 4. Phase 3 — Transport (TransportProviderFactory)

### Activation criteria
The `EndpointOrchestrator` calls `_try_activate_endpoint` only when all
of the following hold:

| Criterion | Field checked |
|---|---|
| a) Definition loaded | `endpoint._manifest_key is not None` |
| b) Required callbacks bound | `endpoint.is_bound() == True` |
| c) Schema resolved | `endpoint._schema_ref is not None` |
| d) Not yet active | `endpoint._transport_key is None` |

### Factory dispatch
```
InterfaceType.SERVICE    → TransportProviderFactory.create_server()
InterfaceType.PUBLISHER  → TransportProviderFactory.create_publisher()
InterfaceType.SUBSCRIBER → TransportProviderFactory.create_subscriber()
InterfaceType.ACTION     → TransportProviderFactory.create_action_server()
```

Each factory method iterates the provider fallback chain:
`Zenoh → ROS2 → Redis → UDS`

The first available provider creates the transport and registers it in
`TransportProviderRegistry`.  The registry key is written back into the
`InterfaceEndpoint` via `EndpointRegistry.set_transport_key()`.

---

## 5. EndpointOrchestrator — Event Loop

```mermaid
sequenceDiagram
    participant MR as ManifestResolver
    participant SR as SchemaResolver
    participant ER as EndpointRegistry
    participant ORC as EndpointOrchestrator
    participant TPF as TransportProviderFactory

    Note over ORC: asyncio.create_task(_run())
    loop _run() — wait for first change event
        MR-->>ORC: change_event (new paths)
        ORC->>ORC: _process_cycle()
        ORC->>MR: load_interface_metadata()
        ORC->>ER: bind_manifest() for each entry
        ORC->>SR: get_interface_for_function() for unresolved
        ORC->>ER: bind_schema() where schema found
        ORC->>ER: list_ready()
        loop each ready endpoint
            ORC->>TPF: create_server / create_publisher …
            ORC->>ER: set_transport_key(name, key)
        end
    end
```

---

## 6. Module Startup Sequence

```mermaid
sequenceDiagram
    participant BS as _base_.py
    participant E as VyraEntity
    participant MR as ManifestResolver
    participant ER as EndpointRegistry
    participant ORC as EndpointOrchestrator
    participant TPF as TransportProviderFactory

    BS->>E: build_entity(settings)
    Note over E: endpoint_registry, manifest_resolver,\nschema_resolver created
    BS->>E: startup_entity()
    E->>TPF: register_provider([Zenoh, ROS2, ...])
    E->>ORC: start()   # spawns background task
    BS->>E: add_manifest_paths([interfaces_path])
    E->>MR: add_manifest_paths(paths)
    MR-->>ORC: change_event
    BS->>ER: bind_callback("initialize", fn)
    BS->>ER: bind_callback("get_interface_list", fn)
    Note over ORC: wakes on change_event
    ORC->>MR: load_interface_metadata()
    ORC->>ER: bind_manifest() × N
    ORC->>ER: bind_schema() × N
    ORC->>ER: list_ready()
    ORC->>TPF: create_server("initialize", ...)
    ORC->>TPF: create_server("get_interface_list", ...)
    Note over E: interfaces live — module ready
```

---

## 7. Decorator-to-Endpoint Flow

```mermaid
flowchart LR
    subgraph classdef [Class Definition Time]
        DEC["@remote_service(name='ping')"]
        BP["ServiceBlueprint\ncreated by decorator"]
        CR["CallbackRegistry\nregister_blueprint()"]
        DEC --> BP --> CR
    end

    subgraph runtime [Module Startup]
        ER["EndpointRegistry\nbind_callback('ping', method)"]
        ORC["EndpointOrchestrator\ndetects new callback"]
        MR["ManifestResolver\nloads ping.meta.json"]
        SR["SchemaResolver\nloads VBASEPing_pb2"]
        TPF["TransportProviderFactory\ncreate_server('ping', ...)"]
        ER --> ORC
        MR --> ORC
        SR --> ORC
        ORC --> TPF
    end

    classdef -->|"entity.endpoint_registry\n.bind_callback()"| runtime
```

---

## 8. Rust Parity Table

| Python | Rust | File |
|---|---|---|
| `InterfaceEndpoint` | `InterfaceEndpoint` | `com/endpoint.rs` |
| `EndpointRegistry` | `EndpointRegistry` | `com/endpoint.rs` |
| `EndpointState` | `EndpointState` | `com/endpoint.rs` |
| `ManifestResolver` | `ManifestResolver` | `com/manifest.rs` |
| `get_manifest_resolver()` | `ManifestResolver::global()` | `com/manifest.rs` |
| `SchemaResolver` | `SchemaResolver` | `com/schema.rs` |
| `SchemaType` (enum) | `SchemaType` (enum) | `com/schema.rs` |
| `Ros2Resolver` | `Ros2Resolver` | `com/schema.rs` |
| `ProtoResolver` | `ProtoResolver` | `com/schema.rs` |
| `EndpointOrchestrator` | `EndpointOrchestrator` | `com/orchestrator.rs` |
| `TransportProviderFactory` | `TransportProviderFactory` | `com/core/factory.rs` |
| `TransportProviderRegistry` | `TransportProviderRegistry` | `com/transport/registry.rs` |
| `EndpointRegistry.change_event` | `EndpointRegistry.subscribe()` → `watch::Receiver<()>` | `com/endpoint.rs` |
| `ManifestResolver.change_event` | `ManifestResolver.subscribe()` | `com/manifest.rs` |
| `asyncio.create_task(orchestrator._run())` | `tokio::spawn(orchestrator.run())` | `core/entity.rs` |
| `load_interface_definitions()` | `load_interface_definitions()` | `src/interface.rs` |
| `add_manifest_paths()` on entity | `entity.add_manifest_paths()` | `core/entity.rs` |
| `add_schema_paths()` on entity | `entity.add_schema_paths()` | `core/entity.rs` |
| `SchemaResolver.load_interface(name, SchemaType)` | `SchemaResolver::load_interface(name, SchemaType)` | `com/schema.rs` |
| `ManifestResolver.add_manifest_paths([...])` | `ManifestResolver::add_manifest_paths(vec![...])` | `com/manifest.rs` |

### Rust-specific notes

1. **Change notification**: Python uses `asyncio.Event`; Rust uses
   `tokio::sync::watch::Sender<()>` + `Receiver<()>`.  The orchestrator
   uses `tokio::select!` over all three receivers.

2. **Callbacks**: Python stores real `Callable` objects.  Rust stores
   `Arc<dyn Any + Send + Sync>` type-erased and downcasts inside the
   transport provider.

3. **Schema ref**: For Rust there is no dynamic import — the `schema_ref`
   is typically `Arc<prost::MessageDescriptor>` or a tonic-generated type.
   `ProtoResolver` loads descriptors from compiled-in stubs.

4. **Singleton**: `ManifestResolver::global()` returns `Arc<ManifestResolver>`;
   Python uses a classical `_instance` class variable.

5. **`#[vyra_module]`** generates `RemoteCallbackProvider::remote_service_names()`
   which the orchestrator queries during `process_cycle` to confirm all
   required callback slots exist in `EndpointRegistry`.
