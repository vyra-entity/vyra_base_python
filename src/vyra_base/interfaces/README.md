# VYRA Interfaces

This directory contains the built-in interface definitions for the VYRA framework, plus the tooling that generates interface files for individual modules. Interfaces define the communication contracts between VYRA entities and are transported via **Zenoh** (default) or other registered transports.

## Directory Structure

```
interfaces/
├── config/         # Metadata JSON files (*.meta.json) for built-in interfaces
│   ├── vyra_core.meta.json      # Core VYRA interfaces (get_interface_list, parameters, volatiles, etc.)
│   ├── vyra_com.meta.json       # Communication interfaces (feeds, events)
│   ├── vyra_security.meta.json  # Security interfaces
│   ├── vyra_state.meta.json     # State-machine interfaces
│   ├── vyra_plugin.meta.json    # Plugin interfaces
│   ├── vyra_skills.meta.json    # Skill interfaces (read_all_skills, add_skill, etc.)
│   └── RESERVED.list            # Reserved function names (module override forbidden)
├── tools/
│   └── generate_interfaces.py   # Interface file generator (consumed by modules)
├── package.xml                   # ROS2 package stub (build tooling only)
└── CMakeLists.template.txt       # CMakeLists template used during generation
```

## Interface Types

The VYRA framework knows four logical interface types, each mapping to a different
communication pattern.

### Speaker — event stream
**Pattern**: one-to-many publish/subscribe  
**Transport**: Zenoh pub/sub  
**Proto pattern**: message type pushed to a topic key expression  
**Use cases**: error feeds, state-change notifications, news feeds, sensor data

```python
# Exposing a Speaker via VyraEntity
await entity.expose_speaker("error_feed", ErrorFeedHandler)
```

### Callable — request/response
**Pattern**: synchronous call, waits for one reply  
**Transport**: Zenoh queryable / remote-service decorator  
**Proto pattern**: request + response message pair  
**Use cases**: health checks, parameter get/set, data queries

```python
# Registering a Callable
@remote_service(entity, "health_check")
async def health_check(self) -> dict:
    return {"alive": True, "state": self.state}
```

### Job — long-running action
**Pattern**: goal → feedback stream → result (ROS2 action style)  
**Transport**: Zenoh (multi-key expression pattern)  
**Use cases**: multi-step workflows, cancelable tasks, progress-based operations

### Proto (`.proto` files) — serialisation contract
**Purpose**: Protocol Buffer definitions used as the wire format for all Zenoh,
gRPC, and external REST/TCP transports.  
**Location**: `.proto` files are *generated* per-module from the `*.meta.json`
config files by `tools/generate_interfaces.py`.  
**Key benefits**:
- Transport-independent schema
- Multi-language support (Python, C++, Go, …)
- Efficient binary serialisation
- Strong typing with auto-generated stubs

## Configuration Files (`config/` directory)

Each `*.meta.json` file is a JSON array that describes one or more interfaces.
The built-in files ship with `vyra_base` and are merged with module-specific
configs during the module build pipeline.

### File Format

```json
[
    {
        "tags": ["zenoh"],
        "type": "service",
        "functionname": "get_interface_list",
        "displayname": "Get Interface List",
        "description": "Retrieve the list of interfaces exposed by this entity.",
        "filetype": ["VBASEGetInterfaceList.proto"],
        "params": [],
        "returns": [
            {
                "name": "interface_list",
                "datatype": "string[]",
                "displayname": "Interface List",
                "description": "JSON-serialised interface config strings"
            }
        ],
        "access_level": 1,
        "displaystyle": {
            "visible": true,
            "published": false
        }
    }
]
```

### Field Reference

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `tags` | `string[]` | yes | Transport tags — use `["zenoh"]` for standard interfaces |
| `type` | `string` | yes | `"service"` / `"message"` / `"action"` |
| `functionname` | `string` | yes | Unique snake_case identifier; becomes the Zenoh key suffix |
| `displayname` | `string` | yes | Human-readable name (UI labels) |
| `description` | `string` | yes | Short description |
| `filetype` | `string[]` | yes | Generated file names; Zenoh-only interfaces use a single `.proto` entry |
| `params` | `object[]` | no | Request parameters (see Parameter Schema below) |
| `returns` | `object[]` | no | Response fields |
| `access_level` | `int` | no | Minimum security level required (1–5, default `1`) |
| `displaystyle` | `object` | no | UI display hints (`visible`, `published`, `category`, `icon`, `order`) |

### Tags

All built-in VYRA interfaces use `"tags": ["zenoh"]`.  Custom modules may extend
this with additional transport tags if they register extra transports, but
`["zenoh"]` must always be present for a callable to be reachable via the
default transport.

### Parameter / Return Schema

```json
{
    "name": "key",
    "datatype": "string",
    "displayname": "Parameter Key",
    "description": "The parameter key to look up.",
    "required": true,
    "default": null,
    "validation": {
        "pattern": "^[a-z_]+$"
    }
}
```

Supported `datatype` values: `bool`, `int`, `int32`, `int64`, `float`, `float64`,
`string`, `string[]`, `int[]`, `float[]`, `bool[]`, `object`, `object[]`

### Display Style Schema

```json
{
    "visible": true,
    "published": false,
    "category": "System",
    "icon": "settings",
    "color": "#4a90e2",
    "order": 10
}
```

`published: true` marks the interface as part of the public module API that is
exposed in the VYRA Dashboard.

## Zenoh Topic Naming

Every interface function maps to a Zenoh key expression following this pattern:

```
{module_name}_{module_id}[/{namespace}]/{functionname}[/{subsection}]
```

Examples:
- `v2_modulemanager_abc123/get_interface_list`
- `v2_modulemanager_abc123/feeder/news_feed` (with namespace)
- `v2_modulemanager_abc123/action/run_task/cancel` (with subsection)

## Interface Generator (`tools/generate_interfaces.py`)

The `generate_interfaces.py` script is called by module build pipelines during
`setup_interfaces.py` execution. It reads the merged base + module `*.meta.json`
files and generates:
- `.proto` files for Zenoh / gRPC serialisation
- (optionally) `.msg` / `.srv` / `.action` stubs for ROS2 transport

```bash
# Run from within a module's interface package:
python3 tools/generate_interfaces.py              # Generate all interfaces
python3 tools/generate_interfaces.py --validate   # Validate configs, no output
python3 tools/generate_interfaces.py --cleanup    # Mark deprecated files
```

### Build Pipeline

`setup_interfaces.py` (module-side) executes:

1. **Copy** config + build files from the installed `vyra_base` library into the
   interface package.
2. **Validate** module-specific config JSONs against the `RESERVED.list` function
   name list — exits with an error on violation.
3. **Schema validation** — validates every `*.meta.json` against
   `assets/schemas/interface_config.json`. Invalid files are excluded from
   generation with a `WARNING` log (non-fatal).
4. **Generate** `.proto` files (and optional `.msg`/`.srv`/`.action` stubs) via
   `generate_interfaces.py`.
5. **Compile** Protobuf stubs (Python `*_pb2.py` / `*_pb2_grpc.py`) via `protoc`.

### RESERVED.list

`config/RESERVED.list` contains function names that are owned by `vyra_base` and
may not be overridden by module config files. Any config entry whose
`functionname` appears in the reserved list will cause the build to abort with a
descriptive error.

## Usage Examples

### Calling a built-in Callable (Python)

```python
from vyra_base.com.transport.t_zenoh import ZenohTransport

transport = ZenohTransport(entity)
response = await transport.call(
    "v2_modulemanager_abc123/get_interface_list",
    payload={}
)
print(response["interface_list"])
```

### Subscribing to a Speaker (Python)

```python
from vyra_base.interfaces import InterfaceBuilder

async def on_error_feed(payload: dict) -> None:
    print(f"Error: {payload['message']}")

await entity.subscribe_speaker("error_feed", on_error_feed)
```

### Custom module config entry

```json
[
    {
        "tags": ["zenoh"],
        "type": "service",
        "functionname": "my_custom_service",
        "displayname": "My Custom Service",
        "description": "Does something module-specific.",
        "filetype": ["MyCustomService.proto"],
        "params": [
            {
                "name": "input",
                "datatype": "string",
                "displayname": "Input",
                "description": "The input value."
            }
        ],
        "returns": [
            {
                "name": "result",
                "datatype": "string",
                "displayname": "Result",
                "description": "The computed result."
            }
        ],
        "access_level": 2,
        "displaystyle": {
            "visible": true,
            "published": true
        }
    }
]
```

## Skill Interfaces (`vyra_skills.meta.json`)

The `vyra_skills.meta.json` file defines the built-in **Skill API** that every
VYRA module exposes automatically. A **Skill** is a named set of mappings that
links a logical skill type (defined in the blueprint) to concrete module resources
(parameters, volatiles, interfaces) along with optional instance-specific defaults.

### Skill Services

| Function Name | Type | Description |
|---------------|------|-------------|
| `read_all_skills` | service | Returns all skill instances as JSON array |
| `get_skill` | service | Returns a single skill instance by ID |
| `add_skill` | service | Creates a new skill instance (fails if ID exists) |
| `update_skill` | service | Partially updates an existing skill instance |
| `delete_skill` | service | Deletes a skill instance by ID |

### Skill Object Fields

```json
{
    "id": "motion_slow",
    "skill_type": "motion_control",
    "is_enabled": true,
    "parameter_mapping": {
        "max_speed": "drive.max_speed_mps",
        "acceleration": "drive.acceleration_ms2"
    },
    "volatile_mapping": {
        "current_position": "robot.current_position"
    },
    "interface_mapping": {
        "move_to": "drive_move_to_position",
        "stop": "drive_emergency_stop"
    },
    "local_defaults": {
        "drive.max_speed_mps": 0.3,
        "drive.acceleration_ms2": 0.5
    },
    "displayname": "Motion — Slow",
    "description": "Safe low-speed motion profile for confined spaces.",
    "tags": ["motion", "safe"],
    "created_at": "2026-04-22T10:00:00",
    "updated_at": "2026-04-22T10:00:00"
}
```

### Multiple Skill Instances per Type

Multiple skill instances can share the same `skill_type` but have different
`local_defaults`, allowing different operational profiles without duplicating
parameter/interface mappings:

```
motion_slow   → skill_type: motion_control — max_speed: 0.3 m/s
motion_normal → skill_type: motion_control — max_speed: 1.2 m/s
motion_rough  → skill_type: motion_control — max_speed: 2.5 m/s
```

### Blueprint Verification

The Module Manager queries `read_all_skills` during startup to verify that a
module satisfies the `required_skills` of its declared blueprint. See the
[Module Manager documentation](../../../../VOS2_WORKSPACE/modules/v2_modulemanager_733256b82d6b48a48bc52b5ec73ebfff/docs/)
for details on the verification flow.

### Automatic Distribution

`vyra_skills.meta.json` ships with the `vyra_base` Python package and is
automatically copied into every module's interface config directory by
`setup_interfaces.py` during the module build. No manual configuration is required.

---

## See Also

- [`src/vyra_base/com/transport/README.md`](../com/transport/README.md) — Zenoh and other transport details
- [`src/vyra_base/core/README.md`](../core/README.md) — VyraEntity and InterfaceBuilder
- [`src/vyra_base/security/README.md`](../security/README.md) — Access levels and security framework

        "type": "callable",
