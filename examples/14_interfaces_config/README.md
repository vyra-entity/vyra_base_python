# Interface Configuration Examples

This directory explains how to write `*.meta.json` interface config files for
a VYRA module.  No Python code is required — the module build pipeline
(`setup_interfaces.py`) reads these files and generates `.proto` stubs
automatically.

## Quickstart

1. Create `src/{module}_interfaces/config/my_module.meta.json`
2. Add your interface definitions (see examples below)
3. Run `python3 tools/setup_interfaces.py` in the module root

The generator merges built-in `vyra_base` interfaces with your custom entries
and produces the corresponding `.proto` files.

## Example: Complete Module Config

```json
[
    {
        "tags": ["zenoh"],
        "type": "service",
        "functionname": "get_device_status",
        "displayname": "Get Device Status",
        "description": "Retrieve the current operational status of the device.",
        "filetype": ["MyModuleGetDeviceStatus.proto"],
        "params": [],
        "returns": [
            {
                "name": "online",
                "datatype": "bool",
                "displayname": "Online",
                "description": "Whether the device is reachable."
            },
            {
                "name": "temperature",
                "datatype": "float",
                "displayname": "Temperature",
                "description": "Current device temperature in °C."
            }
        ],
        "access_level": 1,
        "displaystyle": {
            "visible": true,
            "published": true,
            "category": "Diagnostics",
            "order": 10
        }
    },
    {
        "tags": ["zenoh"],
        "type": "service",
        "functionname": "set_target_temperature",
        "displayname": "Set Target Temperature",
        "description": "Set the desired operating temperature.",
        "filetype": ["MyModuleSetTargetTemperature.proto"],
        "params": [
            {
                "name": "target",
                "datatype": "float",
                "displayname": "Target",
                "description": "Desired temperature in °C.",
                "required": true,
                "validation": {"min": -40, "max": 150}
            }
        ],
        "returns": [
            {
                "name": "success",
                "datatype": "bool",
                "displayname": "Success",
                "description": "Whether the value was accepted."
            },
            {
                "name": "message",
                "datatype": "string",
                "displayname": "Message",
                "description": "Human-readable result message."
            }
        ],
        "access_level": 3,
        "displaystyle": {
            "visible": true,
            "published": true,
            "category": "Control",
            "order": 20
        }
    },
    {
        "tags": ["zenoh"],
        "type": "message",
        "functionname": "temperature_feed",
        "displayname": "Temperature Feed",
        "description": "Streaming temperature measurements (Speaker).",
        "filetype": ["MyModuleTemperatureFeed.proto"],
        "params": [
            {
                "name": "value",
                "datatype": "float",
                "displayname": "Value",
                "description": "Current temperature reading."
            },
            {
                "name": "timestamp",
                "datatype": "int64",
                "displayname": "Timestamp",
                "description": "Unix epoch milliseconds."
            }
        ],
        "returns": [],
        "access_level": 1,
        "displaystyle": {
            "visible": true,
            "published": true,
            "category": "Telemetry",
            "order": 30
        }
    }
]
```

## Field Reference

| Field | Required | Values | Notes |
|-------|----------|--------|-------|
| `tags` | yes | `["zenoh"]` | Always use `["zenoh"]` for Zenoh transport |
| `type` | yes | `"service"` / `"message"` / `"action"` | `service` = request/response; `message` = publisher |
| `functionname` | yes | snake_case | Becomes the Zenoh key suffix and proto service name |
| `displayname` | yes | Any string | Shown in the Dashboard UI |
| `description` | yes | Any string | Shown in API docs and tooltips |
| `filetype` | yes | `["VBASE*.proto"]` | Generated file names; use module prefix + TitleCase |
| `params` | no | array | Input parameters (for `service` / `action`) |
| `returns` | no | array | Output fields |
| `access_level` | no | 1–5 (default 1) | Min security level — see [security README](../../src/vyra_base/security/README.md) |
| `displaystyle.visible` | no | bool | Show in Dashboard (default `true`) |
| `displaystyle.published` | no | bool | Expose as public API (default `false`) |
| `displaystyle.category` | no | string | Dashboard category grouping |
| `displaystyle.order` | no | int | Sort order within category |

## Datatype Reference

| VYRA type | Protobuf type | Python type |
|-----------|---------------|-------------|
| `bool` | `bool` | `bool` |
| `int` / `int32` | `int32` | `int` |
| `int64` | `int64` | `int` |
| `float` / `float32` | `float` | `float` |
| `float64` / `double` | `double` | `float` |
| `string` | `string` | `str` |
| `string[]` | `repeated string` | `list[str]` |
| `int[]` | `repeated int32` | `list[int]` |
| `float[]` | `repeated float` | `list[float]` |
| `bool[]` | `repeated bool` | `list[bool]` |
| `object` | `google.protobuf.Struct` | `dict` |

## RESERVED Function Names

The following names are reserved by `vyra_base` and **cannot** be used in
module configs:

```
get_interface_list
set_parameter
get_parameter
request_access
health_check
get_state
set_state
```

Using a reserved name causes `setup_interfaces.py` to abort with a clear error
message.

## Access Levels

| Level | Name | Meaning |
|-------|------|---------|
| 1 | NONE | Public — no authentication required |
| 2 | BASIC_AUTH | Password required |
| 3 | EXTENDED_AUTH | ID + password authentication |
| 4 | HMAC | HMAC-signed request |
| 5 | DIGITAL_SIGNATURE | Certificate-signed request |

## See Also

- [`src/vyra_base/interfaces/README.md`](../../src/vyra_base/interfaces/README.md) — Interface system reference
- [`src/vyra_base/security/README.md`](../../src/vyra_base/security/README.md) — Access level details
