# Plugin Runtime Examples

This directory demonstrates VYRA's WASM plugin system.

## Requirements

```bash
pip install vyra_base             # Always available (StubRuntime fallback)
pip install wasmtime              # Optional — enables WasmRuntime
```

## Examples

| File | Description |
|------|-------------|
| `01_basic_plugin.py` | Auto-select WasmRuntime vs StubRuntime, call exported functions, host callbacks |

## How it Works

`create_plugin_runtime()` auto-selects the backend:

```python
from vyra_base.plugin.runtime import create_plugin_runtime

runtime = create_plugin_runtime(
    plugin_id="my-plugin",
    wasm_path="/path/to/logic.wasm",
    initial_state={"counter": 0},
)
await runtime.start()
result = await runtime.call("increment", {"step": 1})
```

| Condition | Backend |
|-----------|---------|
| `wasmtime` installed **and** `.wasm` file exists | `WasmRuntime` |
| `wasmtime` missing **or** `.wasm` not found | `StubRuntime` |

## Plugin File Layout

```
my_plugin/
├── logic.wasm          # Compiled WASM module (from any language)
└── metadata.json       # Plugin manifest
```

### `metadata.json` format

```json
{
    "id": "my-plugin",
    "version": "1.0.0",
    "description": "Example counter plugin",
    "exports": ["increment", "reset", "get_value"],
    "imports": ["host_log", "host_emit_event"],
    "initial_state": {
        "counter": {"type": "int", "default": 0},
        "step":    {"type": "int", "default": 1}
    }
}
```

## Host Functions

Host functions are Python callbacks that the WASM plugin can call back into.
Subclass `BaseHostFunctions` and prefix method names with `host_`:

```python
from vyra_base.plugin.host_functions import BaseHostFunctions

class MyHostFunctions(BaseHostFunctions):
    def host_log(self, message: str) -> None:
        print(f"[plugin] {message}")

    def host_emit_event(self, event_type: str, payload: str) -> None:
        print(f"[event:{event_type}] {payload}")
```

Use `NullHostFunctions` when no callbacks are needed.

## See Also

- [`src/vyra_base/plugin/README.md`](../../src/vyra_base/plugin/README.md) — Full API reference
