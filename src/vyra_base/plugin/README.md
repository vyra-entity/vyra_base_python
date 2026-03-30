# vyra_base.plugin

WASM-based plugin runtime for the VYRA framework. Enables loading and executing
sandboxed WASM plugins with controlled host-function access.

## Public API

```python
from vyra_base.plugin import (
    PluginRuntime,       # Abstract base class
    WasmRuntime,         # Real WASM executor (requires wasmtime)
    StubRuntime,         # Python stub — no WASM needed (tests / dev)
    PluginCallError,     # Exception for failed plugin calls
    create_plugin_runtime,  # Factory — auto-selects best runtime
    HostFunctions,          # Protocol for host functions (structural typing)
    BaseHostFunctions,      # Abstract base for module implementations
    NullHostFunctions,      # No-op implementation for tests
)
```

---

## Quickstart

```python
from vyra_base.plugin import create_plugin_runtime, NullHostFunctions

# Automatically picks WasmRuntime if wasmtime is installed, else StubRuntime
runtime = create_plugin_runtime(
    plugin_id="my-plugin",
    wasm_path="/opt/vyra/plugins/my-plugin/1.0.0/logic.wasm",
    host=NullHostFunctions(),
    initial_state={"initial_value": 0, "step": 1},
)

await runtime.start()
result = await runtime.call("increment", {"step": 2})
# → {"result": 2}
```

---

## Plugin Metadata Format

Every WASM plugin ships a `metadata.json` describing its exports:

```json
{
    "exports": [
        {
            "name": "init",
            "args": [
                {"name": "initial_value", "type": "i32"},
                {"name": "step",          "type": "i32"}
            ]
        },
        {
            "name": "increment",
            "args": [{"name": "step", "type": "i32"}]
        }
    ]
}
```

The runtime maps dict keys **in the order defined in `args`** to WASM i32 parameters.

---

## PluginRuntime (Abstract Base)

`PluginRuntime` defines the interface all runtimes must implement:

```python
await runtime.start()                         # Initialize the runtime
result = await runtime.call("fn", {"a": 1})   # Call an exported function
await runtime.stop()                          # Teardown
state  = await runtime.get_state()           # Get current plugin state
```

---

## WasmRuntime

Executes real WASM files using `wasmtime`. Only available if `wasmtime` is installed:

```bash
pip install wasmtime
```

`WasmRuntime` resolves host functions by injecting the `HostFunctions` implementation
into the WASM instance's import namespace. The plugin can only interact with the host
through these explicitly granted functions.

---

## StubRuntime

A pure-Python stub that does not execute WASM. Used for:
- Unit tests (no WASM toolchain required)
- Development without compiled plugin artifacts
- Connectivity checks

```python
from vyra_base.plugin import StubRuntime, NullHostFunctions

runtime = StubRuntime(
    plugin_id="stub-test",
    host=NullHostFunctions(),
    initial_state={},
)
await runtime.start()
result = await runtime.call("any_fn", {})
# → {"stub": True, "fn": "any_fn"}
```

---

## HostFunctions

Host functions are the **only** way a WASM plugin can interact with the outside world
(logging, publishing events, calling services). Direct filesystem or network access
from inside a WASM sandbox is blocked.

### Implementing Host Functions in a Module

```python
from vyra_base.plugin import BaseHostFunctions
from vyra_base.com.core.factory import InterfaceFactory

class MyModuleHostFunctions(BaseHostFunctions):
    def __init__(self, event_publisher):
        super().__init__()
        self._publisher = event_publisher

    async def notify_ui(self, event_name: str, data: dict) -> None:
        await self._publisher.publish({"event": event_name, "data": data})

    async def create_publisher(self, name, module_name, module_id=None, **kwargs):
        return await InterfaceFactory.create_publisher(
            name, module_id=module_id, module_name=module_name, **kwargs
        )

    async def create_client(self, name, module_name, module_id=None, **kwargs):
        return await InterfaceFactory.create_client(
            name, module_id=module_id, module_name=module_name, **kwargs
        )
```

### NullHostFunctions

A no-op implementation for testing — all methods are implemented as silent no-ops:

```python
from vyra_base.plugin import NullHostFunctions

host = NullHostFunctions()
```

---

## PluginFacade (`plugin_facade.py`)

`PluginFacade` sits above the runtime and enforces permissions defined in `metadata.json`.
It prevents plugins from accessing functions not explicitly listed in their exports.
Only functions declared in the metadata are forwarded to the runtime.

---

## Files

| File | Description |
|---|---|
| `runtime.py` | `PluginRuntime`, `WasmRuntime`, `StubRuntime`, `PluginCallError`, `create_plugin_runtime` |
| `host_functions.py` | `HostFunctions`, `BaseHostFunctions`, `NullHostFunctions` |
| `plugin_facade.py` | `PluginFacade` — permission enforcement layer |
