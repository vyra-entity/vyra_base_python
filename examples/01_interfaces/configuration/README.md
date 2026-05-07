# Example 01/interfaces/configuration

This folder provides complete configuration examples for module interfaces.

## Included artifacts

- `json_examples/module_interfaces.meta.json`
  - message, service, and action definitions in one file.
- `decorators_and_set_interfaces.py`
  - callback decoration examples for message/service/action.
  - full `set_interfaces(...)` loading pattern.

## How to use in a module (`setup_interfaces.py` flow)

1. Copy your JSON into your module interface config folder, for example:

```text
src/my_module_interfaces/config/my_module.meta.json
```

2. Run the module setup script from module root:

```bash
python3 tools/setup_interfaces.py
```

3. The setup step validates the JSON against the interface schema and then generates `.proto` files.

## Schema validation recommendation

Always validate your JSON before deployment in CI/local checks.

Example command pattern:

```bash
python3 tools/setup_interfaces.py
```

If JSON does not match the expected schema (missing `type`, invalid `datatype`, reserved `functionname`, etc.), setup fails early.

## Runtime loading with `set_interfaces(...)`

At runtime, build `FunctionConfigEntry` objects and load them into your entity:

```python
settings = build_interface_settings(callbacks)
await entity.set_interfaces(settings)
```

The example `decorators_and_set_interfaces.py` covers:

- service callback (`@remote_service`)
- message callback (`@remote_publisher`)
- action callbacks (`@remote_actionServer.on_goal`, `.on_cancel`, `.execute`)
- loading all three via `set_interfaces(...)`
