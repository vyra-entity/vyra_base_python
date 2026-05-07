# VYRA Base Python - Examples

Practical, runnable examples for `vyra_base`.

## Structure

| Directory | Description |
|-----------|-------------|
| `01_interfaces/` | Canonical interface examples split into `implementation/` and `configuration/` |
| `02_state_machine/` | Core unified/operational state-machine walkthroughs |
| `03_feeder/` | Feeder registry, monitor decorators, conditions, execution points |
| `04_external_communication/` | Full external protocol examples (TCP/UDP, gRPC, MQTT, REST, WebSocket, Shared Memory, Modbus, OPC UA, Registry) |
| `05_decorator_blueprints/` | Two-phase decorator/blueprint pattern |
| `06_helper/` | Helper-function usage matrix and file I/O examples |
| `07_format_logging/` | Logging configuration and runtime logging patterns |
| `08_parameter_volatile/` | Parameter and volatile usage patterns |
| `09_statemachine/` | Unified callbacks plus extended state-machine examples |
| `10_defaults_entries/` | `vyra_base.defaults.entries` dataclasses/enums and serialization |
| `11_security/` | Security-level matrix + standalone security framework examples |
| `12_plugin/` | WASM plugin runtime (`WasmRuntime` / `StubRuntime`) |
| `13_skills/` | Skill instances via SkillManager and Zenoh |

## Quick Start

```bash
cd /path/to/vyra_base_python

# Interfaces
python examples/01_interfaces/implementation/service_server.py
python examples/01_interfaces/configuration/decorators_and_set_interfaces.py

# Runtime patterns
python examples/03_feeder/01_basic_feeder.py
python examples/07_format_logging/format_logging_usage.py
python examples/08_parameter_volatile/parameter_volatile_patterns.py
python examples/09_statemachine/unified_state_callbacks.py

# Security and plugins
python examples/11_security/security_level_matrix.py
python examples/12_plugin/01_basic_plugin.py
python examples/11_security/secure_server_node.py
```

## Notes

- `01_interfaces/configuration/json_examples/` contains ready-to-use `*.meta.json` samples for module interface generation.
- `01_interfaces/configuration/decorators_and_set_interfaces.py` demonstrates callback decoration for message/service/action and loading settings via `set_interfaces(...)`.
- `06_helper/README.md` contains the helper coverage matrix and script mapping.
- `09_statemachine/` now includes callback-focused and broader state-machine examples.

