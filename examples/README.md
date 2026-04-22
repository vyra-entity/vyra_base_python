# VYRA Base Python — Examples

Practical, runnable examples for `vyra_base`.

## Structure

| Directory | Description |
|-----------|-------------|
| `01_service/` | Request/response services (`@remote_service`) |
| `02_publisher_subscriber/` | Pub/sub messaging |
| `03_action_server/` | Long-running actions with feedback |
| `04_protocols/` | Protocol-specific examples (Zenoh / Redis / UDS) |
| `05_state_machine/` | Operational + unified state-machine patterns |
| `06_feeder/` | Feeder registry, monitor decorators, conditions, execution points |
| `07_external_tcp_udp/` | External TCP/UDP communication samples |
| `08_decorator_blueprints/` | Two-phase decorator/blueprint pattern (recommended pattern) |
| `09_helper_file_io/` | Async/sync `FileReader`/`FileWriter` usage |
| `10_state_callbacks/` | `UnifiedStateMachine` callbacks and transition diagnostics |
| `11_defaults_entries/` | `vyra_base.defaults.entries` dataclasses/enums and serialisation |
| `12_security_levels/` | Security-level matrix, algorithm mapping and validation helpers |
| `13_plugin/` | WASM plugin runtime (`WasmRuntime` / `StubRuntime`) |
| `14_interfaces_config/` | Writing `*.meta.json` interface configuration files |
| `15_skills/` | Skill instances: add/query/verify via SkillManager and Zenoh |
| `security/` | Standalone security framework examples (no ROS2 / rclpy required) |

## Quick Start

```bash
cd /path/to/vyra_base_python

# Core patterns
python examples/01_service/service_server.py
python examples/06_feeder/01_basic_feeder.py
python examples/08_decorator_blueprints/example_basic_service.py

# Utilities
python examples/09_helper_file_io/file_io_async_sync.py
python examples/10_state_callbacks/unified_state_callbacks.py
python examples/11_defaults_entries/entries_showcase.py

# Security
python examples/12_security_levels/security_level_matrix.py
python examples/security/secure_server_node.py
python examples/security/secure_client_node.py

# Plugin runtime (falls back to StubRuntime without wasmtime)
python examples/13_plugin/01_basic_plugin.py
```

## Notes

- All examples run **without** a live Zenoh/Redis/ROS2 daemon — transports
  gracefully degrade to stubs when not available.
- `08_decorator_blueprints/` demonstrates the recommended two-phase pattern:
  1. Blueprint definition via decorators (at class definition time)
  2. Callback binding during component initialisation
- Security examples in `security/` use the standalone `SecurityManager` class —
  no `rclpy` or ROS2 installation required.
- `13_plugin/` auto-selects `WasmRuntime` (wasmtime installed + `.wasm` exists)
  or `StubRuntime` (pure Python fallback).
- Interface config documentation in `14_interfaces_config/` is reference-only
  (no Python file needed — the JSON format is the artefact).
- `15_skills/` shows both the Zenoh API and the internal `SkillManager` class.
  The `SkillManager` is part of `vyra_base.core` (Phase 1 implementation).

