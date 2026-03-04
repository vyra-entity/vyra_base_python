# VYRA Base Python — Examples

Practical, runnable examples for `vyra_base`.

## Structure

- `01_service/` — request/response services (`@remote_service`)
- `02_publisher_subscriber/` — pub/sub messaging
- `03_action_server/` — long-running actions with feedback
- `04_protocols/` — protocol-specific examples (Zenoh/Redis/UDS/etc.)
- `05_state_machine/` — operational + unified state-machine patterns
- `06_feeder/` — feeder registry, monitor decorators, conditions, execution points
- `07_tcp_udp/` — TCP/UDP communication samples
- `08_decorator_blueprints/` — two-phase decorator/blueprint examples (moved from `docs/examples`)
- `09_helper_file_io/` — async/sync FileReader/FileWriter usage
- `10_state_callbacks/` — UnifiedStateMachine callbacks + transition diagnostics
- `11_defaults_entries/` — `vyra_base.defaults.entries` dataclasses/enums and serialization patterns
- `12_security_levels/` — security-level matrix, algorithm mapping and validation helpers
- `storage_example.py` — SQLAlchemy + Redis storage example
- `DEPRECATED/` — legacy examples (reference only)

## Quick Start

```bash
cd /path/to/vyra_base_python

python examples/01_service/service_server.py
python examples/06_feeder/01_basic_feeder.py
python examples/08_decorator_blueprints/example_basic_service.py
python examples/09_helper_file_io/file_io_async_sync.py
python examples/10_state_callbacks/unified_state_callbacks.py
python examples/11_defaults_entries/entries_showcase.py
python examples/12_security_levels/security_level_matrix.py
```

## Notes

- Some examples require optional runtime dependencies (e.g. Redis, Zenoh).
- `08_decorator_blueprints/` demonstrates the two-phase pattern:
  1. blueprint definition via decorators
  2. callback binding during component initialization
- Prefer these examples over `DEPRECATED/` for new development.
- Security-focused examples are located in `examples/security/` and referenced from `docs/security/examples.rst`.
