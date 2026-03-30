# vyra_base_python Test Suite

Unit and integration tests for the `vyra_base_python` library.

## Test Structure

```
tests/
├── conftest.py                      # Global fixtures & ROS2 mock setup
├── com/                             # Communication module tests
├── plugin/
│   └── test_plugin_runtime.py       # StubRuntime, create_plugin_runtime, PluginCallError
├── security/
│   └── test_security_levels.py      # SecurityLevel enum, is_valid, get_name, ordering
├── transport/                       # Transport layer tests (zenoh/ ignored by default)
├── test_defaults.py                 # Default entries, exceptions, configurations
├── test_file_operations.py          # File I/O helpers
├── test_helper.py                   # Error handling, async utilities
├── test_interfaces.py               # Interface config, protocol contracts
├── test_multicallback_validation.py # Multi-callback validation logic
├── test_operational_metaclass.py    # OperationalMeta metaclass
├── test_state_events.py             # StateEvent and EventType
├── test_state_machine.py            # StateMachine core engine
├── test_state_types.py              # LifecycleState, OperationalState, HealthState
└── test_storage.py                  # Storage interfaces
```

## Running Tests

```bash
# All tests (from repo root)
pytest tests/

# Individual modules
pytest tests/plugin/
pytest tests/security/

# With coverage (configured in pytest.ini)
pytest tests/
```

### Pytest markers

```bash
pytest tests/ -m asyncio       # async tests only
pytest tests/ -m "not slow"    # skip slow tests
```

## Coverage

`pytest.ini` collects coverage for `vyra_base.state`, `vyra_base.helper`,
`vyra_base.plugin`, and `vyra_base.security`. Reports are written to
`htmlcov/` and printed to the terminal after every run.

`tests/transport/zenoh` is excluded from the default run because it
requires a live Zenoh session. Run it manually when a Zenoh broker is
available:

```bash
pytest tests/transport/zenoh/
```

## External Dependencies

All tests in `tests/plugin/` and `tests/security/` are pure-Python and
require no external services.  `conftest.py` provides ROS2 mocks so the
full suite can run without a ROS2 installation.

