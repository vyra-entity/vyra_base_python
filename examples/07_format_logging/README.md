# Example 08 - format_logging

This example shows how logging should be initialized and used in a VYRA module.

## Covered patterns

- Startup configuration via `VyraLoggingConfig.setup(...)`
- Logger retrieval via `get_logger(...)`
- Temporary log level switch with `temporary_log_level(...)`
- Runtime performance logging with `@log_performance(...)`
- Exception logging with `@log_exceptions(...)`

## Run

```bash
python examples/08_format_logging/format_logging_usage.py
```

## Integration in modules

Use this pattern at module startup (for example in `main.py` or entity initialization):

```python
from vyra_base.helper.logging_config import VyraLoggingConfig

VyraLoggingConfig.setup(
    level="INFO",
    module_name="my_module",
    structured=True,
)
```

Then use `get_logger("my_module")` in your components and services.
