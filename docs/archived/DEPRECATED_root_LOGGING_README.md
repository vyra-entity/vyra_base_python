# VYRA Base Python Logging

## Overview

vyra_base uses the standard Python logging module with a centralized configuration system. This provides consistent, professional logging across all modules and components.

## Quick Start

### Basic Usage

```python
import logging

# Get a logger for your module
logger = logging.getLogger(__name__)

# Log messages
logger.debug("Detailed information for debugging")
logger.info("General information")
logger.warning("Warning message")
logger.error("Error occurred")
logger.critical("Critical problem")
```

### VyraEntity Integration

Logging is automatically initialized when creating a VyraEntity:

```python
from vyra_base.core import VyraEntity

# Initialize with default logging configuration
entity = VyraEntity(
    module_entry=module_entry,
    # Logging is initialized automatically
)

# Or customize logging configuration
entity = VyraEntity(
    module_entry=module_entry,
    log_config={
        "log_level": "DEBUG",
        "log_directory": "log/my_module",
        "enable_console": True,
        "enable_file": True
    }
)
```

### Manual Initialization

If you need to initialize logging outside of VyraEntity:

```python
from vyra_base.helper.logging_config import VyraLoggingConfig
from pathlib import Path

# Initialize with defaults
VyraLoggingConfig.initialize()

# Or with custom configuration
VyraLoggingConfig.initialize(
    log_level="INFO",
    log_directory=Path("log/custom"),
    enable_console=True,
    enable_file=True
)
```

## Configuration

### Log Levels

Available log levels (from most verbose to least):
- `DEBUG`: Detailed information for diagnosing problems
- `INFO`: General informational messages
- `WARNING`: Warning messages (default)
- `ERROR`: Error messages
- `CRITICAL`: Critical problems

### Environment Variables

Configure logging via environment variables:

```bash
# Set log level
export VYRA_LOG_LEVEL=DEBUG

# Application will use DEBUG level for all loggers
```

### Configuration Parameters

When initializing logging via VyraEntity or VyraLoggingConfig:

```python
log_config = {
    # Log level (DEBUG, INFO, WARNING, ERROR, CRITICAL)
    "log_level": "INFO",
    
    # Directory for log files (str or Path)
    "log_directory": "log/vyra",
    
    # Enable console output
    "enable_console": True,
    
    # Enable file output
    "enable_file": True,
}
```

## Advanced Usage

### Runtime Log Level Changes

```python
from vyra_base.helper.logging_config import VyraLoggingConfig

# Change log level for specific logger
VyraLoggingConfig.set_level("DEBUG", logger_name="vyra_base.core")

# Change log level for all vyra_base loggers
VyraLoggingConfig.set_level("WARNING", logger_name="vyra_base")
```

### Temporary Log Level Changes

Use the context manager for temporary log level changes:

```python
from vyra_base.helper.logging_config import get_logger, temporary_log_level

logger = get_logger(__name__)

# Temporarily enable DEBUG logging
with temporary_log_level("DEBUG"):
    logger.debug("This will be logged")
    perform_detailed_operation()
# Original level restored automatically
```

### Performance Logging

Use the `@log_performance` decorator to track function execution time:

```python
from vyra_base.helper.logging_config import log_performance

@log_performance(threshold_ms=100)
def process_data(items):
    """Function that may take time."""
    for item in items:
        # process item
        pass
    return len(items)

# Logs warning if execution exceeds 100ms:
# ⚠️ process_data took 152.3ms (threshold: 100ms)
```

### Exception Logging

Use the `@log_exceptions` decorator for automatic exception logging:

```python
from vyra_base.helper.logging_config import log_exceptions

@log_exceptions()
def risky_operation():
    """Function that may raise exceptions."""
    result = 10 / 0  # Will log exception with full traceback
    return result

# Automatically logs:
# ❌ Exception in risky_operation: division by zero
# [Full traceback included]
```

### JSON Logging

Enable JSON-formatted logs for machine parsing:

```python
from vyra_base.helper.logging_config import VyraLoggingConfig

VyraLoggingConfig.initialize(
    enable_json=True,  # File logs will be JSON formatted
    enable_file=True
)

# Produces logs like:
# {"timestamp": "2024-02-16T10:30:45.123456", "level": "INFO", 
#  "logger": "vyra_base.core", "message": "Operation complete", 
#  "module": "entity", "function": "initialize", "line": 123}
```

### Colored Console Output

Enable colored console output for better readability:

```python
from vyra_base.helper.logging_config import VyraLoggingConfig

VyraLoggingConfig.initialize(
    enable_colors=True,  # Console output will be colored
    enable_console=True
)

# DEBUG messages appear in cyan
# INFO messages appear in green
# WARNING messages appear in yellow
# ERROR messages appear in red
# CRITICAL messages appear in magenta
```

### Rate Limiting

Prevent log spam with rate limiting:

```python
from vyra_base.helper.logging_config import VyraLoggingConfig

VyraLoggingConfig.initialize(
    rate_limit=1.0  # Suppress duplicate messages within 1 second
)

# Multiple identical messages within 1 second will be suppressed
for i in range(1000):
    logger.warning("Connection timeout")  # Only logged once per second
```

### Hierarchical Loggers

Use hierarchical logger names for better organization:

```python
import logging

# Parent logger for all vyra_base modules
logger_base = logging.getLogger("vyra_base")

# Specific module loggers (inherit from parent)
logger_core = logging.getLogger("vyra_base.core")
logger_com = logging.getLogger("vyra_base.com.transport")
logger_storage = logging.getLogger("vyra_base.storage.redis")

# All logs from child loggers will be handled by parent handlers
```

### Custom Handlers

Add custom handlers for specialized logging:

```python
import logging
from vyra_base.helper.logging_config import VyraLoggingConfig

# Create a custom handler (e.g., for network logging)
import logging.handlers
syslog_handler = logging.handlers.SysLogHandler(address=('localhost', 514))
syslog_handler.setFormatter(
    logging.Formatter('%(name)s: %(levelname)s - %(message)s')
)

# Add to specific logger
VyraLoggingConfig.add_handler_to_logger("vyra_base", syslog_handler)
```

### Structured Logging

Use structured log messages for better parsing:

```python
import logging

logger = logging.getLogger(__name__)

# Good: Structured message with context
logger.info(
    "Processing complete",
    extra={
        "duration_ms": 1234,
        "items_processed": 100,
        "status": "success"
    }
)

# Better: Use f-strings for simple messages
logger.info(f"Processed {count} items in {duration}ms")
```

## Best Practices

### 1. Use Module-Level Loggers

```python
import logging

# Always use __name__ to create hierarchical loggers
logger = logging.getLogger(__name__)
```

### 2. Log at Appropriate Levels

```python
# DEBUG: Detailed information for debugging
logger.debug(f"Variable state: {variable_state}")

# INFO: Confirmation that things are working as expected
logger.info("✅ Service started successfully")

# WARNING: Something unexpected, but application continues
logger.warning("⚠️ Configuration file not found, using defaults")

# ERROR: Error occurred, but application continues
logger.error("❌ Failed to connect to database")

# CRITICAL: Serious error, application may stop
logger.critical("💀 Critical system failure")
```

### 3. Use Emojis for Quick Scanning

```python
logger.info("✅ Operation successful")
logger.warning("⚠️ Potential issue detected")
logger.error("❌ Operation failed")
logger.debug("🔍 Debug information")
logger.info("📡 Network communication")
logger.info("💾 Database operation")
```

### 4. Include Context

```python
# Good: Include relevant context
logger.error(f"Failed to process file: {filename}, error: {error}")

# Bad: Vague message
logger.error("Processing failed")
```

### 5. Use Exception Logging

```python
try:
    risky_operation()
except Exception as e:
    # Log with full traceback
    logger.exception("Operation failed")
    
    # Or with custom message
    logger.error(f"Failed to process {item}", exc_info=True)
```

## Log File Management

### Default Configuration

- **Location**: `log/vyra/vyra_base.log`
- **Rotation**: Automatic rotation at 10MB
- **Backup Count**: 10 files kept
- **Format**: `TIMESTAMP - LOGGER_NAME - LEVEL - [FILE:LINE] - MESSAGE`

### Custom Log Directory

```python
from pathlib import Path
from vyra_base.helper.logging_config import VyraLoggingConfig

VyraLoggingConfig.initialize(
    log_directory=Path("/var/log/my_application")
)
```

## Performance Considerations

### Lazy Formatting

```python
# Good: String formatting only happens if message is logged
logger.debug("Processing item %s with value %s", item_id, item_value)

# Less efficient: String formatted even if debug is disabled
logger.debug(f"Processing item {item_id} with value {item_value}")
```

### Conditional Logging

```python
# For expensive operations, check level first
if logger.isEnabledFor(logging.DEBUG):
    expensive_debug_info = gather_debug_data()
    logger.debug(f"Debug info: {expensive_debug_info}")
```

## Troubleshooting

### Logs Not Appearing

```python
import logging

# Check logger level
logger = logging.getLogger(__name__)
print(f"Logger level: {logger.level}")
print(f"Effective level: {logger.getEffectiveLevel()}")

# Check handlers
print(f"Handlers: {logger.handlers}")

# Ensure logging is initialized
from vyra_base.helper.logging_config import VyraLoggingConfig
VyraLoggingConfig.initialize()
```

### Duplicate Log Messages

```python
# Disable propagation to parent logger
logger = logging.getLogger(__name__)
logger.propagate = False
```

### Reset Configuration

```python
# For testing, reset logging configuration
from vyra_base.helper.logging_config import VyraLoggingConfig
VyraLoggingConfig.reset()
```

## Migration from Old Logger

If you're migrating from the old custom logger, see [LOGGING_MIGRATION.md](LOGGING_MIGRATION.md) for detailed instructions.

Quick changes:
- Replace `from vyra_base.helper.logger import logger` with `import logging; logger = logging.getLogger(__name__)`
- Replace `logger.log(LogEntry("msg").error())` with `logger.error("msg")`
- Replace `logger.warn()` with `logger.warning()`

## Complete Example: Professional Logging Setup

```python
from vyra_base.helper.logging_config import (
    VyraLoggingConfig,
    get_logger,
    log_performance,
    log_exceptions,
    temporary_log_level
)
import time

# Initialize with all professional features
VyraLoggingConfig.initialize(
    log_level="INFO",
    enable_console=True,
    enable_file=True,
    enable_colors=True,      # Colored console output
    enable_json=True,        # JSON formatted file logs
    rate_limit=1.0           # Suppress duplicates within 1 second
)

# Get logger for your module
logger = get_logger(__name__)

# Basic logging
logger.info("✅ Application started")
logger.warning("⚠️ Using default configuration")

# Performance monitoring
@log_performance(threshold_ms=50)
def process_batch(items):
    """Process items with performance monitoring."""
    time.sleep(0.1)  # Simulated work
    return len(items)

# Exception handling
@log_exceptions()
def risky_operation(value):
    """Operation that may fail."""
    return 10 / value  # May raise ZeroDivisionError

# Temporary debug mode
with temporary_log_level("DEBUG"):
    logger.debug("🔍 Detailed debugging information")
    result = process_batch([1, 2, 3])

# Structured logging with extra fields
logger.info(
    "Processing complete",
    extra={
        "items_processed": result,
        "duration_ms": 100,
        "status": "success"
    }
)

# Exception logging (automatically handled by decorator)
try:
    risky_operation(0)
except ZeroDivisionError:
    logger.error("❌ Operation failed, continuing with fallback")

logger.info("🎉 Application finished")
```

## Examples

See complete examples in:
- `examples/logging_basic.py` - Basic logging setup
- `examples/logging_advanced.py` - Advanced features
- `examples/logging_with_entity.py` - VyraEntity integration

## References

- [Python Logging Documentation](https://docs.python.org/3/library/logging.html)
- [Python Logging Cookbook](https://docs.python.org/3/howto/logging-cookbook.html)
- [vyra_base.helper.logging_config](../src/vyra_base/helper/logging_config.py) module
