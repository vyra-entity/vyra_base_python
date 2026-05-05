# Logging Setup Guide

## Basic Configuration

```python
import logging

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

# Console handler
handler = logging.StreamHandler()
formatter = logging.Formatter(
    '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
handler.setFormatter(formatter)
logger.addHandler(handler)
```

## VYRA Logging

VYRA provides structured logging:

```python
from vyra_base.helper import Logger

logger = Logger("my_module")
logger.info("✅ Module started")
logger.error("❌ Error occurred")
logger.warning("⚠️  Warning")
```

## Levels

| Level | Use Case | Symbol |
|-------|----------|--------|
| DEBUG | Detailed info for debugging | 🔍 |
| INFO | General informational | ✅ |
| WARNING | Warning messages | ⚠️  |
| ERROR | Error messages | ❌ |
| CRITICAL | Critical failures | 🔴 |

## Configuration File

```ini
[logging]
level = DEBUG
format = %(asctime)s - %(name)s - %(levelname)s - %(message)s
file = vyra.log
```

## Best Practices

1. Use structured logging with context
2. Include relevant data (IDs, values, etc.)
3. Use appropriate log levels
4. Avoid logging sensitive data
5. Rotate logs to prevent disk bloat

