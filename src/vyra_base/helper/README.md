# vyra_base.helper

Utility functions and helpers used throughout the VYRA framework.

## Public API

```python
from vyra_base.helper import (
    # Logging
    VyraLoggingConfig,
    get_logger,
    # ROS2 environment
    update_ament_prefix_path,
    update_python_path,
    ensure_interface_package_discoverable,
    ensure_workspace_discoverable,
    # Utilities
    deep_merge,
    fuzzy_match,
)
```

---

## Logging

### `VyraLoggingConfig`

Configures Python logging for VYRA modules. Loads from `logger_config.json`.

```python
from vyra_base.helper import VyraLoggingConfig, get_logger

VyraLoggingConfig.configure()
logger = get_logger("my_module")
logger.info("Module started")
```

### `VyraLogFilter` (`log_filter.py`)

Python `logging.Filter` that optionally suppresses noisy log records by pattern.

---

## File I/O

### `FileReader` (`file_reader.py`)

Async and sync file reading with caching and JSON support:

```python
from vyra_base.helper.file_reader import FileReader

reader = FileReader("/workspace/config/settings.json")
data = await reader.read_json()           # async
data = reader.read_json_sync()            # sync
raw  = await reader.read_text()
```

### `FileWriter` (`file_writer.py`)

Async and sync file writing:

```python
from vyra_base.helper.file_writer import FileWriter

writer = FileWriter("/workspace/config/settings.json")
await writer.write_json({"key": "value"})
await writer.write_text("raw content")
```

### `FileLock` (`file_lock.py`)

Cross-process file locking to avoid concurrent writes:

```python
from vyra_base.helper.file_lock import FileLock

async with FileLock("/workspace/storage/data.db"):
    # exclusive access
    ...
```

---

## Security / Crypto

### `CryptoHelper` (`crypto_helper.py`)

HMAC generation and verification utilities used by the security framework:

```python
from vyra_base.helper.crypto_helper import CryptoHelper

signature = CryptoHelper.sign(payload="data", key="secret")
valid = CryptoHelper.verify(payload="data", key="secret", signature=signature)
```

---

## Error Handling

### `ErrorTraceback` (`error_handler.py`)

Structured exception formatting for VYRA log entries:

```python
from vyra_base.helper.error_handler import ErrorTraceback

try:
    risky_operation()
except Exception as e:
    tb = ErrorTraceback.format(e)
    logger.error(tb)
```

---

## ROS2 Environment

These helpers are only relevant when running inside a ROS2 context. They manipulate
`AMENT_PREFIX_PATH` and `PYTHONPATH` to make generated interface packages discoverable:

```python
from vyra_base.helper import (
    update_ament_prefix_path,
    ensure_interface_package_discoverable,
    ensure_workspace_discoverable,
)

# Make a colcon workspace discoverable for rclpy
ensure_workspace_discoverable("/workspace/install")

# Make a specific interface package available
ensure_interface_package_discoverable("my_module_interfaces")
```

---

## Utility Functions

### `deep_merge` (`func.py`)

Recursively merges two dicts. Right-hand values win on conflicts; nested dicts are merged:

```python
from vyra_base.helper import deep_merge

base   = {"a": 1, "b": {"x": 1, "y": 2}}
patch  = {"b": {"y": 99, "z": 3}, "c": 4}
result = deep_merge(base, patch)
# {"a": 1, "b": {"x": 1, "y": 99, "z": 3}, "c": 4}
```

### `fuzzy_match` (`func.py`)

Returns the closest string matches from a list of candidates — useful for diagnostic
suggestions (e.g. suggesting valid interface names after a lookup miss):

```python
from vyra_base.helper import fuzzy_match

suggestions = fuzzy_match("red_sencor", ["read_sensor", "set_param", "reset"])
# ["read_sensor"]
```

---

## `EnvHandler` (`env_handler.py`)

Reads and validates environment variables with type coercion:

```python
from vyra_base.helper.env_handler import EnvHandler

debug = EnvHandler.get_bool("VYRA_DEBUG", default=False)
port  = EnvHandler.get_int("APP_PORT", default=8443)
```

---

## Files

| File | Description |
|---|---|
| `logging_config.py` | `VyraLoggingConfig`, `get_logger` |
| `log_filter.py` | `VyraLogFilter` |
| `file_reader.py` | `FileReader` — async/sync JSON + text |
| `file_writer.py` | `FileWriter` — async/sync writes |
| `file_lock.py` | `FileLock` — cross-process file locking |
| `crypto_helper.py` | `CryptoHelper` — HMAC sign/verify |
| `error_handler.py` | `ErrorTraceback` — exception formatting |
| `env_handler.py` | `EnvHandler` — env var access |
| `func.py` | `deep_merge`, `fuzzy_match` |
| `ros2_env_helper.py` | ROS2 path helpers |
| `_aiopath.py` | Internal async path utilities |
| `logger_config.json` | Default logging configuration |
