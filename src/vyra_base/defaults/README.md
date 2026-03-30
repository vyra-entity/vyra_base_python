# vyra_base.defaults

Default constants, data entry types, exceptions, and author information used throughout the VYRA framework.

## Public API

```python
from vyra_base.defaults import (
    # Constants
    FeederConstants,
    LoggerConstants,
    TimeoutConstants,
    MessageLengthConstants,
    CyclicRefreshConstants,
    RequiredVersion,
    # Entries
    ErrorEntry,
    NewsEntry,
    StateEntry,
    PullRequestEntry,
    AvailableModuleEntry,
    # Exceptions
    FeederException,
    # Module info
    AuthorInfo,
)
```

---

## Constants

`constants.py` contains frozen dataclass constants used across the framework:

| Class | Purpose |
|---|---|
| `FeederConstants` | Maximum feed lengths (news, error, state, pull-requests) |
| `LoggerConstants` | Logger queue capacity |
| `TimeoutConstants` | Connect / send / receive / closed timeouts (seconds) |
| `MessageLengthConstants` | Max message byte sizes for UDP/TCP |
| `CyclicRefreshConstants` | Refresh interval defaults |
| `RequiredVersion` | Minimum Python version `(3, 9, 0)` |
| `MODULE_ID_NAMESPACE` | UUID v5 namespace for stable module-ID generation |

---

## Entries

`entries.py` defines structured data classes used as payloads in feeders and communication interfaces.

> **Note (since 0.1.8+build.128):** `ErrorEntry`, `NewsEntry`, `StateEntry`, and `PullRequestEntry`
> no longer contain a `_type` field. Message types are resolved internally by the interface layer.

### Core Entry Types

```python
from vyra_base.defaults.entries import ErrorEntry, NewsEntry, StateEntry

error = ErrorEntry(
    error_id="err-001",
    message="Sensor timeout",
    severity=2,
    source="my_module",
)

news = NewsEntry(
    message="Calibration complete",
    source="my_module",
)

state = StateEntry(
    state="RUNNING",
    source="my_module",
)
```

### FunctionConfig Types

Used for describing callable interfaces:

| Class | Purpose |
|---|---|
| `FunctionConfigEntry` | Full descriptor for a callable (name, params, tags) |
| `FunctionConfigParamTypes` | Enum of allowed parameter types (bool, int, float, string, ...) |
| `FunctionConfigBaseTypes` | Base type mapping for service/action interfaces |
| `FunctionConfigTags` | Tag constants for interface metadata |
| `ModuleEntry` | Module registration entry (name, id, version) |
| `AvailableModuleEntry` | Subset of `ModuleEntry` published to the module registry |

---

## Exceptions

```python
from vyra_base.defaults import FeederException

raise FeederException("Feeder handler not initialized")
```

---

## AuthorInfo

```python
from vyra_base.defaults import AuthorInfo

print(AuthorInfo.COMPANY)   # Variobotic GmbH
print(AuthorInfo.EMAIL)
```

---

## Files

| File | Description |
|---|---|
| `constants.py` | Frozen dataclass constants |
| `entries.py` | Data entry dataclasses and enums |
| `exceptions.py` | `FeederException` |
| `info.py` | `AuthorInfo` |
