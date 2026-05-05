# Transport Module Naming Convention

## Overview

VYRA Base Python transport modules use the `t_` prefix to avoid namespace conflicts with external libraries.

## Module Names

| External Library | VYRA Transport Module |
|------------------|----------------------|
| `eclipse-zenoh` (PyPI) | `vyra_base.com.transport.t_zenoh` |
| `redis` (PyPI) | `vyra_base.com.transport.t_redis` |
| `rclpy` (ROS2) | `vyra_base.com.transport.t_ros2` |
| (Unix sockets) | `vyra_base.com.transport.t_uds` |

## Rationale

### Problem
Python's import system prioritizes local modules over site-packages. If a local module has the same name as an external library (e.g., `zenoh`), importing the external library becomes problematic:

```python
# This would load vyra_base.com.transport.zenoh instead of eclipse-zenoh
import zenoh  # ❌ Wrong module!
```

### Solution
By using the `t_` prefix (for "transport"), we ensure that:

1. ✅ **No namespace conflicts**: `t_zenoh` ≠ `zenoh`
2. ✅ **Clean imports**: External libraries always load correctly
3. ✅ **Clear semantics**: `t_` indicates these are transport implementations
4. ✅ **Future-proof**: New transport modules follow the same pattern

## Usage Examples

### Import Transport Modules

```python
# Import transport providers
from vyra_base.com.transport.t_zenoh import ZenohProvider, ZenohSession
from vyra_base.com.transport.t_redis import RedisProvider, RedisClient
from vyra_base.com.transport.t_ros2 import ROS2Provider, VyraNode
from vyra_base.com.transport.t_uds import UDSProvider

# External libraries still work normally
import zenoh  # ✅ Eclipse Zenoh library
import redis  # ✅ Redis library
import rclpy  # ✅ ROS2 Python client
```

### Using InterfaceFactory

```python
from vyra_base.com import InterfaceFactory, ProtocolType

# Factory automatically uses correct transport modules
callable = await InterfaceFactory.create_callable(
    "service_name",
    protocols=[ProtocolType.ZENOH, ProtocolType.REDIS],
    callback=handle_request
)
```

### Direct Transport Usage

```python
# Zenoh Transport
from vyra_base.com.transport.t_zenoh import ZenohProvider

provider = ZenohProvider()
await provider.initialize(config={
    "mode": "client",
    "connect": ["tcp/zenoh-router:7447"]
})
callable = await provider.create_callable("/my_service", callback)

# Redis Transport
from vyra_base.com.transport.t_redis import RedisProvider

provider = RedisProvider(module_name="my_module")
await provider.initialize()
speaker = await provider.create_speaker("my_topic")

# ROS2 Transport
from vyra_base.com.transport.t_ros2 import ROS2Provider

provider = ROS2Provider(node_name="my_node")
await provider.initialize({"namespace": "/my_namespace"})
job = await provider.create_job("/my_action", callbacks)
```

## Migration from Old Names

If you have code using old import paths (before Feb 2026):

### Automatic Migration

```bash
# Update Python imports
find . -name "*.py" -exec sed -i \
  -e 's/vyra_base\.com\.transport\.zenoh/vyra_base.com.transport.t_zenoh/g' \
  -e 's/vyra_base\.com\.transport\.redis/vyra_base.com.transport.t_redis/g' \
  -e 's/vyra_base\.com\.transport\.ros2/vyra_base.com.transport.t_ros2/g' \
  -e 's/vyra_base\.com\.transport\.uds/vyra_base.com.transport.t_uds/g' \
  {} \;

# Update documentation
find . -name "*.md" -exec sed -i \
  -e 's@com/transport/zenoh@com/transport/t_zenoh@g' \
  -e 's@com/transport/redis@com/transport/t_redis@g' \
  -e 's@com/transport/ros2@com/transport/t_ros2@g' \
  -e 's@com/transport/uds@com/transport/t_uds@g' \
  {} \;
```

### Manual Migration Examples

**Before**:
```python
from vyra_base.com.transport.zenoh import ZenohProvider
from vyra_base.com.transport.redis import RedisClient
from vyra_base.com.transport.ros2.node import VyraNode
```

**After**:
```python
from vyra_base.com.transport.t_zenoh import ZenohProvider
from vyra_base.com.transport.t_redis import RedisClient
from vyra_base.com.transport.t_ros2.node import VyraNode
```

## Testing

Verify imports work correctly:

```python
# Test external library imports (should use pip packages)
import zenoh
print(f"Zenoh from: {zenoh.__file__}")
# Should show: .../site-packages/zenoh/...

import redis
print(f"Redis from: {redis.__file__}")
# Should show: .../site-packages/redis/...

# Test VYRA transport imports
from vyra_base.com.transport.t_zenoh import ZENOH_AVAILABLE
from vyra_base.com.transport.t_redis import REDIS_AVAILABLE
from vyra_base.com.transport.t_ros2 import ROS2_AVAILABLE

print(f"Zenoh transport available: {ZENOH_AVAILABLE}")
print(f"Redis transport available: {REDIS_AVAILABLE}")
print(f"ROS2 transport available: {ROS2_AVAILABLE}")
```

## Best Practices

### 1. Always Use Full Import Paths

✅ **Good**:
```python
from vyra_base.com.transport.t_zenoh import ZenohProvider
```

❌ **Avoid**:
```python
from vyra_base.com.transport import t_zenoh
provider = t_zenoh.ZenohProvider()  # Less clear
```

### 2. Use InterfaceFactory for Protocol-Agnostic Code

✅ **Good** (Protocol-independent):
```python
from vyra_base.com import InterfaceFactory, ProtocolType

callable = await InterfaceFactory.create_callable(
    "service", callback=handler,
    protocols=[ProtocolType.ZENOH, ProtocolType.REDIS]
)
```

✅ **Also Good** (Protocol-specific):
```python
from vyra_base.com.transport.t_zenoh import ZenohProvider

provider = ZenohProvider()
callable = await provider.create_callable("service", handler)
```

### 3. Check Availability Before Use

```python
from vyra_base.com.transport.t_zenoh import ZENOH_AVAILABLE

if ZENOH_AVAILABLE:
    from vyra_base.com.transport.t_zenoh import ZenohProvider
    provider = ZenohProvider()
else:
    logger.warning("Zenoh not available, using fallback")
```

## Internal Module Structure

Each transport module maintains its internal structure:

```
src/vyra_base/com/transport/
├── t_zenoh/
│   ├── __init__.py           # Public API
│   ├── provider.py           # ZenohProvider
│   ├── session.py            # Session management
│   ├── communication/        # Low-level Zenoh operations
│   └── vyra_models/          # High-level abstractions
├── t_redis/
│   ├── __init__.py
│   ├── provider.py
│   ├── communication/        # RedisClient
│   └── vyra_models/          # RedisCallable, RedisSpeaker
├── t_ros2/
│   ├── __init__.py
│   ├── provider.py
│   ├── node.py               # VyraNode
│   ├── communication/        # ROS2 primitives
│   └── vyra_models/          # ROS2Callable, ROS2Speaker, ROS2Job
└── t_uds/
    ├── __init__.py
    ├── provider.py
    └── communication/        # Unix domain socket operations
```

## Troubleshooting

### Import Error: "No module named 't_zenoh'"

Make sure you're using the correct import path:
```python
# ❌ Wrong
from vyra_base.com.transport import t_zenoh

# ✅ Correct
from vyra_base.com.transport.t_zenoh import ZenohProvider
```

### AttributeError: "module 'zenoh' has no attribute 'Config'"

This means the local module is being imported instead of the external library. Check:
1. Are you importing from `t_zenoh` correctly?
2. Is eclipse-zenoh installed? (`pip list | grep zenoh`)

### Tests Failing After Migration

Run the automated migration script on your test files:
```bash
find tests -name "*.py" -exec sed -i \
  -e 's/from vyra_base\.com\.transport\.zenoh/from vyra_base.com.transport.t_zenoh/g' \
  {} \;
```

## References

- **Transport README**: [src/vyra_base/com/transport/README.md](../src/vyra_base/com/transport/README.md)
- **Zenoh README**: [src/vyra_base/com/transport/t_zenoh/README.md](../src/vyra_base/com/transport/t_zenoh/README.md)
- **Redis README**: [src/vyra_base/com/transport/t_redis/README.md](../src/vyra_base/com/transport/t_redis/README.md)
- **ROS2 README**: [src/vyra_base/com/transport/t_ros2/README.md](../src/vyra_base/com/transport/t_ros2/README.md)
- **Protocol Types**: [src/vyra_base/com/core/types.py](../src/vyra_base/com/core/types.py)

---

**Last Updated**: February 6, 2026  
**Author**: GitHub Copilot  
**Change Type**: Module Renaming (Breaking Change)
