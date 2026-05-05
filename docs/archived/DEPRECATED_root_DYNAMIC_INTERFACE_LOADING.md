# Dynamic Interface Loading in vyra_base

## Overview

vyra_base now supports **dynamic interface loading** for both ROS2 interfaces (.srv/.msg/.action) and Protocol Buffer definitions (*_pb2.py). This eliminates the need for compile-time imports and enables true runtime interface discovery.

### Key Features

- ✅ **Runtime Interface Discovery**: Load ROS2 and Protobuf interfaces from string names
- ✅ **No Compile-Time Dependencies**: Eliminate hardcoded imports
- ✅ **Cross-Module Communication**: Discover interfaces from other modules at runtime
- ✅ **Multi-Protocol Support**: Works with ROS2, Zenoh, Redis, and UDS transports
- ✅ **Slim Mode Compatible**: Graceful fallback when ROS2 unavailable
- ✅ **Performance Optimized**: Interface caching for efficient reuse

---

## Architecture

### Components

1. **InterfacePathRegistry**: Singleton registry of interface base paths
2. **InterfaceLoader**: Dynamic loader for ROS2 and Protobuf interfaces
3. **TopicBuilder**: Enhanced with interface loading capabilities
4. **ROS2 Environment Helper**: Utilities for runtime path configuration

### Interface Directory Structure

```
<interface_path>/               # Base interface directory
├── config/                     # Interface metadata (JSON)
│   ├── vyra_core_meta.json
│   ├── vyra_state_meta.json
│   └── custom_meta.json
├── callable/                   # ROS2 service definitions
│   ├── VBASEGetInterfaceList.srv
│   └── VBASEHealthCheck.srv
├── speaker/                    # ROS2 message definitions
│   ├── VBASEStateFeed.msg
│   └── VBASENewsFeed.msg
├── job/                        # ROS2 action definitions
│   └── VBASEInitiateUpdate.action
└── proto/                      # Protocol Buffer definitions
    ├── VBASEGetInterfaceList.proto
    ├── VBASEGetInterfaceList_pb2.py    # Generated Python module
    ├── VBASEStateFeed.proto
    └── VBASEStateFeed_pb2.py
```

---

## Usage Guide

### 1. Basic Setup in Module

In your module's `_base_.py` or initialization code:

```python
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from vyra_base.core.entity import VyraEntity


async def build_base():
    # Create entity
    entity = await build_entity(project_settings)
    
    # Configure interface paths BEFORE set_interfaces()
    module_interface_path = Path(
        get_package_share_directory("v2_modulemanager_interfaces")
    )
    vyra_base_path = Path(
        get_package_share_directory("vyra_module_interfaces")
    )
    
    entity.set_interface_paths([
        module_interface_path,
        vyra_base_path
    ])
    
    # Now set_interfaces() will use dynamic loading
    base_interfaces = await _create_base_interfaces()
    await entity.set_interfaces(base_interfaces)
    
    return entity
```

### 2. Using TopicBuilder with Dynamic Loading

```python
from vyra_base.com.core import TopicBuilder, InterfaceType

# Create TopicBuilder with interface loading enabled
builder = TopicBuilder(
    module_name="v2_modulemanager",
    module_id="abc123",
    enable_interface_loading=True  # Default
)

# Option 1: Build topic name only (slim mode compatible)
topic_name = builder.build_topic_name("get_interface_list")
# Result: "v2_modulemanager_abc123/get_interface_list"

# Option 2: Load interface type only
interface_type = builder.load_interface_type(
    "get_interface_list",
    protocol="ros2"  # or "zenoh", "redis", "uds"
)
# Result: <class 'vyra_module_interfaces.srv.VBASEGetInterfaceList'>

# Option 3: Build topic AND load interface together
topic, interface = builder.build_with_interface(
    "get_interface_list",
    interface_type=InterfaceType.CALLABLE,
    protocol="ros2"
)
# Result: ("v2_modulemanager_abc123/get_interface_list", <class>)
```

### 3. Direct Interface Loading

```python
from vyra_base.com.core import InterfaceLoader

# Create loader with custom paths
loader = InterfaceLoader(
    interface_paths=[Path("/workspace/install/mypackage_interfaces/share/mypackage_interfaces")]
)

# Load ROS2 service interface
srv_type = loader.load_ros2_interface("mypackage_interfaces/srv/MyService")

# Load protobuf module for Zenoh/Redis/UDS
pb_module = loader.load_protobuf_interface("MyServicePB")

# Load by function name from metadata
interface = loader.get_interface_for_function(
    "my_function",
    protocol="ros2"  # Loads .srv/.msg/.action
)
interface_pb = loader.get_interface_for_function(
    "my_function",
    protocol="zenoh"  # Loads .proto → *_pb2.py
)
```

### 4. Interface Metadata (JSON Configuration)

Create metadata files in `<interface_path>/config/*.json`:

```json
{
    "functionname": "get_modules",
    "displayname": "Get Modules",
    "description": "Retrieve list of available modules",
    "type": "callable",
    "filetype": [
        "GetModules.srv",
        "GetModules.proto"
    ],
    "tags": ["ros2", "zenoh", "redis", "uds"],
    "params": [],
    "returns": [
        {
            "name": "modules",
            "datatype": "Module[]",
            "description": "List of modules"
        }
    ],
    "access_level": 1,
    "displaystyle": {
        "visible": true,
        "published": false
    }
}
```

**Key Fields**:
- `functionname`: Unique identifier for the interface
- `type`: Interface type (`callable`, `speaker`, or `job`)
- `filetype`: Array of interface files (`.srv`, `.msg`, `.action`, `.proto`)
- `displaystyle.visible`: If `false`, interface is hidden from discovery

---

## Advanced Features

### Global Registry Configuration

```python
from vyra_base.com.core import InterfacePathRegistry

# Access global registry
registry = InterfacePathRegistry.get_instance()

# Set interface paths globally (affects all TopicBuilders)
registry.set_interface_paths([
    "/workspace/install/package1_interfaces/share/package1_interfaces",
    "/workspace/install/package2_interfaces/share/package2_interfaces"
])

# Add path without replacing existing
registry.add_interface_path("/additional/path")

# Get all configured paths
paths = registry.get_interface_paths()

# Get config subdirectories only
config_paths = registry.get_config_paths()

# Get proto subdirectories only
proto_paths = registry.get_proto_paths()

# Reset to default vyra_base path
registry.clear_interface_paths()
```

### Environment Path Management

```python
from vyra_base.com.core import (
    update_ament_prefix_path,
    update_python_path,
    ensure_interface_package_discoverable,
    ensure_workspace_discoverable
)

# Update AMENT_PREFIX_PATH for ROS2 discovery
update_ament_prefix_path("/workspace/install")

# Update sys.path for Python imports
update_python_path("/workspace/install/mypackage/lib/python3.12/site-packages")

# Make single package discoverable
ensure_interface_package_discoverable(
    "/workspace/install",
    "mypackage_interfaces"
)

# Make all packages in workspace discoverable
count = ensure_workspace_discoverable("/workspace/install")
print(f"Made {count} packages discoverable")
```

### Caching and Performance

```python
# Get cache statistics
stats = builder.get_loaded_interfaces_stats()
print(f"Cached ROS2 interfaces: {stats['ros2_interfaces']}")
print(f"Cached Protobuf modules: {stats['protobuf_interfaces']}")
print(f"Metadata entries: {stats['metadata_entries']}")

# Force reload metadata from disk
builder.reload_interface_metadata()

# Clear all caches
loader = builder.get_interface_loader()
if loader:
    loader.clear_cache()
```

---

## Slim Mode (Without ROS2)

TopicBuilder works in "slim mode" when ROS2 is unavailable:

```python
# Topic naming works without ROS2
builder = TopicBuilder(
    "v2_modulemanager",
    "abc123",
    enable_interface_loading=False  # Disable for pure naming
)

topic = builder.build_topic_name("get_modules")
# ✅ Works: "v2_modulemanager_abc123/get_modules"

interface = builder.load_interface_type("get_modules")
# ⚠️ Returns None gracefully (no ROS2)
```

---

## Migration Guide

### Old Way (Compile-Time Imports)

```python
# ❌ Old: Hardcoded imports
from v2_modulemanager_interfaces.srv import VBASEGetInterfaceList

# Service type must be known at compile time
node.create_service(
    VBASEGetInterfaceList,
    "get_interface_list",
    callback
)
```

### New Way (Runtime Loading)

```python
# ✅ New: Dynamic loading
builder = TopicBuilder("v2_modulemanager", "abc123")

topic, srv_type = builder.build_with_interface(
    "get_interface_list",
    protocol="ros2"
)

# Service type loaded at runtime
if srv_type:
    node.create_service(srv_type, topic, callback)
```

---

## Testing

### Run Unit Tests

```bash
# All dynamic loading tests
pytest tests/com/core/test_interface_path_registry.py -v
pytest tests/com/core/test_interface_loader.py -v
pytest tests/com/core/test_topic_builder_dynamic.py -v

# Quick smoke test
pytest tests/com/core/ -k "dynamic or loader or registry" -v
```

### Manual Testing

```python
# Test in Python REPL
from vyra_base.com.core import InterfaceLoader, InterfacePathRegistry

# Check default paths
registry = InterfacePathRegistry.get_instance()
print("Default paths:", registry.get_interface_paths())

# Try loading an interface
loader = InterfaceLoader()
metadata = loader.load_interface_metadata()
print(f"Found {len(metadata)} interfaces")

# Test ROS2 interface loading
if loader.load_ros2_interface:
    srv = loader.load_ros2_interface("vyra_module_interfaces/srv/VBASEHealthCheck")
    print(f"Loaded: {srv}")
```

---

## Troubleshooting

### Interface Not Found

**Problem**: `loader.load_ros2_interface()` returns `None`

**Solutions**:
1. Check interface path is registered:
   ```python
   registry = InterfacePathRegistry.get_instance()
   print(registry.get_interface_paths())
   ```

2. Verify interfaces are built:
   ```bash
   ls /workspace/install/mypackage_interfaces/share/mypackage_interfaces/
   ```

3. Check AMENT_PREFIX_PATH:
   ```python
   import os
   print(os.environ.get('AMENT_PREFIX_PATH'))
   ```

4. Ensure workspace is sourced or use `ensure_workspace_discoverable()`

### Protobuf Module Import Error

**Problem**: `ModuleNotFoundError: No module named 'MyInterface_pb2'`

**Solutions**:
1. Verify _pb2.py file exists:
   ```bash
   find /workspace -name "*_pb2.py"
   ```

2. Check sys.path includes proto directory:
   ```python
   import sys
   print([p for p in sys.path if 'proto' in p])
   ```

3. Manually update path:
   ```python
   from vyra_base.com.core import update_python_path
   update_python_path("/workspace/install/mypackage_interfaces/.../proto")
   ```

### Metadata Not Loaded

**Problem**: `get_interface_for_function()` returns `None`

**Solutions**:
1. Check JSON files exist in config/ directories
2. Verify `displaystyle.visible: true` in JSON
3. Validate JSON syntax:
   ```bash
   python -m json.tool config/my_meta.json
   ```
4. Force reload:
   ```python
   loader.load_interface_metadata(reload=True)
   ```

### ROS2 Not Available in Slim Mode

**Expected**: Interface loading returns `None` gracefully

**Workaround**: Use Protobuf interfaces for non-ROS2 transports:
```python
# Use protobuf for Zenoh/Redis/UDS
interface = loader.get_interface_for_function("my_func", protocol="zenoh")
```

---

## Performance Considerations

- **First Load**: ~10-50ms per interface (file I/O + import)
- **Cached Load**: ~0.1ms (dictionary lookup)
- **Metadata Load**: ~5-20ms for 50 interfaces (JSON parsing)

**Recommendations**:
- Preload common interfaces at startup
- Use `build_with_interface()` to combine operations
- Don't clear cache unless necessary
- Enable interface loading only when needed

---

## API Reference

### InterfacePathRegistry

```python
class InterfacePathRegistry:
    @classmethod
    def get_instance() -> InterfacePathRegistry
    
    def set_interface_paths(paths: list[str | Path]) -> None
    def add_interface_path(path: str | Path) -> None
    def get_interface_paths() -> list[Path]
    def get_config_paths() -> list[Path]
    def get_proto_paths() -> list[Path]
    def clear_interface_paths() -> None
```

### InterfaceLoader

```python
class InterfaceLoader:
    def __init__(
        interface_paths: Optional[list[Path]] = None,
        auto_update_paths: bool = True
    )
    
    def load_ros2_interface(interface_path: str) -> Optional[type]
    def load_protobuf_interface(interface_name: str) -> Optional[Any]
    def load_interface_metadata(reload: bool = False) -> dict[str, dict]
    def get_interface_for_function(
        function_name: str,
        protocol: str = "ros2"
    ) -> Optional[Union[type, Any]]
    
    def clear_cache() -> None
    def get_cache_stats() -> dict[str, int]
```

### TopicBuilder (Enhanced)

```python
class TopicBuilder:
    def __init__(
        module_name: str,
        module_id: str,
        interface_paths: Optional[list[str | Path]] = None,
        enable_interface_loading: bool = True
    )
    
    # New methods
    def build_topic_name(
        function_name: str,
        subaction: Optional[str] = None,
        interface_type: Optional[InterfaceType] = None
    ) -> str
    
    def load_interface_type(
        function_name: str,
        protocol: str = "ros2"
    ) -> Optional[Union[type, Any]]
    
    def build_with_interface(
        function_name: str,
        subaction: Optional[str] = None,
        interface_type: Optional[InterfaceType] = None,
        protocol: str = "ros2"
    ) -> tuple[str, Optional[Union[type, Any]]]
    
    def get_interface_loader() -> Optional[InterfaceLoader]
    def reload_interface_metadata() -> None
    def get_loaded_interfaces_stats() -> dict[str, int]
    
    # Existing methods (backward compatible)
    def build(...) -> str
    def build_with_prefix(...) -> str
    def parse(...) -> TopicComponents
    def validate(...) -> bool
```

### VyraEntity (New Method)

```python
class VyraEntity:
    def set_interface_paths(interface_paths: list[str | Path]) -> None
```

---

## See Also

- [ROS2 Interface Design](../interfaces/README.md)
- [Protocol Buffer Generation](../../tools/generate_proto.md)
- [Module Development Guide](../../../docs/MODULE_DEVELOPMENT.md)
- [Testing Guide](../../../tests/README.md)

---

## Changelog

### Version 1.0.0 (2026-02-12)

- ✨ Initial implementation of dynamic interface loading
- ✨ Added InterfacePathRegistry singleton
- ✨ Added InterfaceLoader with ROS2 and Protobuf support
- ✨ Enhanced TopicBuilder with interface loading
- ✨ Added ROS2 environment helper utilities
- ✨ Added VyraEntity.set_interface_paths() method
- ✅ Comprehensive unit test coverage
- 📚 Complete documentation and usage guide
