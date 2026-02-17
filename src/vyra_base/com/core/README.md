# Core Communication Components

Foundation types, exceptions, factory, and decorators for multi-protocol communication.

## Components

### Types (`types.py`)

Base types and enums for the communication system.

```python
from vyra_base.com.core.types import (
    ProtocolType,      # Protocol enumeration
    InterfaceType,     # Interface type enumeration
    VyraInterface,     # Base interface ABC
    VyraCallable,      # Request/response interface
    VyraSpeaker,       # Publish/subscribe interface
    VyraJob            # Long-running task interface
)
```

#### ProtocolType Enum

```python
class ProtocolType(Enum):
    # Transport Layer (always available)
    ROS2 = "ros2"              # Robot Operating System 2
    SHARED_MEMORY = "shmem"    # POSIX Shared Memory
    UDS = "uds"                # Unix Domain Sockets
    
    # External Layer (optional dependencies)
    REDIS = "redis"            # Redis Pub/Sub + KV
    GRPC = "grpc"              # gRPC over Unix Sockets
    MQTT = "mqtt"              # MQTT Pub/Sub
    REST = "rest"              # HTTP REST API
    WEBSOCKET = "websocket"    # WebSocket bidirectional
    
    # Industrial Layer (northbound)
    MODBUS = "modbus"          # Modbus TCP/RTU
    OPCUA = "opcua"            # OPC Unified Architecture
```

#### Interface Types

```python
class InterfaceType(Enum):
    CALLABLE = "callable"  # Request/response (RPC)
    SPEAKER = "speaker"    # Publish/subscribe
    JOB = "job"            # Long-running tasks
```

#### Base Classes

```python
class VyraInterface(ABC):
    """Base interface for all communication types"""
    
    @abstractmethod
    async def initialize(self) -> None:
        """Initialize the interface"""
    
    @abstractmethod
    async def shutdown(self) -> None:
        """Cleanup resources"""

class VyraCallable(VyraInterface):
    """Request/response interface (RPC pattern)"""
    
    @abstractmethod
    async def call(self, request: Dict[str, Any], timeout: float = 5.0) -> Dict[str, Any]:
        """Send request and wait for response"""

class VyraSpeaker(VyraInterface):
    """Publish/subscribe interface"""
    
    @abstractmethod
    async def shout(self, message: Dict[str, Any]) -> None:
        """Publish message to all subscribers"""

class VyraJob(VyraInterface):
    """Long-running task interface"""
    
    @abstractmethod
    async def execute(self, goal: Dict[str, Any]) -> Dict[str, Any]:
        """Execute job with feedback"""
```

### Exceptions (`exceptions.py`)

```python
from vyra_base.com.core.exceptions import (
    CommunicationError,          # Base exception
    ProtocolUnavailableError,    # Protocol not installed/available
    ProtocolNotInitializedError, # Provider not initialized
    TransportError,              # Transport layer error
    ProviderError,               # Provider-specific error
    InterfaceError,              # Interface creation error
    CallableError,               # Callable-specific error
    SpeakerError,                # Speaker-specific error
    JobError                     # Job-specific error
)
```

### Factory (`factory.py`)

Central factory for creating communication interfaces with automatic protocol selection and fallback.

```python
from vyra_base.com.core.factory import InterfaceFactory

# Create callable with auto-protocol selection
callable = await InterfaceFactory.create_callable(
    name="my_service",
    callback=handle_request
)

# Create speaker with explicit protocols
speaker = await InterfaceFactory.create_speaker(
    name="events",
    protocols=[ProtocolType.REDIS, ProtocolType.MQTT]
)

# Create job
job = await InterfaceFactory.create_job(
    name="long_task",
    execute_callback=process_job
)
```

#### Methods

**`create_callable(name, callback, protocols=None, **kwargs)`**
- Creates request/response interface (server-side)
- **Role**: Service Server (responds to requests)
- **Default fallback**: Zenoh → ROS2 → Redis → UDS
- **Returns**: `VyraCallable` instance
- **Note**: Automatically sets `is_callable=True`

**`create_caller(name, protocols=None, **kwargs)`**
- Creates request/response interface (client-side)
- **Role**: Service Client (makes requests)
- **Default fallback**: Zenoh → ROS2 → Redis → UDS
- **Returns**: `VyraCallable` instance
- **Note**: Automatically sets `is_callable=False`, no callback required

**`create_speaker(name, callback=None, protocols=None, **kwargs)`**
- Creates publish/subscribe interface (publisher)
- **Role**: Publisher (publishes messages)
- **Default fallback**: Zenoh → ROS2 → Redis → UDS
- **Returns**: `VyraSpeaker` instance
- **Note**: Automatically sets `is_publisher=True`

**`create_listener(name, callback, protocols=None, **kwargs)`**
- Creates publish/subscribe interface (subscriber)
- **Role**: Subscriber (receives messages)
- **Default fallback**: Zenoh → ROS2 → Redis → UDS
- **Returns**: `VyraSpeaker` instance
- **Note**: Automatically sets `is_publisher=False`, starts listening immediately

**`create_job(name, callback, protocols=None, **kwargs)`**
- Creates long-running task interface (server-side)
- **Role**: Action Server (executes actions)
- **Default fallback**: Zenoh → ROS2 → Redis → UDS
- **Returns**: `VyraJob` instance
- **Note**: Automatically sets `is_job=True`

**`create_dispatcher(name, protocols=None, **kwargs)`**
- Creates long-running task interface (client-side)
- **Role**: Action Client (sends goals)
- **Default fallback**: Zenoh → ROS2 → Redis → UDS
- **Returns**: `VyraJob` instance
- **Note**: Automatically sets `is_job=False`, no callback required

**`set_fallback_chain(interface_type, protocols)`**
- Customize protocol fallback order
- **Example**: `InterfaceFactory.set_fallback_chain("callable", [ProtocolType.GRPC, ProtocolType.UDS])`

**`get_available_protocols()`**
- Returns list of currently available protocols
- **Returns**: `List[ProtocolType]`

**`register_provider(provider)`**
- Register a protocol provider
- **Parameter**: `AbstractProtocolProvider` instance or list

**`unregister_provider(protocol)`**
- Unregister a protocol provider
- **Parameter**: `ProtocolType` enum value

### Blueprint System (`blueprints.py`)

**NEW**: Two-phase interface initialization pattern for late binding.

The Blueprint System decouples interface definition from implementation, enabling:
- Late binding of callbacks
- Dynamic interface registration
- Better testing (mock callbacks easily)
- Clear separation of concerns

```python
from vyra_base.com.core import (
    ServiceBlueprint,
    ActionBlueprint,
    PublisherBlueprint,
    SubscriberBlueprint,
    CallbackRegistry
)
```

#### ActionBlueprint - Multi-Callback Pattern

ActionBlueprint supports **three separate callbacks** for clean separation of concerns:

```python
# Phase 1: Create and register blueprint
blueprint = ActionBlueprint(
    name="process_batch",
    protocols=[ProtocolType.ROS2],
    action_type=ProcessBatch
)
CallbackRegistry.register_blueprint(blueprint)

# Phase 2: Bind callbacks
blueprint.bind_callbacks(
    on_goal=accept_goal_handler,      # Validate/accept goals
    on_cancel=handle_cancel_handler,  # Handle cancellation
    execute=execute_batch_handler     # Main execution logic
)

# Check binding status
if blueprint.is_fully_bound():
    # Create action server from blueprint
    action_server = await InterfaceFactory.create_from_blueprint(blueprint)
```

#### Using Decorators (Automatic Binding)

```python
from vyra_base.com import remote_actionServer
from vyra_base.com.core import IActionHandler, IGoalHandle, ActionStatus

class Component(IActionHandler):  # REQUIRED interface
    
    @remote_actionServer.on_goal(name="process_batch", auto_register=True)
    async def accept_goal(self, goal_request) -> bool:
        """
        Goal acceptance callback - validates and accepts/rejects goals.
        Returns: True to accept, False to reject
        """
        return goal_request.batch_size <= 100
    
    @remote_actionServer.on_cancel(name="process_batch")
    async def handle_cancel(self, goal_handle: IGoalHandle) -> bool:
        """
        Cancellation callback - handles cancel requests.
        Returns: True to accept cancellation, False to reject
        """
        return True  # Always allow cancellation
    
    @remote_actionServer.execute(name="process_batch")
    async def execute_batch(self, goal_handle: IGoalHandle) -> dict:
        """
        Execution callback - main action logic with feedback.
        Returns: Result dictionary
        """
        for i in range(goal_handle.goal.batch_size):
            # Check for cancellation
            if goal_handle.is_cancel_requested():
                goal_handle.canceled()
                return {"status": ActionStatus.CANCELED, "processed": i}
            
            # Publish feedback
            goal_handle.publish_feedback({"progress": (i+1)*10})
            
            # Do work
            await process_item(i)
        
        # Mark as successful
        goal_handle.succeed()
        return {"status": ActionStatus.SUCCEEDED, "processed": i+1}
```

#### ActionStatus Enum

```python
from vyra_base.com.core import ActionStatus

class ActionStatus(Enum):
    UNKNOWN = 0
    ACCEPTED = 1
    EXECUTING = 2
    CANCELING = 3
    SUCCEEDED = 4
    CANCELED = 5
    ABORTED = 6
```

#### Abstract Handler Interfaces (REQUIRED)

All complex handlers **MUST** implement these interfaces:

```python
from vyra_base.com.core import IServiceHandler, IActionHandler, IGoalHandle

class IServiceHandler(ABC):
    @abstractmethod
    async def handle_request(self, request: Any) -> Any:
        """Handle service request"""
        pass

class IActionHandler(ABC):
    @abstractmethod
    async def on_goal(self, goal: Any) -> bool:
        """Validate and accept/reject goal"""
        pass
    
    @abstractmethod
    async def execute(self, handle: IGoalHandle) -> Dict[str, Any]:
        """Execute action with feedback"""
        pass
    
    @abstractmethod
    async def on_cancel(self) -> bool:
        """Handle cancellation request"""
        pass

class IGoalHandle(ABC):
    @abstractmethod
    def publish_feedback(self, feedback: Dict[str, Any]) -> None:
        """Send progress feedback"""
        pass
    
    @abstractmethod
    def is_cancel_requested(self) -> bool:
        """Check if cancellation was requested"""
        pass
    
    @abstractmethod
    def succeed(self) -> None:
        """Mark action as successful"""
        pass
    
    @abstractmethod
    def abort(self) -> None:
        """Mark action as failed"""
        pass
    
    @abstractmethod
    def canceled(self) -> None:
        """Mark action as canceled"""
        pass
```

**Benefits of Multi-Callback Pattern:**
- ✅ Separation of concerns
- ✅ Early goal rejection (before execution starts)
- ✅ Explicit cancellation handling
- ✅ Better testability (test each callback independently)
- ✅ Cleaner code organization

**Migration from Single Callback:**
```python
# OLD (DEPRECATED)
@remote_actionServer(name="process")
async def execute_action(self, goal_handle):
    # Mixed concerns: validation + cancellation + execution
    ...

# NEW (REQUIRED)
@remote_actionServer.on_goal(name="process")
async def on_goal(self, goal_request): return True

@remote_actionServer.on_cancel(name="process")
async def on_cancel(self, goal_handle): return True

@remote_actionServer.execute(name="process")
async def execute(self, goal_handle): ...
```

See:
- [Implementation Phase 1](../../../IMPLEMENTATION_PHASE1_COMPLETE.md)
- [Implementation Phase 2](../../../IMPLEMENTATION_PHASE2_COMPLETE.md)
- [Example: Multi-Callback ActionServer](../../../docs/examples/example_action_server_multicallback.py)

### Decorators (`decorators.py`)

Protocol-agnostic decorators for automatic interface creation.

```python
from vyra_base.com.core.decorators import (
    remote_service,     # Service decorator (renamed from remote_callable)
    remote_publisher,   # Publisher decorator (renamed from remote_speaker)
    remote_subscriber,  # Subscriber decorator
    remote_actionServer,  # ActionServer decorator (NEW: multi-callback)
    
    # Backward compatibility aliases (DEPRECATED)
    remote_callable,    # Use remote_service instead
    remote_speaker,     # Use remote_publisher instead
    remote_listener,    # Use remote_subscriber instead
    remote_job          # Use remote_actionServer instead
)
```

#### @remote_service (formerly @remote_callable)

Automatically creates service interface for method:

```python
class MyComponent:
    @remote_service
    async def calculate(self, request, response=None):
        """
        Args:
            request: Input data dict
            response: Optional response object (ROS2 compatibility)
        Returns:
            Dict with result
        """
        return {"result": request["x"] + request["y"]}
    
    @remote_service(protocols=[ProtocolType.GRPC, ProtocolType.UDS])
    async def fast_method(self, request, response=None):
        """Explicit protocol selection"""
        return {"data": process(request)}
```

**Decorator Parameters**:
- `name`: Custom interface name (default: method name)
- `protocols`: List of protocols to try (optional)
- `auto_register`: Whether to auto-register blueprint (default: True)
- `namespace`: Optional namespace for blueprint registration
- `**kwargs`: Additional metadata (service_type, qos, etc.)

#### @remote_publisher (formerly @remote_speaker)

Automatically creates publisher interface for method:

```python
class MyComponent:
    @remote_publisher
    async def publish_status(self, message):
        """
        Args:
            message: Data to publish
        """
        pass  # Implementation auto-generated
    
    @remote_publisher(protocols=[ProtocolType.REDIS])
    async def redis_only(self, message):
        """Explicit protocol"""
        pass
```

**Decorator Parameters**:
- `name`: Custom interface name (default: method name)
- `protocols`: List of protocols to try (optional)
- `auto_register`: Whether to auto-register blueprint (default: True)
- `namespace`: Optional namespace for blueprint registration
- `**kwargs`: Additional metadata (message_type, qos, retain, etc.)

#### @remote_actionServer (NEW: Multi-Callback Pattern)

**Class-based decorator** with sub-decorators for goal acceptance, cancellation, and execution:

```python
from vyra_base.com import remote_actionServer
from vyra_base.com.core import IActionHandler, IGoalHandle, ActionStatus

class Component(IActionHandler):  # REQUIRED interface
    
    @remote_actionServer.on_goal(name="process_batch", auto_register=True)
    async def accept_goal(self, goal_request) -> bool:
        """
        Goal acceptance callback (FIRST decorator must set auto_register=True)
        
        Args:
            goal_request: The goal request data
        
        Returns:
            bool: True to accept goal, False to reject
        """
        # Validate goal parameters
        return goal_request.batch_size <= self.max_batch_size
    
    @remote_actionServer.on_cancel(name="process_batch")
    async def handle_cancel(self, goal_handle: IGoalHandle) -> bool:
        """
        Cancellation callback (auto_register=False, blueprint already registered)
        
        Args:
            goal_handle: Handle to the active goal
        
        Returns:
            bool: True to accept cancellation, False to reject
        """
        # Decide whether to allow cancellation
        return not self.in_critical_section
    
    @remote_actionServer.execute(name="process_batch")
    async def execute_batch(self, goal_handle: IGoalHandle) -> dict:
        """
        Execution callback (auto_register=False, blueprint already registered)
        
        Args:
            goal_handle: Provides:
                - goal_handle.goal: Goal request data
                - goal_handle.publish_feedback(dict): Send progress updates
                - goal_handle.is_cancel_requested(): Check for cancellation
                - goal_handle.succeed(): Mark as successful
                - goal_handle.abort(): Mark as failed
                - goal_handle.canceled(): Mark as canceled
        
        Returns:
            dict: Result data (available to action client)
        
        IMPORTANT: Must call succeed(), abort(), or canceled() before returning!
        """
        for i in range(goal_handle.goal.batch_size):
            # Check for cancellation
            if goal_handle.is_cancel_requested():
                goal_handle.canceled()
                return {"status": ActionStatus.CANCELED, "processed": i}
            
            # Publish feedback
            goal_handle.publish_feedback({"progress": (i+1)*10})
            
            # Do work
            await process_item(i)
        
        # Mark as successful
        goal_handle.succeed()
        return {"status": ActionStatus.SUCCEEDED, "processed": i+1}
```

**Decorator Parameters (all sub-decorators)**:
- `name`: Action name (**REQUIRED**, must match across all three decorators)
- `protocols`: List of protocols to try (optional)
- `auto_register`: Whether to auto-register blueprint (default: True for on_goal, False for others)
- `namespace`: Optional namespace for blueprint registration
- `**kwargs`: Additional metadata (action_type, timeout, etc.)

**Important Notes**:
1. **All three decorators are REQUIRED** for full functionality
2. **`on_goal` must be first** with `auto_register=True` to create blueprint
3. **`on_cancel` and `execute`** should set `auto_register=False` (blueprint already exists)
4. **Component MUST implement `IActionHandler`** interface
5. **Name must be identical** across all three decorators

**Legacy Single-Callback (DEPRECATED)**:
```python
@remote_job  # DEPRECATED - Do not use
async def process_dataset(self, goal, feedback_callback=None):
    # Mixed concerns - NOT RECOMMENDED
    ...
```

### Registry (`registry.py`)

Not directly used by applications - internal component for provider management.

### Monitoring (`monitoring.py`)

Prometheus metrics integration:

```python
from vyra_base.com.monitoring import CommunicationMonitor

monitor = CommunicationMonitor()

# Automatically collects:
# - vyra_com_calls_total
# - vyra_com_call_duration_seconds
# - vyra_com_active_connections
# - vyra_com_errors_total
# - vyra_com_message_size_bytes

# ROS2-specific (when available):
# - vyra_ros2_topic_throughput
# - vyra_ros2_qos_violations
# - vyra_ros2_discovery_nodes
```

### Dynamic Interface Loading

Runtime interface discovery and loading without compile-time imports.

#### Components

**InterfacePathRegistry** - Singleton registry for interface base paths:

```python
from vyra_base.com.core import InterfacePathRegistry

registry = InterfacePathRegistry.get_instance()
registry.set_interface_paths([
    "/workspace/install/mypackage_interfaces/share/mypackage_interfaces"
])
```

**InterfaceLoader** - Dynamic loader for ROS2 and Protobuf interfaces:

```python
from vyra_base.com.core import InterfaceLoader

loader = InterfaceLoader()

# Load ROS2 service interface
srv_type = loader.load_ros2_interface("mypackage_interfaces/srv/MyService")

# Load protobuf module for Zenoh/Redis/UDS
pb_module = loader.load_protobuf_interface("MyServicePB")

# Load by function name from metadata
interface = loader.get_interface_for_function("my_function", protocol="ros2")
```

**TopicBuilder** - Enhanced with interface loading:

```python
from vyra_base.com.core import TopicBuilder

builder = TopicBuilder("v2_modulemanager", "abc123")

# Build topic name only (slim mode compatible)
topic = builder.build_topic_name("get_modules")

# Load interface type
interface = builder.load_interface_type("get_modules", protocol="ros2")

# Build topic AND load interface
topic, interface = builder.build_with_interface(
    "get_modules",
    protocol="ros2"
)
```

#### Features

- ✅ **Runtime Interface Discovery**: Load interfaces from string names
- ✅ **No Compile-Time Dependencies**: Eliminate hardcoded imports
- ✅ **Cross-Module Communication**: Discover interfaces from other modules
- ✅ **Multi-Protocol Support**: ROS2, Zenoh, Redis, UDS
- ✅ **Slim Mode Compatible**: Graceful fallback when ROS2 unavailable
- ✅ **Performance Optimized**: Interface caching

#### Usage in Modules

```python
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from vyra_base.core.entity import VyraEntity

# Configure interface paths BEFORE set_interfaces()
entity = VyraEntity(...)
entity.set_interface_paths([
    Path(get_package_share_directory("mymodule_interfaces")),
    Path(get_package_share_directory("vyra_module_interfaces"))
])

# Now interfaces are loaded dynamically
await entity.set_interfaces(base_interfaces)
```

**See detailed documentation**: [Dynamic Interface Loading Guide](../../../docs/DYNAMIC_INTERFACE_LOADING.md)

## Usage Patterns

### Server-Client Pattern (New!)

The factory now provides explicit server/client methods for clearer code:

```python
from vyra_base.com.core.factory import InterfaceFactory

# SERVER SIDE: Responds to requests
server = await InterfaceFactory.create_callable(
    "my_service",
    callback=lambda req, res: {"result": req["value"] * 2}
)

# CLIENT SIDE: Makes requests
caller = await InterfaceFactory.create_caller("my_service")
result = await caller.call({"value": 21})
print(result)  # {"result": 42}
```

### Publisher-Subscriber Pattern

```python
# PUBLISHER: Sends messages
speaker = await InterfaceFactory.create_speaker("events")
await speaker.shout({"event": "update", "data": 123})

# SUBSCRIBER: Receives messages
listener = await InterfaceFactory.create_listener(
    "events",
    callback=lambda msg: print(f"Received: {msg}")
)
```

### Action Server-Client Pattern

```python
# ACTION SERVER: Executes long-running tasks
async def execute_job(goal, feedback_callback=None):
    for i in range(100):
        if feedback_callback:
            await feedback_callback({"progress": i})
    return {"status": "completed"}

job = await InterfaceFactory.create_job("process_data", callback=execute_job)

# ACTION CLIENT: Sends goals
dispatcher = await InterfaceFactory.create_dispatcher(
    "process_data",
    feedback_callback=lambda fb: print(f"Progress: {fb['progress']}%")
)
result = await dispatcher.execute({"dataset": "data.csv"})
```

### Basic Factory Usage (Legacy)

```python
from vyra_base.com.core.factory import InterfaceFactory
from vyra_base.com.core.types import ProtocolType

# Auto-select protocol
callable = await InterfaceFactory.create_callable(
    "service_name",
    callback=lambda req, res: {"result": req["value"] * 2}
)

result = await callable.call({"value": 21})
print(result)  # {"result": 42}
```

### Explicit Protocol Selection

```python
# Try Redis first, fallback to MQTT
speaker = await InterfaceFactory.create_speaker(
    "events",
    protocols=[ProtocolType.REDIS, ProtocolType.MQTT]
)

await speaker.shout({"event": "update", "data": 123})
```

### Decorator-Based (Recommended)

```python
from vyra_base.com.core.decorators import remote_callable, remote_speaker

class DataProcessor:
    def __init__(self):
        self.results = []
    
    @remote_callable
    async def process(self, request, response=None):
        """Automatically creates callable interface"""
        result = await self._compute(request["input"])
        return {"output": result}
    
    @remote_speaker(protocols=[ProtocolType.REDIS])
    async def notify_complete(self, message):
        """Automatically creates speaker interface"""
        pass  # Publishing handled automatically
    
    async def run(self):
        result = await self.process({"input": [1, 2, 3]})
        await self.notify_complete({"status": "done", "result": result})
```

### Custom Fallback Chain

```python
# Change default fallback order
InterfaceFactory.set_fallback_chain(
    "callable",
    [ProtocolType.SHARED_MEMORY, ProtocolType.UDS, ProtocolType.GRPC]
)

# Now all callables will prefer SharedMemory
callable = await InterfaceFactory.create_callable("fast_service")
```

### Check Available Protocols

```python
from vyra_base.com.core.factory import InterfaceFactory

available = InterfaceFactory.get_available_protocols()
print(f"Available protocols: {available}")

if ProtocolType.ROS2 not in available:
    print("ROS2 not available, using fallback")
```

## Integration with Feeders

Feeders use InterfaceFactory internally for multi-protocol support:

```python
from vyra_base.com.feeder import StateFeeder

# Automatically uses Redis → ROS2 → MQTT fallback
feeder = StateFeeder(entity, node=None)  # Works without ROS2
await feeder.start()
await feeder.push_state("RUNNING")
```

## Error Handling

```python
from vyra_base.com.core.exceptions import (
    ProtocolUnavailableError,
    CallableError,
    InterfaceError
)

try:
    callable = await InterfaceFactory.create_callable(
        "service",
        protocols=[ProtocolType.ROS2]  # ROS2 only
    )
except ProtocolUnavailableError:
    # ROS2 not available
    callable = await InterfaceFactory.create_callable(
        "service",
        protocols=[ProtocolType.UDS]  # Fallback
    )

try:
    result = await callable.call({"data": 123}, timeout=10.0)
except CallableError as e:
    logger.error(f"Call failed: {e}")
```

## Testing

Mock providers for testing:

```python
import pytest
from vyra_base.com.core.factory import InterfaceFactory
from vyra_base.com.core.types import ProtocolType

@pytest.fixture
def mock_provider():
    """Mock provider for testing"""
    class MockProvider:
        async def create_callable(self, name, callback=None, **kwargs):
            return MockCallable(name, callback)
    
    return MockProvider()

@pytest.mark.asyncio
async def test_factory(mock_provider):
    # Register mock provider
    from vyra_base.com.providers import ProviderRegistry
    registry = ProviderRegistry()
    registry.register_provider(ProtocolType.UDS, mock_provider)
    
    # Use factory
    callable = await InterfaceFactory.create_callable(
        "test",
        protocols=[ProtocolType.UDS]
    )
    assert callable is not None
```

## Best Practices

### 1. Use Decorators for Components
```python
# ✅ Good - Clear intent, automatic setup
class Component:
    @remote_callable
    async def method(self, request, response=None):
        return result

# ❌ Avoid - Manual factory calls in business logic
class Component:
    async def method(self, request):
        callable = await InterfaceFactory.create_callable(...)
        return await callable.call(request)
```

### 2. Specify Protocols When Performance Matters
```python
# ✅ Good - Explicit fast protocols
@remote_callable(protocols=[ProtocolType.SHARED_MEMORY, ProtocolType.UDS])
async def high_freq_method(self, request, response=None):
    return result
```

### 3. Handle Protocol Unavailability
```python
# ✅ Good - Graceful degradation
available = InterfaceFactory.get_available_protocols()
if ProtocolType.ROS2 not in available:
    logger.warning("ROS2 unavailable, using fallback")
```

### 4. Use Type Hints
```python
# ✅ Good - Clear types
from typing import Dict, Any

@remote_callable
async def method(self, request: Dict[str, Any], response=None) -> Dict[str, Any]:
    return {"result": request["value"]}
```

## See Also

- [Transport Layer](../transport/README.md) - Local IPC protocols
- [External Layer](../external/README.md) - Network protocols
- [Providers](../providers/README.md) - Provider pattern details
- [Main README](../README.md) - Architecture overview
