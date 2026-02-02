# Provider Pattern

Abstract protocol provider pattern for pluggable communication backends.

## Overview

The Provider Pattern enables protocol-agnostic communication by:
- Abstracting protocol details behind uniform interface
- Enabling runtime protocol selection
- Supporting automatic fallback between protocols
- Facilitating testing with mock providers

## Architecture

```
┌─────────────────────────────────────────┐
│      AbstractProtocolProvider           │
│  (initialize, create_*, shutdown)       │
└───────────────┬─────────────────────────┘
                │
        ┌───────┴─────────┐
        │                 │
┌───────▼──────┐  ┌───────▼────────┐
│  Transport   │  │  External      │
│  Providers   │  │  Providers     │
├──────────────┤  ├────────────────┤
│• SharedMemory│  │• Redis         │
│• ROS2        │  │• gRPC          │
│• UDS         │  │• MQTT          │
│              │  │• REST          │
│              │  │• WebSocket     │
└──────────────┘  └────────────────┘
         │                │
         └────────┬───────┘
                  │
        ┌─────────▼──────────┐
        │  ProviderRegistry  │
        │   (Singleton)      │
        └────────────────────┘
```

## AbstractProtocolProvider

Base class for all protocol implementations.

### Interface

```python
from abc import ABC, abstractmethod
from typing import Optional, Dict, Any
from vyra_base.com.core.types import ProtocolType, VyraCallable, VyraSpeaker, VyraJob

class AbstractProtocolProvider(ABC):
    """Base class for protocol providers"""
    
    def __init__(self, protocol_type: ProtocolType):
        self._protocol_type = protocol_type
        self._initialized = False
    
    @abstractmethod
    async def initialize(self) -> None:
        """Initialize provider resources"""
        pass
    
    @abstractmethod
    async def shutdown(self) -> None:
        """Cleanup provider resources"""
        pass
    
    @abstractmethod
    async def is_available(self) -> bool:
        """Check if protocol is available"""
        pass
    
    @abstractmethod
    async def create_callable(
        self,
        name: str,
        callback: Optional[callable] = None,
        **kwargs
    ) -> VyraCallable:
        """Create request/response interface"""
        pass
    
    @abstractmethod
    async def create_speaker(
        self,
        name: str,
        callback: Optional[callable] = None,
        **kwargs
    ) -> VyraSpeaker:
        """Create publish/subscribe interface"""
        pass
    
    @abstractmethod
    async def create_job(
        self,
        name: str,
        execute_callback: Optional[callable] = None,
        **kwargs
    ) -> VyraJob:
        """Create long-running task interface"""
        pass
    
    @property
    def protocol_type(self) -> ProtocolType:
        return self._protocol_type
    
    @property
    def is_initialized(self) -> bool:
        return self._initialized
```

## ProviderRegistry

Singleton registry for managing protocol providers.

### Usage

```python
from vyra_base.com.providers import ProviderRegistry
from vyra_base.com.transport import SharedMemoryProvider
from vyra_base.com.core.types import ProtocolType

# Get singleton instance
registry = ProviderRegistry()

# Register provider
shmem_provider = SharedMemoryProvider()
await shmem_provider.initialize()
registry.register_provider(ProtocolType.SHARED_MEMORY, shmem_provider)

# Get provider
provider = registry.get_provider(ProtocolType.SHARED_MEMORY)

# Check availability
is_available = await registry.is_protocol_available(ProtocolType.ROS2)

# List all available protocols
available = registry.list_available_protocols()
print(f"Available: {available}")

# Unregister provider
registry.unregister_provider(ProtocolType.SHARED_MEMORY)
```

### Methods

**`register_provider(protocol_type: ProtocolType, provider: AbstractProtocolProvider)`**
- Register a provider for specific protocol
- Raises `ValueError` if provider already registered

**`unregister_provider(protocol_type: ProtocolType)`**
- Remove provider from registry
- Calls `shutdown()` on provider

**`get_provider(protocol_type: ProtocolType) -> Optional[AbstractProtocolProvider]`**
- Get provider for protocol
- Returns `None` if not registered

**`is_protocol_available(protocol_type: ProtocolType) -> bool`**
- Check if protocol is available and initialized
- Returns `False` if provider not registered

**`list_available_protocols() -> List[ProtocolType]`**
- Get list of all registered and available protocols

**`clear()`**
- Remove all providers
- Calls `shutdown()` on all providers

## Implementing a Provider

### Basic Structure

```python
from vyra_base.com.providers import AbstractProtocolProvider
from vyra_base.com.core.types import ProtocolType
from vyra_base.com.core.exceptions import ProviderError

class MyProtocolProvider(AbstractProtocolProvider):
    """Custom protocol provider"""
    
    def __init__(self, host: str, port: int):
        super().__init__(ProtocolType.CUSTOM)  # Add to ProtocolType enum
        self._host = host
        self._port = port
        self._connection = None
    
    async def initialize(self) -> None:
        """Setup connection"""
        if self._initialized:
            return
        
        try:
            self._connection = await connect(self._host, self._port)
            self._initialized = True
        except Exception as e:
            raise ProviderError(f"Failed to initialize: {e}")
    
    async def shutdown(self) -> None:
        """Close connection"""
        if self._connection:
            await self._connection.close()
            self._connection = None
        self._initialized = False
    
    async def is_available(self) -> bool:
        """Check if connection is alive"""
        if not self._initialized or not self._connection:
            return False
        return await self._connection.ping()
    
    async def create_callable(self, name: str, callback=None, **kwargs):
        """Create RPC interface"""
        if not self._initialized:
            raise ProviderError("Provider not initialized")
        
        return MyCallable(self._connection, name, callback)
    
    async def create_speaker(self, name: str, callback=None, **kwargs):
        """Create pub/sub interface"""
        if not self._initialized:
            raise ProviderError("Provider not initialized")
        
        return MySpeaker(self._connection, name, callback)
    
    async def create_job(self, name: str, execute_callback=None, **kwargs):
        """Jobs not supported"""
        raise NotImplementedError("Jobs not supported by this protocol")
```

### Callable Implementation

```python
from vyra_base.com.core.types import VyraCallable
from vyra_base.com.core.exceptions import CallableError

class MyCallable(VyraCallable):
    """Custom protocol callable"""
    
    def __init__(self, connection, name: str, callback=None):
        self._connection = connection
        self._name = name
        self._callback = callback
        self._initialized = False
    
    async def initialize(self) -> None:
        if self._initialized:
            return
        
        if self._callback:
            # Server mode - register handler
            await self._connection.register_handler(self._name, self._callback)
        
        self._initialized = True
    
    async def shutdown(self) -> None:
        if self._callback:
            await self._connection.unregister_handler(self._name)
        self._initialized = False
    
    async def call(self, request: Dict[str, Any], timeout: float = 5.0) -> Dict[str, Any]:
        """Make RPC call"""
        if not self._initialized:
            raise CallableError("Callable not initialized")
        
        try:
            response = await self._connection.call(
                self._name,
                request,
                timeout=timeout
            )
            return response
        except TimeoutError:
            raise CallableError(f"Call to '{self._name}' timed out")
        except Exception as e:
            raise CallableError(f"Call failed: {e}")
```

### Speaker Implementation

```python
from vyra_base.com.core.types import VyraSpeaker
from vyra_base.com.core.exceptions import SpeakerError

class MySpeaker(VyraSpeaker):
    """Custom protocol speaker"""
    
    def __init__(self, connection, name: str, callback=None):
        self._connection = connection
        self._name = name
        self._callback = callback
        self._initialized = False
    
    async def initialize(self) -> None:
        if self._initialized:
            return
        
        if self._callback:
            # Subscribe mode
            await self._connection.subscribe(self._name, self._callback)
        
        self._initialized = True
    
    async def shutdown(self) -> None:
        if self._callback:
            await self._connection.unsubscribe(self._name)
        self._initialized = False
    
    async def shout(self, message: Dict[str, Any]) -> None:
        """Publish message"""
        if not self._initialized:
            raise SpeakerError("Speaker not initialized")
        
        try:
            await self._connection.publish(self._name, message)
        except Exception as e:
            raise SpeakerError(f"Publish failed: {e}")
```

## Testing with Mock Providers

### Mock Provider

```python
from vyra_base.com.providers import AbstractProtocolProvider
from vyra_base.com.core.types import ProtocolType

class MockProvider(AbstractProtocolProvider):
    """Mock provider for testing"""
    
    def __init__(self):
        super().__init__(ProtocolType.MOCK)
        self.calls = []
        self.messages = []
    
    async def initialize(self) -> None:
        self._initialized = True
    
    async def shutdown(self) -> None:
        self._initialized = False
    
    async def is_available(self) -> bool:
        return True
    
    async def create_callable(self, name, callback=None, **kwargs):
        return MockCallable(name, callback, self.calls)
    
    async def create_speaker(self, name, callback=None, **kwargs):
        return MockSpeaker(name, self.messages)
    
    async def create_job(self, name, execute_callback=None, **kwargs):
        return MockJob(name, execute_callback)

class MockCallable:
    def __init__(self, name, callback, calls_list):
        self.name = name
        self.callback = callback
        self.calls_list = calls_list
    
    async def initialize(self): pass
    async def shutdown(self): pass
    
    async def call(self, request, timeout=5.0):
        self.calls_list.append((self.name, request))
        if self.callback:
            return await self.callback(request)
        return {"mock": "response"}

class MockSpeaker:
    def __init__(self, name, messages_list):
        self.name = name
        self.messages_list = messages_list
    
    async def initialize(self): pass
    async def shutdown(self): pass
    
    async def shout(self, message):
        self.messages_list.append((self.name, message))
```

### Test Example

```python
import pytest
from vyra_base.com.core.factory import InterfaceFactory
from vyra_base.com.providers import ProviderRegistry

@pytest.fixture
async def mock_env():
    """Setup mock environment"""
    mock_provider = MockProvider()
    await mock_provider.initialize()
    
    registry = ProviderRegistry()
    registry.register_provider(ProtocolType.MOCK, mock_provider)
    
    # Set factory to use mock
    InterfaceFactory.set_fallback_chain("callable", [ProtocolType.MOCK])
    
    yield mock_provider
    
    # Cleanup
    registry.clear()
    InterfaceFactory.reset_providers()

@pytest.mark.asyncio
async def test_callable(mock_env):
    """Test callable with mock provider"""
    callable = await InterfaceFactory.create_callable(
        "test_service",
        callback=lambda req: {"result": req["value"] * 2}
    )
    
    result = await callable.call({"value": 21})
    assert result == {"result": 42}
    
    # Verify mock recorded call
    assert len(mock_env.calls) == 1
    assert mock_env.calls[0][0] == "test_service"

@pytest.mark.asyncio
async def test_speaker(mock_env):
    """Test speaker with mock provider"""
    speaker = await InterfaceFactory.create_speaker("test_topic")
    
    await speaker.shout({"event": "test"})
    
    # Verify mock recorded message
    assert len(mock_env.messages) == 1
    assert mock_env.messages[0][1] == {"event": "test"}
```

## Provider Lifecycle

### Initialization

```python
# 1. Create provider
provider = MyProtocolProvider(host="localhost", port=1234)

# 2. Initialize resources
await provider.initialize()

# 3. Register with registry
registry = ProviderRegistry()
registry.register_provider(ProtocolType.CUSTOM, provider)

# 4. Use via InterfaceFactory
callable = await InterfaceFactory.create_callable(
    "service",
    protocols=[ProtocolType.CUSTOM]
)
```

### Cleanup

```python
# Automatic cleanup via registry
registry.unregister_provider(ProtocolType.CUSTOM)

# Manual cleanup
await provider.shutdown()
```

### Context Manager Support

```python
async with MyProtocolProvider(host="localhost", port=1234) as provider:
    callable = await provider.create_callable("service")
    result = await callable.call({"data": 123})
# Auto-cleanup on exit
```

## Best Practices

### 1. Initialize Once, Use Many Times
```python
# ✅ Good - Reuse provider
provider = SharedMemoryProvider()
await provider.initialize()
callable1 = await provider.create_callable("service1")
callable2 = await provider.create_callable("service2")

# ❌ Avoid - Multiple provider instances
provider1 = SharedMemoryProvider()
await provider1.initialize()
callable1 = await provider1.create_callable("service1")

provider2 = SharedMemoryProvider()
await provider2.initialize()
callable2 = await provider2.create_callable("service2")
```

### 2. Always Clean Up
```python
# ✅ Good - Proper cleanup
try:
    provider = MyProtocolProvider()
    await provider.initialize()
    # Use provider
finally:
    await provider.shutdown()
```

### 3. Check Availability
```python
# ✅ Good - Check before use
if await provider.is_available():
    callable = await provider.create_callable("service")
else:
    logger.error("Provider not available")
```

### 4. Use Registry for Global Access
```python
# ✅ Good - Central registry
registry = ProviderRegistry()
registry.register_provider(ProtocolType.REDIS, redis_provider)

# Anywhere in code
provider = registry.get_provider(ProtocolType.REDIS)
```

## See Also

- [Core Components](../core/README.md) - Factory and decorators
- [Transport Layer](../transport/README.md) - Built-in transport providers
- [External Layer](../external/README.md) - Built-in external providers
- [Main README](../README.md) - Architecture overview
