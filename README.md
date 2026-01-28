# vyra_base_python - VYRA Core Library

[![Documentation](https://img.shields.io/badge/docs-sphinx-blue)](https://vyra-entity.github.io/vyra_base_python/)
[![Python Version](https://img.shields.io/badge/python-3.11+-blue)](https://www.python.org/downloads/)
[![ROS2](https://img.shields.io/badge/ROS2-Kilted-blue)](https://docs.ros.org/en/kilted/index.html)

**Core Python library for building VYRA modules** - Provides state management, ROS2 communication, storage, and security primitives.

## 📋 Overview

`vyra_base_python` is the foundational library for all VYRA modules, providing:

- **Unified State Machine**: 3-layer state management (Lifecycle, Operational, Health)
- **ROS2 Integration**: Service/topic communication with SROS2 security
- **Storage Layer**: Redis client with TLS support
- **gRPC Communication**: Inter-process communication primitives
- **Security Framework**: Certificate management and validation
- **Parameter System**: Runtime configuration and validation

## 🚀 Quick Start

### Installation

```bash
# Install from source (development)
cd vyra_base_python
pip install -e .

# Install from wheel (production)
pip install vyra_base-<version>-py3-none-any.whl
```

### Basic Usage

```python
from vyra_base.state import OperationalStateMachine
from vyra_base.com.datalayer.interface_factory import remote_callable

class MyComponent(OperationalStateMachine):
    """Example VYRA component with automatic state management."""
    
    @remote_callable
    def initialize(self, request=None, response=None) -> bool:
        """Initialize component - state transition handled automatically."""
        # Your initialization logic here
        return True
    
    @remote_callable
    def start_operation(self, request=None, response=None) -> bool:
        """Start operation - transitions to RUNNING state."""
        # Your operation logic here
        return True

# Use the component
component = MyComponent()
component.initialize()  # IDLE → READY
component.start_operation()  # READY → RUNNING
```

## 📚 Documentation

### Core Documentation

- **[Full API Documentation](https://vyra-entity.github.io/vyra_base_python/)** - Complete Sphinx documentation
- **[State Machine Guide](docs/statemachine/README.md)** - 3-layer state management
- **[Security Framework](docs/security/SECURITY_FRAMEWORK.md)** - Security implementation
- **[Translation Guide](docs/TRANSLATION_GUIDE.md)** - Multilingual documentation

### Key Concepts

#### 1. State Management

The Unified State Machine provides three coordinated layers:

```python
from vyra_base.state import UnifiedStateMachine

usm = UnifiedStateMachine()
usm.start()                     # Lifecycle: Offline → Initializing
usm.complete_initialization()   # Lifecycle: Initializing → Active
usm.set_ready()                 # Operational: Idle → Ready
usm.start_task({'id': '123'})  # Operational: Ready → Running

# Query states
states = usm.get_all_states()
print(states)  # {'lifecycle': 'Active', 'operational': 'Running', 'health': 'Healthy'}
```

See [State Machine Documentation](docs/statemachine/README.md) for details.

#### 2. ROS2 Communication

```python
from vyra_base.core import VyraEntity

# Create entity with ROS2 node
entity = await VyraEntity.create("my_module")

# Publish news
entity.publish_news("Module started")

# Call remote service
result = await entity.call_service('other_module/service_name', request_data)
```

#### 3. Redis Storage

```python
from vyra_base.storage.redis_client import RedisClient

redis = RedisClient(
    host="redis",
    port=6379,
    ssl=True,
    ssl_ca_certs="/workspace/storage/certificates/redis/ca-cert.pem"
)

await redis.set("key", "value")
value = await redis.get("key")
```

## 🏗️ Architecture

### Module Structure

```python
# Typical module structure using vyra_base

from vyra_base.core import VyraEntity
from vyra_base.state import OperationalStateMachine

async def build_entity(settings):
    """Create VyraEntity with ROS2 node."""
    entity = await VyraEntity.create(settings.module_name)
    
    # Register interfaces (creates ROS2 services)
    base_interfaces = await create_base_interfaces()
    await entity.set_interfaces(base_interfaces)
    
    return entity

class Application(OperationalStateMachine):
    """Application logic with state management."""
    
    def __init__(self, entity: VyraEntity):
        super().__init__()
        self.entity = entity
    
    @remote_callable
    def initialize(self) -> bool:
        # Initialization logic
        return True

async def main():
    entity = await build_entity(settings)
    app = Application(entity)
    app.initialize()
    # ...
```

## 🔧 Development

### Setup Development Environment

```bash
# Clone repository
cd vyra_base_python

# Install development dependencies
pip install -e ".[dev]"

# Run tests
pytest -v

# Generate documentation
cd docs
./build_multilingual.sh
```

### Running Tests

```bash
# Unit tests (fast, no external dependencies)
pytest -m unit

# Integration tests (requires Redis, ROS2)
pytest -m integration

# End-to-end tests
pytest -m e2e

# With coverage
pytest --cov=vyra_base --cov-report=html
```

## 📦 Package Structure

```
vyra_base/
├── com/                    # Communication layer
│   ├── datalayer/         # ROS2 interfaces
│   ├── feeder/            # Data feeders
│   └── ros2_service.py    # ROS2 service utilities
├── core/                   # Core primitives
│   ├── entity.py          # VyraEntity (main interface)
│   ├── parameter.py       # Parameter management
│   └── volatile.py        # Volatile storage
├── state/                  # State machine
│   ├── unified.py         # UnifiedStateMachine
│   ├── operational_state_machine.py  # Metaclass for modules
│   ├── lifecycle_layer.py
│   ├── operational_layer.py
│   └── health_layer.py
├── storage/               # Storage backends
│   └── redis_client.py   # Redis with TLS
├── helper/                # Utilities
│   └── error_traceback.py
└── security/              # Security framework
```

## 🔐 Security

Security features:

- **SROS2 Integration**: Certificate-based ROS2 security
- **TLS Support**: Encrypted Redis communication
- **Certificate Management**: Automatic cert generation and validation
- **Access Control**: Role-based permissions

See [Security Documentation](docs/security/SECURITY_FRAMEWORK.md).

## 🌍 Multilingual Documentation

Documentation available in:

- **English** (default): https://vyra-entity.github.io/vyra_base_python/en/
- **Deutsch**: https://vyra-entity.github.io/vyra_base_python/de/

See [Translation Guide](docs/TRANSLATION_GUIDE.md) for contributing translations.

## 📝 Versioning

See [VERSIONING.md](VERSIONING.md) and [CHANGELOG.md](CHANGELOG.md).

## 📄 License

See [LICENSE](LICENSE) file.

## 🏢 Maintainer

**Variobotic GmbH**

Internal project for VYRA framework development.

