# VYRA Security Framework

Comprehensive 5-level security framework for inter-module communication in the VYRA ecosystem.

## Features

- **5 Security Levels**: From no security to certificate-based PKI
- **Automatic Metadata**: Transparent SafetyMetadata handling for messages
- **Session Management**: Token-based authentication with configurable expiration
- **Replay Protection**: Timestamp and nonce validation
- **HMAC Signatures**: Message integrity with HMAC-SHA256 (Level 4)
- **PKI Support**: Full certificate infrastructure (Level 5)
- **Easy Integration**: Mixin-based design for simple module integration

## Security Levels

| Level | Name | Features | Use Case |
|-------|------|----------|----------|
| 1 | NONE | No security | Public information |
| 2 | BASIC_AUTH | Module ID verification | Basic access control |
| 3 | TIMESTAMP | ID + timestamp | Replay attack prevention |
| 4 | HMAC | HMAC-SHA256 signatures | Message integrity |
| 5 | DIGITAL_SIGNATURE | Certificate-based PKI | Maximum security |

## Quick Start

### Server Module

```python
from vyra_base.security.security_manager import SecurityManager
from vyra_base.security.security_levels import SecurityLevel

class MyModule(Node, SecurityManager):
    def __init__(self):
        Node.__init__(self, 'my_module')
        SecurityManager.__init__(
            self,
            max_security_level=SecurityLevel.HMAC
        )
```

### Client Module

```python
from vyra_base.security.security_client import (
    create_security_context,
    SecurePublisher
)

# 1. Authenticate
response = await access_client.call_async(request)

# 2. Create context
ctx = create_security_context(
    module_id=str(uuid),
    security_level=response.granted_sl,
    session_token=response.session_token,
    hmac_key=response.hmac_key
)

# 3. Use secure communication
pub = SecurePublisher(node, MsgType, 'topic', ctx)
pub.publish(msg)  # SafetyMetadata added automatically
```

## Installation

```bash
# Install dependencies
pip install cryptography

# Build interfaces
cd /path/to/vyra_base_python
colcon build
source install/setup.bash
```

## Documentation

- **[Security Framework Guide](docs/security/SECURITY_FRAMEWORK.md)** - Complete documentation
- **[Integration Guide](docs/security/INTEGRATION_GUIDE.md)** - Quick integration reference
- **[Sphinx Docs](docs/security.rst)** - API reference
- **[Examples](examples/security/)** - Working examples

## Architecture

```
┌─────────────────────────────────────────┐
│         Module A (Client)               │
│  ┌────────────┐      ┌──────────────┐   │
│  │  Security  │ ───> │   Secure     │   │
│  │  Context   │      │  Publisher   │   │
│  └────────────┘      └──────────────┘   │
└─────────────────────────────────────────┘
              │
              │ ROS2 (with SafetyMetadata)
              ▼
┌─────────────────────────────────────────┐
│         Module B (Server)               │
│  ┌────────────┐      ┌──────────────┐   │
│  │  Security  │ <─── │  Security    │   │
│  │  Manager   │      │  Validator   │   │
│  └────────────┘      └──────────────┘   │
└─────────────────────────────────────────┘
```

## Components

### SecurityManager (Server-side)
- Provides `RequestAccess` service
- Manages sessions and tokens
- Issues HMAC keys and certificates
- Validates authentication requests

### SecurityClient (Client-side)
- Requests authentication
- Wraps publishers/clients
- Generates SafetyMetadata
- Signs messages automatically

### SecurityValidator (Server-side)
- Validates incoming SafetyMetadata
- Checks signatures and timestamps
- Tracks nonces (replay protection)
- Enforces security levels

### Crypto Helpers
- HMAC-SHA256 operations
- RSA key management
- Certificate operations
- Secure random generation

## Message Flow

```
1. Client → Server: RequestAccess(module_id, requested_sl)
2. Server → Client: Response(token, hmac_key, granted_sl)
3. Client creates SecurityContext
4. Client → Server: Message with SafetyMetadata (signed)
5. Server validates SafetyMetadata
6. Server processes message if valid
```

## Usage Examples

### Access Level Configuration in Interface JSON

Each ROS2 service defined in `src/*/interfaces/config/*.json` must specify an `access_level` (1-5):

```json
{
    "functionname": "set_param",
    "displayname": "Set Parameter",
    "description": "Set a parameter in the system",
    "filetype": "VBASESetParam.srv",
    "access_level": 3,  // Requires EXTENDED_AUTH or higher
    "params": [...],
    "returns": [...]
}
```

**Access Level Mapping:**
- `1` = NONE - Public access
- `2` = BASIC_AUTH - Module ID verification  
- `3` = EXTENDED_AUTH - ID + password
- `4` = HMAC - Message integrity check
- `5` = DIGITAL_SIGNATURE - Full PKI

### Using @security_required Decorator

The `@security_required` decorator enforces access levels defined in interface configs:

```python
from vyra_base.com.datalayer.interface_factory import remote_callable
from vyra_base.security import security_required, SecurityLevel

class Application(OperationalStateMachine):
    def __init__(self, entity: VyraEntity, *args, **kwargs):
        super().__init__(entity.unified_state_machine)
        self.entity = entity
    
    @remote_callable
    @security_required(security_level=SecurityLevel.EXTENDED_AUTH)  # Level 3
    def set_param(self, request=None, response=None):
        """
        Set parameter - requires Level 3 access or higher.
        Matches access_level: 3 in vyra_core_meta.json
        """
        # Decorator validates:
        # 1. SafetyMetadata exists
        # 2. Valid session token
        # 3. Module granted level >= 3
        
        response.success = True
        response.message = f"Parameter {request.key} set to {request.value}"
        return response
    
    @remote_callable
    @security_required(security_level=SecurityLevel.HMAC)  # Level 4
    def critical_operation(self, request=None, response=None):
        """
        Critical operation - requires Level 4 (HMAC) or higher.
        """
        response.success = True
        return response
```

### Session Registration and Access Control

When a module authenticates via `RequestAccess`, it receives a granted security level:

```python
# Client requests access
request = VBASERequestAccess.Request()
request.module_name = "client_module"
request.module_id = module_uuid
request.requested_sl = 4  # Request HMAC level

# Server grants access (via SecurityManager)
response = await access_client.call_async(request)
# response.granted_sl = 4 (could be lower if max_level < 4)
# response.session_token = "abc123..."

# Session is stored with granted_sl = 4
# Now client can call services requiring level 1-4
```

**Access Control Flow:**
1. Module requests access with `requested_sl`
2. SecurityManager validates credentials and grants `granted_sl`
3. Session stored: `{module_id: session_token, granted_sl: 4}`
4. Service call includes `session_token` in SafetyMetadata
5. `@security_required` checks: `granted_sl >= required_level`
6. Access granted or denied

### Basic Authentication

```python
# Server: Handle access requests
async def handle_request_access(self, request):
    result = await self.handle_request_access(
        request.module_name,
        request.module_id,
        request.requested_role,
        request.requested_sl,
        request.certificate_csr
    )
    return result

# Client: Request access
response = await client.call_async(request)
if response.success:
    ctx = create_security_context(...)
```

### Secure Publishing

```python
from vyra_base.security.security_client import SecurePublisher

pub = SecurePublisher(node, MsgType, 'topic', security_context)
msg = MsgType()
msg.data = "Hello"
pub.publish(msg)  # Automatic signing
```

### Message Validation

```python
from vyra_base.security.security_validator import SecurityValidator

validator = SecurityValidator(security_manager)

def callback(self, request, response):
    try:
        validator.validate_metadata(
            request.safety_metadata,
            required_level=SecurityLevel.HMAC
        )
        # Process request
    except SecurityError as e:
        response.success = False
        response.message = str(e)
    return response
```

## Performance

| Level | CPU Overhead | Latency | Recommended For |
|-------|-------------|---------|-----------------|
| 1-2   | <1%         | <1ms    | High frequency |
| 3     | ~1%         | ~1ms    | Standard ops |
| 4     | 2-5%        | 1-3ms   | Sensitive data |
| 5     | 10-20%      | 5-10ms  | Critical ops |

## Testing

```python
import pytest
from vyra_base.security.security_manager import SecurityManager

@pytest.mark.asyncio
async def test_authentication():
    manager = SecurityManager(max_security_level=SecurityLevel.HMAC)
    
    result = await manager.handle_request_access(
        "test_client", uuid.uuid4(), "test", 4, ""
    )
    
    assert result[0] is True  # success
    assert result[6] == 4     # granted_sl
```

## Integration with VYRA Modules

### Using VyraEntity (Recommended)

As of version 1.0.0, `VyraEntity` supports optional SecurityManager integration through module configuration.

**Configuration in module's `_base_.py`:**

```python
from vyra_base.security import SecurityLevel

module_config = {
    'security': {
        'enabled': True,
        'max_level': SecurityLevel.HMAC,  # 4
        'session_duration': 3600,  # seconds
        'module_passwords': {
            'client_module_id': 'password123'  # for Level 3+
        },
        # For Level 5 (optional)
        'ca_key_path': '/workspace/storage/certificates/ca_key.pem',
        'ca_cert_path': '/workspace/storage/certificates/ca_cert.pem'
    }
}

entity = VyraEntity(
    state_entry=state_entry,
    news_entry=news_entry,
    error_entry=error_entry,
    module_entry=module_entry,
    module_config=module_config
)

# Access security manager
if entity.security_manager:
    print(f"Security enabled: Level {entity.security_manager.max_security_level}")
```

**Using SecurityManager in components:**

```python
from vyra_base.security import SecurityValidator, SecurityError

class Application(OperationalStateMachine):
    def __init__(self, entity: VyraEntity, *args, **kwargs):
        super().__init__(entity.unified_state_machine)
        self.entity = entity
        
        # Create validator if security is enabled
        self.validator = None
        if entity.security_manager:
            self.validator = SecurityValidator(entity.security_manager)
    
    @remote_callable
    def secure_operation(self, request=None, response=None):
        # Validate request if security is enabled
        if self.validator and hasattr(request, 'safety_metadata'):
            try:
                self.validator.validate_metadata(
                    request.safety_metadata,
                    required_level=SecurityLevel.HMAC
                )
            except SecurityError as e:
                response.success = False
                response.message = f"Security validation failed: {e}"
                return response
        
        # Process request...
        response.success = True
        return response
```

### Legacy Integration (deprecated)

For modules using SecurityManager as mixin:

```python
from vyra_base.security.security_manager import SecurityManager

class MyVyraModule(VyraEntity, SecurityManager):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        SecurityManager.__init__(
            self,
            max_security_level=SecurityLevel.HMAC
        )
```

### In your module's `application.py`:

```python
from vyra_base.security.security_validator import SecurityValidator

class Application(OperationalStateMachine):
    def __init__(self, entity, *args, **kwargs):
        super().__init__(entity.unified_state_machine)
        
        if hasattr(entity, 'get_session'):
            self.validator = SecurityValidator(entity)
    
    @remote_callable
    def secure_operation(self, request=None, response=None):
        try:
            if hasattr(request, 'safety_metadata'):
                self.validator.validate_metadata(
                    request.safety_metadata,
                    required_level=SecurityLevel.HMAC
                )
            # Process...
        except SecurityError as e:
            response['success'] = False
            response['message'] = str(e)
        return response
```

## Configuration

Environment variables:

```bash
# .env
SECURITY_LEVEL=4
SESSION_DURATION=3600
CA_CERT_PATH=/workspace/storage/certificates/ca_cert.pem
CA_KEY_PATH=/workspace/storage/certificates/ca_key.pem
```

## Common Issues

| Error | Cause | Solution |
|-------|-------|----------|
| "HMAC signature mismatch" | Wrong key | Verify HMAC key |
| "Timestamp drift too large" | Clock skew | Sync system time |
| "Session expired" | Token old | Re-authenticate |
| "Missing SafetyMetadata" | Wrong msg type | Add field to message |

## Dependencies

- Python 3.8+
- ROS2 (Humble or later)
- cryptography >= 3.4
- rclpy

## Contributing

See the main VYRA documentation for contribution guidelines.

## License

Proprietary - Variobotic GmbH

## Support

- Documentation: [docs/security/](docs/security/)
- Examples: [examples/security/](examples/security/)
- API Reference: Build Sphinx docs with `make html`

## Version

1.0.0 (2026-01-19)
