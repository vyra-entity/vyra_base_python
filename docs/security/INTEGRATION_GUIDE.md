# VYRA Security Framework - Integration Guide

Quick reference for integrating VYRA security into your module.

## Prerequisites

```bash
pip install cryptography
```

## 1. Add SafetyMetadata to Your Messages

### For Custom Messages

```
# MyMessage.msg
string data
int32 value
vyra_base/SafetyMetadata safety_metadata
```

### For Custom Services

```
# MyService.srv

# Request
string parameter
vyra_base/SafetyMetadata safety_metadata
---
# Response
bool success
string message
```

Rebuild interfaces:
```bash
colcon build --packages-select your_interfaces_package
```

## 2. Server Module (Provides Security)

### Step 1: Inherit SecurityManager

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

### Step 2: Add RequestAccess Service

```python
from vyra_base.interfaces.srv import VBASERequestAccess

self.access_service = self.create_service(
    VBASERequestAccess,
    'my_module/request_access',
    self.handle_access_callback
)
```

### Step 3: Implement Callback

```python
import asyncio

def handle_access_callback(self, request, response):
    loop = asyncio.new_event_loop()
    result = loop.run_until_complete(
        self.handle_request_access(
            request.module_name,
            request.module_id,
            request.requested_role,
            request.requested_sl,
            request.certificate_csr
        )
    )
    loop.close()
    
    (success, message, token, hmac, cert, expires, granted) = result
    
    response.success = success
    response.message = message
    response.session_token = token
    response.hmac_key = hmac
    response.certificate = cert
    response.expires_at.sec = int(expires.timestamp())
    response.granted_sl = granted
    
    return response
```

### Step 4: Validate Incoming Messages

```python
from vyra_base.security.security_validator import SecurityValidator

# In __init__:
self.validator = SecurityValidator(self, strict_mode=True)

# In service callback:
def my_service(self, request, response):
    try:
        self.validator.validate_metadata(
            request.safety_metadata,
            required_level=SecurityLevel.HMAC
        )
        # Process request
    except SecurityError as e:
        response.success = False
        response.message = str(e)
    return response
```

## 3. Client Module (Consumes Security)

### Step 1: Request Access

```python
import uuid
from vyra_base.interfaces.srv import VBASERequestAccess
from unique_identifier_msgs.msg import UUID as UUIDMsg

# Create UUID
module_uuid = uuid.uuid4()

# Call service
client = node.create_client(VBASERequestAccess, 'server/request_access')
client.wait_for_service(timeout_sec=10.0)

request = VBASERequestAccess.Request()
request.module_name = "my_client"
request.module_id = UUIDMsg(uuid=list(module_uuid.bytes))
request.requested_role = "operator"
request.requested_sl = 4  # HMAC

response = await client.call_async(request)
```

### Step 2: Create Security Context

```python
from vyra_base.security.security_client import create_security_context

security_ctx = create_security_context(
    module_id=str(module_uuid),
    security_level=response.granted_sl,
    session_token=response.session_token,
    hmac_key=response.hmac_key
)
```

### Step 3: Use Secure Communication

```python
from vyra_base.security.security_client import SecurePublisher, SecureServiceClient

# For publishers:
pub = SecurePublisher(node, MyMessage, 'topic', security_ctx)
pub.publish(msg)  # SafetyMetadata added automatically

# For service clients:
client = SecureServiceClient(node, MyService, 'service', security_ctx)
response = await client.call_async(request)
```

## 4. Integration with VyraEntity

### In _base_.py

```python
from vyra_base.security.security_manager import SecurityManager
from vyra_base.security.security_levels import SecurityLevel

class MyVyraModule(VyraEntity, SecurityManager):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        SecurityManager.__init__(
            self,
            max_security_level=SecurityLevel.HMAC,
            session_duration_seconds=3600
        )
        
        # Add RequestAccess to interfaces
        self._setup_security_service()
    
    def _setup_security_service(self):
        """Add security service to module interfaces."""
        # Service is auto-registered via vyra_security_meta.json
        pass
```

### In application.py

```python
from vyra_base.security.security_validator import SecurityValidator

class Application(OperationalStateMachine):
    def __init__(self, entity, *args, **kwargs):
        super().__init__(entity.unified_state_machine)
        
        # Create validator if entity has SecurityManager
        if hasattr(entity, 'get_session'):
            self.validator = SecurityValidator(entity, strict_mode=True)
    
    @remote_callable
    def secure_operation(self, request=None, response=None):
        """Example secure service."""
        try:
            if hasattr(request, 'safety_metadata'):
                self.validator.validate_metadata(
                    request.safety_metadata,
                    required_level=SecurityLevel.HMAC
                )
            
            # Process request
            # ...
            
        except SecurityError as e:
            response['success'] = False
            response['message'] = str(e)
        
        return response
```

## 5. Configuration

### Environment Variables

Add to `.env`:
```bash
# Security Configuration
SECURITY_LEVEL=4
SESSION_DURATION=3600
CA_CERT_PATH=/workspace/storage/certificates/ca_cert.pem
CA_KEY_PATH=/workspace/storage/certificates/ca_key.pem
```

### Load in Module

```python
import os

security_level = int(os.getenv('SECURITY_LEVEL', '3'))
session_duration = int(os.getenv('SESSION_DURATION', '3600'))
```

## 6. Testing

### Unit Test Example

```python
import pytest
from vyra_base.security.security_manager import SecurityManager
from vyra_base.security.security_levels import SecurityLevel

@pytest.mark.asyncio
async def test_request_access():
    manager = SecurityManager(max_security_level=SecurityLevel.HMAC)
    
    result = await manager.handle_request_access(
        module_name="test_client",
        module_id=uuid.uuid4(),
        requested_role="test",
        requested_sl=4,
        certificate_csr=""
    )
    
    success, message, token, hmac, cert, expires, granted = result
    
    assert success is True
    assert granted == 4
    assert len(token) > 0
    assert len(hmac) > 0
```

### Integration Test

```python
def test_secure_communication():
    # Start server
    server = SecureServerNode()
    
    # Start client
    client = SecureClientNode()
    
    # Client authenticates
    assert client.authenticate_with_server() is True
    
    # Client publishes secure message
    msg = MyMessage()
    msg.data = "test"
    client.secure_pub.publish(msg)
    
    # Verify server receives and validates
    # ...
```

## 7. Docker Integration

### Dockerfile

```dockerfile
# Install cryptography
RUN pip install cryptography

# Copy certificates (if using Level 5)
COPY storage/certificates /workspace/storage/certificates
RUN chmod 600 /workspace/storage/certificates/*.pem
```

### docker-compose.yml

```yaml
services:
  my_module:
    environment:
      - SECURITY_LEVEL=4
      - SESSION_DURATION=3600
    volumes:
      - ./storage/certificates:/workspace/storage/certificates:ro
```

## Quick Reference

### Security Levels
- **1**: None (no checks)
- **2**: Basic Auth (ID only)
- **3**: Extended Auth (ID + password)
- **4**: HMAC (signatures)
- **5**: Digital Signature (certificates)

### Common Errors

| Error | Cause | Solution |
|-------|-------|----------|
| "HMAC signature mismatch" | Wrong key | Verify HMAC key |
| "Timestamp drift too large" | Clock skew | Sync system time |
| "Session expired" | Token old | Re-authenticate |
| "Missing SafetyMetadata" | Wrong message type | Add field to message |

### Performance Impact

| Level | Latency | Use For |
|-------|---------|---------|
| 1-2   | <1ms    | High frequency |
| 3     | ~1ms    | Standard |
| 4     | 1-3ms   | Sensitive |
| 5     | 5-10ms  | Critical |

## Complete Example

See `/examples/security/` for full working examples:
- `secure_server_node.py` - Server implementation
- `secure_client_node.py` - Client implementation

## Next Steps

1. Read full documentation: [SECURITY_FRAMEWORK.md](SECURITY_FRAMEWORK.md)
2. Review examples: `/examples/security/`
3. Test integration with your module
4. Consider Level 5 for critical operations

---

**Quick Help:**
- For questions: Check [SECURITY_FRAMEWORK.md](SECURITY_FRAMEWORK.md)
- For bugs: Check troubleshooting section
- For API reference: See generated Sphinx docs
