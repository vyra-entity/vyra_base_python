# VYRA Security Framework - Quick Reference Card

## Import Statements

```python
# All-in-one import
from vyra_base.security import (
    SecurityLevel,
    SecurityManager,
    SecurityValidator,
    SecurePublisher,
    create_security_context
)

# Or individual imports
from vyra_base.security.security_manager import SecurityManager
from vyra_base.security.security_client import SecurePublisher
from vyra_base.security.security_validator import SecurityValidator
from vyra_base.security.security_levels import SecurityLevel
```

## Security Levels

| Level | Name | Validation | Performance | Use Case |
|-------|------|------------|-------------|----------|
| 1 | NONE | None | Instant | Public data |
| 2 | BASIC_AUTH | Module ID | <1ms | Basic access |
| 3 | EXTENDED_AUTH | ID + Password | ~1ms | Standard |
| 4 | HMAC | HMAC-SHA256 | 1-3ms | Sensitive |
| 5 | DIGITAL_SIGNATURE | Certificates | 5-10ms | Critical |

## Server Setup (3 lines)

```python
class MyModule(Node, SecurityManager):
    def __init__(self):
        SecurityManager.__init__(self, max_security_level=SecurityLevel.HMAC)
```

## Client Setup (3 steps)

```python
# 1. Request access
response = await access_client.call_async(request)

# 2. Create context
ctx = create_security_context(
    module_id=str(uuid),
    security_level=response.granted_sl,
    session_token=response.session_token,
    hmac_key=response.hmac_key
)

# 3. Use secure publisher
pub = SecurePublisher(node, MsgType, 'topic', ctx)
pub.publish(msg)
```

## Message Definition

```
# YourMessage.msg
string data
int32 value
vyra_base/SafetyMetadata safety_metadata
```

## Validation (Server)

```python
validator = SecurityValidator(self, strict_mode=True)

def callback(self, request, response):
    try:
        validator.validate_metadata(
            request.safety_metadata,
            required_level=SecurityLevel.HMAC
        )
        # Process
    except SecurityError as e:
        response.success = False
    return response
```

## RequestAccess Service

### Request
```python
request.module_name = "my_client"
request.module_id = UUID(...)
request.requested_role = "operator"
request.requested_sl = 4
request.certificate_csr = ""  # For Level 5
```

### Response
```python
response.success  # bool
response.message  # str
response.session_token  # str
response.hmac_key  # str (Level 4+)
response.certificate  # str (Level 5)
response.expires_at  # Time
response.granted_sl  # uint8
```

## SafetyMetadata Fields

```python
metadata.security_level  # uint8 (1-5)
metadata.sender_id  # string (UUID)
metadata.sender_name  # Module name
metadata.timestamp  # Time
metadata.nonce  # uint64
metadata.security_payload  # string (signature)
metadata.algorithm_id  # string (e.g., "HMAC-SHA256")
```

## Common Patterns

### Server with Validation
```python
class SecureServer(Node, SecurityManager):
    def __init__(self):
        Node.__init__(self, 'server')
        SecurityManager.__init__(self, max_security_level=SecurityLevel.HMAC)
        self.validator = SecurityValidator(self)
```

### Client with Publisher
```python
class SecureClient(Node):
    def authenticate_and_setup(self):
        # Authenticate
        response = await self.request_access()
        # Create context
        self.ctx = create_security_context(...)
        # Create publisher
        self.pub = SecurePublisher(self, MsgType, 'topic', self.ctx)
```

### Service with Security
```python
@security_required(security_level=SecurityLevel.HMAC)
def my_service(self, request, response):
    # Automatically validated
    return response
```

## Error Handling

```python
from vyra_base.security import (
    SecurityError,
    SignatureValidationError,
    TimestampValidationError,
    SessionExpiredError
)

try:
    validator.validate_metadata(...)
except SignatureValidationError:
    # Invalid signature
    pass
except TimestampValidationError:
    # Timestamp too old/new
    pass
except SessionExpiredError:
    # Re-authenticate needed
    pass
except SecurityError:
    # Generic security error
    pass
```

## Session Management

```python
# Get active sessions
count = manager.get_session_count()

# Get session details
session = manager.get_session(token)

# Check expiration
if session.is_expired():
    # Re-authenticate

# Cleanup expired
await manager.cleanup_expired_sessions()
```

## Crypto Helpers

```python
from vyra_base.helper.crypto_helper import (
    generate_hmac_key,
    compute_hmac_signature,
    verify_hmac_signature,
    generate_session_token,
    generate_nonce
)

# Generate key
key = generate_hmac_key()

# Sign message
sig = compute_hmac_signature("message", key)

# Verify
valid = verify_hmac_signature("message", sig, key)
```

## Certificate Operations (Level 5)

```python
from vyra_base.helper.crypto_helper import (
    generate_rsa_keypair,
    create_csr,
    sign_csr,
    verify_certificate
)

# Generate keypair
private_key, public_key = generate_rsa_keypair()

# Create CSR
csr = create_csr("module_name", "uuid", private_key)

# Sign CSR (server)
cert = sign_csr(csr, ca_key, ca_cert)

# Verify
valid = verify_certificate(cert, ca_cert)
```

## Constants

```python
from vyra_base.security.security_levels import (
    DEFAULT_SESSION_DURATION_SECONDS,  # 3600
    MAX_TIMESTAMP_DRIFT_SECONDS,       # 300
    HMAC_KEY_LENGTH,                   # 32
    DEFAULT_TOKEN_LENGTH,              # 32
)
```

## Environment Variables

```bash
SECURITY_LEVEL=4
SESSION_DURATION=3600
CA_CERT_PATH=/workspace/storage/certificates/ca_cert.pem
CA_KEY_PATH=/workspace/storage/certificates/ca_key.pem
```

## Testing

```python
import pytest

@pytest.mark.asyncio
async def test_auth():
    manager = SecurityManager(max_security_level=SecurityLevel.HMAC)
    result = await manager.handle_request_access(...)
    assert result[0] is True  # success
```

## Troubleshooting

| Error | Fix |
|-------|-----|
| "HMAC mismatch" | Check HMAC key |
| "Timestamp drift" | Sync time (ntpdate) |
| "Session expired" | Re-authenticate |
| "Missing metadata" | Add safety_metadata to message |

## Build & Run

```bash
# Build
colcon build
source install/setup.bash

# Run server
python3 secure_server.py

# Run client
python3 secure_client.py
```

## Quick Links

- **Full Docs**: `docs/security/SECURITY_FRAMEWORK.md`
- **Integration**: `docs/security/INTEGRATION_GUIDE.md`
- **Examples**: `examples/security/`
- **API**: `docs/security/api.rst`

---

**Version**: 1.0.0 | **Updated**: 2026-01-19
