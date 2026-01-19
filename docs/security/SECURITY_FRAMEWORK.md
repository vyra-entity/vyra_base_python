# VYRA Security Framework - Developer Guide

## Table of Contents

1. [Overview](#overview)
2. [Security Levels](#security-levels)
3. [Architecture](#architecture)
4. [Quick Start](#quick-start)
5. [Server Implementation](#server-implementation)
6. [Client Implementation](#client-implementation)
7. [Message Security](#message-security)
8. [Advanced Topics](#advanced-topics)
9. [API Reference](#api-reference)
10. [Troubleshooting](#troubleshooting)

---

## Overview

The VYRA Security Framework provides a comprehensive 5-level security model for secure inter-module communication in Docker Swarm environments using ROS2. It implements authentication, message integrity, and protection against replay attacks.

### Key Features

- **5 Security Levels**: From no security to certificate-based digital signatures
- **Automatic Message Security**: Transparent SafetyMetadata handling
- **Session Management**: Token-based authentication with expiration
- **Replay Attack Prevention**: Timestamp and nonce validation (Level 4+)
- **HMAC Signatures**: Message integrity with HMAC-SHA256
- **PKI Support**: Full certificate-based authentication (Level 5)

### When to Use

Use the VYRA Security Framework when:
- Modules need authenticated communication over Docker overlay networks
- Message integrity verification is required
- Protection against replay attacks is necessary
- Different security levels are needed for different operations

---

## Security Levels

The framework implements 5 security levels, each building upon the previous:

### Level 1: NONE
- **Description**: No security checks performed
- **Use Case**: Public information, non-sensitive data
- **Overhead**: Minimal
- **Validation**: None

### Level 2: BASIC_AUTH
- **Description**: Module ID verification only
- **Use Case**: Basic access control, internal networks
- **Overhead**: Low
- **Validation**: Sender ID verification

### Level 3: EXTENDED_AUTH
- **Description**: Module ID + password authentication
- **Use Case**: Enhanced authentication with shared secrets
- **Overhead**: Low
- **Validation**: Sender ID + password verification

### Level 4: HMAC
- **Description**: HMAC-SHA256 signature validation
- **Use Case**: Message integrity, authenticated communication, replay attack prevention
- **Overhead**: Moderate
- **Validation**: Sender ID + timestamp (±5 minutes drift) + HMAC signature
- **Algorithm**: HMAC-SHA256

### Level 5: DIGITAL_SIGNATURE
- **Description**: Certificate-based digital signatures (RSA/ECDSA)
- **Use Case**: Full PKI, non-repudiation, maximum security
- **Overhead**: High
- **Validation**: Certificate chain + digital signature
- **Algorithm**: RSA-SHA256 or ECDSA-P256

---

## Architecture

### Components

The framework consists of four main components:

```
┌─────────────────────────────────────────────────────────┐
│                     Module A (Client)                   │
│  ┌──────────────────┐         ┌──────────────────┐      │
│  │ SecurityContext  │         │ SecurePublisher  │      │
│  │  - session_token │  ────>  │ SecureService    │      │
│  │  - hmac_key      │         │ Client           │      │
│  └──────────────────┘         └──────────────────┘      │
└─────────────────────────────────────────────────────────┘
                         │
                         │ ROS2 (with SafetyMetadata)
                         ▼
┌─────────────────────────────────────────────────────────┐
│                     Module B (Server)                   │
│  ┌──────────────────┐         ┌──────────────────┐      │
│  │ SecurityManager  │  <────  │ SecurityValidator│      │
│  │  - sessions      │         │  - validate()    │      │
│  │  - request_access│         └──────────────────┘      │
│  └──────────────────┘                                   │
└─────────────────────────────────────────────────────────┘
```

#### 1. SecurityManager (Server-side)
- Provides `RequestAccess` service
- Manages security sessions
- Issues session tokens and HMAC keys
- Signs certificates (Level 5)

#### 2. SecurityClient (Client-side)
- Requests authentication via `RequestAccess`
- Stores security context (tokens, keys)
- Wraps publishers/clients with automatic security
- Generates SafetyMetadata

#### 3. SecurityValidator (Server-side)
- Validates incoming SafetyMetadata
- Verifies passwords for Level 3+
- Checks timestamps for replay attacks (Level 4+)
- Verifies HMAC signatures (Level 4)
- Validates digital signatures (Level 5)
- Tracks nonces to prevent replay attacks (Level 4+)

#### 4. Crypto Helpers
- HMAC-SHA256 operations
- RSA key generation and signing
- Certificate operations (CSR, signing, validation)
- Secure random generation

### Message Flow

```
1. Client → Server: RequestAccess(module_id, requested_sl)
2. Server → Client: Response(session_token, hmac_key, granted_sl)
3. Client creates SecurityContext with credentials
4. Client → Server: Message with SafetyMetadata (signed)
5. Server validates SafetyMetadata
6. Server processes message if validation passes
```

---

## Quick Start

### Prerequisites

```bash
# Install Python dependencies
pip install cryptography

# Ensure ROS2 is sourced
source /opt/ros/<distro>/setup.bash

# Build vyra_base interfaces
cd /path/to/vyra_base_python
colcon build
source install/setup.bash
```

### Minimal Server Example

```python
from vyra_base.security.security_manager import SecurityManager
from vyra_base.security.security_levels import SecurityLevel
from vyra_base.interfaces.srv import VBASERequestAccess

class MySecureModule(Node, SecurityManager):
    def __init__(self):
        Node.__init__(self, 'my_module')
        SecurityManager.__init__(self, max_security_level=SecurityLevel.HMAC)
        
        # Create RequestAccess service
        self.srv = self.create_service(
            VBASERequestAccess,
            'my_module/request_access',
            self.handle_request_access_callback
        )
    
    def handle_request_access_callback(self, request, response):
        # Handle authentication
        result = await self.handle_request_access(
            request.module_name,
            request.module_id,
            request.requested_role,
            request.requested_sl,
            request.certificate_csr
        )
        # Fill response...
        return response
```

### Minimal Client Example

```python
from vyra_base.security.security_client import create_security_context
from vyra_base.interfaces.srv import VBASERequestAccess

# 1. Request access
client = node.create_client(VBASERequestAccess, 'server/request_access')
request = VBASERequestAccess.Request()
request.module_name = "my_client"
request.requested_sl = 4  # HMAC level

response = await client.call_async(request)

# 2. Create security context
security_ctx = create_security_context(
    module_id=str(my_uuid),
    security_level=response.granted_sl,
    session_token=response.session_token,
    hmac_key=response.hmac_key
)

# 3. Use secure communication
from vyra_base.security.security_client import SecurePublisher

secure_pub = SecurePublisher(
    node, MessageType, 'topic', security_ctx
)
secure_pub.publish(my_message)
```

---

## Server Implementation

### Step 1: Inherit SecurityManager

Your module should inherit from both `Node` and `SecurityManager`:

```python
from rclpy.node import Node
from vyra_base.security.security_manager import SecurityManager
from vyra_base.security.security_levels import SecurityLevel

class MyModule(Node, SecurityManager):
    def __init__(self):
        # Initialize Node
        Node.__init__(self, 'my_module')
        
        # Initialize SecurityManager
        SecurityManager.__init__(
            self,
            max_security_level=SecurityLevel.HMAC,  # Support up to Level 4
            session_duration_seconds=3600,  # 1 hour sessions
            ca_key_path=None,  # For Level 5
            ca_cert_path=None   # For Level 5
        )
```

### Step 2: Create RequestAccess Service

```python
from vyra_base.interfaces.srv import VBASERequestAccess

def __init__(self):
    # ... previous initialization ...
    
    self.access_service = self.create_service(
        VBASERequestAccess,
        f'{self.get_name()}/request_access',
        self.handle_request_access_callback
    )
```

### Step 3: Implement Service Callback

```python
import asyncio

def handle_request_access_callback(self, request, response):
    """Handle access requests from clients."""
    
    # Call async handler
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    
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
    
    # Unpack result
    (success, message, session_token, hmac_key, 
     certificate, expires_at, granted_sl) = result
    
    # Fill response
    response.success = success
    response.message = message
    response.session_token = session_token
    response.hmac_key = hmac_key
    response.certificate = certificate
    response.expires_at.sec = int(expires_at.timestamp())
    response.expires_at.nanosec = 0
    response.granted_sl = granted_sl
    
    return response
```

### Step 4: Validate Incoming Messages

```python
from vyra_base.security.security_validator import SecurityValidator

def __init__(self):
    # ... previous initialization ...
    
    # Create validator
    self.validator = SecurityValidator(
        security_manager=self,
        strict_mode=True
    )

def my_service_callback(self, request, response):
    """Service callback with validation."""
    
    try:
        # Validate security metadata
        session = self.validator.validate_metadata(
            request.safety_metadata,
            required_level=SecurityLevel.HMAC
        )
        
        # Process authenticated request
        # ...
        
    except SecurityError as e:
        response.success = False
        response.message = f"Security validation failed: {e}"
        return response
    
    return response
```

### Step 5: Add Session Cleanup

```python
def __init__(self):
    # ... previous initialization ...
    
    # Periodic session cleanup
    self.cleanup_timer = self.create_timer(
        60.0,  # Every 60 seconds
        self.cleanup_sessions_callback
    )

def cleanup_sessions_callback(self):
    """Clean up expired sessions."""
    import asyncio
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(self.cleanup_expired_sessions())
    loop.close()
```

---

## Client Implementation

### Step 1: Create Access Request

```python
import uuid
from vyra_base.interfaces.srv import VBASERequestAccess
from unique_identifier_msgs.msg import UUID as UUIDMsg

# Generate or load module UUID
module_uuid = uuid.uuid4()
module_id_str = str(module_uuid)

# Create service client
access_client = node.create_client(
    VBASERequestAccess,
    'server_module/request_access'
)

# Wait for service
if not access_client.wait_for_service(timeout_sec=10.0):
    raise RuntimeError("RequestAccess service not available")

# Create request
request = VBASERequestAccess.Request()
request.module_name = "my_client_module"
request.module_id = UUIDMsg(uuid=list(module_uuid.bytes))
request.requested_role = "operator"
request.requested_sl = 4  # Request Level 4 (HMAC)
request.certificate_csr = ""  # Not needed for Level 4
```

### Step 2: Call Service and Handle Response

```python
# Call service asynchronously
future = access_client.call_async(request)
rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

if not future.done():
    raise RuntimeError("Access request timed out")

response = future.result()

if not response.success:
    raise RuntimeError(f"Access denied: {response.message}")

# Extract credentials
session_token = response.session_token
hmac_key = response.hmac_key
granted_sl = response.granted_sl

print(f"✅ Access granted with Security Level {granted_sl}")
```

### Step 3: Create Security Context

```python
from vyra_base.security.security_client import create_security_context

security_context = create_security_context(
    module_id=module_id_str,
    security_level=granted_sl,
    session_token=session_token,
    hmac_key=hmac_key if granted_sl >= 4 else None,
    certificate=response.certificate if granted_sl >= 5 else None
)
```

### Step 4: Use Secure Publishers

```python
from vyra_base.security.security_client import SecurePublisher

# Create secure publisher
secure_pub = SecurePublisher(
    node,
    MyMessageType,  # Must have safety_metadata field
    'my_secure_topic',
    security_context,
    qos_profile=10
)

# Publish messages (SafetyMetadata added automatically)
msg = MyMessageType()
msg.data = "Hello, secure world!"
secure_pub.publish(msg)
```

### Step 5: Use Secure Service Clients

```python
from vyra_base.security.security_client import SecureServiceClient

# Create secure service client
secure_client = SecureServiceClient(
    node,
    MyServiceType,  # Request must have safety_metadata field
    'my_secure_service',
    security_context
)

# Call service (SafetyMetadata added automatically)
request = MyServiceType.Request()
request.parameter = "value"

response = await secure_client.call_async(request)
```

---

## Message Security

### Adding SafetyMetadata to Custom Messages

Your ROS2 message definitions must include a `SafetyMetadata` field:

```
# MySecureMessage.msg
string data
int32 value
vyra_base/SafetyMetadata safety_metadata
```

### SafetyMetadata Structure

```
# vyra_base/SafetyMetadata.msg
uint8 security_level                    # 1-5
string sender_id                        # Module UUID
string sender_name                      # Module name (human-readable)
builtin_interfaces/Time timestamp       # Message creation time
uint64 nonce                           # Random number (replay prevention)
string security_payload                 # Signature (Level 4-5)
string algorithm_id                     # e.g., "HMAC-SHA256"
```

### Automatic vs Manual SafetyMetadata

**Automatic (Recommended):**
```python
# Use SecurePublisher - metadata added automatically
secure_pub.publish(msg)
```

**Manual (Advanced):**
```python
from vyra_base.security.security_client import SafetyMetadataBuilder

# Build metadata manually
metadata_dict = SafetyMetadataBuilder.build(security_context)

# Apply to message
msg.safety_metadata.security_level = metadata_dict['security_level']
msg.safety_metadata.sender_id = metadata_dict['sender_id']
# ... etc
```

---

## Advanced Topics

### Certificate-Based Security (Level 5)

#### Server Setup

1. **Generate CA Key and Certificate:**

```python
from vyra_base.helper.crypto_helper import (
    generate_rsa_keypair,
    create_self_signed_cert,
    save_private_key
)
from pathlib import Path

# Generate CA key pair
ca_private_key, ca_public_key = generate_rsa_keypair()

# Create self-signed CA certificate
ca_cert = create_self_signed_cert("MyModule CA", ca_private_key)

# Save to disk
ca_key_path = Path("/workspace/storage/certificates/ca_key.pem")
ca_cert_path = Path("/workspace/storage/certificates/ca_cert.pem")

save_private_key(ca_private_key, ca_key_path, password="secure_password")
ca_cert_path.write_bytes(
    ca_cert.public_bytes(serialization.Encoding.PEM)
)
```

2. **Initialize SecurityManager with CA:**

```python
SecurityManager.__init__(
    self,
    max_security_level=SecurityLevel.DIGITAL_SIGNATURE,
    ca_key_path=Path("/workspace/storage/certificates/ca_key.pem"),
    ca_cert_path=Path("/workspace/storage/certificates/ca_cert.pem")
)
```

#### Client Setup

1. **Generate Key Pair and CSR:**

```python
from vyra_base.helper.crypto_helper import (
    generate_rsa_keypair,
    create_csr,
    save_private_key
)

# Generate client key pair
private_key, public_key = generate_rsa_keypair()

# Save private key
private_key_path = Path("/workspace/storage/certificates/client_key.pem")
save_private_key(private_key, private_key_path)

# Create CSR
csr = create_csr(
    module_name="my_client",
    module_id=module_id_str,
    private_key=private_key
)
```

2. **Request Access with CSR:**

```python
request = VBASERequestAccess.Request()
# ... set other fields ...
request.requested_sl = 5  # Level 5
request.certificate_csr = csr  # Include CSR

response = await access_client.call_async(request)

# Server will sign CSR and return certificate
client_certificate = response.certificate
```

3. **Use Certificate for Signing:**

```python
security_context = create_security_context(
    module_id=module_id_str,
    security_level=5,
    session_token=response.session_token,
    certificate=client_certificate,
    private_key_path=str(private_key_path)
)
```

### Custom Security Policies

You can customize security behavior by extending the base classes:

```python
from vyra_base.security.security_manager import SecurityManager

class CustomSecurityManager(SecurityManager):
    async def handle_request_access(self, *args, **kwargs):
        # Add custom authorization logic
        if not self.check_custom_policy(args[0]):  # module_name
            return (False, "Custom policy check failed", "", "", "", datetime.utcnow(), 0)
        
        # Call parent implementation
        return await super().handle_request_access(*args, **kwargs)
    
    def check_custom_policy(self, module_name: str) -> bool:
        # Custom logic (e.g., whitelist, database lookup)
        return module_name in self.allowed_modules
```

### Replay Attack Prevention

The framework uses two mechanisms to prevent replay attacks:

1. **Timestamp Validation (Level 4+)**
   - Messages older than 5 minutes are rejected
   - Configurable via `MAX_TIMESTAMP_DRIFT_SECONDS`

2. **Nonce Tracking (Level 4+)**
   - Each message includes a random nonce
   - Server tracks used nonces per session
   - Duplicate nonces are rejected

### Session Management

Sessions are managed automatically:

```python
# Get active sessions
active_sessions = security_manager.get_session_count()

# Get session details
session = security_manager.get_session(session_token)

# Get all active modules
modules = security_manager.get_active_modules()
# Returns: [{"module_name": "...", "module_id": "...", "role": "...", ...}]

# Manual session invalidation
security_manager._invalidate_session(session_token)
```

### Performance Considerations

**Security Level vs Overhead:**

| Level | CPU Overhead | Latency Impact | Recommended For |
|-------|-------------|----------------|-----------------|
| 1     | None        | <1ms           | Public topics   |
| 2     | Minimal     | <1ms           | Internal network|
| 3     | Low         | ~1ms           | Standard use    |
| 4     | Moderate    | 1-3ms          | Sensitive data  |
| 5     | High        | 5-10ms         | Critical ops    |

**Optimization Tips:**
- Use Level 1-3 for high-frequency topics
- Use Level 4 for sensitive services
- Reserve Level 5 for critical operations only
- Cache security contexts to avoid repeated authentication
- Use connection pooling for multiple services

---

## API Reference

### SecurityManager

```python
class SecurityManager:
    def __init__(
        self,
        max_security_level: SecurityLevel = SecurityLevel.HMAC,
        session_duration_seconds: int = 3600,
        ca_key_path: Optional[Path] = None,
        ca_cert_path: Optional[Path] = None
    )
    
    async def handle_request_access(
        self,
        module_name: str,
        module_id: uuid.UUID,
        requested_role: str,
        requested_sl: int,
        certificate_csr: str = ""
    ) -> Tuple[bool, str, str, str, str, datetime, int]
    
    def get_session(self, session_token: str) -> Optional[SecuritySession]
    async def cleanup_expired_sessions(self) -> None
    def get_session_count(self) -> int
    def get_active_modules(self) -> list
```

### SecurityValidator

```python
class SecurityValidator:
    def __init__(
        self,
        security_manager: Optional[SecurityManager] = None,
        strict_mode: bool = True
    )
    
    def validate_metadata(
        self,
        metadata: Any,
        required_level: SecurityLevel = SecurityLevel.NONE,
        additional_data: str = ""
    ) -> SecuritySession
```

### SecurePublisher

```python
class SecurePublisher:
    def __init__(
        self,
        node: Node,
        msg_type: Type,
        topic: str,
        security_context: SecurityContext,
        qos_profile: Any = 10
    )
    
    def publish(self, msg: Any, additional_data: str = "") -> None
    def destroy(self) -> None
```

### SecureServiceClient

```python
class SecureServiceClient:
    def __init__(
        self,
        node: Node,
        srv_type: Type,
        service_name: str,
        security_context: SecurityContext
    )
    
    async def call_async(self, request: Any, additional_data: str = "") -> Any
    def destroy(self) -> None
```

### Helper Functions

```python
# Crypto helpers
def generate_hmac_key() -> str
def compute_hmac_signature(message: str, hmac_key: str) -> str
def verify_hmac_signature(message: str, signature: str, hmac_key: str) -> bool
def generate_session_token() -> str
def generate_nonce() -> int

# Certificate helpers
def generate_rsa_keypair() -> Tuple[RSAPrivateKey, RSAPublicKey]
def create_csr(module_name: str, module_id: str, private_key: RSAPrivateKey) -> str
def sign_csr(csr_pem: str, ca_private_key: RSAPrivateKey, ca_cert: Certificate) -> str
def verify_certificate(cert_pem: str, ca_cert_pem: str) -> bool

# Client helpers
def create_security_context(...) -> SecurityContext
```

---

## Troubleshooting

### Common Issues

#### 1. "HMAC signature mismatch"

**Cause:** HMAC key mismatch or incorrect message content

**Solution:**
- Verify HMAC key is correctly stored in security context
- Ensure `additional_data` parameter matches between client and server
- Check that message fields used in signature are identical

#### 2. "Timestamp drift too large"

**Cause:** Client and server clocks are not synchronized

**Solution:**
```bash
# Sync system time
sudo ntpdate -u time.nist.gov

# Or use systemd-timesyncd
sudo timedatectl set-ntp true
```

#### 3. "Certificate validation failed"

**Cause:** Certificate expired, CA mismatch, or invalid signature

**Solution:**
- Verify CA certificate is correct
- Check certificate expiration date
- Ensure CSR was signed by the correct CA

#### 4. "Session expired"

**Cause:** Session token exceeded validity period

**Solution:**
- Request new access with `RequestAccess`
- Increase `session_duration_seconds` on server
- Implement automatic re-authentication

#### 5. "Message missing SafetyMetadata"

**Cause:** Message type doesn't include `safety_metadata` field

**Solution:**
Add field to message definition:
```
# MyMessage.msg
# ... other fields ...
vyra_base/SafetyMetadata safety_metadata
```

Then rebuild:
```bash
colcon build --packages-select my_interfaces
```

### Debug Mode

Enable debug logging:

```python
from vyra_base.helper.logger import Logger
import logging

Logger.setLevel(logging.DEBUG)
```

### Validation Testing

Test security validation:

```python
# Create invalid metadata
metadata = SafetyMetadata()
metadata.security_level = 1  # Wrong level
metadata.sender_id = "fake_id"  # Wrong ID

try:
    validator.validate_metadata(metadata, required_level=SecurityLevel.HMAC)
except SecurityError as e:
    print(f"Expected error: {e}")  # Should fail
```

---

## Best Practices

1. **Always use Level 3+ for production**: Protects against replay attacks
2. **Store session tokens securely**: Don't log or expose in APIs
3. **Rotate HMAC keys periodically**: Re-authenticate after key rotation
4. **Use Level 5 for critical operations**: Maximum security for sensitive actions
5. **Implement session cleanup**: Remove expired sessions regularly
6. **Validate all inputs**: Never trust client-provided security metadata
7. **Monitor failed authentications**: Log and alert on repeated failures
8. **Use strong entropy sources**: Rely on `secrets` module for random generation
9. **Protect CA private keys**: Encrypt at rest, restrict file permissions
10. **Test security scenarios**: Include security tests in CI/CD pipeline

---

## Related Documentation

- [ROS2 Security Guide](../ROS2_SECURITY_GUIDE.md)
- [SROS2 Integration](../SROS2_INTEGRATION.md)
- [API Documentation](../api/security.html)
- [Examples](../../examples/security/)

---

**Version:** 1.0.0  
**Last Updated:** 2026-01-19  
**Author:** VYRA Framework Team
