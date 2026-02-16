Security Framework Overview
===========================

The VYRA Security Framework provides a comprehensive 5-level security model for secure inter-module communication.

Introduction
------------

In disributed systems like VYRA, where multiple modules communicate over Docker overlay networks, security is paramount. The Security Framework addresses:

* **Authentication**: Verify module identity
* **Authorization**: Control access levels
* **Integrity**: Ensure messages haven't been tampered with
* **Replay Protection**: Prevent replay attacks
* **Non-repudiation**: Cryptographic proof of origin (Level 5)

Security Levels Explained
--------------------------

Level 1: NONE
~~~~~~~~~~~~~

**Description**: No security checks performed.

**Use Cases**:
- Public information broadcasting
- Non-sensitive monitoing data
- Development/testing environments

**Performance**: Minimal overhead (<1ms)

**Implementation**:

.. code-block:: python

    # No special handling required
    publisher.publish(msg)

Level 2: BASIC_AUTH
~~~~~~~~~~~~~~~~~~~

**Description**: Module ID verification only.

**Use Cases**:
- Basic access control
- Trusted internal networks
- Module identification

**Performance**: Low overhead (<1ms)

**Validation**:
- Sender ID present
- Sender ID format valid

Level 3: EXTENDED_AUTH
~~~~~~~~~~~~~~~~~~~~~~

**Description**: Module ID + password authentication.

**Use Cases**:
- Enhanced authentication with shared secrets
- Password-based access control
- Standard security baseline

**Performance**: Low overhead (~1ms)

**Validation**:
- Sender ID verification
- Password verification against configured credentials

**Configuration**:

.. code-block:: python

    security_manager = SecurityManager(
        module_passwords={"module-uuid": "secret-password"}
    )

Level 4: HMAC
~~~~~~~~~~~~~

**Description**: HMAC-SHA256 signature validation for message integrity.

**Use Cases**:
- Sensitive data transmission
- Critical commands
- Authenticated services

**Performance**: Morate overhead (1-3ms)

**Validation**:
- All Level 3 validations
- HMAC signature verification
- Nonce uniqueness check

**Algorithm**: HMAC-SHA256 with 256-bit keys

**Security Properties**:
- Message integrity
- Authentication
- Tamper detection

Level 5: DIGITAL_SIGNATURE
~~~~~~~~~~~~~~~~~~~~~~~~~~

**Description**: Certificate-based digital signatures with full PKI.

**Use Cases**:
- Maximum security operations
- Regulated environments
- Non-repudiation requirements

**Performance**: High overhead (5-10ms)

**Validation**:
- All Level 4 validations
- Certificate chain validation
- Digital signature verification
- Certificate expiration check

**Algorithms**: RSA-SHA256 (2048-bit) or ECDSA-P256

**Security Properties**:
- All Level 4 properties
- Non-repudiation
- Certificate-based trust

Architecture
------------

Component Overview
~~~~~~~~~~~~~~~~~~

The framework consists of four main components:

1. **SecurityManager** (Server-side)
   
   - Provides RequestAccess service
   - Manages sessions and tokens
   - Issues HMAC keys
   - Signs certificates (Level 5)

2. **SecurityClient** (Client-side)
   
   - Requests authentication
   - Wraps publishers/clients
   - Generates SafetyMetadata
   - Signs messages automatically

3. **SecurityValidator** (Server-side)
   
   - Validates SafetyMetadata
   - Checks signatures
   - Enforces security levels
   - Tracks nonces

4. **Crypto Helpers**
   
   - HMAC operations
   - RSA key management
   - Certificate operations
   - Secure random generation

Authentication Flow
~~~~~~~~~~~~~~~~~~~

.. code-block:: text

    Client                          Server
      |                               |
      |  1. RequestAccess             |
      |  (module_id, requested_sl)    |
      |------------------------------>|
      |                               |
      |                          [Validate]
      |                          [Generate tokens]
      |                               |
      |  2. Response                  |
      |  (token, key, granted_sl)     |
      |<------------------------------|
      |                               |
    [Stoe context]                   |
      |                               |
      |  3. Secure Message            |
      |  (with SafetyMetadata)        |
      |------------------------------>|
      |                               |
      |                          [Validate metadata]
      |                          [Process if valid]
      |                               |

Message Security
~~~~~~~~~~~~~~~~

SafetyMetadata Structure
^^^^^^^^^^^^^^^^^^^^^^^^

All secure messages include a SafetyMetadata field:

.. code-block:: text

    SafetyMetadata:
      security_level: uint8        # 1-5
      sender_id: string            # Module UUID
      timestamp: Time              # Creation time
      nonce: uint64                # Random number
      security_payload: string     # Signature
      algorithm_id: string         # e.g., "HMAC-SHA256"

Signature Generation (Level 4)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

    message = f"{sender_id}|{timestamp}|{nonce}|{additional_data}"
    signature = HMAC-SHA256(message, hmac_key)

Signature Generation (Level 5)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

    message = f"{sender_id}|{timestamp}|{nonce}|{additional_data}"
    signature = RSA-Sign(message, private_key)

Session Management
------------------

Session Lifecycle
~~~~~~~~~~~~~~~~~

1. **Creation**: Via RequestAccess
2. **Active**: Until expiration or invalidation
3. **Cleanup**: Periodic removal of expired sessions

Session Properties
~~~~~~~~~~~~~~~~~~

.. code-block:: python

    @dataclass
    class SecuritySession:
        module_name: str
        module_id: str
        role: str
        security_level: int
        session_token: str
        hmac_key: Optional[str]
        certificate: Optional[str]
        created_at: datetime
        expires_at: datetime
        nonces_used: set

Session Expiration
~~~~~~~~~~~~~~~~~~

Default: 1 hour (3600 seconds)

Configurable:

.. code-block:: python

    SecurityManager.__init__(
        self,
        session_duration_seconds=7200  # 2 hours
    )

Automatic Re-authentication
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

    def publish_with_retry(self, msg):
        try:
            self.secure_pub.publish(msg)
        except SessionExpiredError:
            # Re-authenticate
            self.authenticate_with_server()
            # Retry
            self.secure_pub.publish(msg)

Replay Attack Prevention
------------------------

Timestamp Validation
~~~~~~~~~~~~~~~~~~~~

Messages with timestamps outside the acceptable drift window are rejected:

.. code-block:: python

    time_diff = abs(current_time - msg_timestamp)
    if time_diff > MAX_TIMESTAMP_DRIFT_SECONDS:
        raise TimestampValidationError()

Nonce Tracking
~~~~~~~~~~~~~~

Each session tracks used nonces:

.. code-block:: python

    if session.has_nonce(nonce):
        raise NonceValidationError("Replay attack detected")
    
    session.add_nonce(nonce)

Best Practices:
- Use Level 3+ for all production communications
- Synchronize system clocks (NTP)
- Monito for repeated nonces
- Clean up old nonces periodically

Certificate Management (Level 5)
---------------------------------

CA Setup
~~~~~~~~

Server-side certificate authority:

.. code-block:: python

    from vyra_base.helper.crypto_helper import (
        generate_rsa_keypair,
        create_self_signed_cert
    )

    # Generate CA
    ca_key, _ = generate_rsa_keypair()
    ca_cert = create_self_signed_cert("MyModule CA", ca_key)

    # Save securely
    save_private_key(ca_key, ca_key_path, password="strong_password")

Client CSR Generation
~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

    from vyra_base.helper.crypto_helper import create_csr

    # Generate key pair
    private_key, _ = generate_rsa_keypair()

    # Create CSR
    csr = create_csr(
        module_name="my_client",
        module_id=module_id_str,
        private_key=private_key
    )

Certificate Validation
~~~~~~~~~~~~~~~~~~~~~~

Automatic validation includes:
- Certificate not expired
- Signed by trusted CA
- Subject matches sender_id

Performance Considerations
--------------------------

Benchmarks
~~~~~~~~~~

Typical overhead per operation:

.. list-table::
   :header-rows: 1

   * - Level
     - CPU
     - Latency
     - Throughput Impact
   * - 1 (NONE)
     - 0%
     - <1ms
     - None
   * - 2 (BASIC)
     - <1%
     - <1ms
     - Negligible
   * - 3 (TIMESTAMP)
     - ~1%
     - ~1ms
     - <5%
   * - 4 (HMAC)
     - 2-5%
     - 1-3ms
     - 10-15%
   * - 5 (SIGNATURE)
     - 10-20%
     - 5-10ms
     - 30-40%

Optimization Tips
~~~~~~~~~~~~~~~~~

1. **Choose Appropriate Level**: Don't use Level 5 for all operations
2. **Cache Contexts**: Reuse SecurityContext across operations
3. **Batch Validations**: Validate in bulk when possible
4. **Connection Pooling**: Reuse authenticated connections
5. **Hardware Acceleration**: Use AES-NI, crypto acceleratos

Example Optimization:

.. code-block:: python

    # Bad: Create new context per message
    for msg in messages:
        ctx = create_context(...)
        pub = SecurePublisher(node, MsgType, 'topic', ctx)
        pub.publish(msg)

    # Good: Reuse context and publisher
    ctx = create_context(...)
    pub = SecurePublisher(node, MsgType, 'topic', ctx)
    for msg in messages:
        pub.publish(msg)

Error Handling
--------------

Exception Hierarchy
~~~~~~~~~~~~~~~~~~~

.. code-block:: text

    SecurityError
    ├── InvalidSecurityLevelError
    ├── SignatureValidationError
    ├── CertificateValidationError
    ├── TimestampValidationError
    ├── SessionExpiredError
    └── NonceValidationError

Handling Validation Errors
~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

    try:
        validato.validate_metadata(
            request.safety_metadata,
            required_level=SecurityLevel.HMAC
        )
        # Process request
    except SignatureValidationError as e:
        logger.error(f"Signature invalid: {e}")
        response.success = False
        response.message = "Authentication failed"
    except TimestampValidationError as e:
        logger.warn(f"Timestamp validation failed: {e}")
        response.success = False
        response.message = "Request too old"
    except SecurityError as e:
        logger.error(f"Security error: {e}")
        response.success = False
        response.message = "Security validation failed"

Graceful Degradation
~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

    def try_secure_operation(self, operation, fallback_level):
        try:
            return operation(SecurityLevel.HMAC)
        except SecurityError:
            logger.warn("HMAC failed, falling back to TIMESTAMP")
            return operation(fallback_level)

Security Monitoing
-------------------

Metrics
~~~~~~~

Track:
- Authentication success/failure rate
- Average validation time
- Active sessions count
- Nonce collision rate

.. code-block:: python

    # Get session count
    count = security_manager.get_session_count()

    # Get active modules
    modules = security_manager.get_active_modules()

Alerting
~~~~~~~~

Alert on:
- Repeated authentication failures
- Unusual nonce patterns
- Certificate validation failures
- High validation latency

Testing
-------

Unit Tests
~~~~~~~~~~

.. code-block:: python

    import pytest
    from vyra_base.security.security_manager import SecurityManager

    @pytest.mark.asyncio
    async def test_hmac_authentication():
        manager = SecurityManager(
            max_security_level=SecurityLevel.HMAC
        )
        
        result = await manager.handle_request_access(
            module_name="test",
            module_id=uuid.uuid4(),
            requested_role="operato",
            requested_sl=4,
            certificate_csr=""
        )
        
        success, _, token, hmac, _, _, granted = result
        
        assert success is True
        assert granted == 4
        assert len(token) > 0
        assert len(hmac) == 64  # 32 bytes hex-encoded

Integration Tests
~~~~~~~~~~~~~~~~~

Test complete authentication flow:

.. code-block:: python

    def test_end_to_end_security():
        # Start server
        server = SecureServerNode()
        
        # Start client
        client = SecureClientNode()
        
        # Authenticate
        assert client.authenticate_with_server()
        
        # Send secure message
        msg = create_test_message()
        client.secure_pub.publish(msg)
        
        # Verify server received and validated
        assert server.received_messages > 0

Security Tests
~~~~~~~~~~~~~~

Test attack scenarios:

.. code-block:: python

    def test_replay_attack_prevention():
        # Send valid message
        msg1 = create_secure_message(nonce=12345)
        server.process(msg1)  # Should succeed
        
        # Replay same message
        msg2 = create_secure_message(nonce=12345)  # Same nonce
        with pytest.raises(NonceValidationError):
            server.process(msg2)

    def test_timestamp_tampering():
        # Create message with old timestamp
        msg = create_secure_message(
            timestamp=time.time() - 3600  # 1 hour old
        )
        with pytest.raises(TimestampValidationError):
            server.process(msg)

References
----------

- :doc:`quickstart` - Integration guide
- :doc:`api` - Complete API reference
- :doc:`examples` - Working examples
- `RFC 2104 <https://www.rfc-edito.org/rfc/rfc2104>`_ - HMAC specification
- `RFC 5280 <https://www.rfc-edito.org/rfc/rfc5280>`_ - X.509 certificates
