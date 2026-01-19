Security Framework API Reference
=================================

Complete API documentation for the VYRA Security Framework.

.. currentmodule:: vyra_base.security

Security Levels
---------------

.. currentmodule:: vyra_base.security.security_levels

Classes
~~~~~~~

.. autoclass:: SecurityLevel
   :members:
   :undoc-members:
   :no-index:

.. autoclass:: AccessStatus
   :members:
   :undoc-members:
   :no-index:

.. autoclass:: AlgorithmId
   :members:
   :undoc-members:
   :no-index:

Exceptions
~~~~~~~~~~

.. autoexception:: SecurityError
   :no-index:
.. autoexception:: InvalidSecurityLevelError
   :no-index:
.. autoexception:: SignatureValidationError
   :no-index:
.. autoexception:: CertificateValidationError
   :no-index:
.. autoexception:: TimestampValidationError
   :no-index:
.. autoexception:: SessionExpiredError
   :no-index:
.. autoexception:: NonceValidationError
   :no-index:

Security Manager
----------------

.. currentmodule:: vyra_base.security.security_manager

Classes
~~~~~~~

.. autoclass:: SecuritySession
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: SecurityManager
   :members:
   :undoc-members:
   :show-inheritance:

Security Client
---------------

.. currentmodule:: vyra_base.security.security_client

Classes
~~~~~~~

.. autoclass:: SecurityContext
   :members:
   :undoc-members:

.. autoclass:: SafetyMetadataBuilder
   :members:
   :undoc-members:

.. autoclass:: SecurePublisher
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: SecureServiceClient
   :members:
   :undoc-members:
   :show-inheritance:

Functions
~~~~~~~~~

.. autofunction:: security_required
.. autofunction:: create_security_context

Security Validator
------------------

.. currentmodule:: vyra_base.security.security_validator

Classes
~~~~~~~

.. autoclass:: SecurityValidator
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: MessageSecurityFilter
   :members:
   :undoc-members:
   :show-inheritance:

Functions
~~~~~~~~~

.. autofunction:: create_secure_subscription

Crypto Helpers
--------------

.. currentmodule:: vyra_base.helper.crypto_helper

HMAC Functions
~~~~~~~~~~~~~~

.. autofunction:: generate_hmac_key
.. autofunction:: compute_hmac_signature
.. autofunction:: verify_hmac_signature
.. autofunction:: create_hmac_payload

Token Functions
~~~~~~~~~~~~~~~

.. autofunction:: generate_session_token
.. autofunction:: generate_nonce

RSA Key Functions
~~~~~~~~~~~~~~~~~

.. autofunction:: generate_rsa_keypair
.. autofunction:: save_private_key
.. autofunction:: load_private_key

Certificate Functions
~~~~~~~~~~~~~~~~~~~~~

.. autofunction:: create_csr
.. autofunction:: sign_csr
.. autofunction:: create_self_signed_cert
.. autofunction:: verify_certificate
.. autofunction:: sign_message_rsa
.. autofunction:: verify_message_rsa

Constants
---------

.. py:data:: DEFAULT_SESSION_DURATION_SECONDS
   :type: int
   :value: 3600

   Default session validity duration (1 hour).

.. py:data:: DEFAULT_TOKEN_LENGTH
   :type: int
   :value: 32

   Default session token length in bytes.

.. py:data:: DEFAULT_NONCE_LENGTH
   :type: int
   :value: 16

   Default nonce length in bytes.

.. py:data:: MAX_TIMESTAMP_DRIFT_SECONDS
   :type: int
   :value: 300

   Maximum allowed timestamp drift (5 minutes).

.. py:data:: HMAC_KEY_LENGTH
   :type: int
   :value: 32

   HMAC key length in bytes (256 bits).

.. py:data:: CERT_KEY_SIZE
   :type: int
   :value: 2048

   RSA key size for certificates.

.. py:data:: CERT_VALIDITY_DAYS
   :type: int
   :value: 365

   Default certificate validity period in days.

.. py:data:: MAX_SECURITY_PAYLOAD_SIZE
   :type: int
   :value: 4096

   Maximum size for security payloads (4KB).

.. py:data:: MAX_CERTIFICATE_SIZE
   :type: int
   :value: 8192

   Maximum size for certificates (8KB).

ROS2 Interfaces
---------------

VBASERequestAccess Service
~~~~~~~~~~~~~~~~~~~~~~~~~~

Service definition for requesting authenticated access.

**Request Fields:**

.. code-block:: python

    string module_name           # Name of requesting module
    UUID module_id              # UUID of requesting module
    string requested_role        # Role identifier
    uint8 requested_sl          # Requested security level (1-5)
    string certificate_csr       # CSR for Level 5 (optional)

**Response Fields:**

.. code-block:: python

    bool success                # Access granted
    string message              # Result message
    string session_token        # Session token
    string hmac_key            # HMAC key (Level 4+)
    string certificate          # Signed certificate (Level 5)
    Time expires_at            # Token expiration
    uint8 granted_sl           # Granted security level

SafetyMetadata Message
~~~~~~~~~~~~~~~~~~~~~~

Message type containing security metadata.

**Fields:**

.. code-block:: python

    uint8 security_level        # Security level (1-5)
    string sender_id           # Sender module UUID
    Time timestamp             # Message creation time
    uint64 nonce              # Random nonce
    string security_payload    # Signature/HMAC
    string algorithm_id        # Algorithm identifier

Usage Examples
--------------

Authentication
~~~~~~~~~~~~~~

**Server Side:**

.. code-block:: python

    from vyra_base.security.security_manager import SecurityManager

    class MyModule(Node, SecurityManager):
        def __init__(self):
            SecurityManager.__init__(
                self,
                max_security_level=SecurityLevel.HMAC
            )

**Client Side:**

.. code-block:: python

    from vyra_base.security.security_client import create_security_context

    response = await access_client.call_async(request)
    
    ctx = create_security_context(
        module_id=str(uuid),
        security_level=response.granted_sl,
        session_token=response.session_token,
        hmac_key=response.hmac_key
    )

Secure Publishing
~~~~~~~~~~~~~~~~~

.. code-block:: python

    from vyra_base.security.security_client import SecurePublisher

    pub = SecurePublisher(
        node,
        MessageType,
        'topic_name',
        security_context,
        qos_profile=10
    )

    msg = MessageType()
    msg.data = "Hello"
    pub.publish(msg)  # Automatic SafetyMetadata

Validation
~~~~~~~~~~

.. code-block:: python

    from vyra_base.security.security_validator import SecurityValidator

    validator = SecurityValidator(security_manager, strict_mode=True)

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

Certificate Operations
~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

    from vyra_base.helper.crypto_helper import (
        generate_rsa_keypair,
        create_csr,
        sign_csr
    )

    # Generate key pair
    private_key, public_key = generate_rsa_keypair()

    # Create CSR
    csr = create_csr("my_module", "uuid-string", private_key)

    # Sign CSR (server side)
    certificate = sign_csr(csr, ca_private_key, ca_cert)

Type Hints
----------

.. code-block:: python

    from typing import Optional, Tuple
    from datetime import datetime
    import uuid

    # SecurityManager
    async def handle_request_access(
        self,
        module_name: str,
        module_id: uuid.UUID,
        requested_role: str,
        requested_sl: int,
        certificate_csr: str = ""
    ) -> Tuple[bool, str, str, str, str, datetime, int]:
        ...

    # SecurityValidator
    def validate_metadata(
        self,
        metadata: Any,
        required_level: SecurityLevel = SecurityLevel.NONE,
        additional_data: str = ""
    ) -> Optional[SecuritySession]:
        ...

    # Crypto helpers
    def compute_hmac_signature(
        message: str,
        hmac_key: str
    ) -> str:
        ...

Error Handling
--------------

Exception Handling Pattern
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

    from vyra_base.security.security_levels import (
        SecurityError,
        SignatureValidationError,
        TimestampValidationError,
        SessionExpiredError
    )

    try:
        validator.validate_metadata(metadata, SecurityLevel.HMAC)
    except SignatureValidationError:
        # Handle invalid signature
        pass
    except TimestampValidationError:
        # Handle timestamp issues
        pass
    except SessionExpiredError:
        # Re-authenticate
        pass
    except SecurityError as e:
        # Handle general security error
        pass

Logging
~~~~~~~

.. code-block:: python

    from vyra_base.helper.logger import Logger
    import logging

    # Set log level
    Logger.setLevel(logging.DEBUG)

    # Log security events
    Logger.info("✅ Access granted")
    Logger.warn("⚠️ Timestamp drift detected")
    Logger.error("❌ Signature validation failed")

Testing
-------

Unit Test Example
~~~~~~~~~~~~~~~~~

.. code-block:: python

    import pytest
    from vyra_base.security.security_manager import SecurityManager

    @pytest.mark.asyncio
    async def test_authentication():
        manager = SecurityManager(
            max_security_level=SecurityLevel.HMAC
        )
        
        result = await manager.handle_request_access(
            "test_module",
            uuid.uuid4(),
            "tester",
            4,
            ""
        )
        
        assert result[0] is True  # success
        assert result[6] == 4     # granted_sl

Mock Testing
~~~~~~~~~~~~

.. code-block:: python

    from unittest.mock import Mock, patch

    @patch('vyra_base.helper.crypto_helper.generate_hmac_key')
    def test_with_mock_key(mock_key):
        mock_key.return_value = "0" * 64
        
        manager = SecurityManager()
        # Test with predictable key
        ...

See Also
--------

- :doc:`overview` - Framework overview
- :doc:`quickstart` - Quick start guide
- :doc:`examples` - Example implementations
- Module index: :ref:`modindex`
- Index: :ref:`genindex`
