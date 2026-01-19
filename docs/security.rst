Security Framework
==================

The VYRA Security Framework provides comprehensive security features for inter-module communication in Docker Swarm environments.

.. toctree::
   :maxdepth: 2
   :caption: Contents:

   security/overview
   security/quickstart
   security/api
   security/examples

Overview
--------

The framework implements a 5-level security model:

.. list-table:: Security Levels
   :header-rows: 1
   :widths: 10 30 30 30

   * - Level
     - Name
     - Features
     - Use Case
   * - 1
     - NONE
     - No security
     - Public information
   * - 2
     - BASIC_AUTH
     - Module ID verification
     - Internal networks
   * - 3
     - TIMESTAMP
     - ID + timestamp validation
     - Replay attack prevention
   * - 4
     - HMAC
     - HMAC-SHA256 signatures
     - Message integrity
   * - 5
     - DIGITAL_SIGNATURE
     - Certificate-based PKI
     - Maximum security

Key Features
------------

* **Multi-level Security**: Choose appropriate security for each operation
* **Automatic Metadata**: Transparent SafetyMetadata handling
* **Session Management**: Token-based authentication with expiration
* **Replay Protection**: Timestamp and nonce validation
* **PKI Support**: Full certificate infrastructure for Level 5

Quick Start
-----------

Server Module
~~~~~~~~~~~~~

.. code-block:: python

    from vyra_base.security.security_manager import SecurityManager
    from vyra_base.security.security_levels import SecurityLevel

    class MyModule(Node, SecurityManager):
        def __init__(self):
            Node.__init__(self, 'my_module')
            SecurityManager.__init__(
                self,
                max_security_level=SecurityLevel.HMAC
            )

Client Module
~~~~~~~~~~~~~

.. code-block:: python

    from vyra_base.security.security_client import (
        create_security_context,
        SecurePublisher
    )

    # Request access
    response = await client.call_async(request)

    # Create context
    ctx = create_security_context(
        module_id=str(uuid),
        security_level=response.granted_sl,
        session_token=response.session_token,
        hmac_key=response.hmac_key
    )

    # Use secure communication
    pub = SecurePublisher(node, MsgType, 'topic', ctx)
    pub.publish(msg)

Documentation
-------------

For detailed documentation, see:

* :doc:`security/overview` - Complete framework documentation
* :doc:`security/quickstart` - Integration guide
* :doc:`security/api` - API reference
* :doc:`security/examples` - Example implementations

Architecture
------------

The VYRA Security Framework follows a client-server architecture:

1. **Client Module** (SecurityClient) - Requests authentication and sends secure messages
2. **Server Module** (SecurityManager) - Provides authentication service and manages sessions
3. **SecurityValidator** - Validates incoming messages on the server side

**Flow:**

1. Client → Server: RequestAccess
2. Server → Client: Token + HMAC Key
3. Client → Server: Secure Message (with SafetyMetadata)
4. Server → Validato: Validate metadata
5. Validato → Server: Pass/Fail result

Modules
-------

The Security Framework consists of the following modules:

* **Security Levels** - Defines the 5 security levels and related enums
* **Security Manager** - Server-side authentication and session management
* **Security Client** - Client-side wrappers for secure communication
* **Security Validato** - Validates incoming messages and signatures
* **Crypto Helpers** - Cryptographic operations (HMAC, RSA, certificates)

For detailed API documentation, see :doc:`security/api`.

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
