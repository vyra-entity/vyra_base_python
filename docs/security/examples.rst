Security Framework Examples
===========================

This section provides complete, working examples of the VYRA Security Framework.

Example Overview
----------------

.. list-table::
   :header-rows: 1
   :widths: 30 50 20

   * - Example
     - Description
     - Level
   * - :ref:`basic-server`
     - Minimal secure server
     - 4 (HMAC)
   * - :ref:`basic-client`
     - Minimal secure client
     - 4 (HMAC)
   * - :ref:`secure-publisher`
     - Secure message publishing
     - 4 (HMAC)
   * - :ref:`secure-service`
     - Secure service implementation
     - 4 (HMAC)
   * - :ref:`certificate-auth`
     - Certificate-based auth
     - 5 (Digital)
   * - :ref:`vyra-integration`
     - Integration with VyraEntity
     - 4 (HMAC)

.. _basic-server:

Basic Secure Server
-------------------

A minimal server providing authentication services.

.. code-block:: python

    """
    Basic Secure Server Example
    
    Demonstrates:
    - SecurityManager integration
    - RequestAccess service
    - Session management
    """

    import rclpy
    from rclpy.node import Node
    from vyra_base.security.security_manager import SecurityManager
    from vyra_base.security.security_levels import SecurityLevel
    from vyra_base.interfaces.srv import VBASERequestAccess
    import asyncio

    class BasicSecureServer(Node, SecurityManager):
        """Basic secure server with HMAC support."""
        
        def __init__(self):
            # Initialize Node
            Node.__init__(self, 'basic_secure_server')
            
            # Initialize SecurityManager
            SecurityManager.__init__(
                self,
                max_security_level=SecurityLevel.HMAC,
                session_duration_seconds=3600
            )
            
            # Create RequestAccess service
            self.service = self.create_service(
                VBASERequestAccess,
                f'{self.get_name()}/request_access',
                self.handle_request_access_callback
            )
            
            # Session cleanup timer
            self.cleanup_timer = self.create_timer(
                60.0,
                self.cleanup_callback
            )
            
            self.get_logger().info(
                f"Server '{self.get_name()}' ready with "
                f"max security level {SecurityLevel.HMAC.name}"
            )
        
        def handle_request_access_callback(self, request, response):
            """Handle access requests from clients."""
            
            self.get_logger().info(
                f"Access request from {request.module_name} "
                f"for SL{request.requested_sl}"
            )
            
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
            
            if success:
                self.get_logger().info(
                    f"✅ Access granted to {request.module_name} "
                    f"with SL{granted_sl}"
                )
            else:
                self.get_logger().warn(
                    f"❌ Access denied to {request.module_name}: {message}"
                )
            
            return response
        
        def cleanup_callback(self):
            """Periodic session cleanup."""
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(self.cleanup_expired_sessions())
            loop.close()
            
            count = self.get_session_count()
            if count > 0:
                self.get_logger().debug(f"Active sessions: {count}")

    def main(args=None):
        rclpy.init(args=args)
        node = BasicSecureServer()
        
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()

    if __name__ == '__main__':
        main()

.. _basic-client:

Basic Secure Client
-------------------

A minimal client that authenticates with a server.

.. code-block:: python

    """
    Basic Secure Client Example
    
    Demonstrates:
    - Requesting access
    - Creating security context
    - Session management
    """

    import rclpy
    from rclpy.node import Node
    import uuid
    from vyra_base.interfaces.srv import VBASERequestAccess
    from vyra_base.security.security_client import create_security_context
    from vyra_base.security.security_levels import SecurityLevel
    from unique_identifier_msgs.msg import UUID as UUIDMsg

    class BasicSecureClient(Node):
        """Basic secure client that authenticates with server."""
        
        def __init__(self):
            super().__init__('basic_secure_client')
            
            # Module identification
            self.module_name = 'basic_secure_client'
            self.module_uuid = uuid.uuid4()
            
            # Security context (set after authentication)
            self.security_context = None
            
            # Create access client
            self.access_client = self.create_client(
                VBASERequestAccess,
                'basic_secure_server/request_access'
            )
            
            self.get_logger().info(
                f"Client initialized with UUID: {self.module_uuid}"
            )
            
            # Authenticate
            self.authenticate()
        
        def authenticate(self):
            """Request access from server."""
            
            self.get_logger().info("Requesting access from server...")
            
            # Wait for service
            if not self.access_client.wait_for_service(timeout_sec=10.0):
                self.get_logger().error(
                    "RequestAccess service not available!"
                )
                return False
            
            # Create request
            request = VBASERequestAccess.Request()
            request.module_name = self.module_name
            request.module_id = UUIDMsg(uuid=list(self.module_uuid.bytes))
            request.requested_role = "client_operator"
            request.requested_sl = SecurityLevel.HMAC.value
            request.certificate_csr = ""
            
            # Call service
            try:
                future = self.access_client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                
                if not future.done():
                    self.get_logger().error("Access request timed out!")
                    return False
                
                response = future.result()
                
                if response.success:
                    self.get_logger().info(
                        f"✅ Access granted with SL{response.granted_sl}"
                    )
                    self.get_logger().info(
                        f"Session token: {response.session_token[:16]}..."
                    )
                    
                    # Create security context
                    self.security_context = create_security_context(
                        module_id=str(self.module_uuid),
                        security_level=response.granted_sl,
                        session_token=response.session_token,
                        hmac_key=response.hmac_key
                    )
                    
                    return True
                else:
                    self.get_logger().error(
                        f"❌ Access denied: {response.message}"
                    )
                    return False
                    
            except Exception as e:
                self.get_logger().error(f"Authentication error: {e}")
                return False

    def main(args=None):
        rclpy.init(args=args)
        node = BasicSecureClient()
        
        if node.security_context:
            try:
                rclpy.spin(node)
            except KeyboardInterrupt:
                pass
            finally:
                node.destroy_node()
        else:
            node.destroy_node()
        
        rclpy.shutdown()

    if __name__ == '__main__':
        main()

.. _secure-publisher:

Secure Publisher Example
-------------------------

Publishing messages with automatic security metadata.

.. code-block:: python

    """
    Secure Publisher Example
    
    Demonstrates:
    - SecurePublisher usage
    - Automatic SafetyMetadata
    - Message signing
    """

    from vyra_base.security.security_client import SecurePublisher
    from std_msgs.msg import String

    class PublisherClient(BasicSecureClient):
        """Client with secure publishing capability."""
        
        def __init__(self):
            super().__init__()
            
            if self.security_context:
                self.setup_publisher()
        
        def setup_publisher(self):
            """Setup secure publisher after authentication."""
            
            # Note: In production, use message types with safety_metadata field
            # For this example, we'll show the API usage
            
            self.get_logger().info("Setting up secure publisher...")
            
            # Create secure publisher
            # self.secure_pub = SecurePublisher(
            #     self,
            #     YourSecureMessage,  # Must have safety_metadata field
            #     'secure_topic',
            #     self.security_context,
            #     10
            # )
            
            # Create timer to publish
            self.timer = self.create_timer(
                2.0,
                self.publish_message
            )
        
        def publish_message(self):
            """Publish a secure message."""
            
            # Example publication:
            # msg = YourSecureMessage()
            # msg.data = f"Secure message at {time.time()}"
            # 
            # # SafetyMetadata added automatically
            # self.secure_pub.publish(msg)
            # 
            # self.get_logger().debug("Published secure message")
            
            pass

.. _secure-service:

Secure Service Example
-----------------------

Implementing a secure service with validation.

.. code-block:: python

    """
    Secure Service Example
    
    Demonstrates:
    - Service with security validation
    - SecurityValidator usage
    - Error handling
    """

    from vyra_base.security.security_validator import SecurityValidator
    from vyra_base.security.security_levels import (
        SecurityLevel,
        SecurityError
    )

    class SecureServiceServer(BasicSecureServer):
        """Server with secure service."""
        
        def __init__(self):
            super().__init__()
            
            # Create validator
            self.validator = SecurityValidator(
                security_manager=self,
                strict_mode=True
            )
            
            # Create secure service
            # from your_interfaces.srv import YourSecureService
            # self.secure_service = self.create_service(
            #     YourSecureService,
            #     'secure_operation',
            #     self.secure_operation_callback
            # )
            
            self.get_logger().info("Secure service ready")
        
        def secure_operation_callback(self, request, response):
            """Service callback with security validation."""
            
            try:
                # Validate security metadata
                session = self.validator.validate_metadata(
                    request.safety_metadata,
                    required_level=SecurityLevel.HMAC
                )
                
                self.get_logger().info(
                    f"Validated request from {session.module_name}"
                )
                
                # Process authenticated request
                response.success = True
                response.data = "Operation completed securely"
                
            except SecurityError as e:
                self.get_logger().warn(f"Security validation failed: {e}")
                response.success = False
                response.message = f"Security error: {str(e)}"
            
            except Exception as e:
                self.get_logger().error(f"Processing error: {e}")
                response.success = False
                response.message = f"Internal error: {str(e)}"
            
            return response

.. _certificate-auth:

Certificate-Based Authentication (Level 5)
-------------------------------------------

Using digital signatures with certificates.

.. code-block:: python

    """
    Certificate-Based Authentication Example
    
    Demonstrates:
    - Level 5 security
    - Certificate generation
    - CSR signing
    """

    from pathlib import Path
    from vyra_base.helper.crypto_helper import (
        generate_rsa_keypair,
        create_csr,
        create_self_signed_cert,
        save_private_key
    )
    from vyra_base.security.security_levels import SecurityLevel
    from cryptography import x509
    from cryptography.hazmat.primitives import serialization

    class CertificateServer(Node, SecurityManager):
        """Server with certificate signing capability."""
        
        def __init__(self):
            # Setup CA infrastructure
            ca_key_path = Path("/tmp/vyra_ca_key.pem")
            ca_cert_path = Path("/tmp/vyra_ca_cert.pem")
            
            if not ca_key_path.exists():
                self.setup_ca(ca_key_path, ca_cert_path)
            
            # Initialize with Level 5 support
            Node.__init__(self, 'certificate_server')
            SecurityManager.__init__(
                self,
                max_security_level=SecurityLevel.DIGITAL_SIGNATURE,
                ca_key_path=ca_key_path,
                ca_cert_path=ca_cert_path
            )
            
            self.service = self.create_service(
                VBASERequestAccess,
                'certificate_server/request_access',
                self.handle_request_access_callback
            )
        
        @staticmethod
        def setup_ca(key_path: Path, cert_path: Path):
            """Setup Certificate Authority."""
            print("Setting up CA infrastructure...")
            
            # Generate CA key pair
            ca_private_key, _ = generate_rsa_keypair()
            
            # Create self-signed CA certificate
            ca_cert = create_self_signed_cert(
                "VYRA Test CA",
                ca_private_key
            )
            
            # Save CA key
            save_private_key(ca_private_key, key_path)
            
            # Save CA certificate
            ca_cert_pem = ca_cert.public_bytes(
                serialization.Encoding.PEM
            )
            cert_path.write_bytes(ca_cert_pem)
            
            print(f"✓ CA certificate saved to {cert_path}")

    class CertificateClient(BasicSecureClient):
        """Client with certificate-based authentication."""
        
        def __init__(self):
            # Generate key pair
            self.private_key, _ = generate_rsa_keypair()
            
            # Save private key
            self.private_key_path = Path("/tmp/client_key.pem")
            save_private_key(self.private_key, self.private_key_path)
            
            super().__init__()
        
        def authenticate(self):
            """Authenticate with certificate."""
            
            # Create CSR
            csr = create_csr(
                module_name=self.module_name,
                module_id=str(self.module_uuid),
                private_key=self.private_key
            )
            
            # Wait for service
            if not self.access_client.wait_for_service(timeout_sec=10.0):
                return False
            
            # Request with CSR
            request = VBASERequestAccess.Request()
            request.module_name = self.module_name
            request.module_id = UUIDMsg(uuid=list(self.module_uuid.bytes))
            request.requested_role = "certificate_client"
            request.requested_sl = SecurityLevel.DIGITAL_SIGNATURE.value
            request.certificate_csr = csr
            
            future = self.access_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            
            response = future.result()
            
            if response.success:
                # Save certificate
                cert_path = Path("/tmp/client_cert.pem")
                cert_path.write_text(response.certificate)
                
                # Create context with certificate
                self.security_context = create_security_context(
                    module_id=str(self.module_uuid),
                    security_level=response.granted_sl,
                    session_token=response.session_token,
                    certificate=response.certificate,
                    private_key_path=str(self.private_key_path)
                )
                
                self.get_logger().info(
                    "✅ Certificate-based authentication successful"
                )
                return True
            
            return False

.. _vyra-integration:

Integration with VyraEntity
----------------------------

Integrating security into a VYRA module.

.. code-block:: python

    """
    VyraEntity Integration Example
    
    Demonstrates:
    - SecurityManager as mixin
    - Integration with module lifecycle
    - Security service registration
    """

    from vyra_base.com.entity import VyraEntity
    from vyra_base.security.security_manager import SecurityManager
    from vyra_base.security.security_validator import SecurityValidator
    from vyra_base.security.security_levels import SecurityLevel

    class SecureVyraModule(VyraEntity, SecurityManager):
        """VyraEntity with integrated security."""
        
        def __init__(self, *args, **kwargs):
            # Initialize VyraEntity
            super().__init__(*args, **kwargs)
            
            # Initialize SecurityManager
            SecurityManager.__init__(
                self,
                max_security_level=SecurityLevel.HMAC,
                session_duration_seconds=3600
            )
            
            # Create validator
            self.security_validator = SecurityValidator(
                security_manager=self,
                strict_mode=True
            )
            
            self.get_logger().info(
                "Secure VYRA module initialized with HMAC support"
            )
        
        async def setup_security_service(self):
            """Setup RequestAccess service."""
            # Service is auto-registered via vyra_security_meta.json
            # Additional setup can be done here
            pass
        
        def validate_incoming_request(self, request):
            """Validate request security metadata."""
            try:
                session = self.security_validator.validate_metadata(
                    request.safety_metadata,
                    required_level=SecurityLevel.HMAC
                )
                return True, session
            except SecurityError as e:
                self.get_logger().warn(f"Validation failed: {e}")
                return False, None

Running the Examples
--------------------

**Prerequisites:**

.. code-block:: bash

    cd /path/to/vyra_base_python
    colcon build
    source install/setup.bash

**Run Basic Server:**

.. code-block:: bash

    python3 examples/security/secure_server_node.py

**Run Basic Client:**

.. code-block:: bash

    python3 examples/security/secure_client_node.py

**Expected Output:**

.. code-block:: text

    # Server:
    [INFO] Server 'basic_secure_server' ready with max security level HMAC
    [INFO] Access request from basic_secure_client for SL4
    [INFO] ✅ Access granted to basic_secure_client with SL4

    # Client:
    [INFO] Client initialized with UUID: abc123...
    [INFO] Requesting access from server...
    [INFO] ✅ Access granted with SL4
    [INFO] Session token: 1a2b3c4d5e6f7g8h...

Further Reading
---------------

- :doc:`overview` - Complete framework documentation
- :doc:`quickstart` - Quick start guide
- :doc:`api` - API reference
- Example code: ``/examples/security/``

