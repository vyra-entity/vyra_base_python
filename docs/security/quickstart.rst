Security Framework Quick Start
==============================

This guide will get you started with the VYRA Security Framework in 15 minutes.

Installation
------------

1. **Install Dependencies**

   .. code-block:: bash

      pip install cryptography

2. **Build Interfaces**

   .. code-block:: bash

      cd /path/to/vyra_base_python
      colcon build
      source install/setup.bash

3. **Verify Installation**

   .. code-block:: python

      from vyra_base.security.security_manager import SecurityManager
      from vyra_base.security.security_levels import SecurityLevel
      print("✓ Security framework imported successfully")

Server Implementation (5 minutes)
----------------------------------

Create a secure server module that authenticates clients.

**Step 1: Create Node with SecurityManager**

.. code-block:: python

    from rclpy.node import Node
    from vyra_base.security.security_manager import SecurityManager
    from vyra_base.security.security_levels import SecurityLevel
    from vyra_base.interfaces.srv import VBASERequestAccess
    import asyncio

    class MySecureServer(Node, SecurityManager):
        def __init__(self):
            # Initialize Node
            Node.__init__(self, 'secure_server')
            
            # Initialize SecurityManager with HMAC support (Level 4)
            SecurityManager.__init__(
                self,
                max_security_level=SecurityLevel.HMAC,
                session_duration_seconds=3600  # 1 hour
            )
            
            # Create RequestAccess service
            self.srv = self.create_service(
                VBASERequestAccess,
                'secure_server/request_access',
                self.handle_access
            )
            
            print("✓ Secure server ready")

**Step 2: Implement Access Handler**

.. code-block:: python

    def handle_access(self, request, response):
        """Handle client authentication requests."""
        
        # Call async handler
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
        
        # Unpack result
        (success, message, token, hmac, cert, expires, granted) = result
        
        # Fill response
        response.success = success
        response.message = message
        response.session_token = token
        response.hmac_key = hmac
        response.certificate = cert
        response.expires_at.sec = int(expires.timestamp())
        response.granted_sl = granted
        
        return response

**Step 3: Run Server**

.. code-block:: python

    import rclpy

    def main():
        rclpy.init()
        node = MySecureServer()
        rclpy.spin(node)
        rclpy.shutdown()

    if __name__ == '__main__':
        main()

Client Implementation (5 minutes)
----------------------------------

Create a client that authenticates and sends secure messages.

**Step 1: Request Access**

.. code-block:: python

    import rclpy
    from rclpy.node import Node
    import uuid
    from vyra_base.interfaces.srv import VBASERequestAccess
    from unique_identifier_msgs.msg import UUID as UUIDMsg

    class MySecureClient(Node):
        def __init__(self):
            super().__init__('secure_client')
            
            # Generate module UUID
            self.module_uuid = uuid.uuid4()
            
            # Create access client
            self.client = self.create_client(
                VBASERequestAccess,
                'secure_server/request_access'
            )
            
            # Authenticate
            self.authenticate()

**Step 2: Authenticate**

.. code-block:: python

    def authenticate(self):
        """Request access from server."""
        
        # Wait for service
        if not self.client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("Server not available")
        
        # Create request
        request = VBASERequestAccess.Request()
        request.module_name = "my_client"
        request.module_id = UUIDMsg(uuid=list(self.module_uuid.bytes))
        request.requested_role = "operator"
        request.requested_sl = 4  # Request HMAC (Level 4)
        request.certificate_csr = ""
        
        # Call service
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        
        if not response.success:
            raise RuntimeError(f"Access denied: {response.message}")
        
        # Store credentials
        self.security_context = self._create_context(response)
        
        print(f"✓ Authenticated with Security Level {response.granted_sl}")

**Step 3: Create Security Context**

.. code-block:: python

    from vyra_base.security.security_client import create_security_context

    def _create_context(self, response):
        """Create security context from authentication response."""
        return create_security_context(
            module_id=str(self.module_uuid),
            security_level=response.granted_sl,
            session_token=response.session_token,
            hmac_key=response.hmac_key
        )

**Step 4: Use Secure Communication**

.. code-block:: python

    from vyra_base.security.security_client import SecurePublisher

    def setup_communication(self):
        """Setup secure publishers after authentication."""
        
        # Create secure publisher
        # Note: Message type must have safety_metadata field
        self.secure_pub = SecurePublisher(
            self,
            MyMessage,  # Your message type
            'secure_topic',
            self.security_context,
            10
        )
    
    def send_message(self):
        """Send a secure message."""
        msg = MyMessage()
        msg.data = "Hello, secure world!"
        self.secure_pub.publish(msg)  # SafetyMetadata added automatically

Message Definition (2 minutes)
-------------------------------

Add SafetyMetadata to your custom messages.

**Create Message File**

.. code-block:: text

    # my_interfaces/msg/MyMessage.msg
    string data
    int32 value
    vyra_base/SafetyMetadata safety_metadata

**Build Interfaces**

.. code-block:: bash

    cd your_workspace
    colcon build --packages-select my_interfaces
    source install/setup.bash

Validation (3 minutes)
-----------------------

Add validation to server-side service callbacks.

**Step 1: Create Validator**

.. code-block:: python

    from vyra_base.security.security_validator import SecurityValidator

    def __init__(self):
        # ... previous initialization ...
        
        # Create validator
        self.validator = SecurityValidator(
            security_manager=self,
            strict_mode=True
        )

**Step 2: Validate in Callbacks**

.. code-block:: python

    def my_service_callback(self, request, response):
        """Service callback with security validation."""
        
        try:
            # Validate request
            self.validator.validate_metadata(
                request.safety_metadata,
                required_level=SecurityLevel.HMAC
            )
            
            # Process authenticated request
            response.success = True
            response.data = "Processed securely"
            
        except SecurityError as e:
            response.success = False
            response.message = f"Security validation failed: {e}"
        
        return response

Complete Example
----------------

Here's a minimal working example combining all steps.

**Server (secure_server.py)**

.. code-block:: python

    import rclpy
    from rclpy.node import Node
    from vyra_base.security.security_manager import SecurityManager
    from vyra_base.security.security_validator import SecurityValidator
    from vyra_base.security.security_levels import SecurityLevel
    from vyra_base.interfaces.srv import VBASERequestAccess
    import asyncio

    class QuickServer(Node, SecurityManager):
        def __init__(self):
            Node.__init__(self, 'quick_server')
            SecurityManager.__init__(self, max_security_level=SecurityLevel.HMAC)
            
            self.srv = self.create_service(
                VBASERequestAccess,
                'quick_server/request_access',
                self.handle_access
            )
            
            self.validator = SecurityValidator(self, strict_mode=True)
            print("Server ready on 'quick_server/request_access'")
        
        def handle_access(self, request, response):
            loop = asyncio.new_event_loop()
            result = loop.run_until_complete(
                self.handle_request_access(
                    request.module_name, request.module_id,
                    request.requested_role, request.requested_sl,
                    request.certificate_csr
                )
            )
            loop.close()
            
            (success, msg, token, hmac, cert, expires, granted) = result
            
            response.success = success
            response.message = msg
            response.session_token = token
            response.hmac_key = hmac
            response.certificate = cert
            response.expires_at.sec = int(expires.timestamp())
            response.granted_sl = granted
            
            return response

    def main():
        rclpy.init()
        node = QuickServer()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()

    if __name__ == '__main__':
        main()

**Client (secure_client.py)**

.. code-block:: python

    import rclpy
    from rclpy.node import Node
    import uuid
    from vyra_base.interfaces.srv import VBASERequestAccess
    from vyra_base.security.security_client import create_security_context
    from unique_identifier_msgs.msg import UUID as UUIDMsg

    class QuickClient(Node):
        def __init__(self):
            super().__init__('quick_client')
            
            self.module_uuid = uuid.uuid4()
            self.client = self.create_client(
                VBASERequestAccess,
                'quick_server/request_access'
            )
            
            if self.authenticate():
                print(f"✓ Authenticated successfully!")
                # Now you can use secure communication
        
        def authenticate(self):
            if not self.client.wait_for_service(timeout_sec=10.0):
                print("✗ Server not available")
                return False
            
            request = VBASERequestAccess.Request()
            request.module_name = "quick_client"
            request.module_id = UUIDMsg(uuid=list(self.module_uuid.bytes))
            request.requested_role = "operator"
            request.requested_sl = 4
            request.certificate_csr = ""
            
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            
            response = future.result()
            
            if response.success:
                self.security_context = create_security_context(
                    module_id=str(self.module_uuid),
                    security_level=response.granted_sl,
                    session_token=response.session_token,
                    hmac_key=response.hmac_key
                )
                return True
            else:
                print(f"✗ Access denied: {response.message}")
                return False

    def main():
        rclpy.init()
        node = QuickClient()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()

    if __name__ == '__main__':
        main()

Running the Example
--------------------

**Terminal 1 (Server):**

.. code-block:: bash

    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    python3 secure_server.py

**Terminal 2 (Client):**

.. code-block:: bash

    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    python3 secure_client.py

Expected Output:

.. code-block:: text

    # Terminal 1:
    Server ready on 'quick_server/request_access'
    ✅ Access granted to quick_client with SL4

    # Terminal 2:
    ✓ Authenticated successfully!

Next Steps
----------

Now that you have a working secure system:

1. **Add Message Security**
   
   - Define messages with SafetyMetadata
   - Use SecurePublisher for automatic signing
   - See :doc:`overview` for details

2. **Implement Validation**
   
   - Add SecurityValidator to services
   - Handle SecurityError exceptions
   - Test with invalid credentials

3. **Explore Higher Levels**
   
   - Try Level 5 (Digital Signatures)
   - Set up certificate infrastructure
   - See certificate examples

4. **Read Full Documentation**
   
   - :doc:`overview` - Complete framework guide
   - :doc:`api` - API reference
   - :doc:`examples` - More examples

Common Issues
-------------

**Service Not Available**

.. code-block:: python

    if not client.wait_for_service(timeout_sec=10.0):
        raise RuntimeError("Server not available")
    
    # Solution: Ensure server is running first

**Missing SafetyMetadata Field**

.. code-block:: text

    # Error: Message type doesn't have safety_metadata field
    
    # Solution: Add to message definition:
    vyra_base/SafetyMetadata safety_metadata

**HMAC Signature Mismatch**

.. code-block:: python

    # Cause: HMAC key not stored correctly
    
    # Solution: Ensure key is saved from authentication:
    hmac_key = response.hmac_key  # Save this!

**Import Errors**

.. code-block:: bash

    # Solution: Source workspace
    source install/setup.bash

Resources
---------

- **Examples**: ``/examples/security/``
- **Full Guide**: :doc:`overview`
- **API Docs**: :doc:`api`
- **Integration**: ``docs/security/INTEGRATION_GUIDE.md``

Support
-------

For issues:

1. Check :doc:`overview` troubleshooting section
2. Review example code
3. Enable debug logging:

   .. code-block:: python

      from vyra_base.helper.logger import Logger
      import logging
      Logger.setLevel(logging.DEBUG)

