"""
VYRA Security Framework - Example Client Node
==============================================

This example demonstrates how to create a secure client that:
1. Authenticates with a server using RequestAccess
2. Sends secure messages with SafetyMetadata
3. Calls secure services with automatic signature generation

This node acts as "Module A" that authenticates with Module B.

Author: VYRA Framework Team
License: Proprietary
"""

import rclpy
import uuid

from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from unique_identifier_msgs.msg import UUID
from typing import Optional
from vyra_base.interfaces.srv import VBASERequestAccess
from vyra_base.security.security_client import (
    SecurityContext,
    SecurePublisher,
    create_security_context
)
from vyra_base.security.security_levels import SecurityLevel
from vyra_base.helper.logger import Logger


class SecureClientNode(Node):
    """
    Example secure client node with VYRA security framework.
    
    This node demonstrates:
    - Requesting access from a secure server
    - Creating a security context
    - Using SecurePublisher for secure communication
    """

    def __init__(self):
        """Initialize the secure client node."""
        super().__init__('secure_client_example')
        
        # Module identification
        self.module_name = 'secure_client_example'
        self.module_id = str(uuid.uuid4())
        
        # Security context (will be set after authentication)
        self.security_context: Optional[SecurityContext] = None
        
        # Create client for RequestAccess service
        self.access_client = self.create_client(
            VBASERequestAccess,
            'secure_server_example/request_access'
        )
        
        Logger.info(f"SecureClientNode initialized (ID: {self.module_id})")
        
        # Authenticate with server
        self.authenticate_with_server()

    def authenticate_with_server(self):
        """
        Authenticate with the secure server using RequestAccess.
        """
        Logger.info("Requesting access from secure server...")
        
        # Wait for service
        if not self.access_client.wait_for_service(timeout_sec=10.0):
            Logger.error("RequestAccess service not available!")
            return False
        
        # Create request
        request = VBASERequestAccess.Request()
        request.module_name = self.module_name
        
        uuid_obj = uuid.UUID(self.module_id)
        request.module_id = UUID(uuid=list(uuid_obj.bytes))
        
        request.requested_role = "client_operator"
        request.requested_sl = SecurityLevel.HMAC.value  # Request Level 4
        request.password = ""  # Required for Level 3+, set to module's password
        request.certificate_csr = ""  # Not needed for Level 4
        
        # Call service
        try:
            future = self.access_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if not future.done():
                Logger.error("Access request timed out!")
                return False
            
            response = future.result()
            
            if response is None:
                Logger.error("No response from access service!")
                return False

            if response.success:
                Logger.info(f"✅ Access granted with SL{response.granted_sl}")
                Logger.info(f"Session token: {response.session_token[:16]}...")
                
                # Create security context
                self.security_context = create_security_context(
                    module_name=self.module_name,
                    module_id=self.module_id,
                    security_level=response.granted_sl,
                    session_token=response.session_token,
                    hmac_key=response.hmac_key
                )
                
                # Now we can create secure publishers/clients
                self.setup_secure_communication()
                
                return True
            else:
                Logger.error(f"❌ Access denied: {response.message}")
                return False
                
        except Exception as e:
            Logger.error(f"Error during authentication: {e}")
            return False

    def setup_secure_communication(self):
        """
        Set up secure publishers and clients after authentication.
        """
        # Example: Create a secure publisher
        # Note: The message type must have a safety_metadata field
        # self.secure_pub = SecurePublisher(
        #     self,
        #     SecureMessage,  # Custom message type
        #     'secure_topic',
        #     self.security_context,
        #     10
        # )
        
        # Create a timer to send secure messages
        # self.publish_timer = self.create_timer(
        #     2.0,  # Every 2 seconds
        #     self.publish_secure_message
        # )
        
        Logger.info("Secure communication channels established")

    def publish_secure_message(self):
        """
        Publish a secure message with automatic SafetyMetadata.
        """
        # Example of publishing secure message:
        # msg = SecureMessage()
        # msg.data = f"Secure message at {time.time()}"
        # self.secure_pub.publish(msg)
        # Logger.debug("Published secure message")
        pass


def main(args=None):
    """Main entry point for the secure client node."""
    rclpy.init(args=args)
    
    try:
        node = SecureClientNode()
        
        if node.security_context:
            Logger.info("Secure Client Node is running with authentication")
            
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            
            try:
                executor.spin()
            except KeyboardInterrupt:
                Logger.info("Shutting down...")
            finally:
                executor.shutdown()
                node.destroy_node()
        else:
            Logger.error("Failed to authenticate, shutting down")
            node.destroy_node()
            
    except Exception as e:
        Logger.error(f"Error in main: {e}")
        raise
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
