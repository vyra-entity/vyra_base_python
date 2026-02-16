"""
VYRA Security Framework - Example Server Node
==============================================

This example demonstrates how to create a secure VYRA module that:
1. Provides the RequestAccess service for authentication
2. Validates incoming messages with SafetyMetadata
3. Enforces security levels for service calls

This node acts as "Module B" that other modules authenticate with.

Author: VYRA Framework Team
License: Proprietary
"""
import logging
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

from vyra_base.interfaces.srv import VBASERequestAccess
from vyra_base.security.security_manager import SecurityManager
from vyra_base.security.security_validator import SecurityValidator, create_secure_subscription
from vyra_base.security.security_levels import SecurityLevel

logger = logging.getLogger(__name__)


class SecureServerNode(Node, SecurityManager):
    """
    Example secure server node with VYRA security framework.
    
    This node demonstrates:
    - Implementing RequestAccess service
    - Validating incoming messages with SafetyMetadata
    - Using security decorators
    """

    def __init__(self):
        """Initialize the secure server node."""
        # Initialize ROS2 Node
        Node.__init__(self, 'secure_server_example')
        
        # Initialize SecurityManager with Level 4 (HMAC)
        # For Level 3+, provide module_passwords dictionary
        SecurityManager.__init__(
            self,
            max_security_level=SecurityLevel.HMAC,
            session_duration_seconds=3600,
            module_passwords={
                # Add authorized module IDs and their passwords
                # "module-uuid-1": "secret-password-1",
                # "module-uuid-2": "secret-password-2",
            }
        )
        
        # Create RequestAccess service
        self.access_service = self.create_service(
            VBASERequestAccess,
            'secure_server_example/request_access',
            self.handle_request_access_callback
        )
        
        # Create validator
        self.validator = SecurityValidator(
            security_manager=self,
            strict_mode=True
        )
        
        # Create a secure subscription example
        # (Requires a message type with SafetyMetadata field)
        # self.subscription = create_secure_subscription(
        #     self,
        #     SecureMessage,  # Custom message type with safety_metadata field
        #     'secure_topic',
        #     self.secure_message_callback,
        #     self.validator,
        #     required_level=SecurityLevel.HMAC
        # )
        
        # Create a timer for session cleanup
        self.cleanup_timer = self.create_timer(
            60.0,  # Every 60 seconds
            self.cleanup_expired_sessions_callback
        )
        
        logger.info("SecureServerNode initialized with HMAC security (Level 4)")

    def handle_request_access_callback(self, request, response):
        """
        Handle RequestAccess service calls.
        
        This is the main authentication entry point for clients.
        """
        logger.info(
            f"Access request from {request.module_name} "
            f"({request.module_id.uuid}) for SL{request.requested_sl}"
        )
        
        try:
            # Call SecurityManager's handle method
            import asyncio
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            
            result = loop.run_until_complete(
                self.request_access_impl(
                    request.module_name,
                    request.module_id,
                    request.requested_role,
                    request.requested_sl,
                    request.password,
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
                logger.info(f"✅ Access granted to {request.module_name} with SL{granted_sl}")
            else:
                logger.warn(f"❌ Access denied to {request.module_name}: {message}")
            
        except Exception as e:
            logger.error(f"Error handling access request: {e}")
            response.success = False
            response.message = f"Internal error: {str(e)}"
            response.granted_sl = 0
        
        return response

    def secure_message_callback(self, msg):
        """
        Example callback for secure messages.
        
        Messages are automatically validated by the SecurityValidator
        before reaching this callback.
        """
        logger.info(f"Received secure message: {msg.data}")
        # Process message knowing it passed security validation

    def cleanup_expired_sessions_callback(self):
        """Timer callback to clean up expired sessions."""
        import asyncio
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self.cleanup_expired_sessions())
        loop.close()
        
        active_count = self.get_session_count()
        if active_count > 0:
            logger.debug(f"Active sessions: {active_count}")


def main(args=None):
    """Main entry point for the secure server node."""
    rclpy.init(args=args)
    
    try:
        node = SecureServerNode()
        
        logger.info("Secure Server Node is running...")
        logger.info("Waiting for access requests on 'secure_server_example/request_access'")
        
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            logger.info("Shutting down...")
        finally:
            executor.shutdown()
            node.destroy_node()
            
    except Exception as e:
        logger.error(f"Error in main: {e}")
        raise
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
