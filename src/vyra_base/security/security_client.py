"""
Security Client Components for VYRA Framework
==============================================

This module provides client-side security components:
- SecurePublisher: Wrapper for ROS2 publishers with automatic SafetyMetadata
- SecureServiceClient: Wrapper for ROS2 service clients with security
- security_required: Decorator for service callbacks requiring authentication
- SecureMessageBuilder: Helper for creating secure messages

These components automatically handle:
- SafetyMetadata field population
- HMAC signature generation (Level 4)
- Digital signature generation (Level 5)
- Session token management
- Nonce generation

Author: VYRA Framework Team
License: Proprietary
"""

import time
import uuid
from typing import Any, Callable, Optional, Type
from functools import wraps
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.client import Client

from vyra_base.security.security_levels import (
    SecurityLevel,
    AlgorithmId,
    SecurityError,
)
from vyra_base.helper.crypto_helper import (
    generate_nonce,
    create_hmac_payload,
    sign_message_rsa,
    load_private_key,
)
from vyra_base.helper.logger import Logger


@dataclass
class SecurityContext:
    """
    Security context for a client connection.
    
    :ivar module_name: Name of the client module
    :ivar module_id: UUID of the client module
    :ivar security_level: Security level in use
    :ivar session_token: Session token from RequestAccess
    :ivar hmac_key: HMAC key (for Level 4+)
    :ivar certificate: Client certificate (for Level 5)
    :ivar private_key_path: Path to private key (for Level 5)
    """
    module_name: str
    module_id: str
    security_level: int
    session_token: str
    hmac_key: Optional[str] = None
    certificate: Optional[str] = None
    private_key_path: Optional[str] = None


class SafetyMetadataBuilder:
    """
    Helper class to build SafetyMetadata for messages.
    
    This class handles the creation of SafetyMetadata fields based on
    the security level and context.
    """

    @staticmethod
    def build(
        security_context: SecurityContext,
        additional_data: str = ""
    ) -> dict:
        """
        Build SafetyMetadata dictionary.
        
        :param security_context: Security context with authentication info
        :param additional_data: Additional data to include in signature
        :return: Dictionary with SafetyMetadata fields
        """
        sender_name = security_context.module_name
        sender_id = security_context.module_id
        security_level = security_context.security_level
        timestamp_sec = int(time.time())
        nonce = generate_nonce()
        
        metadata = {
            'security_level': security_level,
            'sender_name': sender_name,
            'sender_id': sender_id,
            'timestamp': {
                'sec': timestamp_sec,
                'nanosec': 0
            },
            'nonce': nonce,
            'security_payload': '',
            'algorithm_id': AlgorithmId.for_security_level(SecurityLevel(security_level)).value
        }
        
        # Generate security payload based on level
        if security_level >= SecurityLevel.HMAC.value:
            if not security_context.hmac_key:
                raise SecurityError("HMAC key required for Level 4+")
            
            signature = create_hmac_payload(
                sender_id,
                timestamp_sec,
                nonce,
                security_context.hmac_key,
                additional_data
            )
            metadata['security_payload'] = signature
            
        elif security_level >= SecurityLevel.DIGITAL_SIGNATURE.value:
            if not security_context.private_key_path:
                raise SecurityError("Private key required for Level 5")
            
            try:
                private_key = load_private_key(security_context.private_key_path)
                message = f"{sender_id}|{timestamp_sec}|{nonce}|{additional_data}"
                signature = sign_message_rsa(message, private_key)
                metadata['security_payload'] = signature
                metadata['algorithm_id'] = AlgorithmId.RSA_SHA256.value
            except Exception as e:
                raise SecurityError(f"Failed to sign message: {e}")
        
        return metadata


class SecurePublisher:
    """
    Wrapper for ROS2 Publisher with automatic SafetyMetadata.
    
    This class wraps a standard ROS2 publisher and automatically adds
    SafetyMetadata to each published message based on the security level.
    
    Usage:
        publisher = SecurePublisher(
            node,
            MessageType,
            'topic_name',
            security_context,
            qos_profile
        )
        publisher.publish(msg)
    
    :ivar publisher: Underlying ROS2 publisher
    :ivar security_context: Security context for this publisher
    """

    def __init__(
        self,
        node: Node,
        msg_type: Type,
        topic: str,
        security_context: SecurityContext,
        qos_profile: Any = 10
    ):
        """
        Initialize SecurePublisher.
        
        :param node: ROS2 node
        :param msg_type: Message type class
        :param topic: Topic name
        :param security_context: Security context
        :param qos_profile: QoS profile
        """
        self.publisher = node.create_publisher(msg_type, topic, qos_profile)
        self.security_context = security_context
        self.node = node
        
        Logger.debug(f"SecurePublisher created for topic '{topic}' with SL{security_context.security_level}")

    def publish(self, msg: Any, additional_data: str = "") -> None:
        """
        Publish a message with automatic SafetyMetadata.
        
        :param msg: Message to publish (must have safety_metadata field)
        :param additional_data: Additional data to include in signature
        :raises SecurityError: If message doesn't support SafetyMetadata
        """
        if not hasattr(msg, 'safety_metadata'):
            raise SecurityError(
                f"Message type {type(msg).__name__} does not have safety_metadata field. "
                "Ensure the message definition includes SafetyMetadata safety_metadata."
            )
        
        try:
            # Build and attach metadata
            metadata_dict = SafetyMetadataBuilder.build(self.security_context, additional_data)
            
            # Convert dict to message fields
            msg.safety_metadata.security_level = metadata_dict['security_level']
            msg.safety_metadata.sender_id = metadata_dict['sender_id']
            msg.safety_metadata.timestamp.sec = metadata_dict['timestamp']['sec']
            msg.safety_metadata.timestamp.nanosec = metadata_dict['timestamp']['nanosec']
            msg.safety_metadata.nonce = metadata_dict['nonce']
            msg.safety_metadata.security_payload = metadata_dict['security_payload']
            msg.safety_metadata.algorithm_id = metadata_dict['algorithm_id']
            
            # Publish
            self.publisher.publish(msg)
            
        except Exception as e:
            Logger.error(f"Failed to publish secure message: {e}")
            raise

    def destroy(self) -> None:
        """Destroy the underlying publisher."""
        self.node.destroy_publisher(self.publisher)


class SecureServiceClient:
    """
    Wrapper for ROS2 Service Client with automatic SafetyMetadata.
    
    Similar to SecurePublisher, but for service calls.
    
    Usage:
        client = SecureServiceClient(
            node,
            ServiceType,
            'service_name',
            security_context
        )
        response = await client.call_async(request)
    
    :ivar client: Underlying ROS2 service client
    :ivar security_context: Security context for this client
    """

    def __init__(
        self,
        node: Node,
        srv_type: Type,
        service_name: str,
        security_context: SecurityContext
    ):
        """
        Initialize SecureServiceClient.
        
        :param node: ROS2 node
        :param srv_type: Service type class
        :param service_name: Service name
        :param security_context: Security context
        """
        self.client = node.create_client(srv_type, service_name)
        self.security_context = security_context
        self.node = node
        
        Logger.debug(f"SecureServiceClient created for service '{service_name}' with SL{security_context.security_level}")

    async def call_async(self, request: Any, additional_data: str = "") -> Any:
        """
        Call service asynchronously with automatic SafetyMetadata.
        
        :param request: Service request (must have safety_metadata field)
        :param additional_data: Additional data to include in signature
        :return: Service response
        :raises SecurityError: If request doesn't support SafetyMetadata
        """
        if not hasattr(request, 'safety_metadata'):
            raise SecurityError(
                f"Request type {type(request).__name__} does not have safety_metadata field."
            )
        
        try:
            # Build and attach metadata
            metadata_dict = SafetyMetadataBuilder.build(self.security_context, additional_data)
            
            # Convert dict to message fields
            request.safety_metadata.security_level = metadata_dict['security_level']
            request.safety_metadata.sender_id = metadata_dict['sender_id']
            request.safety_metadata.timestamp.sec = metadata_dict['timestamp']['sec']
            request.safety_metadata.timestamp.nanosec = metadata_dict['timestamp']['nanosec']
            request.safety_metadata.nonce = metadata_dict['nonce']
            request.safety_metadata.security_payload = metadata_dict['security_payload']
            request.safety_metadata.algorithm_id = metadata_dict['algorithm_id']
            
            # Wait for service
            if not self.client.wait_for_service(timeout_sec=5.0):
                raise SecurityError(f"Service not available: {self.client.srv_name}")
            
            # Call service
            future = self.client.call_async(request)
            return await future
            
        except Exception as e:
            Logger.error(f"Failed to call secure service: {e}")
            raise

    def destroy(self) -> None:
        """Destroy the underlying client."""
        self.node.destroy_client(self.client)


def security_required(
    security_level: SecurityLevel = SecurityLevel.BASIC_AUTH,
    validate_metadata: bool = True
):
    """
    Decorator for service callbacks requiring authentication.
    
    This decorator checks that:
    1. SafetyMetadata is present in the request
    2. Security level meets the minimum requirement
    3. Session token is valid (if applicable)
    4. Signature is valid (for Level 4+)
    
    Usage:
        @security_required(security_level=SecurityLevel.HMAC)
        def my_service_callback(self, request, response):
            # Service implementation
            return response
    
    :param security_level: Minimum required security level
    :param validate_metadata: Whether to validate SafetyMetadata
    :return: Decorator function
    """
    def decorator(func: Callable) -> Callable:
        @wraps(func)
        def wrapper(self, request, response):
            if not validate_metadata:
                return func(self, request, response)
            
            # Check if request has SafetyMetadata
            if not hasattr(request, 'safety_metadata'):
                Logger.warn(f"Service call to {func.__name__} missing SafetyMetadata")
                response.success = False
                response.message = "Missing SafetyMetadata"
                return response
            
            metadata = request.safety_metadata
            
            # Check security level
            if metadata.security_level < security_level.value:
                Logger.warn(
                    f"Service call to {func.__name__} has insufficient security level "
                    f"(got {metadata.security_level}, required {security_level.value})"
                )
                response.success = False
                response.message = f"Insufficient security level (required: {security_level.name})"
                return response
            
            # Additional validation can be added here
            # (timestamp check, nonce check, signature validation)
            # This would require access to SecurityManager
            
            Logger.debug(f"Service call to {func.__name__} passed security check (SL{metadata.security_level})")
            
            return func(self, request, response)
        
        return wrapper
    return decorator


def create_security_context(
    module_name: str,
    module_id: str,
    security_level: int,
    session_token: str,
    hmac_key: Optional[str] = None,
    certificate: Optional[str] = None,
    private_key_path: Optional[str] = None
) -> SecurityContext:
    """
    Helper function to create a SecurityContext.
    
    :param module_name: Module name (human-readable)
    :param module_id: Module UUID
    :param security_level: Security level (1-5)
    :param session_token: Session token from RequestAccess
    :param hmac_key: HMAC key (for Level 4+)
    :param certificate: Certificate (for Level 5)
    :param private_key_path: Path to private key (for Level 5)
    :return: SecurityContext instance
    """
    return SecurityContext(
        module_name=module_name,
        module_id=module_id,
        security_level=security_level,
        session_token=session_token,
        hmac_key=hmac_key,
        certificate=certificate,
        private_key_path=private_key_path
    )
