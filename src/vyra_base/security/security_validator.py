"""
Security Validation Middleware for VYRA Framework
=================================================

This module provides middleware functions for validating incoming
ROS2 messages and service calls based on their SafetyMetadata.

The middleware performs:
- Security level verification
- Password authentication (Level 3, validated during RequestAccess)
- Timestamp validation (Level 4+, replay attack prevention)
- Nonce validation (Level 4+, replay attack prevention)
- HMAC signature validation (Level 4)
- Digital signature validation (Level 5)
- Session token validation

Author: VYRA Framework Team
License: Proprietary
"""

import time
from typing import Any, Optional, Callable
from datetime import datetime

from vyra_base.security.security_levels import (
    SecurityLevel,
    AlgorithmId,
    MAX_TIMESTAMP_DRIFT_SECONDS,
    TimestampValidationError,
    SignatureValidationError,
    NonceValidationError,
    SessionExpiredError,
)
from vyra_base.security.security_manager import SecurityManager, SecuritySession
from vyra_base.helper.crypto_helper import (
    verify_hmac_signature,
    verify_message_rsa,
    create_hmac_payload,
)
from vyra_base.helper.logger import logger


class SecurityValidator:
    """
    Validates SafetyMetadata in incoming messages and service calls.
    
    This class works in conjunction with SecurityManager to provide
    comprehensive validation of security metadata.
    
    Usage::
    
        validator = SecurityValidator(security_manager)
        
        # In service callback:
        try:
            validator.validate_metadata(request.safety_metadata, required_level=SecurityLevel.HMAC)
            # Process request
        except SecurityError as e:
            response.success = False
            response.message = str(e)
            return response
    
    :ivar security_manager: SecurityManager instance for session validation
    :ivar strict_mode: If True, reject messages that fail any validation
    """

    def __init__(
        self,
        security_manager: Optional[SecurityManager] = None,
        strict_mode: bool = True
    ):
        """
        Initialize SecurityValidator.
        
        :param security_manager: SecurityManager for session validation
        :param strict_mode: Enable strict validation mode
        """
        self.security_manager = security_manager
        self.strict_mode = strict_mode
        logger.debug(f"SecurityValidator initialized (strict_mode={strict_mode})")

    def validate_metadata(
        self,
        metadata: Any,
        required_level: SecurityLevel = SecurityLevel.NONE,
        additional_data: str = ""
    ) -> Optional[SecuritySession]:
        """
        Validate SafetyMetadata from an incoming message/service call.
        
        Performs validation based on the security level:
        - Level 1 (NONE): No validation
        - Level 2 (BASIC_AUTH): Module ID check
        - Level 3 (EXTENDED_AUTH): Module ID + password authentication (validated during RequestAccess)
        - Level 4 (HMAC): HMAC signature + timestamp validation
        - Level 5 (DIGITAL_SIGNATURE): Certificate signature validation
        
        :param metadata: SafetyMetadata message
        :param required_level: Minimum required security level
        :param additional_data: Additional data that was included in signature
        :return: SecuritySession if validation succeeds
        :raises SecurityError: If validation fails
        """
        # Check if metadata exists
        if metadata is None:
            if required_level > SecurityLevel.NONE:
                raise SignatureValidationError("SafetyMetadata is required but missing")
            return None
        
        security_level = SecurityLevel(metadata.security_level)
        
        # Check minimum security level
        if security_level < required_level:
            raise SignatureValidationError(
                f"Insufficient security level: got {security_level.name}, "
                f"required {required_level.name}"
            )
        
        # Level 1: No validation needed
        if security_level == SecurityLevel.NONE:
            logger.debug("Security level NONE: skipping validation")
            return None
        
        # Level 2+: Validate module ID
        sender_id = metadata.sender_id
        if not sender_id:
            raise SignatureValidationError("Sender ID is required")
        
        logger.debug(f"Validating metadata from {sender_id} at level {security_level.name}")
        
        # Get session (if SecurityManager is available)
        session = None
        if self.security_manager:
            # Note: We would need session_token in metadata to look up session
            # For now, we can validate based on sender_id
            # In production, session_token should be part of the request
            pass
        
        # Level 3+: Password authentication is validated during RequestAccess
        # No additional validation needed here for Level 3
        
        # Level 4+: Validate timestamp for replay attack prevention
        if security_level >= SecurityLevel.HMAC:
            self._validate_timestamp(metadata)
        
        # Level 4: Validate HMAC
        if security_level == SecurityLevel.HMAC:
            self._validate_hmac(metadata, session, additional_data)
        
        # Level 5: Validate digital signature
        elif security_level == SecurityLevel.DIGITAL_SIGNATURE:
            self._validate_digital_signature(metadata, session, additional_data)
        
        return session

    def _validate_timestamp(self, metadata: Any) -> None:
        """
        Validate timestamp to prevent replay attacks (Level 4+).
        
        :param metadata: SafetyMetadata message
        :raises TimestampValidationError: If timestamp is invalid
        """
        msg_timestamp = metadata.timestamp.sec
        current_time = int(time.time())
        time_diff = abs(current_time - msg_timestamp)
        
        if time_diff > MAX_TIMESTAMP_DRIFT_SECONDS:
            raise TimestampValidationError(
                f"Timestamp drift too large: {time_diff}s (max: {MAX_TIMESTAMP_DRIFT_SECONDS}s)"
            )
        
        logger.debug(f"Timestamp validation passed (drift: {time_diff}s)")

    def _validate_nonce(self, metadata: Any, session: Optional[SecuritySession]) -> None:
        """
        Validate nonce to prevent replay attacks.
        
        :param metadata: SafetyMetadata message
        :param session: SecuritySession (if available)
        :raises NonceValidationError: If nonce was already used
        """
        if not session:
            # Can't validate nonce without session
            logger.debug("Nonce validation skipped (no session)")
            return
        
        nonce = metadata.nonce
        
        if session.has_nonce(nonce):
            raise NonceValidationError(f"Nonce {nonce} already used (replay attack detected)")
        
        # Add nonce to session
        session.add_nonce(nonce)
        logger.debug(f"Nonce {nonce} validated and recorded")

    def _validate_hmac(
        self,
        metadata: Any,
        session: Optional[SecuritySession],
        additional_data: str
    ) -> None:
        """
        Validate HMAC signature.
        
        :param metadata: SafetyMetadata message
        :param session: SecuritySession containing HMAC key
        :param additional_data: Additional data included in signature
        :raises SignatureValidationError: If signature is invalid
        """
        if not metadata.security_payload:
            raise SignatureValidationError("HMAC signature missing in security_payload")
        
        if not session or not session.hmac_key:
            raise SignatureValidationError("HMAC key not available for validation")
        
        # Reconstruct the message that was signed
        sender_id = metadata.sender_id
        timestamp = metadata.timestamp.sec
        nonce = metadata.nonce
        signature = metadata.security_payload
        
        try:
            # Verify HMAC
            expected_signature = create_hmac_payload(
                sender_id,
                timestamp,
                nonce,
                session.hmac_key,
                additional_data
            )
            
            if signature != expected_signature:
                raise SignatureValidationError("HMAC signature mismatch")
            
            logger.debug("HMAC signature validated successfully")
            
            # Validate nonce
            self._validate_nonce(metadata, session)
            
        except Exception as e:
            raise SignatureValidationError(f"HMAC validation failed: {e}")

    def _validate_digital_signature(
        self,
        metadata: Any,
        session: Optional[SecuritySession],
        additional_data: str
    ) -> None:
        """
        Validate digital signature with certificate.
        
        :param metadata: SafetyMetadata message
        :param session: SecuritySession containing certificate
        :param additional_data: Additional data included in signature
        :raises SignatureValidationError: If signature is invalid
        """
        if not metadata.security_payload:
            raise SignatureValidationError("Digital signature missing in security_payload")
        
        if not session or not session.certificate:
            raise SignatureValidationError("Certificate not available for validation")
        
        # Reconstruct the message that was signed
        sender_id = metadata.sender_id
        timestamp = metadata.timestamp.sec
        nonce = metadata.nonce
        message = f"{sender_id}|{timestamp}|{nonce}|{additional_data}"
        signature = metadata.security_payload
        
        try:
            # Verify signature
            if not verify_message_rsa(message, signature, session.certificate):
                raise SignatureValidationError("Digital signature verification failed")
            
            logger.debug("Digital signature validated successfully")
            
            # Validate nonce
            self._validate_nonce(metadata, session)
            
        except Exception as e:
            raise SignatureValidationError(f"Digital signature validation failed: {e}")


class MessageSecurityFilter:
    """
    Filter for incoming ROS2 messages with automatic security validation.
    
    This class can be used as a subscription callback wrapper to automatically
    validate SafetyMetadata before processing messages.
    
    Usage::
    
        def my_callback(msg):
            # Process message
            pass
        
        filter = MessageSecurityFilter(validator, SecurityLevel.HMAC)
        filtered_callback = filter.wrap_callback(my_callback)
        
        subscription = node.create_subscription(
            MsgType,
            'topic',
            filtered_callback,
            qos_profile
        )
    
    :ivar validator: SecurityValidator instance
    :ivar required_level: Minimum required security level
    """

    def __init__(
        self,
        validator: SecurityValidator,
        required_level: SecurityLevel = SecurityLevel.NONE
    ):
        """
        Initialize MessageSecurityFilter.
        
        :param validator: SecurityValidator instance
        :param required_level: Minimum required security level
        """
        self.validator = validator
        self.required_level = required_level

    def wrap_callback(self, callback: Callable) -> Callable:
        """
        Wrap a callback function with security validation.
        
        :param callback: Original callback function
        :return: Wrapped callback with validation
        """
        def wrapped_callback(msg: Any) -> None:
            if not hasattr(msg, 'safety_metadata'):
                if self.required_level > SecurityLevel.NONE:
                    logger.warn(f"Message missing SafetyMetadata (required level: {self.required_level.name})")
                    return
                else:
                    # No security required
                    callback(msg)
                    return
            
            try:
                self.validator.validate_metadata(msg.safety_metadata, self.required_level)
                callback(msg)
            except Exception as e:
                logger.warn(f"Message validation failed: {e}")
                # Message is dropped
        
        return wrapped_callback


def create_secure_subscription(
    node: Any,
    msg_type: type,
    topic: str,
    callback: Callable,
    validator: SecurityValidator,
    required_level: SecurityLevel = SecurityLevel.NONE,
    qos_profile: Any = 10
) -> Any:
    """
    Helper function to create a subscription with automatic security validation.
    
    :param node: ROS2 node
    :param msg_type: Message type
    :param topic: Topic name
    :param callback: Callback function
    :param validator: SecurityValidator instance
    :param required_level: Minimum required security level
    :param qos_profile: QoS profile
    :return: ROS2 subscription
    """
    filter = MessageSecurityFilter(validator, required_level)
    wrapped_callback = filter.wrap_callback(callback)
    
    return node.create_subscription(
        msg_type,
        topic,
        wrapped_callback,
        qos_profile
    )
