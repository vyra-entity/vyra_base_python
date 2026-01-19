"""
VYRA Security Framework
=======================

Comprehensive security framework for inter-module communication.

This package provides a 5-level security model for the VYRA framework,
including authentication, message integrity, and protection against
replay attacks.

Security Levels:
    - Level 1 (NONE): No security checks
    - Level 2 (BASIC_AUTH): Module ID verification
    - Level 3 (TIMESTAMP): ID + timestamp validation
    - Level 4 (HMAC): HMAC-SHA256 signatures
    - Level 5 (DIGITAL_SIGNATURE): Certificate-based PKI

Quick Start:
    Server:
        >>> from vyra_base.security import SecurityManager, SecurityLevel
        >>> class MyModule(Node, SecurityManager):
        ...     def __init__(self):
        ...         SecurityManager.__init__(self, max_security_level=SecurityLevel.HMAC)
    
    Client:
        >>> from vyra_base.security import SecurePublisher, create_security_context
        >>> ctx = create_security_context(module_id, level, token, hmac_key)
        >>> pub = SecurePublisher(node, MsgType, 'topic', ctx)
        >>> pub.publish(msg)

Author: VYRA Framework Team
License: Proprietary
"""

# Security Levels and Constants
from vyra_base.security.security_levels import (
    SecurityLevel,
    AccessStatus,
    AlgorithmId,
    SecurityError,
    InvalidSecurityLevelError,
    SignatureValidationError,
    CertificateValidationError,
    TimestampValidationError,
    SessionExpiredError,
    NonceValidationError,
    DEFAULT_SESSION_DURATION_SECONDS,
    MAX_TIMESTAMP_DRIFT_SECONDS,
)

# Security Manager (Server-side)
from vyra_base.security.security_manager import (
    SecurityManager,
    SecuritySession,
)

# Security Client (Client-side)
from vyra_base.security.security_client import (
    SecurityContext,
    SecurePublisher,
    SecureServiceClient,
    SafetyMetadataBuilder,
    security_required,
    create_security_context,
)

# Security Validator (Server-side)
from vyra_base.security.security_validator import (
    SecurityValidator,
    MessageSecurityFilter,
    create_secure_subscription,
)

__all__ = [
    # Security Levels
    'SecurityLevel',
    'AccessStatus',
    'AlgorithmId',
    
    # Exceptions
    'SecurityError',
    'InvalidSecurityLevelError',
    'SignatureValidationError',
    'CertificateValidationError',
    'TimestampValidationError',
    'SessionExpiredError',
    'NonceValidationError',
    
    # Constants
    'DEFAULT_SESSION_DURATION_SECONDS',
    'MAX_TIMESTAMP_DRIFT_SECONDS',
    
    # Server Components
    'SecurityManager',
    'SecuritySession',
    'SecurityValidator',
    'MessageSecurityFilter',
    
    # Client Components
    'SecurityContext',
    'SecurePublisher',
    'SecureServiceClient',
    'SafetyMetadataBuilder',
    
    # Decorators and Helpers
    'security_required',
    'create_security_context',
    'create_secure_subscription',
]