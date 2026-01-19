"""
Security Level Definitions for VYRA Framework
==============================================

This module defines the security levels, constants, and enumerations
used throughout the VYRA security framework.

Security Levels:
    - Level 1 (NONE): No security checks, open communication
    - Level 2 (BASIC_AUTH): Module ID verification only
    - Level 3 (EXTENDED_AUTH): Module ID + password authentication
    - Level 4 (HMAC): HMAC-SHA256 signature validation (message integrity)
    - Level 5 (DIGITAL_SIGNATURE): Certificate-based digital signature (full PKI)

Author: VYRA Framework Team
License: Proprietary
"""

from enum import IntEnum, Enum
from typing import Final


class SecurityLevel(IntEnum):
    """
    Enumeration of security levels for module communication.
    
    Each level builds upon the previous level, adding additional
    security mechanisms.
    
    :cvar NONE: No security checks performed
    :cvar BASIC_AUTH: Module ID verification
    :cvar EXTENDED_AUTH: Module ID + password authentication
    :cvar HMAC: HMAC-SHA256 signature validation
    :cvar DIGITAL_SIGNATURE: Certificate-based digital signature
    """
    NONE = 1
    BASIC_AUTH = 2
    EXTENDED_AUTH = 3
    HMAC = 4
    DIGITAL_SIGNATURE = 5

    @classmethod
    def is_valid(cls, level: int) -> bool:
        """
        Check if a given integer is a valid security level.
        
        :param level: Security level to validate
        :return: True if valid, False otherwise
        """
        return level in [sl.value for sl in cls]

    @classmethod
    def get_name(cls, level: int) -> str:
        """
        Get the name of a security level.
        
        :param level: Security level integer
        :return: Name of the security level
        :raises ValueError: If level is invalid
        """
        if not cls.is_valid(level):
            raise ValueError(f"Invalid security level: {level}")
        return cls(level).name

    def requires_hmac(self) -> bool:
        """Check if this security level requires HMAC validation."""
        return self >= SecurityLevel.HMAC

    def requires_certificate(self) -> bool:
        """Check if this security level requires certificate validation."""
        return self >= SecurityLevel.DIGITAL_SIGNATURE


class AccessStatus(IntEnum):
    """
    Status codes for access request results.
    
    :cvar GRANTED: Access was granted successfully
    :cvar DENIED: Access was denied
    :cvar INVALID_REQUEST: Request was malformed or missing required fields
    :cvar EXPIRED: Session or token has expired
    :cvar INVALID_SIGNATURE: Signature validation failed
    :cvar CERTIFICATE_ERROR: Certificate validation failed
    :cvar INTERNAL_ERROR: Internal server error
    """
    GRANTED = 0
    DENIED = 1
    INVALID_REQUEST = 2
    EXPIRED = 3
    INVALID_SIGNATURE = 4
    CERTIFICATE_ERROR = 5
    INTERNAL_ERROR = 6


class AlgorithmId(str, Enum):
    """
    Supported cryptographic algorithms.
    
    :cvar NONE: No algorithm (security level 1-3)
    :cvar HMAC_SHA256: HMAC with SHA-256 (security level 4)
    :cvar RSA_SHA256: RSA signature with SHA-256 (security level 5)
    :cvar ECDSA_P256: ECDSA with P-256 curve (security level 5)
    """
    NONE = "NONE"
    HMAC_SHA256 = "HMAC-SHA256"
    RSA_SHA256 = "RSA-SHA256"
    ECDSA_P256 = "ECDSA-P256"

    @classmethod
    def for_security_level(cls, level: SecurityLevel) -> 'AlgorithmId':
        """
        Get the default algorithm for a security level.
        
        :param level: Security level
        :return: Default algorithm for that level
        """
        if level < SecurityLevel.HMAC:
            return cls.NONE
        elif level == SecurityLevel.HMAC:
            return cls.HMAC_SHA256
        else:  # DIGITAL_SIGNATURE
            return cls.RSA_SHA256


# Constants
DEFAULT_SESSION_DURATION_SECONDS: Final[int] = 3600  # 1 hour
DEFAULT_TOKEN_LENGTH: Final[int] = 32  # bytes
DEFAULT_NONCE_LENGTH: Final[int] = 16  # bytes
MAX_TIMESTAMP_DRIFT_SECONDS: Final[int] = 300  # 5 minutes
HMAC_KEY_LENGTH: Final[int] = 32  # bytes (256 bits)

# Certificate constants
CERT_KEY_SIZE: Final[int] = 2048  # RSA key size
CERT_VALIDITY_DAYS: Final[int] = 365  # Certificate validity period
CERT_HASH_ALGORITHM: Final[str] = "sha256"

# Security payload size limits (to prevent DoS attacks)
MAX_SECURITY_PAYLOAD_SIZE: Final[int] = 4096  # 4KB max
MAX_CERTIFICATE_SIZE: Final[int] = 8192  # 8KB max
MAX_CSR_SIZE: Final[int] = 4096  # 4KB max


class SecurityError(Exception):
    """Base exception for security-related errors."""
    pass


class InvalidSecurityLevelError(SecurityError):
    """Raised when an invalid security level is specified."""
    pass


class SignatureValidationError(SecurityError):
    """Raised when signature validation fails."""
    pass


class CertificateValidationError(SecurityError):
    """Raised when certificate validation fails."""
    pass


class TimestampValidationError(SecurityError):
    """Raised when timestamp validation fails."""
    pass


class SessionExpiredError(SecurityError):
    """Raised when a session has expired."""
    pass


class NonceValidationError(SecurityError):
    """Raised when nonce validation fails (replay attack detected)."""
    pass
