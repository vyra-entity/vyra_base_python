"""
Security Manager for VYRA Framework
====================================

This module provides the SecurityManager class which handles:
- Access request validation and token generation
- Session management
- HMAC key distribution
- Certificate management (for Level 5)
- Security level enforcement

The SecurityManager can be used as a base class or mixin for modules
that need to provide the RequestAccess service.

Author: VYRA Framework Team
License: Proprietary
"""

import uuid
import asyncio
from datetime import datetime, timedelta
from typing import Dict, Optional, Tuple
from pathlib import Path
from dataclasses import dataclass, field

try:
    import rclpy
    from rclpy.node import Node
    from builtin_interfaces.msg import Time
    ROS2_AVAILABLE = True
except ImportError:
    rclpy = None
    Node = None
    Time = None
    ROS2_AVAILABLE = False

from vyra_base.com import remote_service
from vyra_base.security.security_levels import (
    SecurityLevel,
    AccessStatus,
    AlgorithmId,
    DEFAULT_SESSION_DURATION_SECONDS,
    SessionExpiredError,
    InvalidSecurityLevelError,
)
from vyra_base.helper.crypto_helper import (
    generate_hmac_key,
    generate_session_token,
    sign_csr,
    verify_certificate,
    load_private_key,
    create_self_signed_cert,
)
import logging
logger = logging.getLogger(__name__)
@dataclass
class SecuritySession:
    """
    Represents an active security session.
    
    :ivar module_name: Name of the authenticated module
    :ivar module_id: UUID of the authenticated module
    :ivar role: Role identifier of the module
    :ivar security_level: Granted security level
    :ivar session_token: Unique session token
    :ivar hmac_key: HMAC key (for Level 4+)
    :ivar certificate: Client certificate (for Level 5)
    :ivar created_at: Session creation timestamp
    :ivar expires_at: Session expiration timestamp
    :ivar nonces_used: Set of nonces used (for replay attack prevention)
    """
    module_name: str
    module_id: str
    role: str
    security_level: int
    session_token: str
    hmac_key: Optional[str] = None
    certificate: Optional[str] = None
    created_at: datetime = field(default_factory=datetime.utcnow)
    expires_at: datetime = field(default_factory=lambda: datetime.utcnow() + timedelta(seconds=DEFAULT_SESSION_DURATION_SECONDS))
    nonces_used: set = field(default_factory=set)

    def is_expired(self) -> bool:
        """Check if the session has expired."""
        return datetime.utcnow() > self.expires_at

    def add_nonce(self, nonce: int) -> None:
        """Add a nonce to the used nonces set."""
        self.nonces_used.add(nonce)

    def has_nonce(self, nonce: int) -> bool:
        """Check if a nonce has been used (replay attack detection)."""
        return nonce in self.nonces_used


class SecurityManager:
    """
    Manages security sessions and access control for a VYRA module.
    
    This class can be used as a mixin or standalone component to provide
    security functionality. It handles:
    - Request access service implementation
    - Session token generation and validation
    - HMAC key distribution
    - Certificate signing (for Level 5)
    
    Usage:
        class MyModule(Node, SecurityManager):
            def __init__(self):
                Node.__init__(self, 'my_module')
                SecurityManager.__init__(self, max_security_level=SecurityLevel.HMAC)
                self.setup_security_service()
    
    :ivar max_security_level: Maximum security level this module supports
    :ivar sessions: Dictionary mapping session tokens to SecuritySession objects
    :ivar sessions_by_module: Dictionary mapping module IDs to session tokens
    :ivar ca_private_key: CA private key (for Level 5 certificate signing)
    :ivar ca_cert: CA certificate (for Level 5 certificate validation)
    """

    def __init__(
        self,
        max_security_level: SecurityLevel = SecurityLevel.HMAC,
        session_duration_seconds: int = DEFAULT_SESSION_DURATION_SECONDS,
        ca_key_path: Optional[Path] = None,
        ca_cert_path: Optional[Path] = None,
        module_passwords: Optional[Dict[str, str]] = None
    ):
        """
        Initialize the SecurityManager.
        
        :param max_security_level: Maximum security level to support
        :param session_duration_seconds: Session validity duration
        :param ca_key_path: Path to CA private key (required for Level 5)
        :param ca_cert_path: Path to CA certificate (required for Level 5)
        :param module_passwords: Dictionary mapping module_id to password (for Level 3+)
        """
        self.max_security_level = max_security_level
        self.session_duration_seconds = session_duration_seconds
        self.sessions: Dict[str, SecuritySession] = {}
        self.sessions_by_module: Dict[str, str] = {}
        self.module_passwords: Dict[str, str] = module_passwords or {}
        
        # CA infrastructure for Level 5
        self.ca_private_key = None
        self.ca_cert = None
        self.ca_cert_pem = None
        
        if max_security_level >= SecurityLevel.DIGITAL_SIGNATURE:
            self._load_or_create_ca(ca_key_path, ca_cert_path)
        
        logger.info(f"SecurityManager initialized with max level: {SecurityLevel.get_name(max_security_level)}")

    def _load_or_create_ca(self, ca_key_path: Optional[Path], ca_cert_path: Optional[Path]) -> None:
        """Load or create CA infrastructure for Level 5."""
        try:
            if ca_key_path and ca_key_path.exists():
                self.ca_private_key = load_private_key(ca_key_path)
                logger.info(f"Loaded CA private key from {ca_key_path}")
            else:
                logger.warn("CA key not found, Level 5 authentication will not be available")
                
            if ca_cert_path and ca_cert_path.exists():
                # Load both PEM string and parsed certificate object
                self.ca_cert_pem = ca_cert_path.read_text()
                from cryptography import x509
                from cryptography.hazmat.backends import default_backend
                cert_bytes = ca_cert_path.read_bytes()
                self.ca_cert = x509.load_pem_x509_certificate(cert_bytes, default_backend())
                logger.info(f"Loaded CA certificate from {ca_cert_path}")
            else:
                self.ca_cert = None
        except Exception as e:
            logger.error(f"Failed to load CA infrastructure: {e}")

    @remote_service()
    async def request_access(self, request, response) -> bool:
        """
        Remote callable wrapper for request_access_impl.
        
        :param request: Request object with access parameters
        :param response: Response object to populate
        :return: True if handled successfully, False otherwise
        """
        try:
            (
                success,
                message,
                session_token,
                hmac_key,
                certificate,
                expires_at,
                granted_sl
            ) = await self.request_access_impl(
                module_name=request.module_name,
                module_id=request.module_id,
                requested_role=request.requested_role,
                requested_sl=request.requested_sl,
                password=request.password,
                certificate_csr=request.certificate_csr
            )
            
            response.success = success
            response.message = message
            response.session_token = session_token
            response.hmac_key = hmac_key
            response.certificate = certificate
            if Time is not None:
                response.expires_at = Time(sec=int(expires_at.timestamp()), nanosec=0)
            response.granted_sl = granted_sl
            
            return True
        except Exception as e:
            logger.error(f"Error handling request_access: {e}")
            return False
    

    async def request_access_impl(
        self,
        module_name: str,
        module_id: uuid.UUID,
        requested_role: str,
        requested_sl: int,
        password: str = "",
        certificate_csr: str = ""
    ) -> Tuple[bool, str, str, str, str, datetime, int]:
        """
        Handle an access request from another module.
        
        This is the main entry point for the RequestAccess service.
        
        :param module_name: Name of the requesting module
        :param module_id: UUID of the requesting module
        :param requested_role: Role identifier
        :param requested_sl: Requested security level (1-5)
        :param password: Password for Level 3+ authentication
        :param certificate_csr: CSR for Level 5 (base64-encoded PEM)
        :return: Tuple of (success, message, session_token, hmac_key, certificate, expires_at, granted_sl)
        """
        module_id_str = str(module_id)
        
        # Validate security level
        if not SecurityLevel.is_valid(requested_sl):
            logger.warn(f"Invalid security level requested: {requested_sl}")
            return (
                False,
                f"Invalid security level: {requested_sl}",
                "",
                "",
                "",
                datetime.utcnow(),
                0
            )
        
        requested_level = SecurityLevel(requested_sl)
        
        # Password validation for Level 3+
        if requested_level >= SecurityLevel.EXTENDED_AUTH:
            if module_id_str not in self.module_passwords:
                logger.warn(f"No password configured for module {module_name} ({module_id_str})")
                return (
                    False,
                    "Password required for Level 3+ authentication",
                    "",
                    "",
                    "",
                    datetime.utcnow(),
                    0
                )
            
            if not password or password != self.module_passwords[module_id_str]:
                logger.warn(f"Invalid password for module {module_name} ({module_id_str})")
                return (
                    False,
                    "Invalid password",
                    "",
                    "",
                    "",
                    datetime.utcnow(),
                    0
                )
        
        # Check if requested level is supported
        if requested_level > self.max_security_level:
            logger.warn(f"Security level {requested_level.name} not supported (max: {self.max_security_level.name})")
            # Downgrade to maximum supported level
            granted_level = self.max_security_level
            logger.info(f"Downgrading to security level {granted_level.name}")
        else:
            granted_level = requested_level
        
        # Invalidate any existing session for this module (latest request wins)
        self._invalidate_module_session(module_id_str)
        
        # Generate session token
        session_token = generate_session_token()
        expires_at = datetime.utcnow() + timedelta(seconds=self.session_duration_seconds)
        
        # Generate HMAC key for Level 4+
        hmac_key = ""
        if granted_level >= SecurityLevel.HMAC:
            hmac_key = generate_hmac_key()
        
        # Handle certificate for Level 5
        certificate = ""
        if granted_level >= SecurityLevel.DIGITAL_SIGNATURE:
            if not certificate_csr:
                logger.warn("Certificate CSR required for Level 5 but not provided")
                return (
                    False,
                    "Certificate CSR required for Level 5 authentication",
                    "",
                    "",
                    "",
                    datetime.utcnow(),
                    0
                )
            
            if not self.ca_private_key or not self.ca_cert:
                logger.error("CA infrastructure not available for Level 5")
                return (
                    False,
                    "Certificate signing not available",
                    "",
                    "",
                    "",
                    datetime.utcnow(),
                    0
                )
            
            try:
                # Import here to avoid circular import
                from vyra_base.helper.crypto_helper import sign_csr as crypto_sign_csr
                certificate = crypto_sign_csr(
                    certificate_csr,
                    self.ca_private_key,
                    self.ca_cert,  # Type is now guaranteed to be Certificate (not None)
                    validity_days=365
                )
            except Exception as e:
                logger.error(f"Failed to sign certificate: {e}")
                return (
                    False,
                    f"Certificate signing failed: {str(e)}",
                    "",
                    "",
                    "",
                    datetime.utcnow(),
                    0
                )
        
        # Create session
        session = SecuritySession(
            module_name=module_name,
            module_id=module_id_str,
            role=requested_role,
            security_level=granted_level.value,
            session_token=session_token,
            hmac_key=hmac_key,
            certificate=certificate,
            expires_at=expires_at
        )
        
        # Store session
        self.sessions[session_token] = session
        self.sessions_by_module[module_id_str] = session_token
        
        logger.info(
            f"Access granted to {module_name} ({module_id_str}) "
            f"with security level {granted_level.name}, role: {requested_role}"
        )
        
        return (
            True,
            f"Access granted with security level {granted_level.name}",
            session_token,
            hmac_key,
            certificate,
            expires_at,
            granted_level.value
        )

    def _invalidate_module_session(self, module_id: str) -> None:
        """Invalidate any existing session for a module."""
        if module_id in self.sessions_by_module:
            old_token = self.sessions_by_module[module_id]
            if old_token in self.sessions:
                del self.sessions[old_token]
                logger.debug(f"Invalidated old session for module {module_id}")
            del self.sessions_by_module[module_id]

    def get_session(self, session_token: str) -> Optional[SecuritySession]:
        """
        Retrieve a session by token.
        
        :param session_token: Session token
        :return: SecuritySession if valid and not expired, None otherwise
        """
        session = self.sessions.get(session_token)
        if not session:
            return None
        
        if session.is_expired():
            logger.warn(f"Session {session_token} has expired")
            self._invalidate_session(session_token)
            return None
        
        return session

    def _invalidate_session(self, session_token: str) -> None:
        """Invalidate a session."""
        if session_token in self.sessions:
            session = self.sessions[session_token]
            if session.module_id in self.sessions_by_module:
                del self.sessions_by_module[session.module_id]
            del self.sessions[session_token]

    async def cleanup_expired_sessions(self) -> None:
        """Remove all expired sessions. Should be called periodically."""
        expired_tokens = [
            token for token, session in self.sessions.items()
            if session.is_expired()
        ]
        
        for token in expired_tokens:
            self._invalidate_session(token)
        
        if expired_tokens:
            logger.debug(f"Cleaned up {len(expired_tokens)} expired sessions")

    def get_session_count(self) -> int:
        """Get the number of active sessions."""
        return len(self.sessions)

    def get_active_modules(self) -> list:
        """
        Get list of active authenticated modules.
        
        :return: List of dicts with module info
        """
        return [
            {
                "module_name": session.module_name,
                "module_id": session.module_id,
                "role": session.role,
                "security_level": SecurityLevel.get_name(session.security_level),
                "expires_at": session.expires_at.isoformat()
            }
            for session in self.sessions.values()
            if not session.is_expired()
        ]
