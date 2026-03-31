"""
Cryptographic Helper Functions for VYRA Security Framework
==========================================================

This module provides cryptographic utilities for the VYRA security framework,
including HMAC generation/validation, certificate management, and key generation.

Features:
    - HMAC-SHA256 signature generation and validation
    - RSA key pair generation
    - Certificate Signing Request (CSR) creation
    - Certificate signing and validation
    - Secure random token generation
    - Nonce generation for replay attack prevention

Dependencies:
    - cryptography: For PKI operations (Level 5)
    - hmac/hashlib: For HMAC operations (Level 4)
    - secrets: For secure random generation

Author: VYRA Framework Team
License: Proprietary
"""

import hmac
import hashlib
import secrets
import base64
import uuid
from datetime import datetime, timedelta, timezone
from typing import Optional, Tuple, Dict, Any
from pathlib import Path

from cryptography import x509
from cryptography.x509.oid import NameOID, ExtensionOID
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import rsa, padding
from cryptography.hazmat.backends import default_backend

from vyra_base.security.security_levels import (
    HMAC_KEY_LENGTH,
    DEFAULT_TOKEN_LENGTH,
    DEFAULT_NONCE_LENGTH,
    CERT_KEY_SIZE,
    CERT_VALIDITY_DAYS,
    CERT_HASH_ALGORITHM,
    SignatureValidationError,
    CertificateValidationError,
)


# ============================================================================
# HMAC Operations (Security Level 4)
# ============================================================================

def generate_hmac_key() -> str:
    """
    Generate a secure random HMAC key.
    
    :return: Hex-encoded HMAC key
    """
    key_bytes = secrets.token_bytes(HMAC_KEY_LENGTH)
    return key_bytes.hex()


def compute_hmac_signature(message: str, hmac_key: str) -> str:
    """
    Compute HMAC-SHA256 signature for a message.
    
    :param message: Message to sign
    :param hmac_key: Hex-encoded HMAC key
    :return: Hex-encoded HMAC signature
    :raises ValueError: If hmac_key is invalid
    """
    try:
        key_bytes = bytes.fromhex(hmac_key)
    except ValueError as e:
        raise ValueError(f"Invalid HMAC key format: {e}")
    
    message_bytes = message.encode('utf-8')
    signature = hmac.new(key_bytes, message_bytes, hashlib.sha256).digest()
    return signature.hex()


def verify_hmac_signature(message: str, signature: str, hmac_key: str) -> bool:
    """
    Verify HMAC-SHA256 signature for a message.
    
    :param message: Original message
    :param signature: Hex-encoded signature to verify
    :param hmac_key: Hex-encoded HMAC key
    :return: True if signature is valid, False otherwise
    :raises SignatureValidationError: If signature format is invalid
    """
    try:
        expected_signature = compute_hmac_signature(message, hmac_key)
        # Use constant-time comparison to prevent timing attacks
        return hmac.compare_digest(signature, expected_signature)
    except (ValueError, TypeError) as e:
        raise SignatureValidationError(f"Invalid signature format: {e}")


def create_hmac_payload(
    sender_id: str,
    timestamp: int,
    nonce: int,
    hmac_key: str,
    additional_data: str = ""
) -> str:
    """
    Create HMAC payload for SafetyMetadata.
    
    The message signed includes: sender_id, timestamp, nonce, and optional additional data.
    
    :param sender_id: Sender module ID
    :param timestamp: Message timestamp (seconds since epoch)
    :param nonce: Random nonce
    :param hmac_key: Hex-encoded HMAC key
    :param additional_data: Optional additional data to include in signature
    :return: Hex-encoded HMAC signature
    """
    message_parts = [
        sender_id,
        str(timestamp),
        str(nonce),
    ]
    if additional_data:
        message_parts.append(additional_data)
    
    message = "|".join(message_parts)
    return compute_hmac_signature(message, hmac_key)


# ============================================================================
# Token and Nonce Generation
# ============================================================================

def generate_session_token() -> str:
    """
    Generate a secure random session token.
    
    :return: Hex-encoded session token
    """
    return secrets.token_hex(DEFAULT_TOKEN_LENGTH)


def generate_nonce() -> int:
    """
    Generate a random nonce for replay attack prevention.
    
    :return: Random 64-bit unsigned integer
    """
    return secrets.randbits(64)


# ============================================================================
# RSA Key Operations (Security Level 5)
# ============================================================================

def generate_rsa_keypair() -> Tuple[rsa.RSAPrivateKey, rsa.RSAPublicKey]:
    """
    Generate RSA key pair for certificate operations.
    
    :return: Tuple of (private_key, public_key)
    """
    private_key = rsa.generate_private_key(
        public_exponent=65537,
        key_size=CERT_KEY_SIZE,
        backend=default_backend()
    )
    public_key = private_key.public_key()
    return private_key, public_key


def save_private_key(private_key: rsa.RSAPrivateKey, filepath: Path, password: Optional[str] = None) -> None:
    """
    Save RSA private key to file.
    
    :param private_key: RSA private key
    :param filepath: Path to save the key
    :param password: Optional password for encryption
    """
    encryption_algorithm = (
        serialization.BestAvailableEncryption(password.encode())
        if password else serialization.NoEncryption()
    )
    
    pem = private_key.private_bytes(
        encoding=serialization.Encoding.PEM,
        format=serialization.PrivateFormat.PKCS8,
        encryption_algorithm=encryption_algorithm
    )
    
    filepath.parent.mkdir(parents=True, exist_ok=True)
    filepath.write_bytes(pem)


def load_private_key(filepath: Path, password: Optional[str] = None) -> rsa.RSAPrivateKey:
    """
    Load RSA private key from file.
    
    :param filepath: Path to the key file
    :param password: Optional password if key is encrypted
    :return: RSA private key
    :raises FileNotFoundError: If key file doesn't exist
    """
    pem_data = filepath.read_bytes()
    password_bytes = password.encode() if password else None
    
    return serialization.load_pem_private_key(
        pem_data,
        password=password_bytes,
        backend=default_backend()
    )


# ============================================================================
# Certificate Operations (Security Level 5)
# ============================================================================

def create_csr(
    module_name: str,
    module_id: str,
    private_key: rsa.RSAPrivateKey,
    organization: str = "VYRA Framework",
    country: str = "DE"
) -> str:
    """
    Create a Certificate Signing Request (CSR).
    
    :param module_name: Name of the module
    :param module_id: UUID of the module
    :param private_key: RSA private key
    :param organization: Organization name
    :param country: Country code (ISO 3166-1 alpha-2)
    :return: Base64-encoded PEM CSR
    """
    subject = x509.Name([
        x509.NameAttribute(NameOID.COUNTRY_NAME, country),
        x509.NameAttribute(NameOID.ORGANIZATION_NAME, organization),
        x509.NameAttribute(NameOID.COMMON_NAME, module_name),
        x509.NameAttribute(NameOID.SERIAL_NUMBER, module_id),
    ])
    
    csr = x509.CertificateSigningRequestBuilder().subject_name(subject).sign(
        private_key, hashes.SHA256(), backend=default_backend()
    )
    
    pem = csr.public_bytes(serialization.Encoding.PEM)
    return base64.b64encode(pem).decode('utf-8')


def sign_csr(
    csr_pem: str,
    ca_private_key: rsa.RSAPrivateKey,
    ca_cert: x509.Certificate,
    validity_days: int = CERT_VALIDITY_DAYS
) -> str:
    """
    Sign a Certificate Signing Request with CA key.
    
    :param csr_pem: Base64-encoded PEM CSR
    :param ca_private_key: CA private key
    :param ca_cert: CA certificate
    :param validity_days: Certificate validity in days
    :return: PEM-encoded signed certificate
    :raises CertificateValidationError: If CSR is invalid
    """
    try:
        csr_bytes = base64.b64decode(csr_pem)
        csr = x509.load_pem_x509_csr(csr_bytes, backend=default_backend())
    except Exception as e:
        raise CertificateValidationError(f"Invalid CSR: {e}")
    
    # Create certificate
    now = datetime.utcnow()
    cert = (
        x509.CertificateBuilder()
        .subject_name(csr.subject)
        .issuer_name(ca_cert.subject)
        .public_key(csr.public_key())
        .serial_number(x509.random_serial_number())
        .not_valid_before(now)
        .not_valid_after(now + timedelta(days=validity_days))
        .add_extension(
            x509.KeyUsage(
                digital_signature=True,
                key_encipherment=True,
                content_commitment=False,
                data_encipherment=False,
                key_agreement=False,
                key_cert_sign=False,
                crl_sign=False,
                encipher_only=False,
                decipher_only=False,
            ),
            critical=True,
        )
        .sign(ca_private_key, hashes.SHA256(), backend=default_backend())
    )
    
    return cert.public_bytes(serialization.Encoding.PEM).decode('utf-8')


def create_self_signed_cert(
    module_name: str,
    private_key: rsa.RSAPrivateKey,
    validity_days: int = CERT_VALIDITY_DAYS
) -> x509.Certificate:
    """
    Create a self-signed certificate (for CA or testing).
    
    :param module_name: Name for the certificate
    :param private_key: RSA private key
    :param validity_days: Certificate validity in days
    :return: Self-signed certificate
    """
    subject = issuer = x509.Name([
        x509.NameAttribute(NameOID.COUNTRY_NAME, "DE"),
        x509.NameAttribute(NameOID.ORGANIZATION_NAME, "VYRA Framework"),
        x509.NameAttribute(NameOID.COMMON_NAME, module_name),
    ])
    
    now = datetime.utcnow()
    cert = (
        x509.CertificateBuilder()
        .subject_name(subject)
        .issuer_name(issuer)
        .public_key(private_key.public_key())
        .serial_number(x509.random_serial_number())
        .not_valid_before(now)
        .not_valid_after(now + timedelta(days=validity_days))
        .add_extension(
            x509.BasicConstraints(ca=True, path_length=0),
            critical=True,
        )
        .sign(private_key, hashes.SHA256(), backend=default_backend())
    )
    
    return cert


def verify_certificate(
    cert_pem: str,
    ca_cert_pem: str
) -> bool:
    """
    Verify a certificate against a CA certificate.
    
    :param cert_pem: PEM-encoded certificate to verify
    :param ca_cert_pem: PEM-encoded CA certificate
    :return: True if certificate is valid
    :raises CertificateValidationError: If certificate is invalid
    """
    try:
        cert = x509.load_pem_x509_certificate(
            cert_pem.encode('utf-8'),
            backend=default_backend()
        )
        ca_cert = x509.load_pem_x509_certificate(
            ca_cert_pem.encode('utf-8'),
            backend=default_backend()
        )
        
        # Check certificate validity period
        now = datetime.utcnow()
        if now < cert.not_valid_before or now > cert.not_valid_after:
            raise CertificateValidationError("Certificate expired or not yet valid")
        
        # Verify signature
        ca_public_key = ca_cert.public_key()
        ca_public_key.verify(
            cert.signature,
            cert.tbs_certificate_bytes,
            padding.PKCS1v15(),
            cert.signature_hash_algorithm,
        )
        
        return True
    except Exception as e:
        raise CertificateValidationError(f"Certificate validation failed: {e}")


def sign_message_rsa(message: str, private_key: rsa.RSAPrivateKey) -> str:
    """
    Sign a message with RSA private key.
    
    :param message: Message to sign
    :param private_key: RSA private key
    :return: Base64-encoded signature
    """
    message_bytes = message.encode('utf-8')
    signature = private_key.sign(
        message_bytes,
        padding.PSS(
            mgf=padding.MGF1(hashes.SHA256()),
            salt_length=padding.PSS.MAX_LENGTH
        ),
        hashes.SHA256()
    )
    return base64.b64encode(signature).decode('utf-8')


def verify_message_rsa(message: str, signature: str, cert_pem: str) -> bool:
    """
    Verify RSA signature of a message.
    
    :param message: Original message
    :param signature: Base64-encoded signature
    :param cert_pem: PEM-encoded certificate containing public key
    :return: True if signature is valid
    :raises SignatureValidationError: If signature is invalid
    """
    try:
        cert = x509.load_pem_x509_certificate(
            cert_pem.encode('utf-8'),
            backend=default_backend()
        )
        public_key = cert.public_key()
        
        message_bytes = message.encode('utf-8')
        signature_bytes = base64.b64decode(signature)
        
        public_key.verify(
            signature_bytes,
            message_bytes,
            padding.PSS(
                mgf=padding.MGF1(hashes.SHA256()),
                salt_length=padding.PSS.MAX_LENGTH
            ),
            hashes.SHA256()
        )
        return True
    except Exception as e:
        raise SignatureValidationError(f"Signature verification failed: {e}")


# ============================================================================
# JWT Operations (RS256)
# ============================================================================

def _ensure_jwt_crypto() -> None:
    """
    Ensure PyJWT has RS256 and other cryptographic algorithms registered.

    ``redis-py`` may import ``jwt`` before ``cryptography`` is fully
    available (e.g. during pytest collection), causing
    ``jwt.algorithms.has_crypto`` to be stuck at ``False``.  This
    function detects the situation and re-registers the missing
    crypto algorithms on the global PyJWS instance.
    """
    import jwt
    import jwt.algorithms as jwt_alg
    from jwt.api_jws import _jws_global_obj

    if "RS256" in _jws_global_obj._algorithms:
        return  # already registered, nothing to do

    # cryptography is guaranteed to be available (top-level import in this file).
    # Reload jwt.algorithms so it re-evaluates the try/except cryptography block.
    import importlib
    importlib.reload(jwt_alg)

    # Now get_default_algorithms() includes the crypto algorithms
    for name, algo in jwt_alg.get_default_algorithms().items():
        if name not in _jws_global_obj._algorithms:
            _jws_global_obj._algorithms[name] = algo


def create_jwt_token(
    payload: Dict[str, Any],
    private_key: rsa.RSAPrivateKey,
    algorithm: str = "RS256",
    expires_delta: Optional[timedelta] = None,
) -> str:
    """
    Create a JWT token signed with an RSA private key.

    :param payload: Claims to include in the token
    :param private_key: RSA private key for signing
    :param algorithm: Signing algorithm (default RS256)
    :param expires_delta: Optional token lifetime; defaults to 8 hours
    :return: Encoded JWT string
    """
    import jwt  # lazy import to avoid circular dependency
    _ensure_jwt_crypto()

    now = datetime.now(timezone.utc)
    claims = {
        **payload,
        "iat": now,
        "jti": uuid.uuid4().hex,
    }
    if expires_delta is None:
        expires_delta = timedelta(hours=8)
    claims["exp"] = now + expires_delta

    pem_bytes = private_key.private_bytes(
        encoding=serialization.Encoding.PEM,
        format=serialization.PrivateFormat.PKCS8,
        encryption_algorithm=serialization.NoEncryption(),
    )
    return jwt.encode(claims, pem_bytes, algorithm=algorithm)


def verify_jwt_token(
    token: str,
    public_key_pem: bytes,
    algorithms: Optional[list] = None,
) -> Dict[str, Any]:
    """
    Verify and decode a JWT token using a public key (PEM-encoded bytes).

    :param token: Encoded JWT string
    :param public_key_pem: PEM-encoded public key or certificate bytes
    :param algorithms: Allowed algorithms (default ``["RS256"]``)
    :return: Decoded claims dictionary
    :raises jwt.ExpiredSignatureError: If the token has expired
    :raises jwt.InvalidTokenError: If the token is invalid
    """
    import jwt  # lazy import to avoid circular dependency
    _ensure_jwt_crypto()

    if algorithms is None:
        algorithms = ["RS256"]
    return jwt.decode(token, public_key_pem, algorithms=algorithms)


def decode_jwt_token_unverified(token: str) -> Dict[str, Any]:
    """
    Decode a JWT token **without** verifying the signature.

    Useful for inspecting claims (e.g. ``jti``, ``sub``) before full
    verification.

    :param token: Encoded JWT string
    :return: Decoded claims dictionary
    """
    import jwt  # lazy import to avoid circular dependency
    _ensure_jwt_crypto()

    return jwt.decode(token, options={"verify_signature": False})


# ============================================================================
# Bcrypt Password Hashing
# ============================================================================

def hash_password_bcrypt(password: str) -> str:
    """
    Hash a password using bcrypt.

    :param password: Plain-text password
    :return: Bcrypt hash string (includes salt)
    """
    import bcrypt  # lazy import to avoid circular dependency

    return bcrypt.hashpw(password.encode("utf-8"), bcrypt.gensalt()).decode("utf-8")


def verify_password_bcrypt(password: str, hashed: str) -> bool:
    """
    Verify a plain-text password against a bcrypt hash.

    :param password: Plain-text password
    :param hashed: Bcrypt hash to check against
    :return: ``True`` if the password matches
    """
    import bcrypt  # lazy import to avoid circular dependency

    return bcrypt.checkpw(password.encode("utf-8"), hashed.encode("utf-8"))
