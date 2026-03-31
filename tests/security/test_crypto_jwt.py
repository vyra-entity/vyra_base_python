"""
Unit tests for JWT and bcrypt functions in crypto_helper.
"""

import pytest
from datetime import timedelta

from vyra_base.helper.crypto_helper import (
    generate_rsa_keypair,
    create_jwt_token,
    verify_jwt_token,
    decode_jwt_token_unverified,
    hash_password_bcrypt,
    verify_password_bcrypt,
)
from cryptography.hazmat.primitives import serialization
import jwt as pyjwt


@pytest.fixture
def rsa_keys():
    """Generate a fresh RSA key pair for tests."""
    private_key, public_key = generate_rsa_keypair()
    public_pem = public_key.public_bytes(
        encoding=serialization.Encoding.PEM,
        format=serialization.PublicFormat.SubjectPublicKeyInfo,
    )
    return private_key, public_pem


class TestJWTToken:
    """Tests for JWT create / verify / decode functions."""

    def test_create_and_verify(self, rsa_keys):
        private_key, public_pem = rsa_keys
        payload = {"sub": "user123", "roles": ["admin"]}
        token = create_jwt_token(payload, private_key)

        decoded = verify_jwt_token(token, public_pem)
        assert decoded["sub"] == "user123"
        assert decoded["roles"] == ["admin"]
        assert "iat" in decoded
        assert "exp" in decoded
        assert "jti" in decoded

    def test_expired_token_rejected(self, rsa_keys):
        private_key, public_pem = rsa_keys
        token = create_jwt_token(
            {"sub": "user1"},
            private_key,
            expires_delta=timedelta(seconds=-1),
        )
        with pytest.raises(pyjwt.ExpiredSignatureError):
            verify_jwt_token(token, public_pem)

    def test_invalid_signature_rejected(self, rsa_keys):
        private_key, _ = rsa_keys
        # Sign with one key, verify with another
        other_private, other_public = generate_rsa_keypair()
        other_public_pem = other_public.public_bytes(
            encoding=serialization.Encoding.PEM,
            format=serialization.PublicFormat.SubjectPublicKeyInfo,
        )
        token = create_jwt_token({"sub": "x"}, private_key)

        with pytest.raises(pyjwt.InvalidSignatureError):
            verify_jwt_token(token, other_public_pem)

    def test_decode_unverified(self, rsa_keys):
        private_key, _ = rsa_keys
        token = create_jwt_token({"sub": "peek"}, private_key)
        claims = decode_jwt_token_unverified(token)
        assert claims["sub"] == "peek"

    def test_custom_expiry(self, rsa_keys):
        private_key, public_pem = rsa_keys
        token = create_jwt_token(
            {"sub": "u"},
            private_key,
            expires_delta=timedelta(hours=24),
        )
        decoded = verify_jwt_token(token, public_pem)
        assert decoded["sub"] == "u"

    def test_jti_is_unique(self, rsa_keys):
        private_key, public_pem = rsa_keys
        t1 = create_jwt_token({"sub": "a"}, private_key)
        t2 = create_jwt_token({"sub": "a"}, private_key)
        c1 = decode_jwt_token_unverified(t1)
        c2 = decode_jwt_token_unverified(t2)
        assert c1["jti"] != c2["jti"]


class TestBcryptPassword:
    """Tests for bcrypt hash / verify functions."""

    def test_hash_and_verify(self):
        pw = "SecurePassword123!"
        hashed = hash_password_bcrypt(pw)
        assert verify_password_bcrypt(pw, hashed)

    def test_wrong_password_rejected(self):
        hashed = hash_password_bcrypt("correct")
        assert not verify_password_bcrypt("wrong", hashed)

    def test_hash_is_unique(self):
        pw = "same"
        h1 = hash_password_bcrypt(pw)
        h2 = hash_password_bcrypt(pw)
        assert h1 != h2  # bcrypt salts differ
        assert verify_password_bcrypt(pw, h1)
        assert verify_password_bcrypt(pw, h2)
