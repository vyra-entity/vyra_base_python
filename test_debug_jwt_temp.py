"""Debug test to check jwt state inside pytest context."""
import jwt as pyjwt
from vyra_base.helper.crypto_helper import (
    generate_rsa_keypair,
    create_jwt_token,
)
from cryptography.hazmat.primitives import serialization


def test_debug_jwt():
    pk, pub = generate_rsa_keypair()
    # Check jwt state
    import jwt
    print(f"\njwt.__file__: {jwt.__file__}")
    print(f"has_crypto: {jwt.algorithms.has_crypto}")
    print(f"RS256 registered: {'RS256' in jwt.PyJWS()._algorithms}")

    pub_pem = pub.public_bytes(
        encoding=serialization.Encoding.PEM,
        format=serialization.PublicFormat.SubjectPublicKeyInfo,
    )
    token = create_jwt_token({"sub": "test"}, pk)
    print(f"Token: {token[:30]}")
