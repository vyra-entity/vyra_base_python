"""
Security-specific test fixtures.

Restores the real ``cryptography`` and ``jwt`` packages that are mocked
globally in ``tests/conftest.py`` so that JWT signing / verification and
bcrypt hashing work with actual cryptographic operations.
"""

import importlib
import sys

# ── Restore real cryptography ────────────────────────────────────────────
# The root conftest.py replaces every ``cryptography.*`` entry in
# sys.modules with an AutoMock **at import time** (before collection).
# We need to undo that so the security tests can use the real library.

_crypto_keys = [k for k in sys.modules if k == "cryptography" or k.startswith("cryptography.")]
for _k in _crypto_keys:
    del sys.modules[_k]

# Force a fresh, real import of cryptography
import cryptography  # noqa: E402, F811

# ── Restore real PyJWT ───────────────────────────────────────────────────
# PyJWT caches ``has_crypto`` at first import.  If it was imported while
# cryptography was still mocked, the flag is stuck on False.
_jwt_keys = [k for k in sys.modules if k == "jwt" or k.startswith("jwt.")]
for _k in _jwt_keys:
    del sys.modules[_k]

import jwt  # noqa: E402, F811

# ── Re-import crypto_helper so it picks up the real modules ─────────────
_helper_keys = [
    k for k in sys.modules
    if "crypto_helper" in k or k.startswith("vyra_base.helper.crypto_helper")
]
for _k in _helper_keys:
    del sys.modules[_k]

from vyra_base.helper import crypto_helper  # noqa: E402, F811

# Sanity: make sure RS256 is actually available
assert jwt.algorithms.has_crypto, "cryptography was not restored correctly"
