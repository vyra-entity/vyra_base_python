"""
Unit tests for vyra_base.security.security_manager.SecurityManager
and SecuritySession.

These tests avoid real crypto: max_security_level is kept below
DIGITAL_SIGNATURE so _load_or_create_ca() is never called.
"""

import pytest
from datetime import datetime, timedelta
from unittest.mock import MagicMock, patch

from vyra_base.security.security_manager import SecurityManager, SecuritySession
from vyra_base.security.security_levels import (
    SecurityLevel,
    DEFAULT_SESSION_DURATION_SECONDS,
)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture
def manager():
    """SecurityManager at HMAC level (no CA infrastructure needed)."""
    return SecurityManager(max_security_level=SecurityLevel.HMAC)


@pytest.fixture
def manager_with_password():
    """SecurityManager at EXTENDED_AUTH with a known module password."""
    return SecurityManager(
        max_security_level=SecurityLevel.EXTENDED_AUTH,
        module_passwords={"module_abc": "s3cr3t"},
    )


@pytest.fixture
def session():
    """A fresh, non-expired SecuritySession."""
    return SecuritySession(
        module_name="test_mod",
        module_id="mod-uuid-123",
        role="operator",
        security_level=SecurityLevel.HMAC,
        session_token="tok-abc",
    )


# ---------------------------------------------------------------------------
# SecurityManager.__init__
# ---------------------------------------------------------------------------


class TestSecurityManagerInit:
    """Verify SecurityManager initialises cleanly."""

    def test_default_security_level(self, manager):
        assert manager.max_security_level == SecurityLevel.HMAC

    def test_sessions_initially_empty(self, manager):
        assert manager.sessions == {}

    def test_sessions_by_module_initially_empty(self, manager):
        assert manager.sessions_by_module == {}

    def test_module_passwords_stored(self, manager_with_password):
        assert "module_abc" in manager_with_password.module_passwords

    def test_ca_not_loaded_below_level5(self, manager):
        """Level 4 (HMAC) must not trigger CA loading."""
        assert manager.ca_private_key is None
        assert manager.ca_cert is None


# ---------------------------------------------------------------------------
# SecuritySession
# ---------------------------------------------------------------------------


class TestSecuritySession:
    """Tests for SecuritySession helper methods."""

    def test_new_session_is_not_expired(self, session):
        assert session.is_expired() is False

    def test_expired_session_detected(self):
        s = SecuritySession(
            module_name="m",
            module_id="id",
            role="viewer",
            security_level=SecurityLevel.NONE,
            session_token="tok",
            expires_at=datetime.utcnow() - timedelta(seconds=1),
        )
        assert s.is_expired() is True

    def test_add_and_check_nonce(self, session):
        session.add_nonce(42)
        assert session.has_nonce(42) is True

    def test_nonce_not_present_before_add(self, session):
        assert session.has_nonce(999) is False

    def test_multiple_nonces_tracked(self, session):
        for n in (1, 2, 3):
            session.add_nonce(n)
        assert all(session.has_nonce(n) for n in (1, 2, 3))

    def test_nonce_duplication_is_detectable(self, session):
        session.add_nonce(7)
        # Replay: nonce already exists
        assert session.has_nonce(7) is True

    def test_session_created_at_is_set(self, session):
        assert isinstance(session.created_at, datetime)

    def test_session_expires_in_future(self, session):
        assert session.expires_at > datetime.utcnow()

    def test_session_duration_matches_default(self):
        s = SecuritySession(
            module_name="m",
            module_id="i",
            role="r",
            security_level=1,
            session_token="t",
        )
        delta = s.expires_at - s.created_at
        # Allow ±2s tolerance for test execution time
        assert abs(delta.total_seconds() - DEFAULT_SESSION_DURATION_SECONDS) < 2
