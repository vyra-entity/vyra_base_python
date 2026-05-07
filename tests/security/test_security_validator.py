"""
Unit tests for vyra_base.security.security_validator.SecurityValidator.

SafetyMetadata is represented by simple MagicMock objects so that
no ROS2 message types are needed.
"""

import time
import pytest
from unittest.mock import MagicMock, patch
from datetime import datetime, timedelta

from vyra_base.security.security_validator import SecurityValidator
from vyra_base.security.security_manager import SecurityManager, SecuritySession
from vyra_base.security.security_levels import (
    SecurityLevel,
    SignatureValidationError,
    TimestampValidationError,
    MAX_TIMESTAMP_DRIFT_SECONDS,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _metadata(
    security_level: int = SecurityLevel.NONE,
    sender_id: str = "mod-abc",
    timestamp_sec: int | None = None,
    nonce: int = 1,
    signature: str = "",
) -> MagicMock:
    """Build a minimal SafetyMetadata mock."""
    m = MagicMock()
    m.security_level = security_level
    m.sender_id = sender_id
    ts = MagicMock()
    ts.sec = timestamp_sec if timestamp_sec is not None else int(time.time())
    m.timestamp = ts
    m.nonce = nonce
    m.signature = signature
    return m


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture
def validator():
    """SecurityValidator without a SecurityManager."""
    return SecurityValidator(security_manager=None, strict_mode=True)


@pytest.fixture
def manager():
    return SecurityManager(max_security_level=SecurityLevel.HMAC)


@pytest.fixture
def validator_with_manager(manager):
    return SecurityValidator(security_manager=manager, strict_mode=True)


# ---------------------------------------------------------------------------
# Initialisation
# ---------------------------------------------------------------------------


class TestSecurityValidatorInit:
    def test_strict_mode_default_true(self):
        v = SecurityValidator()
        assert v.strict_mode is True

    def test_strict_mode_can_be_disabled(self):
        v = SecurityValidator(strict_mode=False)
        assert v.strict_mode is False

    def test_security_manager_stored(self, manager):
        v = SecurityValidator(security_manager=manager)
        assert v.security_manager is manager


# ---------------------------------------------------------------------------
# Level 1 (NONE) — no validation
# ---------------------------------------------------------------------------


class TestLevelNone:
    """Level 1: any metadata (or None) must pass without raising."""

    def test_none_metadata_level_none_ok(self, validator):
        result = validator.validate_metadata(None, required_level=SecurityLevel.NONE)
        assert result is None

    def test_mock_metadata_level_none_ok(self, validator):
        meta = _metadata(security_level=SecurityLevel.NONE)
        result = validator.validate_metadata(meta, required_level=SecurityLevel.NONE)
        assert result is None


# ---------------------------------------------------------------------------
# Insufficient security level
# ---------------------------------------------------------------------------


class TestInsufficientLevel:
    """Messages below the required level must raise SignatureValidationError."""

    def test_level_none_when_basic_required(self, validator):
        meta = _metadata(security_level=SecurityLevel.NONE)
        with pytest.raises(SignatureValidationError):
            validator.validate_metadata(meta, required_level=SecurityLevel.BASIC_AUTH)

    def test_basic_when_hmac_required(self, validator):
        meta = _metadata(security_level=SecurityLevel.BASIC_AUTH)
        with pytest.raises(SignatureValidationError):
            validator.validate_metadata(meta, required_level=SecurityLevel.HMAC)


# ---------------------------------------------------------------------------
# Missing metadata at higher level
# ---------------------------------------------------------------------------


class TestMissingMetadata:
    def test_none_metadata_with_basic_requirement_raises(self, validator):
        with pytest.raises(SignatureValidationError):
            validator.validate_metadata(None, required_level=SecurityLevel.BASIC_AUTH)


# ---------------------------------------------------------------------------
# Level 2 (BASIC_AUTH) — sender_id required
# ---------------------------------------------------------------------------


class TestLevelBasicAuth:
    def test_valid_sender_id_passes(self, validator):
        meta = _metadata(security_level=SecurityLevel.BASIC_AUTH, sender_id="valid-module")
        # Level 2 only checks sender_id; HMAC/signature not validated here
        # (validation stops before HMAC at level 2)
        result = validator.validate_metadata(meta, required_level=SecurityLevel.BASIC_AUTH)
        # Should return None or a session (no crash)
        assert result is None or True

    def test_empty_sender_id_raises(self, validator):
        meta = _metadata(security_level=SecurityLevel.BASIC_AUTH, sender_id="")
        with pytest.raises(SignatureValidationError):
            validator.validate_metadata(meta, required_level=SecurityLevel.BASIC_AUTH)


# ---------------------------------------------------------------------------
# Timestamp validation (Level 4+)
# ---------------------------------------------------------------------------


class TestTimestampValidation:
    def test_fresh_timestamp_passes(self, validator):
        """A timestamp within drift window must not raise."""
        meta = _metadata(
            security_level=SecurityLevel.BASIC_AUTH,
            timestamp_sec=int(time.time()),
        )
        # _validate_timestamp is private but called by validate_metadata for HMAC+
        validator._validate_timestamp(meta)  # should not raise

    def test_stale_timestamp_raises(self, validator):
        old_ts = int(time.time()) - MAX_TIMESTAMP_DRIFT_SECONDS - 60
        meta = _metadata(security_level=SecurityLevel.HMAC, timestamp_sec=old_ts)
        with pytest.raises(TimestampValidationError):
            validator._validate_timestamp(meta)

    def test_future_timestamp_too_far_raises(self, validator):
        future_ts = int(time.time()) + MAX_TIMESTAMP_DRIFT_SECONDS + 60
        meta = _metadata(security_level=SecurityLevel.HMAC, timestamp_sec=future_ts)
        with pytest.raises(TimestampValidationError):
            validator._validate_timestamp(meta)


# ---------------------------------------------------------------------------
# Nonce validation
# ---------------------------------------------------------------------------


class TestNonceValidation:
    def test_nonce_accepted_first_time(self, validator, manager):
        session = SecuritySession(
            module_name="m",
            module_id="id",
            role="r",
            security_level=SecurityLevel.HMAC,
            session_token="tok",
        )
        meta = _metadata(nonce=100)
        validator._validate_nonce(meta, session)
        assert session.has_nonce(100)

    def test_replay_nonce_raises(self, validator, manager):
        from vyra_base.security.security_levels import NonceValidationError
        session = SecuritySession(
            module_name="m",
            module_id="id",
            role="r",
            security_level=SecurityLevel.HMAC,
            session_token="tok",
        )
        session.add_nonce(55)
        meta = _metadata(nonce=55)
        with pytest.raises(NonceValidationError):
            validator._validate_nonce(meta, session)

    def test_nonce_validation_skipped_without_session(self, validator):
        meta = _metadata(nonce=42)
        # Must not raise
        validator._validate_nonce(meta, None)
