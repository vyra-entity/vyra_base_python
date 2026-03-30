"""
Unit tests for vyra_base.security.security_levels

Tests cover SecurityLevel enum values, helper classmethods,
and instance methods (requires_hmac, requires_certificate).
"""

import pytest

from vyra_base.security.security_levels import SecurityLevel


# ---------------------------------------------------------------------------
# Enum values
# ---------------------------------------------------------------------------


class TestSecurityLevelValues:
    """Verify that each SecurityLevel has the expected integer value."""

    def test_none_is_1(self):
        assert SecurityLevel.NONE == 1

    def test_basic_auth_is_2(self):
        assert SecurityLevel.BASIC_AUTH == 2

    def test_extended_auth_is_3(self):
        assert SecurityLevel.EXTENDED_AUTH == 3

    def test_hmac_is_4(self):
        assert SecurityLevel.HMAC == 4

    def test_digital_signature_is_5(self):
        assert SecurityLevel.DIGITAL_SIGNATURE == 5

    def test_exactly_five_levels(self):
        """Ensure no extra levels have been added without updating tests."""
        assert len(SecurityLevel) == 5


# ---------------------------------------------------------------------------
# is_valid()
# ---------------------------------------------------------------------------


class TestIsValid:
    """Tests for SecurityLevel.is_valid() classmethod."""

    def test_all_valid_levels(self):
        for value in (1, 2, 3, 4, 5):
            assert SecurityLevel.is_valid(value) is True

    def test_zero_is_invalid(self):
        assert SecurityLevel.is_valid(0) is False

    def test_six_is_invalid(self):
        assert SecurityLevel.is_valid(6) is False

    def test_negative_is_invalid(self):
        assert SecurityLevel.is_valid(-1) is False


# ---------------------------------------------------------------------------
# get_name()
# ---------------------------------------------------------------------------


class TestGetName:
    """Tests for SecurityLevel.get_name() classmethod."""

    def test_get_name_none(self):
        assert SecurityLevel.get_name(1) == "NONE"

    def test_get_name_basic_auth(self):
        assert SecurityLevel.get_name(2) == "BASIC_AUTH"

    def test_get_name_extended_auth(self):
        assert SecurityLevel.get_name(3) == "EXTENDED_AUTH"

    def test_get_name_hmac(self):
        assert SecurityLevel.get_name(4) == "HMAC"

    def test_get_name_digital_signature(self):
        assert SecurityLevel.get_name(5) == "DIGITAL_SIGNATURE"

    def test_get_name_invalid_raises_value_error(self):
        with pytest.raises(ValueError):
            SecurityLevel.get_name(99)

    def test_get_name_zero_raises_value_error(self):
        with pytest.raises(ValueError):
            SecurityLevel.get_name(0)


# ---------------------------------------------------------------------------
# Ordering (IntEnum)
# ---------------------------------------------------------------------------


class TestLevelOrdering:
    """SecurityLevel is an IntEnum so comparison operators must work."""

    def test_none_less_than_basic_auth(self):
        assert SecurityLevel.NONE < SecurityLevel.BASIC_AUTH

    def test_basic_auth_less_than_extended_auth(self):
        assert SecurityLevel.BASIC_AUTH < SecurityLevel.EXTENDED_AUTH

    def test_extended_auth_less_than_hmac(self):
        assert SecurityLevel.EXTENDED_AUTH < SecurityLevel.HMAC

    def test_hmac_less_than_digital_signature(self):
        assert SecurityLevel.HMAC < SecurityLevel.DIGITAL_SIGNATURE

    def test_digital_signature_is_highest(self):
        for level in (
            SecurityLevel.NONE,
            SecurityLevel.BASIC_AUTH,
            SecurityLevel.EXTENDED_AUTH,
            SecurityLevel.HMAC,
        ):
            assert level < SecurityLevel.DIGITAL_SIGNATURE


# ---------------------------------------------------------------------------
# requires_hmac()
# ---------------------------------------------------------------------------


class TestRequiresHmac:
    """Tests for SecurityLevel.requires_hmac() instance method."""

    def test_none_does_not_require_hmac(self):
        assert SecurityLevel.NONE.requires_hmac() is False

    def test_basic_auth_does_not_require_hmac(self):
        assert SecurityLevel.BASIC_AUTH.requires_hmac() is False

    def test_extended_auth_does_not_require_hmac(self):
        assert SecurityLevel.EXTENDED_AUTH.requires_hmac() is False

    def test_hmac_requires_hmac(self):
        assert SecurityLevel.HMAC.requires_hmac() is True

    def test_digital_signature_requires_hmac(self):
        assert SecurityLevel.DIGITAL_SIGNATURE.requires_hmac() is True


# ---------------------------------------------------------------------------
# requires_certificate()
# ---------------------------------------------------------------------------


class TestRequiresCertificate:
    """Tests for SecurityLevel.requires_certificate() instance method."""

    def test_levels_below_digital_signature_do_not_require_cert(self):
        for level in (
            SecurityLevel.NONE,
            SecurityLevel.BASIC_AUTH,
            SecurityLevel.EXTENDED_AUTH,
            SecurityLevel.HMAC,
        ):
            assert level.requires_certificate() is False

    def test_digital_signature_requires_certificate(self):
        assert SecurityLevel.DIGITAL_SIGNATURE.requires_certificate() is True
