"""Unit tests for shared auth models, exceptions, and DI utilities."""

from __future__ import annotations

import pytest
from fastapi import HTTPException
from fastapi.security import HTTPAuthorizationCredentials

from vyra_base.auth.exceptions import UsermanagerUnavailableError
from vyra_base.auth.models import (
    AuthToken,
    CreateUserRequest,
    LoginRequest,
    LoginResponse,
)
from vyra_base.auth.router_utils import _extract_token, make_auth_service_di


class TestAuthModels:
    """Test shared authentication request/response models."""

    def test_auth_token_requires_full_payload(self):
        """AuthToken should map required fields exactly."""
        token = AuthToken(
            token="abc",
            username="alice",
            user_id=42,
            role="admin",
            level=10,
            created_at="2026-05-07T10:00:00Z",
            expires_at="2026-05-07T18:00:00Z",
            module_id="module_1",
        )

        assert token.token == "abc"
        assert token.username == "alice"
        assert token.user_id == 42

    def test_login_request_defaults(self):
        """LoginRequest should provide default auth mode and module name."""
        request = LoginRequest(username="alice", password="secret")

        assert request.auth_mode == "local"
        assert request.module_name is None

    def test_login_response_defaults(self):
        """LoginResponse should provide empty default output fields."""
        response = LoginResponse(success=True)

        assert response.success is True
        assert response.token == ""
        assert response.username == ""
        assert response.auth_mode == "local"
        assert response.message == ""

    def test_create_user_request_defaults(self):
        """CreateUserRequest should set default role/level/permissions."""
        request = CreateUserRequest(username="bob", password="pw")

        assert request.role == "viewer"
        assert request.level == 0
        assert request.permissions == "{}"


class TestRouterUtils:
    """Test auth token extraction and DI helpers."""

    def test_extract_token_prefers_cookie_over_header(self):
        """Cookie token should take precedence over Authorization header."""
        credentials = HTTPAuthorizationCredentials(scheme="Bearer", credentials="header-token")

        token = _extract_token(auth_token="cookie-token", credentials=credentials)

        assert token == "cookie-token"

    def test_extract_token_falls_back_to_header(self):
        """Header token should be used when cookie token is missing."""
        credentials = HTTPAuthorizationCredentials(scheme="Bearer", credentials="header-token")

        token = _extract_token(auth_token=None, credentials=credentials)

        assert token == "header-token"

    def test_extract_token_raises_when_missing(self):
        """Missing credentials should raise HTTP 401."""
        with pytest.raises(HTTPException) as exc:
            _extract_token(auth_token=None, credentials=None)

        assert exc.value.status_code == 401
        assert exc.value.detail == "No authentication token provided"

    def test_make_auth_service_di_lifecycle(self):
        """DI factory should return unset error before injection and value after injection."""
        set_auth_service, get_auth_service = make_auth_service_di()

        with pytest.raises(HTTPException) as exc:
            get_auth_service()

        assert exc.value.status_code == 503
        assert exc.value.detail == "Auth service not initialized"

        service = object()
        set_auth_service(service)

        assert get_auth_service() is service


class TestAuthExceptions:
    """Test custom shared auth exceptions."""

    def test_usermanager_unavailable_error_is_exception(self):
        """Custom auth exception should preserve the error message."""
        message = "service offline"
        err = UsermanagerUnavailableError(message)

        assert isinstance(err, Exception)
        assert str(err) == message
