"""
vyra_base.auth — shared authentication primitives for VYRA modules.

Public API
----------
Exceptions:
    UsermanagerUnavailableError

Models:
    AuthToken, LoginRequest, LoginResponse,
    ChangePasswordRequest, CreateUserRequest

Base service:
    BaseAuthService

Router utilities:
    TOKEN_COOKIE_MAX_AGE, security, _extract_token, make_auth_service_di
"""

from .exceptions import UsermanagerUnavailableError
from .models import (
    AuthToken,
    ChangePasswordRequest,
    CreateUserRequest,
    LoginRequest,
    LoginResponse,
)
from .base_auth_service import BaseAuthService
from .router_utils import TOKEN_COOKIE_MAX_AGE, _extract_token, make_auth_service_di, security

__all__ = [
    # Exceptions
    "UsermanagerUnavailableError",
    # Models
    "AuthToken",
    "LoginRequest",
    "LoginResponse",
    "ChangePasswordRequest",
    "CreateUserRequest",
    # Base service
    "BaseAuthService",
    # Router utilities
    "TOKEN_COOKIE_MAX_AGE",
    "security",
    "_extract_token",
    "make_auth_service_di",
]
