"""
Shared FastAPI dependency-injection utilities for VYRA authentication routers.

Provides:
- ``TOKEN_COOKIE_MAX_AGE`` — session cookie lifetime (8 h).
- ``security`` — ``HTTPBearer`` scheme used on protected routes.
- ``_extract_token()`` — resolves a bearer token from cookie or header.
- ``make_auth_service_di()`` — factory returning ``(set_auth_service, get_auth_service)``
  closures that every module router uses for the auth-service DI pattern.
"""

from typing import Any, Callable, Optional, Tuple

from fastapi import HTTPException
from fastapi.security import HTTPAuthorizationCredentials, HTTPBearer

TOKEN_COOKIE_MAX_AGE: int = 3600 * 8  # 8 hours

security = HTTPBearer(auto_error=False)


def _extract_token(
    auth_token: Optional[str],
    credentials: Optional[HTTPAuthorizationCredentials],
) -> str:
    """
    Resolve the bearer token from either a cookie value or an Authorization header.

    Args:
        auth_token:   Value of the ``auth_token`` cookie (may be ``None``).
        credentials:  ``HTTPBearer`` credentials extracted from the request header.

    Returns:
        The raw token string.

    Raises:
        HTTPException(401): when neither source provides a token.
    """
    if auth_token:
        return auth_token
    if credentials:
        return credentials.credentials
    raise HTTPException(status_code=401, detail="No authentication token provided")


def make_auth_service_di() -> Tuple[Callable[[Any], None], Callable[[], Any]]:
    """
    Factory that creates a ``(set_auth_service, get_auth_service)`` pair.

    Each module router calls this once at import time::

        from vyra_base.auth import make_auth_service_di
        set_auth_service, get_auth_service = make_auth_service_di()

    ``set_auth_service`` is called during application startup (in ``main_rest.py``).
    ``get_auth_service`` is used as a FastAPI ``Depends()`` target on protected routes.

    Returns:
        A tuple ``(set_auth_service, get_auth_service)``.
    """
    _service: list = [None]  # mutable single-element list as a closure cell

    def set_auth_service(auth_service: Any) -> None:
        """Inject the ``AuthenticationService`` instance (called once at startup)."""
        _service[0] = auth_service

    def get_auth_service() -> Any:
        """Return the injected service, or raise HTTP 503 if not yet set."""
        if _service[0] is None:
            raise HTTPException(status_code=503, detail="Auth service not initialized")
        return _service[0]

    return set_auth_service, get_auth_service
