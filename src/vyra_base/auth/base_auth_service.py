"""
BaseAuthService — shared Redis-session authentication logic for VYRA modules.

Concrete modules extend this class and implement only the two abstract methods:
  - ``_get_user_manager()``: return the module's user-manager DI instance.
  - ``_validate_usermanager_credentials()``: validate against the external UM.

Everything else (local validation, session creation, token validation/revocation,
password change, user listing/creation) is implemented here once.
"""

import json as _json
import logging
import secrets
from abc import ABC, abstractmethod
from datetime import datetime, timedelta
from typing import Any, Dict, Optional

from vyra_base.com.transport.t_redis.communication import RedisClient

from .exceptions import UsermanagerUnavailableError  # noqa: F401 — re-export for convenience
from .models import AuthToken

_TOKEN_TTL = 3600 * 8  # 8 hours


class BaseAuthService(ABC):
    """
    Abstract base class for Redis-session-based authentication services.

    Subclasses must implement:
        - :meth:`_get_user_manager` — resolve the DI user manager instance.
        - :meth:`_validate_usermanager_credentials` — delegate to the external UM.
        - :meth:`check_usermanager_available` — probe whether the UM is reachable.
    """

    def __init__(
        self,
        redis_client: RedisClient,
        module_id: str,
        logger: Optional[logging.Logger] = None,
    ) -> None:
        """
        Args:
            redis_client: Connected Redis client used for session storage.
            module_id:    Unique module identifier (e.g. ``"v2_dashboard"``).
            logger:       Optional pre-configured logger; falls back to the
                          standard ``logging`` logger for this module.
        """
        self.redis = redis_client
        self.module_id = module_id
        self._token_prefix = f"{module_id}:auth:tokens"
        self._token_ttl = _TOKEN_TTL
        self._logger = logger if logger is not None else logging.getLogger(__name__)
        self._logger.info(f"✅ Authentication service initialized for {module_id} (direct DI)")

    # ------------------------------------------------------------------
    # Abstract interface — implemented per module
    # ------------------------------------------------------------------

    @abstractmethod
    def _get_user_manager(self) -> Any:
        """Return the user-manager instance from container injection."""

    @abstractmethod
    async def _validate_usermanager_credentials(
        self, username: str, password: str
    ) -> Optional[Dict[str, Any]]:
        """
        Validate credentials against the external UserManager service.

        Returns a user-info dict on success or raises on error.
        """

    @abstractmethod
    async def check_usermanager_available(self) -> Dict[str, Any]:
        """
        Probe whether the external UserManager is reachable.

        Returns a dict with at least ``{"available": bool, "message": str}``.
        """

    # ------------------------------------------------------------------
    # Shared credential validation
    # ------------------------------------------------------------------

    async def validate_credentials(
        self,
        username: str,
        password: str,
        auth_mode: str = "local",
    ) -> Optional[Dict[str, Any]]:
        """
        Dispatch credential validation to the correct backend.

        Args:
            username:  Plaintext username.
            password:  Plaintext password.
            auth_mode: ``"local"`` or ``"usermanager"``.

        Returns:
            User-info dict on success, ``None`` otherwise.
        """
        if auth_mode == "local":
            return await self._validate_local_credentials(username, password)
        if auth_mode == "usermanager":
            return await self._validate_usermanager_credentials(username, password)
        self._logger.error(f"Unknown auth_mode: {auth_mode}")
        return None

    async def _validate_local_credentials(
        self, username: str, password: str
    ) -> Optional[Dict[str, Any]]:
        """Validate credentials via the InternalUserManager (direct DI)."""
        try:
            user_manager = self._get_user_manager()
            if not user_manager or not user_manager.internal_usermanager:
                raise Exception("InternalUserManager not available")

            user_info = await user_manager.internal_usermanager.authenticate(username, password)
            if user_info:
                self._logger.info(f"✅ User authenticated via direct DI: {username}")
                return {**user_info, "auth_mode": "local"}

            self._logger.warning(f"❌ Authentication failed for: {username}")
            raise Exception("Invalid username or password")
        except Exception as exc:
            self._logger.error(f"❌ Authentication error: {exc}", exc_info=True)
            raise Exception("General authentication error") from exc

    # ------------------------------------------------------------------
    # Session management
    # ------------------------------------------------------------------

    async def create_session(self, username: str, user_info: Dict[str, Any]) -> str:
        """
        Create a Redis-backed session and return the session token.

        Args:
            username:  Authenticated username.
            user_info: User-info dict returned by ``validate_credentials``.

        Returns:
            A ``secrets.token_urlsafe(32)`` session token.
        """
        token = secrets.token_urlsafe(32)
        auth_token = AuthToken(
            token=token,
            username=username,
            user_id=user_info.get("user_id", 0),
            role=user_info.get("role", "viewer"),
            level=user_info.get("level", 3),
            created_at=datetime.utcnow().isoformat(),
            expires_at=(datetime.utcnow() + timedelta(seconds=self._token_ttl)).isoformat(),
            module_id=self.module_id,
        )
        token_key = f"{self._token_prefix}:{token}"
        await self.redis.set(token_key, auth_token.model_dump_json())
        client = await self.redis._ensure_connected()
        await client.expire(token_key, self._token_ttl)
        self._logger.info(f"✅ Created session for user: {username}")
        return token

    async def validate_token(self, token: str) -> Optional[Dict[str, Any]]:
        """
        Validate a session token.

        Returns a user-info dict if the token is valid and not expired,
        ``None`` otherwise.
        """
        token_key = f"{self._token_prefix}:{token}"
        token_json = await self.redis.get(token_key)
        if not token_json:
            self._logger.debug("Invalid or expired token")
            return None
        try:
            auth_token = AuthToken.model_validate_json(token_json)
            expires_at = datetime.fromisoformat(auth_token.expires_at)
            if datetime.utcnow() > expires_at:
                self._logger.debug("Token expired")
                await self.redis.delete(token_key)
                return None
            return {
                "user_id": auth_token.user_id,
                "username": auth_token.username,
                "role": auth_token.role,
                "level": auth_token.level,
                "token": auth_token.token,
                "module_id": auth_token.module_id,
            }
        except Exception as exc:
            self._logger.error(f"Error validating token: {exc}")
            return None

    async def revoke_token(self, token: str) -> bool:
        """
        Revoke a session token (logout).

        Returns ``True`` if the token was deleted from Redis.
        """
        token_key = f"{self._token_prefix}:{token}"
        result = await self.redis.delete(token_key)
        return result > 0

    # ------------------------------------------------------------------
    # User management helpers (local UserManager DI)
    # ------------------------------------------------------------------

    async def change_password(
        self, username: str, old_password: str, new_password: str
    ) -> bool:
        """
        Change a local user's password via InternalUserManager.

        Returns ``True`` on success.
        """
        try:
            user_manager = self._get_user_manager()
            if not user_manager or not user_manager.internal_usermanager:
                raise Exception("InternalUserManager not available")
            result = await user_manager.internal_usermanager.change_password_impl(
                username, old_password, new_password
            )
            return result.get("success", False) if result else False
        except Exception as exc:
            self._logger.error(f"Error changing password: {exc}", exc_info=True)
            return False

    async def list_local_users(self) -> list:
        """Return all local users via InternalUserManager."""
        try:
            user_manager = self._get_user_manager()
            if not user_manager or not user_manager.internal_usermanager:
                raise Exception("InternalUserManager not available")
            result = await user_manager.internal_usermanager.list_users_impl()
            return result.get("users", []) if result else []
        except Exception as exc:
            self._logger.error(f"Error listing users: {exc}", exc_info=True)
            return []

    async def create_local_user(
        self,
        username: str,
        password: str,
        role: str = "viewer",
        level: int = 0,
        permissions: str | dict = "{}",
    ) -> Dict[str, Any]:
        """
        Create a new local user via InternalUserManager.

        ``permissions`` may be a JSON string or a plain dict.
        """
        try:
            user_manager = self._get_user_manager()
            if not user_manager or not user_manager.internal_usermanager:
                raise Exception("InternalUserManager not available")

            if isinstance(permissions, str):
                try:
                    permissions_parsed: dict = _json.loads(permissions)
                except _json.JSONDecodeError:
                    self._logger.error("Invalid permissions JSON string")
                    return {"success": False, "message": "Invalid permissions format"}
            else:
                permissions_parsed = permissions

            result = await user_manager.internal_usermanager.create_user_impl(
                username=username,
                password=password,
                role=role,
                level=level,
                permissions=permissions_parsed,
            )
            if result and result.get("success"):
                return {
                    "success": True,
                    "message": "User created successfully",
                    "user": result.get("user"),
                }
            return {
                "success": False,
                "message": result.get("message", "Failed to create user") if result else "Failed to create user",
            }
        except Exception as exc:
            self._logger.error(f"Error creating user: {exc}", exc_info=True)
            return {"success": False, "message": f"Error: {exc}"}

    async def delete_local_user(self, username: str) -> Dict[str, Any]:
        """Delete a local user by username via InternalUserManager.

        Args:
            username: The username to delete.

        Returns:
            Dict with ``success`` bool and ``message`` string.
        """
        try:
            user_manager = self._get_user_manager()
            if not user_manager or not user_manager.internal_usermanager:
                raise Exception("InternalUserManager not available")
            deleted = await user_manager.internal_usermanager.delete_user_impl(username)
            if deleted:
                return {"success": True, "message": f"User '{username}' deleted"}
            return {"success": False, "message": f"Failed to delete user '{username}' (may be last admin)"}
        except Exception as exc:
            self._logger.error(f"Error deleting user: {exc}", exc_info=True)
            return {"success": False, "message": f"Error: {exc}"}
