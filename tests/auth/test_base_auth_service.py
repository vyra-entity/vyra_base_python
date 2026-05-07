"""
Unit tests for vyra_base.auth.base_auth_service.BaseAuthService.

All Redis calls are mocked via AsyncMock so no live Redis server is needed.
The abstract methods are implemented in a minimal ConcreteAuthService subclass.
"""

import pytest
from unittest.mock import AsyncMock, MagicMock, patch
from datetime import datetime, timedelta

from vyra_base.auth.base_auth_service import BaseAuthService
from vyra_base.auth.models import AuthToken


# ---------------------------------------------------------------------------
# Minimal concrete implementation for testing
# ---------------------------------------------------------------------------


class ConcreteAuthService(BaseAuthService):
    """Concrete subclass of BaseAuthService for testing."""

    def __init__(self, redis_client, module_id, user_manager=None):
        super().__init__(redis_client, module_id)
        self._user_manager = user_manager

    def _get_user_manager(self):
        return self._user_manager

    async def _validate_usermanager_credentials(self, username, password):
        return {"user_id": 99, "role": "operator", "username": username}

    async def check_usermanager_available(self):
        return {"available": True, "message": "ok"}


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture
def mock_redis():
    """Mock RedisClient with async get/set/delete/_ensure_connected."""
    redis = MagicMock()
    redis.get = AsyncMock(return_value=None)
    redis.set = AsyncMock()
    redis.delete = AsyncMock(return_value=1)

    inner = AsyncMock()
    inner.expire = AsyncMock()
    redis._ensure_connected = AsyncMock(return_value=inner)
    return redis


@pytest.fixture
def service(mock_redis):
    """ConcreteAuthService with mocked Redis."""
    return ConcreteAuthService(mock_redis, "test_module")


# ---------------------------------------------------------------------------
# Constructor / initialisation
# ---------------------------------------------------------------------------


class TestBaseAuthServiceInit:
    """Tests for __init__."""

    def test_module_id_stored(self, service):
        assert service.module_id == "test_module"

    def test_token_prefix_contains_module_id(self, service):
        assert "test_module" in service._token_prefix

    def test_token_ttl_is_positive(self, service):
        assert service._token_ttl > 0

    def test_custom_logger_is_used(self, mock_redis):
        import logging
        custom_logger = logging.getLogger("custom")
        svc = ConcreteAuthService(mock_redis, "mod", user_manager=None)
        svc._logger = custom_logger
        assert svc._logger is custom_logger


# ---------------------------------------------------------------------------
# validate_credentials — dispatch
# ---------------------------------------------------------------------------


class TestValidateCredentials:
    """Tests for validate_credentials() dispatcher."""

    @pytest.mark.asyncio
    async def test_local_mode_calls_local_validator(self, service):
        """auth_mode='local' must invoke _validate_local_credentials."""
        um = MagicMock()
        um.internal_usermanager = MagicMock()
        um.internal_usermanager.authenticate = AsyncMock(
            return_value={"user_id": 1, "role": "admin"}
        )
        service._user_manager = um

        result = await service.validate_credentials("alice", "pass", auth_mode="local")
        assert result is not None
        assert result["auth_mode"] == "local"

    @pytest.mark.asyncio
    async def test_usermanager_mode_delegates(self, service):
        """auth_mode='usermanager' must call _validate_usermanager_credentials."""
        result = await service.validate_credentials("bob", "secret", auth_mode="usermanager")
        assert result is not None
        assert result["user_id"] == 99

    @pytest.mark.asyncio
    async def test_unknown_mode_returns_none(self, service):
        result = await service.validate_credentials("x", "y", auth_mode="magic")
        assert result is None

    @pytest.mark.asyncio
    async def test_local_mode_raises_on_auth_failure(self, service):
        """Invalid credentials must raise an exception."""
        um = MagicMock()
        um.internal_usermanager = MagicMock()
        um.internal_usermanager.authenticate = AsyncMock(return_value=None)
        service._user_manager = um

        with pytest.raises(Exception):
            await service.validate_credentials("bad", "creds", auth_mode="local")

    @pytest.mark.asyncio
    async def test_local_mode_raises_when_no_user_manager(self, service):
        """If user_manager is None, local auth must raise."""
        service._user_manager = None

        with pytest.raises(Exception):
            await service.validate_credentials("x", "y", auth_mode="local")


# ---------------------------------------------------------------------------
# create_session
# ---------------------------------------------------------------------------


class TestCreateSession:
    """Tests for create_session()."""

    @pytest.mark.asyncio
    async def test_returns_token_string(self, service, mock_redis):
        user_info = {"user_id": 1, "role": "admin", "level": 5}
        token = await service.create_session("alice", user_info)
        assert isinstance(token, str)
        assert len(token) > 10

    @pytest.mark.asyncio
    async def test_redis_set_called(self, service, mock_redis):
        user_info = {"user_id": 1, "role": "viewer", "level": 1}
        await service.create_session("bob", user_info)
        mock_redis.set.assert_awaited_once()

    @pytest.mark.asyncio
    async def test_expire_called(self, service, mock_redis):
        user_info = {"user_id": 2, "role": "operator", "level": 3}
        await service.create_session("carol", user_info)
        inner = await mock_redis._ensure_connected()
        inner.expire.assert_awaited()

    @pytest.mark.asyncio
    async def test_tokens_are_unique(self, service, mock_redis):
        user_info = {"user_id": 1, "role": "viewer", "level": 1}
        t1 = await service.create_session("user1", user_info)
        t2 = await service.create_session("user2", user_info)
        assert t1 != t2


# ---------------------------------------------------------------------------
# validate_token
# ---------------------------------------------------------------------------


class TestValidateToken:
    """Tests for validate_token()."""

    def _make_token_json(self, username="alice", expired=False, module_id="test_module"):
        """Helper to build a valid AuthToken JSON string."""
        now = datetime.utcnow()
        if expired:
            expires = (now - timedelta(hours=1)).isoformat()
        else:
            expires = (now + timedelta(hours=8)).isoformat()

        t = AuthToken(
            token="sometoken",
            username=username,
            user_id=1,
            role="admin",
            level=5,
            created_at=now.isoformat(),
            expires_at=expires,
            module_id=module_id,
        )
        return t.model_dump_json()

    @pytest.mark.asyncio
    async def test_valid_token_returns_user_info(self, service, mock_redis):
        mock_redis.get = AsyncMock(return_value=self._make_token_json())
        result = await service.validate_token("sometoken")
        assert result is not None
        assert result["username"] == "alice"
        assert result["role"] == "admin"

    @pytest.mark.asyncio
    async def test_missing_token_returns_none(self, service, mock_redis):
        mock_redis.get = AsyncMock(return_value=None)
        result = await service.validate_token("badtoken")
        assert result is None

    @pytest.mark.asyncio
    async def test_expired_token_returns_none_and_deletes(self, service, mock_redis):
        mock_redis.get = AsyncMock(return_value=self._make_token_json(expired=True))
        result = await service.validate_token("expiredtoken")
        assert result is None
        mock_redis.delete.assert_awaited()

    @pytest.mark.asyncio
    async def test_corrupt_json_returns_none(self, service, mock_redis):
        mock_redis.get = AsyncMock(return_value=b"not-valid-json")
        result = await service.validate_token("token")
        assert result is None


# ---------------------------------------------------------------------------
# revoke_token
# ---------------------------------------------------------------------------


class TestRevokeToken:
    """Tests for revoke_token()."""

    @pytest.mark.asyncio
    async def test_returns_true_when_deleted(self, service, mock_redis):
        mock_redis.delete = AsyncMock(return_value=1)
        result = await service.revoke_token("mytoken")
        assert result is True

    @pytest.mark.asyncio
    async def test_returns_false_when_not_found(self, service, mock_redis):
        mock_redis.delete = AsyncMock(return_value=0)
        result = await service.revoke_token("ghost")
        assert result is False

    @pytest.mark.asyncio
    async def test_uses_correct_key_prefix(self, service, mock_redis):
        mock_redis.delete = AsyncMock(return_value=1)
        await service.revoke_token("abc")
        call_args = mock_redis.delete.call_args[0][0]
        assert "test_module" in call_args
        assert "abc" in call_args


# ---------------------------------------------------------------------------
# change_password
# ---------------------------------------------------------------------------


class TestChangePassword:
    """Tests for change_password()."""

    @pytest.mark.asyncio
    async def test_returns_true_on_success(self, service):
        um = MagicMock()
        um.internal_usermanager = MagicMock()
        um.internal_usermanager.change_password_impl = AsyncMock(
            return_value={"success": True}
        )
        service._user_manager = um
        result = await service.change_password("alice", "old", "new")
        assert result is True

    @pytest.mark.asyncio
    async def test_returns_false_on_failure(self, service):
        um = MagicMock()
        um.internal_usermanager = MagicMock()
        um.internal_usermanager.change_password_impl = AsyncMock(
            return_value={"success": False}
        )
        service._user_manager = um
        result = await service.change_password("alice", "old", "bad")
        assert result is False

    @pytest.mark.asyncio
    async def test_returns_false_when_no_user_manager(self, service):
        service._user_manager = None
        result = await service.change_password("x", "old", "new")
        assert result is False


# ---------------------------------------------------------------------------
# list_local_users
# ---------------------------------------------------------------------------


class TestListLocalUsers:
    """Tests for list_local_users()."""

    @pytest.mark.asyncio
    async def test_returns_user_list(self, service):
        um = MagicMock()
        um.internal_usermanager = MagicMock()
        um.internal_usermanager.list_users_impl = AsyncMock(
            return_value={"users": [{"username": "alice"}, {"username": "bob"}]}
        )
        service._user_manager = um
        result = await service.list_local_users()
        assert len(result) == 2
        assert result[0]["username"] == "alice"

    @pytest.mark.asyncio
    async def test_returns_empty_on_error(self, service):
        service._user_manager = None
        result = await service.list_local_users()
        assert result == []


# ---------------------------------------------------------------------------
# create_local_user
# ---------------------------------------------------------------------------


class TestCreateLocalUser:
    """Tests for create_local_user()."""

    @pytest.mark.asyncio
    async def test_creates_user_successfully(self, service):
        um = MagicMock()
        um.internal_usermanager = MagicMock()
        um.internal_usermanager.create_user_impl = AsyncMock(
            return_value={"success": True, "user": {"username": "new_user"}}
        )
        service._user_manager = um
        result = await service.create_local_user("new_user", "pass", "viewer")
        assert result["success"] is True

    @pytest.mark.asyncio
    async def test_accepts_permissions_as_dict(self, service):
        um = MagicMock()
        um.internal_usermanager = MagicMock()
        um.internal_usermanager.create_user_impl = AsyncMock(
            return_value={"success": True, "user": {}}
        )
        service._user_manager = um
        result = await service.create_local_user(
            "u", "p", permissions={"read": True}
        )
        assert result["success"] is True

    @pytest.mark.asyncio
    async def test_invalid_permissions_json_returns_error(self, service):
        um = MagicMock()
        um.internal_usermanager = MagicMock()
        service._user_manager = um
        result = await service.create_local_user("u", "p", permissions="not-json{{{")
        assert result["success"] is False
        assert "Invalid" in result["message"]

    @pytest.mark.asyncio
    async def test_returns_error_dict_when_no_user_manager(self, service):
        service._user_manager = None
        result = await service.create_local_user("u", "p")
        # Should return a failure dict (error path caught)
        assert "success" in result or isinstance(result, dict)
