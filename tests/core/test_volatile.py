"""
Unit tests for vyra_base.core.volatile.Volatile.

Redis calls and VyraPublishers are mocked so no live services are needed.
The ErrorTraceback decorator wraps some methods — those are tested via their
public signature (no internal decorator is broken).
"""

import asyncio
import pytest
from unittest.mock import AsyncMock, MagicMock, patch


# ---------------------------------------------------------------------------
# Helpers / Factories
# ---------------------------------------------------------------------------


def _make_redis_mock() -> MagicMock:
    """Return a minimal async-capable RedisClient mock."""
    redis = MagicMock()
    redis.get = AsyncMock(return_value=None)
    redis.set = AsyncMock()
    redis.delete = AsyncMock(return_value=1)
    redis.get_active_listeners = AsyncMock(return_value={"active_channels": []})
    redis.create_pubsub_listener = AsyncMock()
    redis.remove_listener_channels = AsyncMock()
    return redis


def _make_transient_types() -> dict:
    """Dummy type map — Volatile stores these in REDIS_TYPE_MAP."""
    return {
        "VolatileString": MagicMock(name="VolatileString"),
        "VolatileHash": MagicMock(name="VolatileHash"),
        "VolatileList": MagicMock(name="VolatileList"),
        "VolatileSet": MagicMock(name="VolatileSet"),
    }


def _make_volatile():
    """Instantiate Volatile with all dependencies mocked."""
    from vyra_base.core.volatile import Volatile

    return Volatile(
        storage_access_transient=_make_redis_mock(),
        module_name="test_mod",
        module_id="abc123",
        node=None,
        transient_base_types=_make_transient_types(),
    )


# ---------------------------------------------------------------------------
# Construction
# ---------------------------------------------------------------------------


class TestVolatileInit:
    """Verify that Volatile initialises correctly."""

    def test_module_name_stored(self):
        v = _make_volatile()
        assert v.module_name == "test_mod"

    def test_module_id_stored(self):
        v = _make_volatile()
        assert v.module_id == "abc123"

    def test_volatile_prefix_contains_module_info(self):
        v = _make_volatile()
        assert "test_mod" in v._volatile_prefix
        assert "abc123" in v._volatile_prefix

    def test_active_shouter_initially_empty(self):
        v = _make_volatile()
        assert v._active_shouter == {}

    def test_listener_initially_none(self):
        v = _make_volatile()
        assert v._listener is None

    def test_redis_type_map_populated(self):
        v = _make_volatile()
        from vyra_base.com.transport.t_redis import REDIS_TYPE
        assert REDIS_TYPE.STRING in v.REDIS_TYPE_MAP
        assert REDIS_TYPE.HASH in v.REDIS_TYPE_MAP
        assert REDIS_TYPE.LIST in v.REDIS_TYPE_MAP
        assert REDIS_TYPE.SET in v.REDIS_TYPE_MAP


# ---------------------------------------------------------------------------
# Key helpers
# ---------------------------------------------------------------------------


class TestQualifiedKey:
    """Tests for _qualified_key() and _external_key()."""

    def test_qualified_key_prefixes_plain_key(self):
        v = _make_volatile()
        qk = v._qualified_key("status")
        assert qk.startswith(v._volatile_prefix)
        assert qk.endswith("status")

    def test_qualified_key_does_not_double_prefix(self):
        v = _make_volatile()
        prefixed = v._volatile_prefix + "status"
        qk = v._qualified_key(prefixed)
        # Should not add prefix twice
        assert qk.count(v._volatile_prefix) == 1

    def test_external_key_strips_prefix(self):
        v = _make_volatile()
        full_key = v._volatile_prefix + "temperature"
        ext = v._external_key(full_key)
        assert ext == "temperature"

    def test_external_key_plain_key_unchanged(self):
        v = _make_volatile()
        ext = v._external_key("mykey")
        assert ext == "mykey"


# ---------------------------------------------------------------------------
# cleanup
# ---------------------------------------------------------------------------


class TestCleanup:
    """Tests for the async cleanup() method."""

    @pytest.mark.asyncio
    async def test_cleanup_cancels_listener(self):
        v = _make_volatile()
        mock_task = MagicMock()
        mock_task.cancel = MagicMock()
        v._listener = mock_task

        await v.cleanup()

        mock_task.cancel.assert_called_once()
        assert v._listener is None

    @pytest.mark.asyncio
    async def test_cleanup_clears_shouters(self):
        v = _make_volatile()
        pub = AsyncMock()
        pub.shutdown = AsyncMock()
        v._active_shouter["chan1"] = pub

        await v.cleanup()

        pub.shutdown.assert_awaited_once()
        assert v._active_shouter == {}

    @pytest.mark.asyncio
    async def test_cleanup_idempotent_when_no_listener(self):
        v = _make_volatile()
        # Should not raise
        await v.cleanup()
        assert v._listener is None


# ---------------------------------------------------------------------------
# deactivate_listener
# ---------------------------------------------------------------------------


class TestDeactivateListener:
    """Tests for deactivate_listener()."""

    @pytest.mark.asyncio
    async def test_deactivates_single_channel(self):
        v = _make_volatile()
        await v.deactivate_listener("my_channel")
        v.redis.remove_listener_channels.assert_awaited_once()

    @pytest.mark.asyncio
    async def test_deactivates_list_of_channels(self):
        v = _make_volatile()
        await v.deactivate_listener(["ch1", "ch2"])
        call_args = v.redis.remove_listener_channels.call_args[1]
        assert "ch1" in call_args.get("channels", [])


# ---------------------------------------------------------------------------
# __del__
# ---------------------------------------------------------------------------


class TestDel:
    """Tests for __del__ destructor."""

    def test_del_cancels_listener(self):
        v = _make_volatile()
        mock_task = MagicMock()
        mock_task.cancel = MagicMock()
        v._listener = mock_task

        v.__del__()

        mock_task.cancel.assert_called_once()
        assert v._listener is None

    def test_del_clears_shouters(self):
        v = _make_volatile()
        v._active_shouter["x"] = MagicMock()
        v.__del__()
        assert v._active_shouter == {}
