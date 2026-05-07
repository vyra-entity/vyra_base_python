"""
Tests for vyra_base.com.feeder.feeder.BaseFeeder

Covers: init, properties, is_alive, is_ready, feed buffering, feed_sync,
feed dedup, add_handler, add_handler_class, _publish, retry logic,
_flush_buffer, _dispatch_to_handlers, register/unregister_condition,
evaluate_conditions, set_interface_paths, _prepare_entry_for_publish.
"""
from __future__ import annotations

import asyncio
import datetime
from collections import deque
from typing import Any
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from vyra_base.com.feeder.feeder import BaseFeeder
from vyra_base.com.handler.communication import CommunicationHandler
from vyra_base.defaults.exceptions import FeederException
from vyra_base.com.feeder.tracking import ExecutionPoint


# ---------------------------------------------------------------------------
# Helpers / Fixtures
# ---------------------------------------------------------------------------

class ConcreteFeeder(BaseFeeder):
    """Minimal concrete subclass for testing."""

    def __init__(self, name: str = "test_feeder") -> None:
        super().__init__()
        self._feederName = name


def _make_mock_publisher(connected: bool = True, has_protocol: bool = False) -> MagicMock:
    pub = MagicMock()
    pub.publish = AsyncMock()
    if connected:
        pub.is_connected = MagicMock(return_value=True)
    if has_protocol:
        pub.protocol = "zenoh"
    return pub


class _FakeHandler(CommunicationHandler):
    """Minimal CommunicationHandler subclass for testing."""

    def __init__(self):
        self.activated = True
        self.dispatched: list = []

    async def dispatch(self, data: Any) -> None:
        self.dispatched.append(data)


# ---------------------------------------------------------------------------
# TestBaseFeederInit
# ---------------------------------------------------------------------------

class TestBaseFeederInit:
    def test_default_values(self):
        f = ConcreteFeeder()
        assert f._feederName == "test_feeder"
        assert f._is_ready is False
        assert f._feed_count == 0
        assert f._error_count == 0
        assert f._last_feed_at is None
        assert f._publisher is None
        assert isinstance(f._feedbuffer, deque)
        assert f._max_retries == 3
        assert f._retry_delay == 1.0
        assert f._interface_paths == []
        assert f._resolved_protocol is None

    def test_feed_count_property(self):
        f = ConcreteFeeder()
        assert f.feed_count == 0

    def test_error_count_property(self):
        f = ConcreteFeeder()
        assert f.error_count == 0

    def test_last_feed_at_property(self):
        f = ConcreteFeeder()
        assert f.last_feed_at is None

    def test_debounced_duplicate_count_property(self):
        f = ConcreteFeeder()
        assert f.debounced_duplicate_count == 0

    def test_get_feeder_name(self):
        f = ConcreteFeeder("my_feeder")
        assert f.get_feeder_name() == "my_feeder"

    def test_get_protocol_before_start(self):
        f = ConcreteFeeder()
        assert f.get_protocol() is None

    def test_get_buffer_returns_deque(self):
        f = ConcreteFeeder()
        assert isinstance(f.get_buffer(), deque)

    def test_is_ready_false_initially(self):
        f = ConcreteFeeder()
        assert f.is_ready() is False


# ---------------------------------------------------------------------------
# TestIsAlive
# ---------------------------------------------------------------------------

class TestIsAlive:
    def test_no_publisher_returns_false(self):
        f = ConcreteFeeder()
        assert f.is_alive() is False

    def test_publisher_with_is_connected_true(self):
        f = ConcreteFeeder()
        f._publisher = _make_mock_publisher(connected=True)
        assert f.is_alive() is True

    def test_publisher_with_is_connected_false(self):
        f = ConcreteFeeder()
        pub = MagicMock()
        pub.is_connected = MagicMock(return_value=False)
        f._publisher = pub
        assert f.is_alive() is False

    def test_publisher_without_is_connected_returns_true(self):
        f = ConcreteFeeder()
        pub = MagicMock(spec=[])  # no is_connected
        f._publisher = pub
        assert f.is_alive() is True


# ---------------------------------------------------------------------------
# TestSetInterfacePaths
# ---------------------------------------------------------------------------

class TestSetInterfacePaths:
    def test_set_list_of_strings(self):
        f = ConcreteFeeder()
        f.set_interface_paths(["/a", "/b"])
        assert f._interface_paths == ["/a", "/b"]

    def test_set_path_objects(self):
        from pathlib import Path
        f = ConcreteFeeder()
        f.set_interface_paths([Path("/tmp/iface.json")])
        assert f._interface_paths == ["/tmp/iface.json"]

    def test_empty_list(self):
        f = ConcreteFeeder()
        f.set_interface_paths([])
        assert f._interface_paths == []


# ---------------------------------------------------------------------------
# TestFeedBuffering
# ---------------------------------------------------------------------------

class TestFeedBuffering:
    @pytest.mark.asyncio
    async def test_feed_before_start_buffers_message(self):
        f = ConcreteFeeder()
        assert not f._is_ready
        await f.feed({"val": 1})
        assert len(f._feedbuffer) == 1

    @pytest.mark.asyncio
    async def test_buffer_maxlen(self):
        f = ConcreteFeeder()
        for i in range(25):
            await f.feed({"val": i})
        assert len(f._feedbuffer) <= 20

    @pytest.mark.asyncio
    async def test_feed_when_ready_calls_publish(self):
        f = ConcreteFeeder()
        f._is_ready = True
        pub = _make_mock_publisher()
        f._publisher = pub
        await f.feed({"val": 42})
        pub.publish.assert_called_once()

    @pytest.mark.asyncio
    async def test_feed_without_publisher_increments_error_count(self):
        f = ConcreteFeeder()
        f._is_ready = True
        f._publisher = None
        await f.feed({"val": 1})
        assert f._error_count == 1

    @pytest.mark.asyncio
    async def test_feed_exception_increments_error_count(self):
        f = ConcreteFeeder()
        f._is_ready = True
        pub = MagicMock()
        pub.publish = AsyncMock(side_effect=RuntimeError("oops"))
        f._publisher = pub
        await f.feed({"val": 1})
        assert f._error_count == 1

    @pytest.mark.asyncio
    async def test_feed_logging_on(self, caplog):
        import logging
        f = ConcreteFeeder()
        f._is_ready = True
        f._loggingOn = True
        pub = _make_mock_publisher()
        f._publisher = pub
        with caplog.at_level(logging.INFO):
            await f.feed({"val": 1})
        assert f._feed_count == 1


# ---------------------------------------------------------------------------
# TestFeedSync
# ---------------------------------------------------------------------------

class TestFeedSync:
    def test_feed_sync_buffers_when_not_ready(self):
        f = ConcreteFeeder()
        f.feed_sync({"x": 1})
        assert len(f._feedbuffer) == 1

    def test_feed_sync_logs_no_publisher(self):
        f = ConcreteFeeder()
        f._is_ready = True
        f._publisher = None
        f.feed_sync({"x": 1})
        assert f._error_count == 1


# ---------------------------------------------------------------------------
# TestPublish
# ---------------------------------------------------------------------------

class TestPublish:
    @pytest.mark.asyncio
    async def test_publish_increments_feed_count(self):
        f = ConcreteFeeder()
        f._publisher = _make_mock_publisher()
        f._resolved_protocol = "zenoh"
        await f._publish({"key": "val"})
        assert f._feed_count == 1
        assert isinstance(f._last_feed_at, datetime.datetime)

    @pytest.mark.asyncio
    async def test_publish_retries_on_failure(self):
        f = ConcreteFeeder()
        f._max_retries = 3
        f._retry_delay = 0.0  # instant
        pub = MagicMock()
        pub.publish = AsyncMock(side_effect=RuntimeError("fail"))
        f._publisher = pub
        # _publish catches all retries and increments error_count, does not reraise
        await f._publish({"k": "v"})
        assert pub.publish.call_count == 3
        assert f._error_count == 1

    @pytest.mark.asyncio
    async def test_publish_no_publisher_raises(self):
        f = ConcreteFeeder()
        f._publisher = None
        with pytest.raises(FeederException):
            await f._publish({"k": "v"})

    @pytest.mark.asyncio
    async def test_publish_publisher_without_publish_method_increments_error(self):
        f = ConcreteFeeder()
        # Object with no publish() method — _publish will catch FeederException internally
        class NoPub:
            pass
        f._publisher = NoPub()
        # _publish catches the error (no publish() method) and increments error_count
        await f._publish({"k": "v"})
        assert f._error_count == 1

    @pytest.mark.asyncio
    async def test_publish_calls_dispatch_on_handlers(self):
        f = ConcreteFeeder()
        f._publisher = _make_mock_publisher()
        handler = _FakeHandler()
        f._handler.append(handler)
        await f._publish({"k": "v"})
        # dispatch is scheduled as asyncio task, give it a cycle
        await asyncio.sleep(0)
        assert len(handler.dispatched) == 1


# ---------------------------------------------------------------------------
# TestFlushBuffer
# ---------------------------------------------------------------------------

class TestFlushBuffer:
    @pytest.mark.asyncio
    async def test_flush_empty_buffer_does_nothing(self):
        f = ConcreteFeeder()
        f._publisher = _make_mock_publisher()
        f._is_ready = True
        await f._flush_buffer()
        assert f._feed_count == 0

    @pytest.mark.asyncio
    async def test_flush_publishes_buffered_messages(self):
        f = ConcreteFeeder()
        f._publisher = _make_mock_publisher()
        f._feedbuffer.appendleft({"a": 1})
        f._feedbuffer.appendleft({"b": 2})
        await f._flush_buffer()
        assert f._publisher.publish.call_count == 2

    @pytest.mark.asyncio
    async def test_flush_handles_publish_error_gracefully(self):
        f = ConcreteFeeder()
        pub = MagicMock()
        pub.publish = AsyncMock(side_effect=RuntimeError("bad"))
        f._publisher = pub
        f._max_retries = 1
        f._retry_delay = 0.0
        f._feedbuffer.appendleft({"a": 1})
        # Should not raise
        await f._flush_buffer()


# ---------------------------------------------------------------------------
# TestAddHandler
# ---------------------------------------------------------------------------

class TestAddHandler:
    def test_add_handler_returns_true(self):
        f = ConcreteFeeder()
        h = _FakeHandler()
        assert f.add_handler(h) is True
        assert h in f._handler

    def test_add_duplicate_handler_returns_false(self):
        f = ConcreteFeeder()
        h = _FakeHandler()
        f.add_handler(h)
        assert f.add_handler(h) is False

    def test_add_invalid_handler_raises(self):
        f = ConcreteFeeder()
        with pytest.raises(TypeError):
            f.add_handler("not_a_handler")  # type: ignore

    def test_add_handler_class(self):
        f = ConcreteFeeder()
        f.add_handler_class(_FakeHandler)
        assert _FakeHandler in f._handler_classes

    def test_add_handler_class_duplicate_ignored(self):
        f = ConcreteFeeder()
        f.add_handler_class(_FakeHandler)
        f.add_handler_class(_FakeHandler)
        assert f._handler_classes.count(_FakeHandler) == 1

    def test_add_invalid_handler_class_raises(self):
        f = ConcreteFeeder()
        with pytest.raises(TypeError):
            f.add_handler_class(object)  # type: ignore


# ---------------------------------------------------------------------------
# TestDispatchToHandlers
# ---------------------------------------------------------------------------

class TestDispatchToHandlers:
    @pytest.mark.asyncio
    async def test_dispatch_skips_inactive_handler(self):
        f = ConcreteFeeder()
        h = _FakeHandler()
        h.activated = False
        f._handler.append(h)
        f._publisher = _make_mock_publisher()
        await f._publish({"k": "v"})
        await asyncio.sleep(0)
        assert len(h.dispatched) == 0

    @pytest.mark.asyncio
    async def test_no_handlers_no_error(self):
        f = ConcreteFeeder()
        f._publisher = _make_mock_publisher()
        # No handlers, should not raise
        await f._publish({"k": "v"})


# ---------------------------------------------------------------------------
# TestConditions
# ---------------------------------------------------------------------------

class TestConditions:
    def test_register_condition_returns_name(self):
        f = ConcreteFeeder()
        name = f.register_condition(lambda ctx: True, name="my_cond")
        assert isinstance(name, str)
        assert "my_cond" in name

    def test_unregister_condition(self):
        f = ConcreteFeeder()
        name = f.register_condition(lambda ctx: True, name="c1")
        result = f.unregister_condition(name)
        assert result is True

    def test_unregister_nonexistent_condition(self):
        f = ConcreteFeeder()
        result = f.unregister_condition("does_not_exist")
        assert result is False

    def test_evaluate_conditions_all_pass(self):
        f = ConcreteFeeder()
        f.register_condition(
            lambda ctx: True,
            name="ok_cond",
            success_message="all good",
        )
        results = f.evaluate_conditions({"data": 1})
        assert isinstance(results, list)

    def test_evaluate_conditions_filter_by_tag(self):
        f = ConcreteFeeder()
        f.register_condition(lambda ctx: True, name="tagged", tag="alert")
        results = f.evaluate_conditions({}, tags=["alert"])
        assert isinstance(results, list)


# ---------------------------------------------------------------------------
# TestPrepareEntry
# ---------------------------------------------------------------------------

class TestPrepareEntry:
    def test_default_prepare_returns_unchanged(self):
        f = ConcreteFeeder()
        data = {"x": 1, "y": 2}
        assert f._prepare_entry_for_publish(data) is data

    def test_subclass_can_override(self):
        class MappingFeeder(BaseFeeder):
            def _prepare_entry_for_publish(self, entry):
                return {"mapped": True}

        f = MappingFeeder()
        result = f._prepare_entry_for_publish({"raw": 1})
        assert result == {"mapped": True}


# ---------------------------------------------------------------------------
# TestCreate (mocked InterfaceFactory)
# ---------------------------------------------------------------------------

class TestCreate:
    @pytest.mark.asyncio
    async def test_create_sets_publisher_and_ready(self):
        f = ConcreteFeeder()
        mock_pub = _make_mock_publisher()
        with patch(
            "vyra_base.com.feeder.feeder.InterfaceFactory.create_publisher",
            new=AsyncMock(return_value=mock_pub),
        ):
            with patch.object(f, "_load_msg_type", new=AsyncMock(return_value=None)):
                await f.create()
        assert f._publisher is mock_pub
        assert f._is_ready is True

    @pytest.mark.asyncio
    async def test_create_raises_feeder_exception_when_publisher_none(self):
        f = ConcreteFeeder()
        with patch(
            "vyra_base.com.feeder.feeder.InterfaceFactory.create_publisher",
            new=AsyncMock(return_value=None),
        ):
            with patch.object(f, "_load_msg_type", new=AsyncMock(return_value=None)):
                with pytest.raises(FeederException):
                    await f.create()

    @pytest.mark.asyncio
    async def test_create_with_resolved_protocol_from_publisher(self):
        f = ConcreteFeeder()
        mock_pub = _make_mock_publisher(has_protocol=True)
        with patch(
            "vyra_base.com.feeder.feeder.InterfaceFactory.create_publisher",
            new=AsyncMock(return_value=mock_pub),
        ):
            with patch.object(f, "_load_msg_type", new=AsyncMock(return_value=None)):
                await f.create()
        assert f._resolved_protocol == "zenoh"

    @pytest.mark.asyncio
    async def test_start_calls_create(self):
        f = ConcreteFeeder()
        with patch.object(f, "create", new=AsyncMock()) as mock_create:
            await f.start()
        mock_create.assert_called_once()

    @pytest.mark.asyncio
    async def test_create_fallback_on_primary_failure(self):
        f = ConcreteFeeder()
        mock_pub = _make_mock_publisher()
        call_count = {"n": 0}

        async def side_effect(**kwargs):
            call_count["n"] += 1
            if call_count["n"] == 1:
                raise RuntimeError("primary fail")
            return mock_pub

        with patch(
            "vyra_base.com.feeder.feeder.InterfaceFactory.create_publisher",
            new=side_effect,
        ):
            with patch.object(f, "_load_msg_type", new=AsyncMock(return_value=None)):
                await f.create()
        assert f._is_ready is True
        assert call_count["n"] == 2

    @pytest.mark.asyncio
    async def test_create_raises_when_both_attempts_fail(self):
        f = ConcreteFeeder()
        with patch(
            "vyra_base.com.feeder.feeder.InterfaceFactory.create_publisher",
            new=AsyncMock(side_effect=RuntimeError("fail")),
        ):
            with pytest.raises(FeederException):
                await f.create()
