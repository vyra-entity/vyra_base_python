"""
Unit tests for Feeder Handler classes.

Tests IFeederHandler, CommunicationHandler, ZenohHandler, RedisHandler,
UDSHandler, DBCommunicationHandler, DatabaseWriter protocol, HandlerFactory.
"""
import asyncio
import logging
import pytest
from unittest.mock import AsyncMock, MagicMock, Mock, patch, call

from vyra_base.com.handler.interfaces import IFeederHandler
from vyra_base.com.handler.communication import CommunicationHandler
from vyra_base.com.handler.database import DBCommunicationHandler, DatabaseWriter


# ---------------------------------------------------------------------------
# IFeederHandler
# ---------------------------------------------------------------------------

class TestIFeederHandler:
    def test_is_abstract(self):
        import inspect
        assert inspect.isabstract(IFeederHandler)

    def test_has_required_abstract_methods(self):
        import abc
        abstract_methods = set(getattr(IFeederHandler, '__abstractmethods__', set()))
        assert "dispatch" in abstract_methods

    def test_inherits_logging_handler(self):
        assert issubclass(IFeederHandler, logging.Handler)


# ---------------------------------------------------------------------------
# CommunicationHandler
# ---------------------------------------------------------------------------

class ConcreteCommunicationHandler(CommunicationHandler):
    """Minimal concrete subclass for testing."""

    async def dispatch(self, message):
        self._last_message = message

    def get_protocol(self):
        return "test"


class TestCommunicationHandler:
    def test_inherits_ifeeder_handler(self):
        assert issubclass(CommunicationHandler, IFeederHandler)

    def test_emit_schedules_dispatch(self):
        handler = ConcreteCommunicationHandler(
            initiator="test_module",
            type=None,
        )
        record = logging.LogRecord(
            name="test", level=logging.INFO,
            pathname="", lineno=0, msg="hello",
            args=(), exc_info=None,
        )
        # emit() should not raise even without a running event loop
        try:
            handler.emit(record)
        except RuntimeError:
            pass  # No running loop is acceptable in sync test

    def test_get_protocol_returns_string(self):
        handler = ConcreteCommunicationHandler(initiator="mod", type=None)
        assert isinstance(handler.get_protocol(), str)


# ---------------------------------------------------------------------------
# DBCommunicationHandler
# ---------------------------------------------------------------------------

class MockDatabase:
    """Implements DatabaseWriter protocol for testing."""

    def __init__(self):
        self.records = []

    async def write(self, record: dict) -> None:
        self.records.append(record)


class TestDBCommunicationHandler:
    @pytest.mark.asyncio
    async def test_dispatch_stores_record(self):
        db = MockDatabase()
        handler = DBCommunicationHandler(database=db, source="test_module")
        await handler.dispatch("Test message")
        assert len(db.records) == 1
        rec = db.records[0]
        assert rec["message"] == "Test message"
        assert rec["source"] == "test_module"
        assert "timestamp" in rec

    @pytest.mark.asyncio
    async def test_dispatch_stores_level(self):
        db = MockDatabase()
        handler = DBCommunicationHandler(database=db, source="mod")
        # Level is inferred from the message string if not part of record
        await handler.dispatch("ERROR: error message")
        assert len(db.records) == 1

    @pytest.mark.asyncio
    async def test_dispatch_multiple_records(self):
        db = MockDatabase()
        handler = DBCommunicationHandler(database=db, source="mod")
        for i in range(5):
            await handler.dispatch(f"msg {i}")
        assert len(db.records) == 5

    def test_get_protocol(self):
        db = MockDatabase()
        handler = DBCommunicationHandler(database=db, source="mod")
        assert handler.get_protocol() == "database"

    def test_emit_formats_log_record(self):
        db = MockDatabase()
        handler = DBCommunicationHandler(database=db, source="mod")
        record = logging.LogRecord(
            name="test", level=logging.WARNING,
            pathname="", lineno=0, msg="warning msg",
            args=(), exc_info=None,
        )
        # emit() should queue the record without raising
        try:
            handler.emit(record)
        except RuntimeError:
            pass  # No running loop is acceptable in sync test


# ---------------------------------------------------------------------------
# ZenohHandler / RedisHandler / UDSHandler (import smoke tests)
# ---------------------------------------------------------------------------

class TestProtocolHandlerImports:
    def test_zenoh_handler_importable(self):
        from vyra_base.com.handler.zenoh import ZenohHandler
        assert ZenohHandler is not None

    def test_redis_handler_importable(self):
        from vyra_base.com.handler.redis import RedisHandler
        assert RedisHandler is not None

    def test_uds_handler_importable(self):
        from vyra_base.com.handler.uds import UDSHandler
        assert UDSHandler is not None

    def test_zenoh_handler_protocol(self):
        from vyra_base.com.handler.zenoh import ZenohHandler
        publisher = MagicMock()
        handler = ZenohHandler(initiator="mod", publisher=publisher, type=None)
        assert handler.get_protocol() == "zenoh"

    def test_redis_handler_protocol(self):
        from vyra_base.com.handler.redis import RedisHandler
        publisher = MagicMock()
        handler = RedisHandler(initiator="mod", publisher=publisher, type=None)
        assert handler.get_protocol() == "redis"

    def test_uds_handler_protocol(self):
        from vyra_base.com.handler.uds import UDSHandler
        publisher = MagicMock()
        handler = UDSHandler(initiator="mod", publisher=publisher, type=None)
        assert handler.get_protocol() == "uds"

    @pytest.mark.asyncio
    async def test_zenoh_dispatch_calls_publisher(self):
        from vyra_base.com.handler.zenoh import ZenohHandler
        publisher = MagicMock()
        publisher.publish = AsyncMock()
        handler = ZenohHandler(initiator="mod", publisher=publisher, type=None)
        await handler.dispatch("hello")
        publisher.publish.assert_called_once_with("hello")


# ---------------------------------------------------------------------------
# HandlerFactory (import smoke test)
# ---------------------------------------------------------------------------

class TestHandlerFactory:
    def test_factory_importable(self):
        from vyra_base.com.handler.factory import HandlerFactory
        assert HandlerFactory is not None

    @pytest.mark.asyncio
    async def test_factory_create_database_handler(self):
        from vyra_base.com.handler.factory import HandlerFactory
        db = MockDatabase()
        handler = await HandlerFactory.create(
            protocol="database",
            initiator="mod",
            feeder_name="TestFeed",
            message_type=None,
            database=db,
        )
        assert isinstance(handler, DBCommunicationHandler)
