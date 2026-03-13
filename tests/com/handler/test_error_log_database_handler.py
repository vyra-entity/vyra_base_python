"""
Unit tests for ErrorLogDatabaseHandler.

Tests:
- FieldSpec validation logic
- Ring-buffer ID rotation
- dispatch() with valid ErrorEntry-like dict
- activated=False suppresses dispatch
- _dispatch_to_handlers() in BaseFeeder: active vs inactive handlers
- configure() activates a deactivated handler
"""
from __future__ import annotations

import asyncio
import pytest
from datetime import datetime
from unittest.mock import AsyncMock, MagicMock, patch, call
from typing import Any

from vyra_base.com.handler.error_log_database import ErrorLogDatabaseHandler, FieldSpec
from vyra_base.storage.tb_error_log import ErrorLog, ERROR_LOG_MAX_ROWS


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

def _make_handler(activated: bool = True, max_rows: int = 10) -> ErrorLogDatabaseHandler:
    """Build an ErrorLogDatabaseHandler with a mocked DbManipulator."""
    handler = ErrorLogDatabaseHandler(
        database=None,  # will inject manipulator manually
        model=ErrorLog,
        field_definitions=ErrorLogDatabaseHandler.default_error_log_fields(),
        max_rows=max_rows,
        activated=activated,
        source="test_module",
    )
    mock_manipulator = MagicMock()
    mock_manipulator.get_by_id = AsyncMock(return_value=MagicMock(status="not_found", value=None))
    mock_manipulator.add = AsyncMock(return_value=MagicMock(status="success"))
    mock_manipulator.update = AsyncMock(return_value=MagicMock(status="success"))
    handler._manipulator = mock_manipulator
    if activated:
        handler.activated = True
    return handler


def _sample_error_dict() -> dict:
    return {
        "timestamp": datetime(2026, 3, 13, 12, 0, 0),
        "code": 42,
        "level": 1,
        "description": "Test error",
        "miscellaneous": {"sensor": "temperature", "value": 99.5},
        "error_id": "test-uuid-1234",
    }


# ---------------------------------------------------------------------------
# FieldSpec validation
# ---------------------------------------------------------------------------

class TestFieldSpec:
    """Tests for _validate() logic in ErrorLogDatabaseHandler."""

    def test_required_field_missing_raises(self):
        handler = _make_handler()
        record = {
            # "occured_at" missing → required
            "severity": 1,
            "message": "test",
        }
        with pytest.raises(ValueError, match="occured_at"):
            handler._validate(record)

    def test_nonnullable_required_field_as_none_raises(self):
        handler = _make_handler()
        record = {
            "occured_at": None,   # required + non-nullable
            "severity": 1,
            "message": "test",
        }
        with pytest.raises(ValueError, match="non-nullable"):
            handler._validate(record)

    def test_nullable_optional_field_as_none_passes(self):
        handler = _make_handler()
        record = {
            "occured_at": datetime.utcnow(),
            "error_code": None,        # optional + nullable → OK
            "severity": 2,
            "context_snap": None,      # optional + nullable → OK
            "message": "ok",
            "acknowledged": None,      # optional + nullable → OK
        }
        validated = handler._validate(record)
        assert validated["error_code"] is None

    def test_type_coercion_int_float(self):
        handler = _make_handler()
        record = {
            "occured_at": datetime.utcnow(),
            "severity": 2.0,    # float instead of int → coerced
            "message": "coercion test",
        }
        validated = handler._validate(record)
        assert isinstance(validated["severity"], int)

    def test_wrong_type_raises_type_error(self):
        handler = _make_handler()
        record = {
            "occured_at": "not-a-datetime",  # str instead of datetime → cannot coerce
            "severity": 1,
            "message": "bad type",
        }
        with pytest.raises((TypeError, ValueError)):
            handler._validate(record)


# ---------------------------------------------------------------------------
# Ring-buffer ID rotation
# ---------------------------------------------------------------------------

class TestRingBufferRotation:
    """Slot IDs should cycle through 1 … max_rows."""

    def test_first_slot_is_one(self):
        handler = _make_handler(max_rows=10)
        assert handler._next_slot_id() == 1

    def test_slots_are_sequential(self):
        handler = _make_handler(max_rows=10)
        slots = [handler._next_slot_id() for _ in range(10)]
        assert slots == list(range(1, 11))

    def test_slot_wraps_at_max_rows(self):
        handler = _make_handler(max_rows=5)
        for _ in range(5):
            handler._next_slot_id()
        assert handler._next_slot_id() == 1  # wraps back to 1

    def test_counter_increments_continuously(self):
        handler = _make_handler(max_rows=5)
        handler._next_slot_id()  # counter=0 → slot 1, counter becomes 1
        assert handler._counter == 1

    def test_full_rotation_sequence(self):
        max_rows = 10
        handler = _make_handler(max_rows=max_rows)
        slots = [handler._next_slot_id() for _ in range(max_rows * 2 + 3)]
        # Every slot must be in [1, max_rows]
        assert all(1 <= s <= max_rows for s in slots)
        # After max_rows the pattern repeats
        assert slots[0] == slots[max_rows] == slots[max_rows * 2]


# ---------------------------------------------------------------------------
# dispatch() behaviour
# ---------------------------------------------------------------------------

class TestDispatch:
    """Tests for the full dispatch path."""

    @pytest.mark.asyncio
    async def test_dispatch_calls_add_when_slot_empty(self):
        handler = _make_handler()
        await handler.dispatch(_sample_error_dict())
        handler._manipulator.add.assert_called_once()

    @pytest.mark.asyncio
    async def test_dispatch_calls_update_when_slot_exists(self):
        handler = _make_handler()
        # Simulate existing row at slot 1
        handler._manipulator.get_by_id = AsyncMock(
            return_value=MagicMock(status="success", value=MagicMock())
        )
        await handler.dispatch(_sample_error_dict())
        handler._manipulator.update.assert_called_once()
        handler._manipulator.add.assert_not_called()

    @pytest.mark.asyncio
    async def test_dispatch_inactive_does_nothing(self):
        handler = _make_handler(activated=False)
        await handler.dispatch(_sample_error_dict())
        handler._manipulator.add.assert_not_called()
        handler._manipulator.update.assert_not_called()

    @pytest.mark.asyncio
    async def test_dispatch_without_manipulator_logs_warning(self):
        from unittest.mock import patch
        handler = _make_handler()
        handler._manipulator = None
        handler.activated = True
        with patch("vyra_base.com.handler.error_log_database.logger") as mock_logger:
            await handler.dispatch(_sample_error_dict())
        mock_logger.warning.assert_called_once()
        assert "no DbManipulator" in mock_logger.warning.call_args[0][0]

    @pytest.mark.asyncio
    async def test_dispatch_maps_error_entry_fields(self):
        handler = _make_handler()
        captured: list[dict] = []

        async def fake_add(data: dict):
            captured.append(data)
            return MagicMock(status="success")

        handler._manipulator.add = fake_add
        raw = {
            "timestamp": datetime(2026, 1, 1),
            "code": 7,
            "level": 2,
            "description": "Overheating",
            "miscellaneous": {"sensor": "temp"},
        }
        await handler.dispatch(raw)

        assert len(captured) == 1
        rec = captured[0]
        assert rec["occured_at"] == datetime(2026, 1, 1)
        assert rec["error_code"] == 7
        assert rec["severity"] == 2
        assert rec["message"] == "Overheating"
        assert rec["context_snap"] == {"sensor": "temp"}

    @pytest.mark.asyncio
    async def test_dispatch_maps_error_id(self):
        handler = _make_handler()
        captured: list[dict] = []

        async def fake_add(data: dict):
            captured.append(data)
            return MagicMock(status="success")

        handler._manipulator.add = fake_add
        raw = {
            "timestamp": datetime(2026, 1, 1),
            "code": 0,
            "level": 0,
            "description": "uuid test",
            "error_id": "abc-123-uuid",
        }
        await handler.dispatch(raw)
        assert captured[0]["error_id"] == "abc-123-uuid"

    @pytest.mark.asyncio
    async def test_dispatch_maps_uuid_as_error_id(self):
        """uuid field in the raw dict should be mapped to error_id when error_id is absent."""
        handler = _make_handler()
        captured: list[dict] = []

        async def fake_add(data: dict):
            captured.append(data)
            return MagicMock(status="success")

        handler._manipulator.add = fake_add
        import uuid as _uuid
        sample_uuid = _uuid.uuid4()
        raw = {
            "timestamp": datetime(2026, 1, 1),
            "code": 0,
            "level": 0,
            "description": "uuid fallback test",
            "uuid": sample_uuid,
        }
        await handler.dispatch(raw)
        assert captured[0]["error_id"] == str(sample_uuid)

    @pytest.mark.asyncio
    async def test_dispatch_sets_slot_id(self):
        handler = _make_handler(max_rows=10)
        captured: list[dict] = []

        async def fake_add(data: dict):
            captured.append(data)
            return MagicMock(status="success")

        handler._manipulator.add = fake_add
        await handler.dispatch(_sample_error_dict())
        assert captured[0]["id"] == 1


# ---------------------------------------------------------------------------
# configure() activates deactivated handler
# ---------------------------------------------------------------------------

class TestConfigure:
    def test_configure_activates_handler(self):
        handler = ErrorLogDatabaseHandler(
            database=None,
            model=ErrorLog,
            field_definitions=ErrorLogDatabaseHandler.default_error_log_fields(),
            activated=False,
        )
        assert handler.activated is False
        assert handler._manipulator is None

        mock_db = MagicMock()
        mock_db.session = MagicMock()

        with patch("vyra_base.com.handler.error_log_database.DbManipulator") as MockManip:
            MockManip.return_value = MagicMock()
            handler.configure(mock_db)

        assert handler.activated is True
        assert handler._manipulator is not None


# ---------------------------------------------------------------------------
# BaseFeeder._dispatch_to_handlers integration
# ---------------------------------------------------------------------------

class TestBaseFeederDispatchToHandlers:
    """Ensure BaseFeeder._dispatch_to_handlers respects the activated flag."""

    def _make_feeder(self):
        import sys
        from unittest.mock import MagicMock
        # Patch the ROS2 import so we can import BaseFeeder
        with patch.dict("sys.modules", {"rclpy": None, "rclpy.qos": None}):
            from vyra_base.com.feeder.feeder import BaseFeeder
        feeder = BaseFeeder.__new__(BaseFeeder)
        feeder._feederName = "TestFeeder"
        feeder._handler = []
        return feeder

    def test_active_handler_is_dispatched(self):
        from vyra_base.com.feeder.feeder import BaseFeeder
        feeder = BaseFeeder.__new__(BaseFeeder)
        feeder._feederName = "TestFeeder"
        feeder._handler = []

        mock_handler = MagicMock()
        mock_handler.activated = True
        mock_handler.dispatch = AsyncMock()
        feeder._handler.append(mock_handler)

        loop = asyncio.new_event_loop()
        try:
            asyncio.set_event_loop(loop)
            feeder._dispatch_to_handlers({"test": "data"})
            # run a tick to execute the task
            loop.run_until_complete(asyncio.sleep(0))
        finally:
            loop.close()
            asyncio.set_event_loop(None)

        mock_handler.dispatch.assert_called_once_with({"test": "data"})

    def test_inactive_handler_is_skipped(self):
        from vyra_base.com.feeder.feeder import BaseFeeder
        feeder = BaseFeeder.__new__(BaseFeeder)
        feeder._feederName = "TestFeeder"
        feeder._handler = []

        mock_handler = MagicMock()
        mock_handler.activated = False
        mock_handler.dispatch = AsyncMock()
        feeder._handler.append(mock_handler)

        loop = asyncio.new_event_loop()
        try:
            asyncio.set_event_loop(loop)
            feeder._dispatch_to_handlers({"test": "data"})
            loop.run_until_complete(asyncio.sleep(0))
        finally:
            loop.close()
            asyncio.set_event_loop(None)

        mock_handler.dispatch.assert_not_called()

    def test_empty_handler_list_does_nothing(self):
        from vyra_base.com.feeder.feeder import BaseFeeder
        feeder = BaseFeeder.__new__(BaseFeeder)
        feeder._feederName = "TestFeeder"
        feeder._handler = []
        # Should not raise
        feeder._dispatch_to_handlers({"test": "data"})


# ---------------------------------------------------------------------------
# IFeederHandler activated flag
# ---------------------------------------------------------------------------

class TestIFeederHandlerActivated:
    """Tests for the activated/deactivate/activate on IFeederHandler."""

    def _concrete_handler(self) -> ErrorLogDatabaseHandler:
        return ErrorLogDatabaseHandler(
            database=None,
            model=ErrorLog,
            field_definitions=ErrorLogDatabaseHandler.default_error_log_fields(),
        )

    def test_default_activated_false_when_no_db(self):
        h = self._concrete_handler()
        assert h.activated is False

    def test_activate_sets_true(self):
        h = self._concrete_handler()
        h.activate()
        assert h.activated is True

    def test_deactivate_sets_false(self):
        h = self._concrete_handler()
        h.activate()
        h.deactivate()
        assert h.activated is False

    def test_emit_skipped_when_deactivated(self):
        h = self._concrete_handler()
        h.activated = False
        h.dispatch = AsyncMock()
        import logging
        record = logging.LogRecord("n", logging.ERROR, "", 0, "msg", (), None)
        h.emit(record)
        h.dispatch.assert_not_called()
