"""Tests for vyra_base.com.feeder.tracking module."""
import asyncio
import time
from typing import Any
from unittest.mock import MagicMock, AsyncMock

import pytest

from vyra_base.com.feeder.tracking import (
    FeedDebouncer,
    FeedConditionRegistry,
    DebounceHit,
    ConditionRule,
    build_message_signature,
    _severity_to_error_level,
    _to_serializable,
    resolve_entity_from_call,
)
from vyra_base.defaults.entries import ErrorEntry


# ---------------------------------------------------------------------------
# _severity_to_error_level
# ---------------------------------------------------------------------------

class TestSeverityToErrorLevel:
    """Tests for _severity_to_error_level helper."""

    def test_debug_returns_minor(self):
        """DEBUG maps to MINOR_FAULT."""
        level = _severity_to_error_level("debug")
        assert level == ErrorEntry.ERROR_LEVEL.MINOR_FAULT.value

    def test_info_returns_minor(self):
        """INFO maps to MINOR_FAULT."""
        level = _severity_to_error_level("INFO")
        assert level == ErrorEntry.ERROR_LEVEL.MINOR_FAULT.value

    def test_warning_returns_major(self):
        """WARNING maps to MAJOR_FAULT."""
        level = _severity_to_error_level("warning")
        assert level == ErrorEntry.ERROR_LEVEL.MAJOR_FAULT.value

    def test_error_returns_critical(self):
        """ERROR maps to CRITICAL_FAULT."""
        level = _severity_to_error_level("error")
        assert level == ErrorEntry.ERROR_LEVEL.CRITICAL_FAULT.value

    def test_critical_returns_critical(self):
        """CRITICAL maps to CRITICAL_FAULT."""
        level = _severity_to_error_level("critical")
        assert level == ErrorEntry.ERROR_LEVEL.CRITICAL_FAULT.value

    def test_emergency_returns_emergency(self):
        """EMERGENCY maps to EMERGENCY_FAULT."""
        level = _severity_to_error_level("emergency")
        assert level == ErrorEntry.ERROR_LEVEL.EMERGENCY_FAULT.value

    def test_unknown_returns_major(self):
        """Unknown severity defaults to MAJOR_FAULT."""
        level = _severity_to_error_level("unknown_severity")
        assert level == ErrorEntry.ERROR_LEVEL.MAJOR_FAULT.value

    def test_none_returns_major(self):
        """None severity defaults to MAJOR_FAULT."""
        level = _severity_to_error_level(None)
        assert level == ErrorEntry.ERROR_LEVEL.MAJOR_FAULT.value

    def test_empty_string_returns_major(self):
        """Empty string defaults to MAJOR_FAULT."""
        level = _severity_to_error_level("")
        assert level == ErrorEntry.ERROR_LEVEL.MAJOR_FAULT.value

    def test_major_fault_alias(self):
        """'MAJOR' alias maps to MAJOR_FAULT."""
        level = _severity_to_error_level("major")
        assert level == ErrorEntry.ERROR_LEVEL.MAJOR_FAULT.value

    def test_minor_fault_alias(self):
        """'MINOR_FAULT' maps to MINOR_FAULT."""
        level = _severity_to_error_level("MINOR_FAULT")
        assert level == ErrorEntry.ERROR_LEVEL.MINOR_FAULT.value


# ---------------------------------------------------------------------------
# _to_serializable
# ---------------------------------------------------------------------------

class TestToSerializable:
    """Tests for _to_serializable helper."""

    def test_primitive_str(self):
        """String passes through unchanged."""
        assert _to_serializable("hello") == "hello"

    def test_primitive_int(self):
        """Int passes through unchanged."""
        assert _to_serializable(42) == 42

    def test_primitive_float(self):
        """Float passes through unchanged."""
        assert _to_serializable(3.14) == 3.14

    def test_primitive_bool(self):
        """Bool passes through unchanged."""
        assert _to_serializable(True) is True

    def test_none(self):
        """None passes through."""
        assert _to_serializable(None) is None

    def test_dict(self):
        """Dict is recursively serialized."""
        result = _to_serializable({"a": 1, "b": "two"})
        assert result == {"a": 1, "b": "two"}

    def test_list(self):
        """List elements are recursively serialized."""
        result = _to_serializable([1, "two", None])
        assert result == [1, "two", None]

    def test_tuple(self):
        """Tuple is converted to list."""
        result = _to_serializable((1, 2))
        assert result == [1, 2]

    def test_set(self):
        """Set is converted to list."""
        result = _to_serializable({42})
        assert 42 in result

    def test_dataclass(self):
        """Dataclass is converted to dict."""
        from dataclasses import dataclass

        @dataclass
        class Point:
            x: int
            y: int

        result = _to_serializable(Point(x=1, y=2))
        assert result == {"x": 1, "y": 2}

    def test_object_with_dict(self):
        """Object with __dict__ is serialized via vars()."""
        class Obj:
            def __init__(self):
                self.val = "test"

        result = _to_serializable(Obj())
        assert result["val"] == "test"

    def test_unknown_type_uses_repr(self):
        """Objects without dict/slots fall back to repr."""

        class Weird:
            __slots__ = ()

        result = _to_serializable(Weird())
        # repr is a string
        assert isinstance(result, str)

    def test_nested_dict(self):
        """Nested dicts are recursively processed."""
        data = {"outer": {"inner": [1, 2]}}
        result = _to_serializable(data)
        assert result["outer"]["inner"] == [1, 2]


# ---------------------------------------------------------------------------
# build_message_signature
# ---------------------------------------------------------------------------

class TestBuildMessageSignature:
    """Tests for build_message_signature."""

    def test_returns_hex_string(self):
        """Returns a 64-char hex string (SHA256)."""
        sig = build_message_signature({"key": "val"})
        assert isinstance(sig, str)
        assert len(sig) == 64

    def test_same_message_same_signature(self):
        """Same message produces same signature."""
        msg = {"a": 1, "b": [1, 2]}
        sig1 = build_message_signature(msg)
        sig2 = build_message_signature(msg)
        assert sig1 == sig2

    def test_different_messages_different_signatures(self):
        """Different messages produce different signatures."""
        sig1 = build_message_signature({"a": 1})
        sig2 = build_message_signature({"a": 2})
        assert sig1 != sig2

    def test_timestamp_key_ignored(self):
        """'timestamp' key is stripped before hashing."""
        msg1 = {"value": 42, "timestamp": "2024-01-01T00:00:00"}
        msg2 = {"value": 42, "timestamp": "2024-12-31T23:59:59"}
        sig1 = build_message_signature(msg1)
        sig2 = build_message_signature(msg2)
        assert sig1 == sig2

    def test_uuid_key_ignored(self):
        """'uuid' key is stripped before hashing."""
        msg1 = {"data": "x", "uuid": "aaa-bbb"}
        msg2 = {"data": "x", "uuid": "ccc-ddd"}
        assert build_message_signature(msg1) == build_message_signature(msg2)

    def test_string_message(self):
        """Works on non-dict messages."""
        sig = build_message_signature("hello")
        assert isinstance(sig, str)
        assert len(sig) == 64

    def test_order_independent(self):
        """Dict with same keys in different insertion order gives same signature."""
        msg1 = {"b": 2, "a": 1}
        msg2 = {"a": 1, "b": 2}
        assert build_message_signature(msg1) == build_message_signature(msg2)


# ---------------------------------------------------------------------------
# FeedDebouncer
# ---------------------------------------------------------------------------

class TestFeedDebouncer:
    """Tests for FeedDebouncer."""

    def test_first_call_allows_publish(self):
        """First call for a signature allows publishing."""
        d = FeedDebouncer(window_seconds=5.0)
        result = d.evaluate("sig-A")
        assert result.should_publish is True
        assert result.duplicate_count == 0

    def test_immediate_repeat_suppressed(self):
        """Second call within window is suppressed."""
        d = FeedDebouncer(window_seconds=5.0)
        d.evaluate("sig-A")
        result = d.evaluate("sig-A")
        assert result.should_publish is False
        assert result.duplicate_count == 1

    def test_duplicate_count_increments(self):
        """duplicate_count increments on each suppressed call."""
        d = FeedDebouncer(window_seconds=5.0)
        d.evaluate("sig-A")
        d.evaluate("sig-A")
        result = d.evaluate("sig-A")
        assert result.duplicate_count == 2

    def test_after_window_allows_publish(self):
        """After window expires, signature is allowed again."""
        d = FeedDebouncer(window_seconds=0.01)
        d.evaluate("sig-B")
        time.sleep(0.05)
        result = d.evaluate("sig-B")
        assert result.should_publish is True

    def test_different_signatures_independent(self):
        """Different signatures are tracked independently."""
        d = FeedDebouncer(window_seconds=5.0)
        r1 = d.evaluate("sig-X")
        r2 = d.evaluate("sig-Y")
        assert r1.should_publish is True
        assert r2.should_publish is True

    def test_custom_window(self):
        """Custom window_seconds is respected."""
        d = FeedDebouncer(window_seconds=0.05)
        d.evaluate("sig-C")
        time.sleep(0.1)
        result = d.evaluate("sig-C")
        assert result.should_publish is True


# ---------------------------------------------------------------------------
# FeedConditionRegistry
# ---------------------------------------------------------------------------

class TestFeedConditionRegistry:
    """Tests for FeedConditionRegistry."""

    def _reg(self):
        return FeedConditionRegistry()

    def test_register_function(self):
        """register returns rule name."""
        reg = self._reg()
        name = reg.register(lambda ctx: True, name="my_rule", tag="info")
        assert name == "my_rule"

    def test_register_uses_function_name_if_no_name(self):
        """register uses function __name__ if name is not provided."""
        reg = self._reg()

        def my_condition(ctx):
            return True

        name = reg.register(my_condition, tag="info")
        assert name == "my_condition"

    def test_register_coroutine_raises(self):
        """Coroutine functions are rejected."""
        reg = self._reg()

        async def bad_condition(ctx):
            return True

        with pytest.raises(TypeError):
            reg.register(bad_condition, tag="info")

    def test_register_invalid_execution_point_raises(self):
        """Invalid execution_point raises ValueError."""
        reg = self._reg()
        with pytest.raises(ValueError):
            reg.register(lambda ctx: True, tag="info", execution_point="INVALID")

    def test_unregister_existing(self):
        """unregister returns True for existing rule."""
        reg = self._reg()
        reg.register(lambda ctx: True, name="r1", tag="info")
        assert reg.unregister("r1") is True

    def test_unregister_missing_returns_false(self):
        """unregister returns False if rule not found."""
        reg = self._reg()
        assert reg.unregister("nonexistent") is False

    def test_evaluate_calls_conditions(self):
        """evaluate calls registered condition function."""
        reg = self._reg()
        calls = []
        reg.register(lambda ctx: calls.append(ctx) or True, name="r1", tag="test")
        reg.evaluate({"x": 1})
        assert len(calls) == 1

    def test_evaluate_success_message(self):
        """Passing condition with success_message returns it."""
        reg = self._reg()
        reg.register(
            lambda ctx: True,
            name="r1",
            tag="info",
            success_message="It passed!",
        )
        outputs = reg.evaluate({})
        assert ("info", "It passed!") in outputs

    def test_evaluate_failure_message(self):
        """Failing condition with failure_message returns it."""
        reg = self._reg()
        reg.register(
            lambda ctx: False,
            name="r1",
            tag="warn",
            failure_message="It failed!",
        )
        outputs = reg.evaluate({})
        assert ("warn", "It failed!") in outputs

    def test_evaluate_no_messages_returns_empty(self):
        """Condition without messages produces no outputs."""
        reg = self._reg()
        reg.register(lambda ctx: True, name="r1", tag="info")
        outputs = reg.evaluate({})
        assert outputs == []

    def test_evaluate_filter_by_rule_names(self):
        """rule_names filter restricts which rules are evaluated."""
        reg = self._reg()
        calls = []
        reg.register(lambda ctx: calls.append("r1") or True, name="r1", tag="a")
        reg.register(lambda ctx: calls.append("r2") or True, name="r2", tag="b")
        reg.evaluate({}, rule_names=["r1"])
        assert "r1" in calls
        assert "r2" not in calls

    def test_evaluate_filter_by_tags(self):
        """tags filter restricts which rules are evaluated."""
        reg = self._reg()
        calls = []
        reg.register(lambda ctx: calls.append("ta") or True, name="ra", tag="alpha")
        reg.register(lambda ctx: calls.append("tb") or True, name="rb", tag="beta")
        reg.evaluate({}, tags=["alpha"])
        assert "ta" in calls
        assert "tb" not in calls

    def test_evaluate_filter_by_execution_point(self):
        """execution_point filter restricts evaluation."""
        reg = self._reg()
        calls = []
        reg.register(
            lambda ctx: calls.append("before") or True,
            name="r_before",
            tag="t",
            execution_point="BEFORE",
        )
        reg.register(
            lambda ctx: calls.append("after") or True,
            name="r_after",
            tag="t",
            execution_point="AFTER",
        )
        reg.evaluate({}, execution_point="BEFORE")
        assert "before" in calls
        assert "after" not in calls

    def test_evaluate_always_rules_included(self):
        """ALWAYS rules run regardless of execution_point filter."""
        reg = self._reg()
        calls = []
        reg.register(
            lambda ctx: calls.append("always") or True,
            name="r_always",
            tag="t",
            execution_point="ALWAYS",
        )
        reg.evaluate({}, execution_point="BEFORE")
        assert "always" in calls

    def test_evaluate_invalid_execution_point_raises(self):
        """Passing invalid execution_point to evaluate raises ValueError."""
        reg = self._reg()
        with pytest.raises(ValueError):
            reg.evaluate({}, execution_point="INVALID")

    def test_evaluate_condition_returning_non_bool_raises(self):
        """Condition returning non-bool raises TypeError (caught as warning)."""
        reg = self._reg()
        reg.register(lambda ctx: "yes", name="bad_r", tag="t")  # type: ignore
        # Should not raise — exception is caught and logged as warning
        outputs = reg.evaluate({})
        assert outputs == []

    def test_evaluate_condition_exception_does_not_propagate(self):
        """Condition raising exception is caught and logged, not propagated."""
        reg = self._reg()

        def broken_cond(ctx):
            raise RuntimeError("broken")

        reg.register(broken_cond, name="broken", tag="t")
        outputs = reg.evaluate({})
        assert outputs == []


# ---------------------------------------------------------------------------
# resolve_entity_from_call
# ---------------------------------------------------------------------------

class TestResolveEntityFromCall:
    """Tests for resolve_entity_from_call helper."""

    def test_explicit_entity_returned_if_provided(self):
        """explicit_entity takes priority."""
        entity = MagicMock()
        result = resolve_entity_from_call(args=(), kwargs={}, explicit_entity=entity)
        assert result is entity

    def test_entity_from_kwargs(self):
        """entity kwarg is returned if explicit is None."""
        entity = MagicMock()
        result = resolve_entity_from_call(args=(), kwargs={"entity": entity})
        assert result is entity

    def test_entity_from_first_arg_with_error_feeder(self):
        """Arg with error_feeder/news_feeder attribute is returned as entity."""
        entity = MagicMock(spec=["error_feeder"])
        result = resolve_entity_from_call(args=(entity, "other"), kwargs={})
        assert result is entity

    def test_entity_from_first_arg_entity_attribute(self):
        """Arg with .entity attribute returns that attribute."""
        inner_entity = MagicMock()
        outer = MagicMock()
        outer.entity = inner_entity
        result = resolve_entity_from_call(args=(outer,), kwargs={})
        assert result is inner_entity

    def test_none_if_no_args(self):
        """Returns None if no args and no explicit entity."""
        result = resolve_entity_from_call(args=(), kwargs={})
        assert result is None
