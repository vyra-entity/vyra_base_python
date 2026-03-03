"""
Unit tests for Feeder Components.

Tests IFeeder, BaseFeeder, CustomBaseFeeder, FeederRegistry,
FeederConfigResolver, StateFeeder, NewsFeeder, ErrorFeeder.
"""
import asyncio
import json
import pytest
import tempfile
import os
from pathlib import Path
from unittest.mock import AsyncMock, MagicMock, Mock, patch

from vyra_base.com.feeder.interfaces import IFeeder
from vyra_base.com.feeder.custom_feeder import CustomBaseFeeder
from vyra_base.com.feeder.config_resolver import FeederConfigResolver, FeederResolverResult
from vyra_base.com.feeder.registry import FeederRegistry, register_feeder
from vyra_base.com.feeder.feeder import BaseFeeder
from vyra_base.com.feeder import feed_tracker
from vyra_base.com.feeder.tracking import FeedConditionRegistry, FeedTracker


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def sample_interface_config():
    """Write a minimal interface config JSON to a temp file and return the path."""
    data = [
        {
            "type": "publisher",
            "functionname": "TemperatureFeed",
            "name": "temperature_feed",
            "tags": ["zenoh"],
            "filetype": ["VBASETemperatureFeed.proto"],
            "description": "Temperature publisher",
        },
        {
            "type": "publisher",
            "functionname": "StateFeed",
            "name": "state_feed",
            "tags": ["zenoh"],
            "filetype": ["VBASEStateFeed.proto"],
            "description": "State publisher",
        },
    ]
    with tempfile.NamedTemporaryFile(
        mode="w", suffix=".json", delete=False
    ) as f:
        json.dump(data, f)
        path = f.name
    yield path
    os.unlink(path)


@pytest.fixture(autouse=True)
def clean_registry():
    """Clear the FeederRegistry before each test."""
    FeederRegistry._instance.clear()
    yield
    FeederRegistry._instance.clear()


# ---------------------------------------------------------------------------
# IFeeder
# ---------------------------------------------------------------------------

class TestIFeeder:
    def test_is_abstract(self):
        """IFeeder cannot be instantiated directly."""
        import inspect
        assert inspect.isabstract(IFeeder)

    def test_has_required_methods(self):
        for method in ("start", "feed", "get_feeder_name"):
            assert hasattr(IFeeder, method), f"IFeeder missing '{method}'"


# ---------------------------------------------------------------------------
# FeederConfigResolver
# ---------------------------------------------------------------------------

class TestFeederConfigResolver:
    def test_resolve_known_name(self, sample_interface_config):
        resolver = FeederConfigResolver()
        result = resolver.resolve("TemperatureFeed", [sample_interface_config])
        assert result is not None
        assert isinstance(result, FeederResolverResult)
        assert result.feeder_name == "TemperatureFeed"
        assert result.protocol == "zenoh"

    def test_resolve_unknown_name_returns_none(self, sample_interface_config, caplog):
        resolver = FeederConfigResolver()
        result = resolver.resolve("CompletelyUnknownFeed", [sample_interface_config])
        assert result is None

    def test_resolve_fuzzy_suggestion_in_log(self, sample_interface_config, caplog):
        import logging
        # Capture all loggers so propagation issues don't hide the message
        with caplog.at_level(logging.ERROR):
            import logging as _lg
            _lg.getLogger("vyra_base.com.feeder.config_resolver").propagate = True
            resolver = FeederConfigResolver()
            resolver.resolve("TemratureFeed", [sample_interface_config])
        # The error is written to stderr by default; also check caplog text
        # (fuzzy matches are logged at ERROR level with "Did you mean")
        combined = caplog.text + "".join(
            r.getMessage() for r in caplog.records
        )
        assert "TemperatureFeed" in combined, (
            "Fuzzy suggestion for close match should appear in error log. "
            f"caplog.records={caplog.records}"
        )

    def test_resolve_case_insensitive(self, sample_interface_config):
        resolver = FeederConfigResolver()
        result = resolver.resolve("temperaturefeed", [sample_interface_config],
                                  case_sensitive=False)
        assert result is not None

    def test_resolve_no_paths_returns_none(self):
        resolver = FeederConfigResolver()
        result = resolver.resolve("StateFeed", [])
        assert result is None


# ---------------------------------------------------------------------------
# FeederRegistry
# ---------------------------------------------------------------------------

class TestFeederRegistry:
    def test_register_and_get(self):
        class DummyFeeder(CustomBaseFeeder):
            async def start(self): pass
            def feed(self, _): pass
            def get_feeder_name(self): return "Dummy"

        FeederRegistry.register("DummyFeed", DummyFeeder)
        assert FeederRegistry.get("DummyFeed") is DummyFeeder

    def test_get_unregistered_returns_none(self):
        assert FeederRegistry.get("NoSuchFeed") is None

    def test_list_feeders(self):
        class A(CustomBaseFeeder):
            async def start(self): pass
            def feed(self, _): pass
            def get_feeder_name(self): return "A"

        FeederRegistry.register("AFeed", A)
        assert "AFeed" in FeederRegistry.list_feeders()

    def test_unregister(self):
        class B(CustomBaseFeeder):
            async def start(self): pass
            def feed(self, _): pass
            def get_feeder_name(self): return "B"

        FeederRegistry.register("BFeed", B)
        FeederRegistry.unregister("BFeed")
        assert FeederRegistry.get("BFeed") is None

    def test_register_feeder_decorator(self):
        @register_feeder("DecoratedFeed")
        class DecoratedFeeder(CustomBaseFeeder):
            async def start(self): pass
            def feed(self, _): pass
            def get_feeder_name(self): return "Decorated"

        assert FeederRegistry.get("DecoratedFeed") is DecoratedFeeder


# ---------------------------------------------------------------------------
# CustomBaseFeeder
# ---------------------------------------------------------------------------

class ConcreteFeeder(CustomBaseFeeder):
    """Minimal concrete feeder for testing."""

    def _build_message(self, raw):
        return {"value": raw, "unit": "°C"}

    def _validate(self, raw):
        return isinstance(raw, (int, float)) and -50 <= raw <= 300


class TestCustomBaseFeeder:
    def _make_feeder(self):
        entity = MagicMock()
        entity.name = "test_module"
        entity.uuid = "aaaa-bbbb"
        return ConcreteFeeder(
            feeder_name="TemperatureFeed",
            module_entity=entity,
        )

    def test_validate_passes_valid_value(self):
        feeder = self._make_feeder()
        assert feeder._validate(25.0) is True

    def test_validate_rejects_out_of_range(self):
        feeder = self._make_feeder()
        assert feeder._validate(999.0) is False

    def test_build_message(self):
        feeder = self._make_feeder()
        msg = feeder._build_message(100.0)
        assert msg == {"value": 100.0, "unit": "°C"}

    def test_feed_skips_invalid(self):
        feeder = self._make_feeder()
        feeder._is_ready = False
        feeder.feed_sync(-999.0)  # invalid — validation should reject
        assert len(feeder.get_buffer()) == 0

    def test_feed_valid_buffers_when_not_ready(self):
        feeder = self._make_feeder()
        feeder._is_ready = False
        feeder.feed_sync(42.0)  # valid — should buffer transformed message
        assert len(feeder.get_buffer()) == 1

    def test_get_feeder_name(self):
        feeder = self._make_feeder()
        assert feeder.get_feeder_name() == "TemperatureFeed"

    def test_metrics_initial_values(self):
        feeder = self._make_feeder()
        assert feeder.feed_count == 0
        assert feeder.error_count == 0

    def test_is_alive_false_before_start(self):
        feeder = self._make_feeder()
        assert feeder.is_alive() is False


# ---------------------------------------------------------------------------
# StateFeeder / NewsFeeder / ErrorFeeder minimal smoke tests
# ---------------------------------------------------------------------------

class TestBuiltinFeeders:
    def _make_builtin(self, klass):
        """Instantiate a builtin feeder with all required mocks."""
        entity = MagicMock()
        entity.name = "test_module"
        entity.uuid = "test-uuid"
        return klass(type=None, node=None, module_entity=entity)

    def test_state_feeder_name(self):
        from vyra_base.com.feeder.state_feeder import StateFeeder
        feeder = self._make_builtin(StateFeeder)
        assert feeder.get_feeder_name() == "StateFeed"

    def test_news_feeder_name(self):
        from vyra_base.com.feeder.news_feeder import NewsFeeder
        feeder = self._make_builtin(NewsFeeder)
        assert feeder.get_feeder_name() == "NewsFeed"

    def test_error_feeder_name(self):
        from vyra_base.com.feeder.error_feeder import ErrorFeeder
        feeder = self._make_builtin(ErrorFeeder)
        assert feeder.get_feeder_name() == "ErrorFeed"


class TestFeederTracking:
    @pytest.mark.asyncio
    async def test_basefeeder_debounce_suppresses_duplicate(self):
        feeder = BaseFeeder()
        feeder._is_ready = False

        await feeder.feed({"message": "same-message"})
        await feeder.feed({"message": "same-message"})

        assert len(feeder.get_buffer()) == 1
        assert feeder.debounced_duplicate_count == 1

    def test_condition_registry_returns_messages(self):
        feeder = BaseFeeder()
        feeder.register_condition(
            lambda context: context.get("ok", False),
            name="ok_condition",
            tag="news",
            success_message="ok",
            failure_message="not ok",
        )

        assert feeder.evaluate_conditions({"ok": True}) == [("news", "ok")]
        assert feeder.evaluate_conditions({"ok": False}) == [("news", "not ok")]

    def test_condition_registry_can_filter_by_rule_name(self):
        feeder = BaseFeeder()
        feeder.register_condition(
            lambda context: context.get("a", False),
            name="rule_a",
            tag="news",
            success_message="A",
        )
        feeder.register_condition(
            lambda context: context.get("b", False),
            name="rule_b",
            tag="custom",
            success_message="B",
        )

        outputs = feeder.evaluate_conditions(
            {"a": True, "b": True},
            rule_names=["rule_b"],
        )
        assert outputs == [("custom", "B")]

    def test_condition_registry_can_filter_by_tag(self):
        feeder = BaseFeeder()
        feeder.register_condition(
            lambda context: context.get("a", False),
            name="rule_a",
            tag="news",
            success_message="A",
        )
        feeder.register_condition(
            lambda context: context.get("b", False),
            name="rule_b",
            tag="custom",
            success_message="B",
        )

        outputs = feeder.evaluate_conditions(
            {"a": True, "b": True},
            tags=["news"],
        )
        assert outputs == [("news", "A")]

    def test_condition_registry_filters_by_execution_point(self):
        feeder = BaseFeeder()
        feeder.register_condition(
            lambda context: context.get("ok", False),
            name="before_rule",
            tag="news",
            execution_point="BEFORE",
            success_message="before",
        )
        feeder.register_condition(
            lambda context: context.get("ok", False),
            name="always_rule",
            tag="news",
            execution_point="ALWAYS",
            success_message="always",
        )

        outputs = feeder.evaluate_conditions(
            {"ok": True},
            execution_point="AFTER",
        )
        assert outputs == [("news", "always")]

    def test_condition_registry_rejects_async_callback(self):
        feeder = BaseFeeder()

        async def async_condition(_):
            return True

        with pytest.raises(TypeError):
            feeder.register_condition(async_condition, name="invalid")

    @pytest.mark.asyncio
    async def test_feed_tracker_lazy_entity_lookup(self):
        class DummyErrorFeeder:
            def __init__(self):
                self.received = []

            async def feed(self, payload):
                self.received.append(payload)

            def evaluate_conditions(self, _context):
                return []

        class DummyEntity:
            def __init__(self):
                self.error_feeder = DummyErrorFeeder()
                self.news_feeder = MagicMock()

        class DummyComponent:
            def __init__(self):
                self.entity = DummyEntity()

            @feed_tracker.monitor(tag="error", label="lazy lookup", severity="WARNING")
            async def boom(self):
                raise RuntimeError("boom")

        component = DummyComponent()
        with pytest.raises(RuntimeError):
            await component.boom()

        assert len(component.entity.error_feeder.received) == 1
        assert "RuntimeError: boom" in component.entity.error_feeder.received[0]["description"]

    def test_feed_tracker_news_during_execution_point(self):
        class ConditionCarrier:
            def __init__(self):
                self._conditions = FeedConditionRegistry()

            def register_condition(self, *args, **kwargs):
                return self._conditions.register(*args, **kwargs)

            def evaluate_conditions(self, context, **kwargs):
                return self._conditions.evaluate(context, **kwargs)

        class DummyNewsSink:
            def __init__(self):
                self.messages = []

            def feed_sync(self, message):
                self.messages.append(message)

        class DummyEntity:
            def __init__(self):
                self.news_feeder = DummyNewsSink()

        class DummyComponent:
            def __init__(self):
                self.entity = DummyEntity()
                self.a = 0
                self.monitor_source = ConditionCarrier()
                self.monitor_source.register_condition(
                    lambda _context: self.a > 0,
                    name="during_a_positive",
                    tag="news",
                    execution_point="DURING",
                    success_message="a is positive during execution",
                )

        component = DummyComponent()

        decorator = FeedTracker(component.monitor_source).monitor(
            tag="news",
            during_interval_seconds=0.01,
        )

        @decorator
        def run_step(self):
            import time
            time.sleep(0.03)
            self.a = 3
            time.sleep(0.04)

        run_step(component)
        assert "a is positive during execution" in component.entity.news_feeder.messages
