"""
Tests for vyra_base.com.core.callback_registry.CallbackRegistry

Covers: initialize, clear, register_blueprint, get_blueprint, bind_callback,
unbind_callback, list_all, list_unbound, list_bound, get_statistics,
remove_blueprint, exists, _find_full_name, debug_print.
"""
from __future__ import annotations

import pytest

from vyra_base.com.core.callback_registry import CallbackRegistry
from vyra_base.com.core.blueprints import (
    ServiceBlueprint,
    PublisherBlueprint,
    SubscriberBlueprint,
    ActionBlueprint,
    InterfaceType,
)
from vyra_base.com.core.types import ProtocolType


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture(autouse=True)
def clear_registry():
    """Save registry state before test, restore after to avoid cross-test pollution."""
    saved = dict(CallbackRegistry._blueprints)
    CallbackRegistry.clear()
    yield
    CallbackRegistry.clear()
    CallbackRegistry._blueprints.update(saved)


def _make_service(name: str = "svc") -> ServiceBlueprint:
    return ServiceBlueprint(name=name, protocols=[ProtocolType.ZENOH])


def _make_publisher(name: str = "pub") -> PublisherBlueprint:
    return PublisherBlueprint(name=name, protocols=[ProtocolType.ZENOH])


def _make_subscriber(name: str = "sub") -> SubscriberBlueprint:
    return SubscriberBlueprint(name=name, protocols=[ProtocolType.ZENOH])


def _make_action(name: str = "act") -> ActionBlueprint:
    return ActionBlueprint(name=name, protocols=[ProtocolType.ZENOH])


# ---------------------------------------------------------------------------
# TestInitializeClear
# ---------------------------------------------------------------------------

class TestInitializeClear:
    def test_initialize_is_idempotent(self):
        CallbackRegistry.initialize()
        CallbackRegistry.initialize()  # second call should not raise
        assert CallbackRegistry._initialized is True

    def test_clear_removes_all(self):
        CallbackRegistry.register_blueprint(_make_service("a"))
        CallbackRegistry.register_blueprint(_make_service("b"))
        CallbackRegistry.clear()
        assert CallbackRegistry.list_all() == []

    def test_clear_resets_count(self):
        for i in range(5):
            CallbackRegistry.register_blueprint(_make_service(f"s{i}"))
        assert len(CallbackRegistry.list_all()) == 5
        CallbackRegistry.clear()
        assert len(CallbackRegistry.list_all()) == 0


# ---------------------------------------------------------------------------
# TestRegisterBlueprint
# ---------------------------------------------------------------------------

class TestRegisterBlueprint:
    def test_register_returns_name(self):
        bp = _make_service("calculate")
        name = CallbackRegistry.register_blueprint(bp)
        assert name == "calculate"

    def test_register_with_namespace(self):
        bp = _make_service("calculate")
        name = CallbackRegistry.register_blueprint(bp, namespace="v2_mm")
        assert name == "v2_mm/calculate"

    def test_register_with_leading_slash_namespace(self):
        bp = _make_service("init")
        name = CallbackRegistry.register_blueprint(bp, namespace="/v2_mm/")
        assert name == "v2_mm/init"

    def test_register_duplicate_raises(self):
        CallbackRegistry.register_blueprint(_make_service("dup"))
        with pytest.raises(ValueError, match="already registered"):
            CallbackRegistry.register_blueprint(_make_service("dup"))

    def test_register_with_override(self):
        CallbackRegistry.register_blueprint(_make_service("dup"))
        # override=True should not raise
        CallbackRegistry.register_blueprint(_make_service("dup"), override=True)
        assert CallbackRegistry.exists("dup")

    def test_register_multiple_types(self):
        CallbackRegistry.register_blueprint(_make_service("svc"))
        CallbackRegistry.register_blueprint(_make_publisher("pub"))
        CallbackRegistry.register_blueprint(_make_subscriber("sub"))
        CallbackRegistry.register_blueprint(_make_action("act"))
        assert len(CallbackRegistry.list_all()) == 4


# ---------------------------------------------------------------------------
# TestGetBlueprint
# ---------------------------------------------------------------------------

class TestGetBlueprint:
    def test_get_by_name(self):
        bp = _make_service("my_svc")
        CallbackRegistry.register_blueprint(bp)
        result = CallbackRegistry.get_blueprint("my_svc")
        assert result is bp

    def test_get_with_namespace(self):
        bp = _make_service("my_svc")
        CallbackRegistry.register_blueprint(bp, namespace="ns")
        result = CallbackRegistry.get_blueprint("my_svc", namespace="ns")
        assert result is bp

    def test_get_nonexistent_returns_none(self):
        assert CallbackRegistry.get_blueprint("nonexistent") is None

    def test_get_by_full_qualified_name(self):
        bp = _make_service("my_svc")
        CallbackRegistry.register_blueprint(bp, namespace="ns")
        result = CallbackRegistry.get_blueprint("ns/my_svc")
        assert result is bp


# ---------------------------------------------------------------------------
# TestBindCallback
# ---------------------------------------------------------------------------

class TestBindCallback:
    def test_bind_callback_returns_true(self):
        bp = _make_service("svc")
        CallbackRegistry.register_blueprint(bp)
        result = CallbackRegistry.bind_callback("svc", lambda req, resp=None: {})
        assert result is True
        assert bp.is_bound()

    def test_bind_callback_not_found_returns_false(self):
        result = CallbackRegistry.bind_callback("missing", lambda: None)
        assert result is False

    def test_bind_action_callback_with_type(self):
        bp = _make_action("process")
        CallbackRegistry.register_blueprint(bp)
        async def execute(gh): ...
        result = CallbackRegistry.bind_callback("process", execute, callback_type="execute")
        assert result is True

    def test_bind_action_with_default_type_uses_bind_callback(self):
        bp = _make_action("process")
        CallbackRegistry.register_blueprint(bp)
        # callback_type='default' on ActionBlueprint falls through to regular bind_callback
        async def execute(gh): ...
        result = CallbackRegistry.bind_callback("process", execute, callback_type="default")
        assert result is True

    def test_bind_with_namespace(self):
        bp = _make_service("svc")
        CallbackRegistry.register_blueprint(bp, namespace="ns")
        result = CallbackRegistry.bind_callback("svc", lambda req, resp=None: {}, namespace="ns")
        assert result is True

    def test_bind_duplicate_raises(self):
        bp = _make_service("svc")
        CallbackRegistry.register_blueprint(bp)
        cb = lambda req, resp=None: {}
        CallbackRegistry.bind_callback("svc", cb)
        with pytest.raises(RuntimeError):
            CallbackRegistry.bind_callback("svc", lambda req, resp=None: {})


# ---------------------------------------------------------------------------
# TestUnbindCallback
# ---------------------------------------------------------------------------

class TestUnbindCallback:
    def test_unbind_returns_true_when_bound(self):
        bp = _make_service("svc")
        CallbackRegistry.register_blueprint(bp)
        CallbackRegistry.bind_callback("svc", lambda req, resp=None: {})
        result = CallbackRegistry.unbind_callback("svc")
        assert result is True
        assert not bp.is_bound()

    def test_unbind_not_found_returns_false(self):
        result = CallbackRegistry.unbind_callback("nonexistent")
        assert result is False

    def test_unbind_unbound_returns_false(self):
        bp = _make_service("svc")
        CallbackRegistry.register_blueprint(bp)
        # Not bound yet — unbind returns False
        result = CallbackRegistry.unbind_callback("svc")
        assert result is False


# ---------------------------------------------------------------------------
# TestListAll
# ---------------------------------------------------------------------------

class TestListAll:
    def test_list_all_empty(self):
        assert CallbackRegistry.list_all() == []

    def test_list_all_sorted(self):
        CallbackRegistry.register_blueprint(_make_service("z_svc"))
        CallbackRegistry.register_blueprint(_make_service("a_svc"))
        names = CallbackRegistry.list_all()
        assert names == sorted(names)

    def test_list_all_filter_namespace(self):
        CallbackRegistry.register_blueprint(_make_service("s1"), namespace="ns1")
        CallbackRegistry.register_blueprint(_make_service("s2"), namespace="ns2")
        names = CallbackRegistry.list_all(namespace="ns1")
        assert all(n.startswith("ns1/") for n in names)
        assert len(names) == 1

    def test_list_all_filter_type(self):
        CallbackRegistry.register_blueprint(_make_service("svc"))
        CallbackRegistry.register_blueprint(_make_publisher("pub"))
        names = CallbackRegistry.list_all(interface_type=InterfaceType.SERVICE)
        assert "svc" in names
        assert "pub" not in names


# ---------------------------------------------------------------------------
# TestListUnboundBound
# ---------------------------------------------------------------------------

class TestListUnboundBound:
    def test_list_unbound(self):
        CallbackRegistry.register_blueprint(_make_service("unbound"))
        unbound = CallbackRegistry.list_unbound()
        assert "unbound" in unbound

    def test_list_bound(self):
        bp = _make_service("bound_svc")
        CallbackRegistry.register_blueprint(bp)
        CallbackRegistry.bind_callback("bound_svc", lambda req, resp=None: {})
        bound = CallbackRegistry.list_bound()
        assert "bound_svc" in bound

    def test_unbound_excludes_bound(self):
        bp = _make_service("svc")
        CallbackRegistry.register_blueprint(bp)
        CallbackRegistry.bind_callback("svc", lambda req, resp=None: {})
        assert "svc" not in CallbackRegistry.list_unbound()

    def test_list_unbound_with_namespace(self):
        CallbackRegistry.register_blueprint(_make_service("s"), namespace="ns")
        unbound = CallbackRegistry.list_unbound(namespace="ns")
        assert "ns/s" in unbound


# ---------------------------------------------------------------------------
# TestGetStatistics
# ---------------------------------------------------------------------------

class TestGetStatistics:
    def test_empty_stats(self):
        stats = CallbackRegistry.get_statistics()
        assert stats["total"] == 0
        assert stats["bound"] == 0
        assert stats["unbound"] == 0

    def test_stats_with_blueprints(self):
        CallbackRegistry.register_blueprint(_make_service("s1"))
        bp = _make_service("s2")
        CallbackRegistry.register_blueprint(bp)
        CallbackRegistry.bind_callback("s2", lambda req, resp=None: {})
        stats = CallbackRegistry.get_statistics()
        assert stats["total"] == 2
        assert stats["bound"] == 1
        assert stats["unbound"] == 1

    def test_stats_contains_type_counts(self):
        CallbackRegistry.register_blueprint(_make_publisher("pub"))
        stats = CallbackRegistry.get_statistics()
        assert "publishers" in stats
        assert stats["publishers"] == 1

    def test_stats_with_namespace_filter(self):
        CallbackRegistry.register_blueprint(_make_service("s"), namespace="ns")
        CallbackRegistry.register_blueprint(_make_service("t"))
        stats = CallbackRegistry.get_statistics(namespace="ns")
        assert stats["total"] == 1


# ---------------------------------------------------------------------------
# TestRemoveBlueprintExists
# ---------------------------------------------------------------------------

class TestRemoveBlueprintExists:
    def test_remove_existing(self):
        CallbackRegistry.register_blueprint(_make_service("svc"))
        result = CallbackRegistry.remove_blueprint("svc")
        assert result is True
        assert not CallbackRegistry.exists("svc")

    def test_remove_with_namespace(self):
        CallbackRegistry.register_blueprint(_make_service("svc"), namespace="ns")
        result = CallbackRegistry.remove_blueprint("svc", namespace="ns")
        assert result is True

    def test_remove_nonexistent_returns_false(self):
        result = CallbackRegistry.remove_blueprint("nonexistent")
        assert result is False

    def test_exists_true(self):
        CallbackRegistry.register_blueprint(_make_service("svc"))
        assert CallbackRegistry.exists("svc") is True

    def test_exists_false(self):
        assert CallbackRegistry.exists("not_there") is False

    def test_exists_with_namespace(self):
        CallbackRegistry.register_blueprint(_make_service("svc"), namespace="ns")
        assert CallbackRegistry.exists("svc", namespace="ns") is True
        assert CallbackRegistry.exists("svc") is False


# ---------------------------------------------------------------------------
# TestDebugPrint
# ---------------------------------------------------------------------------

class TestDebugPrint:
    def test_debug_print_runs_without_error(self, capsys):
        CallbackRegistry.register_blueprint(_make_service("svc"))
        CallbackRegistry.register_blueprint(_make_action("act"))
        CallbackRegistry.debug_print()
        captured = capsys.readouterr()
        assert "CALLBACK REGISTRY" in captured.out

    def test_debug_print_empty_registry(self, capsys):
        CallbackRegistry.debug_print()
        captured = capsys.readouterr()
        assert "none" in captured.out.lower() or "CALLBACK REGISTRY" in captured.out

    def test_debug_print_with_namespace_filter(self, capsys):
        CallbackRegistry.register_blueprint(_make_service("svc"), namespace="ns")
        CallbackRegistry.debug_print(namespace="ns")
        captured = capsys.readouterr()
        assert "CALLBACK REGISTRY" in captured.out


# ---------------------------------------------------------------------------
# TestFindFullName
# ---------------------------------------------------------------------------

class TestFindFullName:
    def test_find_full_name_registered(self):
        bp = _make_service("svc")
        CallbackRegistry.register_blueprint(bp, namespace="ns")
        full = CallbackRegistry._find_full_name(bp)
        assert full == "ns/svc"

    def test_find_full_name_unregistered_falls_back(self):
        bp = _make_service("orphan")
        # Not registered
        full = CallbackRegistry._find_full_name(bp)
        assert full == "orphan"


# ---------------------------------------------------------------------------
# TestThreadSafety
# ---------------------------------------------------------------------------

class TestThreadSafety:
    def test_concurrent_register(self):
        import threading
        errors = []

        def register(i):
            try:
                CallbackRegistry.register_blueprint(_make_service(f"svc_{i}"))
            except Exception as e:
                errors.append(e)

        threads = [threading.Thread(target=register, args=(i,)) for i in range(20)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        assert len(errors) == 0
        assert len(CallbackRegistry.list_all()) == 20
