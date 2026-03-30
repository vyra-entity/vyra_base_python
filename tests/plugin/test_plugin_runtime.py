"""
Unit tests for vyra_base.plugin.runtime

Tests cover create_plugin_runtime() factory, StubRuntime lifecycle,
dispatch table, and PluginCallError exception.
No wasmtime dependency required — all tests use StubRuntime.
"""

import pytest
from pathlib import Path

from vyra_base.plugin.runtime import (
    create_plugin_runtime,
    StubRuntime,
    PluginCallError,
    PluginRuntime,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

NONEXISTENT_WASM = Path("/nonexistent/logic.wasm")


def make_stub(plugin_id: str = "test", initial_state: dict | None = None) -> StubRuntime:
    """Create a StubRuntime with prefer_stub=True so no filesystem is needed."""
    return create_plugin_runtime(  # type: ignore[return-value]
        plugin_id=plugin_id,
        wasm_path=NONEXISTENT_WASM,
        initial_state=initial_state,
        prefer_stub=True,
    )


# ---------------------------------------------------------------------------
# Factory tests
# ---------------------------------------------------------------------------


class TestCreatePluginRuntime:
    """Tests for the create_plugin_runtime() factory function."""

    def test_returns_stub_when_prefer_stub_true(self):
        """prefer_stub=True must always yield a StubRuntime."""
        runtime = create_plugin_runtime("pid", NONEXISTENT_WASM, prefer_stub=True)
        assert isinstance(runtime, StubRuntime)

    def test_returns_stub_when_wasm_missing(self):
        """Missing .wasm file falls back to StubRuntime even without prefer_stub."""
        runtime = create_plugin_runtime("pid", NONEXISTENT_WASM)
        assert isinstance(runtime, StubRuntime)

    def test_returns_plugin_runtime_subclass(self):
        """create_plugin_runtime always returns a PluginRuntime subclass."""
        runtime = create_plugin_runtime("pid", NONEXISTENT_WASM, prefer_stub=True)
        assert isinstance(runtime, PluginRuntime)

    def test_plugin_id_stored(self):
        """factory must preserve the plugin_id on the returned runtime."""
        runtime = create_plugin_runtime("my-plugin", NONEXISTENT_WASM, prefer_stub=True)
        assert runtime.plugin_id == "my-plugin"

    def test_wasm_path_stored(self):
        """factory must store wasm_path as a Path object."""
        runtime = create_plugin_runtime("pid", NONEXISTENT_WASM, prefer_stub=True)
        assert runtime.wasm_path == NONEXISTENT_WASM

    def test_initial_state_is_optional(self):
        """No initial_state should not raise."""
        runtime = create_plugin_runtime("pid", NONEXISTENT_WASM, prefer_stub=True)
        assert runtime is not None


# ---------------------------------------------------------------------------
# StubRuntime lifecycle tests
# ---------------------------------------------------------------------------


class TestStubRuntimeLifecycle:
    """Tests for StubRuntime start / stop / is_running."""

    @pytest.mark.asyncio
    async def test_not_running_before_start(self):
        runtime = make_stub()
        assert runtime.is_running() is False

    @pytest.mark.asyncio
    async def test_running_after_start(self):
        runtime = make_stub()
        await runtime.start()
        assert runtime.is_running() is True

    @pytest.mark.asyncio
    async def test_stopped_after_stop(self):
        runtime = make_stub()
        await runtime.start()
        await runtime.stop()
        assert runtime.is_running() is False

    @pytest.mark.asyncio
    async def test_double_start_is_safe(self):
        """Calling start() twice must not raise."""
        runtime = make_stub()
        await runtime.start()
        await runtime.start()  # should log a warning, not raise
        assert runtime.is_running() is True

    @pytest.mark.asyncio
    async def test_stop_without_start_is_safe(self):
        """Calling stop() on a never-started runtime must not raise."""
        runtime = make_stub()
        await runtime.stop()
        assert runtime.is_running() is False


# ---------------------------------------------------------------------------
# StubRuntime call / dispatch tests
# ---------------------------------------------------------------------------


class TestStubRuntimeDispatch:
    """Tests for StubRuntime.call() dispatch table."""

    @pytest.mark.asyncio
    async def test_call_raises_if_not_started(self):
        runtime = make_stub()
        with pytest.raises(PluginCallError):
            await runtime.call("ping", {})

    @pytest.mark.asyncio
    async def test_ping_returns_ok(self):
        runtime = make_stub("my-plugin")
        await runtime.start()
        result = await runtime.call("ping", {})
        assert result["status"] == "ok"
        assert result["plugin_id"] == "my-plugin"
        assert result["runtime"] == "stub"

    @pytest.mark.asyncio
    async def test_get_state_returns_dict(self):
        runtime = make_stub(initial_state={"counter": 5})
        await runtime.start()
        result = await runtime.call("get_state", {})
        assert isinstance(result, dict)
        assert result["counter"] == 5

    @pytest.mark.asyncio
    async def test_set_state_updates_state(self):
        runtime = make_stub()
        await runtime.start()
        await runtime.call("set_state", {"value": 42})
        result = await runtime.call("get_state", {})
        assert result["value"] == 42

    @pytest.mark.asyncio
    async def test_unknown_function_raises_plugin_call_error(self):
        runtime = make_stub()
        await runtime.start()
        with pytest.raises(PluginCallError) as exc_info:
            await runtime.call("increment", {"step": 1})
        assert exc_info.value.plugin_id == "test"
        assert exc_info.value.function_name == "increment"

    @pytest.mark.asyncio
    async def test_on_event_does_not_raise(self):
        """on_event must silently swallow unknown events in StubRuntime."""
        runtime = make_stub()
        await runtime.start()
        await runtime.on_event("some.event", {"key": "value"})  # should not raise


# ---------------------------------------------------------------------------
# StubRuntime initial_state tests
# ---------------------------------------------------------------------------


class TestStubRuntimeInitialState:
    """Tests for initial_state handling."""

    @pytest.mark.asyncio
    async def test_initial_state_available_via_get_state(self):
        runtime = make_stub(initial_state={"x": 10, "y": 20})
        await runtime.start()
        state = await runtime.call("get_state", {})
        assert state["x"] == 10
        assert state["y"] == 20

    @pytest.mark.asyncio
    async def test_initial_state_not_mutated_by_factory(self):
        """The factory must copy initial_state, not hold a reference to it."""
        original = {"a": 1}
        runtime = make_stub(initial_state=original)
        await runtime.start()
        await runtime.call("set_state", {"a": 99})
        assert original["a"] == 1  # original dict must be untouched

    @pytest.mark.asyncio
    async def test_empty_initial_state(self):
        runtime = make_stub(initial_state={})
        await runtime.start()
        state = await runtime.call("get_state", {})
        assert state == {}


# ---------------------------------------------------------------------------
# PluginCallError tests
# ---------------------------------------------------------------------------


class TestPluginCallError:
    """Tests for the PluginCallError exception class."""

    def test_attributes_stored(self):
        exc = PluginCallError("pid", "fn", "reason")
        assert exc.plugin_id == "pid"
        assert exc.function_name == "fn"
        assert exc.reason == "reason"

    def test_message_contains_plugin_id(self):
        exc = PluginCallError("my-plugin", "do_thing", "not found")
        assert "my-plugin" in str(exc)

    def test_message_contains_function_name(self):
        exc = PluginCallError("pid", "do_thing", "not found")
        assert "do_thing" in str(exc)

    def test_is_exception(self):
        exc = PluginCallError("pid", "fn", "reason")
        assert isinstance(exc, Exception)
