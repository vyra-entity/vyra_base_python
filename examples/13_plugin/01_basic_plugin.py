"""
Example 13 — WASM Plugin Runtime
=================================

Demonstrates:
- ``create_plugin_runtime()`` auto-selecting WasmRuntime / StubRuntime
- Calling exported plugin functions
- Injecting host functions (callbacks from Python into the WASM sandbox)
- Graceful fallback when wasmtime is not installed

No .wasm file is required to run this example — it automatically falls back
to StubRuntime which simulates the WASM interface in pure Python.

Run:
    python examples/13_plugin/01_basic_plugin.py
"""
import asyncio
import logging
from pathlib import Path

from vyra_base.plugin.runtime import create_plugin_runtime, NullHostFunctions
from vyra_base.plugin.host_functions import BaseHostFunctions

logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Custom host functions — Python callbacks exposed to the WASM plugin
# ---------------------------------------------------------------------------


class CounterHostFunctions(BaseHostFunctions):
    """
    Host-function implementation that logs all plugin events.

    Methods prefixed with ``host_`` are automatically forwarded to the
    plugin sandbox when using WasmRuntime.
    """

    def __init__(self) -> None:
        """Initialise event log."""
        self.events: list[str] = []

    def host_log(self, message: str) -> None:
        """
        Log a message emitted by the plugin.

        :param message: Log message string from the plugin.
        """
        entry = f"[plugin] {message}"
        self.events.append(entry)
        logger.info(entry)

    def host_emit_event(self, event_type: str, payload: str) -> None:
        """
        Handle an event emitted by the plugin.

        :param event_type: Event type identifier.
        :param payload: JSON-serialised event payload.
        """
        entry = f"[event:{event_type}] {payload}"
        self.events.append(entry)
        logger.info(entry)


# ---------------------------------------------------------------------------
# Main demo
# ---------------------------------------------------------------------------


async def main() -> None:
    """Run the plugin demo end-to-end."""
    # Path to a hypothetical .wasm file (doesn't need to exist for the demo)
    wasm_path = Path(__file__).parent / "counter_plugin" / "logic.wasm"

    host = CounterHostFunctions()

    # create_plugin_runtime selects WasmRuntime if wasmtime is installed AND
    # the .wasm file exists; otherwise it falls back to StubRuntime.
    runtime = create_plugin_runtime(
        plugin_id="counter-plugin",
        wasm_path=wasm_path,
        host=host,
        initial_state={"counter": 0, "step": 1},
    )

    logger.info("Runtime type: %s", type(runtime).__name__)

    # Start the runtime (loads WASM module or initialises stub state)
    await runtime.start()
    logger.info("Plugin started — state: %s", await runtime.get_state())

    # Call an exported function
    result = await runtime.call("increment", {"step": 5})
    logger.info("increment(5) → %s", result)

    result = await runtime.call("increment", {"step": 3})
    logger.info("increment(3) → %s", result)

    result = await runtime.call("get_value", {})
    logger.info("get_value() → %s", result)

    # Reset
    result = await runtime.call("reset", {})
    logger.info("reset() → %s", result)

    # Stop / cleanup
    await runtime.stop()
    logger.info("Plugin stopped.")
    logger.info("Host events recorded: %d", len(host.events))


if __name__ == "__main__":
    asyncio.run(main())
