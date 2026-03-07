"""
vyra_base.plugin
================

Plugin-Runtime-Abstraktion für VYRA-Plugins.

Enthält:
  - PluginRuntime          : Abstrakte Basis (ABC)
  - WasmRuntime            : Echter WASM-Executor via wasmtime
  - StubRuntime            : Python-Stub ohne WASM (Fallback / Tests)
  - create_plugin_runtime  : Factory — wählt automatisch beste Runtime
  - HostFunctions          : Protokoll für Host-Funktionen (structural typing)
  - BaseHostFunctions      : Abstrakte Basisklasse für Modul-Implementierungen
  - NullHostFunctions      : No-op Implementierung für Tests
"""

from vyra_base.plugin.host_functions import HostFunctions, BaseHostFunctions, NullHostFunctions
from vyra_base.plugin.runtime import (
    PluginRuntime,
    WasmRuntime,
    StubRuntime,
    PluginCallError,
    create_plugin_runtime,
)

__all__ = [
    "HostFunctions",
    "BaseHostFunctions",
    "NullHostFunctions",
    "PluginRuntime",
    "WasmRuntime",
    "StubRuntime",
    "PluginCallError",
    "create_plugin_runtime",
]
