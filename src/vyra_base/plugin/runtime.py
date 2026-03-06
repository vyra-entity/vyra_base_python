"""
vyra_base.plugin.runtime
=========================

Plugin-Runtime-Implementierungen.

Klassen:
    PluginCallError  — Basis-Exception für Fehler beim Plugin-Aufruf
    PluginRuntime    — Abstrakte Basis / WASM-Schnittstelle
    WasmRuntime      — Echter WASM-Executor via wasmtime (aktiv wenn verfügbar)
    StubRuntime      — Python-Stub-Implementierung (Fallback ohne wasmtime)

Die WasmRuntime führt die echte logic.wasm aus (init, increment, reset, get_count,
get_step) und ist die bevorzugte Implementierung wenn wasmtime installiert ist.
Die StubRuntime ist der Pure-Python-Fallback für Umgebungen ohne wasmtime.

Nutze create_plugin_runtime() um automatisch die beste verfügbare Runtime zu erhalten.

Integrationsbeispiel (v2_modulemanager)::

    from vyra_base.plugin.runtime import create_plugin_runtime
    from vyra_base.plugin.host_functions import ModuleManagerHostFunctions

    host_fns = ModuleManagerHostFunctions(zenoh_session, redis)
    runtime  = create_plugin_runtime(
        plugin_id   = "counter-widget",
        wasm_path   = "/nfs/plugins/pool/counter-widget/1.0.0/logic.wasm",
        host        = host_fns,
        initial_state = {"count": 0, "step": 1},
    )
    await runtime.start()

    result = await runtime.call("increment", {})
    # WasmRuntime:  WASM increment(0) ausgeführt -> {"count": 1}
    # StubRuntime:  Python-Logik -> {"count": 1}

    result = await runtime.call("get_state", {})
    # -> {"count": 1, "step": 1}
"""

from __future__ import annotations

import logging
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Any

from vyra_base.plugin.host_functions import HostFunctions, NullHostFunctions

logger = logging.getLogger(__name__)

# Prüfen ob wasmtime verfügbar ist (optionale Abhängigkeit)
try:
    from wasmtime import Store, Module as WasmModule, Instance, Linker, Engine  # type: ignore[import]
    _WASMTIME_AVAILABLE = True
except ImportError:
    _WASMTIME_AVAILABLE = False


class PluginCallError(Exception):
    """Wird geworfen, wenn ein Plugin-Aufruf fehlschlägt."""

    def __init__(self, plugin_id: str, function_name: str, reason: str):
        self.plugin_id     = plugin_id
        self.function_name = function_name
        self.reason        = reason
        super().__init__(f"[{plugin_id}] call '{function_name}' failed: {reason}")


class PluginRuntime(ABC):
    """
    Abstrakte Basis-Klasse für Plugin-Runtimes.

    Definiert die gemeinsame API, die sowohl die StubRuntime (aktuell)
    als auch die zukünftige ExtismRuntime (Phase 2) implementieren.
    """

    def __init__(
        self,
        plugin_id: str,
        wasm_path: str | Path,
        host: HostFunctions | None = None,
    ) -> None:
        self.plugin_id = plugin_id
        self.wasm_path = Path(wasm_path)
        self.host      = host or NullHostFunctions()
        self._started  = False

    @abstractmethod
    async def start(self) -> None:
        """Lädt das WASM-Modul und initialisiert die Runtime."""
        ...

    @abstractmethod
    async def stop(self) -> None:
        """Gibt Ressourcen der Runtime frei."""
        ...

    @abstractmethod
    async def call(self, function_name: str, data: dict[str, Any]) -> dict[str, Any]:
        """
        Ruft eine exportierte Funktion des Plugins auf.

        :param function_name: Name der WASM-Funktion (z.B. "increment", "get_state")
        :param data:          Eingabe-Parameter als JSON-serialisierbares Dict
        :returns:             Rückgabe-Wert als Dict
        :raises PluginCallError: Bei Fehler im Plugin oder nicht unterstützter Funktion
        """
        ...

    @abstractmethod
    async def on_event(self, event_name: str, data: dict[str, Any]) -> None:
        """
        Leitet ein externes Event (Zenoh/Redis) an das Plugin weiter.

        :param event_name: Event-Name (z.B. "zenoh.message", "module.state_changed")
        :param data:       Event-Payload
        """
        ...

    def is_running(self) -> bool:
        """Gibt an, ob die Runtime aktiv ist."""
        return self._started

    def __repr__(self) -> str:
        status = "running" if self._started else "stopped"
        return f"<{self.__class__.__name__} plugin_id={self.plugin_id!r} wasm={self.wasm_path.name!r} {status}>"


# ---------------------------------------------------------------------------
# WasmRuntime — führt die echte logic.wasm via wasmtime aus
# ---------------------------------------------------------------------------

class WasmRuntime(PluginRuntime):
    """
    Echter WASM-Executor für VYRA-Plugins via wasmtime.

    Lädt das kompilierte logic.wasm, instanziiert es und führt die
    exportierten Funktionen direkt im WASM aus:

      init(initial_count: i32, step: i32) -> i32
      increment(step_override: i32) -> i32   (0 = gespeicherten step nutzen)
      reset() -> i32
      get_count() -> i32
      get_step() -> i32

    Nur verfügbar wenn `wasmtime` installiert ist.
    Nutze create_plugin_runtime() für automatische Auswahl.
    """

    def __init__(
        self,
        plugin_id: str,
        wasm_path: str | Path,
        host: HostFunctions | None = None,
        initial_state: dict[str, Any] | None = None,
    ) -> None:
        if not _WASMTIME_AVAILABLE:
            raise ImportError(
                "wasmtime ist nicht installiert. Installiere es mit: pip install wasmtime"
            )
        super().__init__(plugin_id, wasm_path, host)
        self._initial_state: dict[str, Any] = initial_state.copy() if initial_state else {}
        self._store: Any = None
        self._instance: Any = None
        self._exports: dict[str, Any] = {}

    async def start(self) -> None:
        if self._started:
            logger.warning("[%s] WasmRuntime already started", self.plugin_id)
            return

        if not self.wasm_path.exists():
            raise FileNotFoundError(
                f"[{self.plugin_id}] WASM-Datei nicht gefunden: {self.wasm_path}"
            )

        engine = Engine()
        self._store = Store(engine)
        wasm_module = WasmModule(engine, self.wasm_path.read_bytes())
        linker = Linker(engine)
        self._instance = linker.instantiate(self._store, wasm_module)

        # Exportierte Funktionen cachen
        for fn_name in ("init", "increment", "reset", "get_count", "get_step"):
            fn = self._instance.exports(self._store).get(fn_name)
            if fn is not None:
                self._exports[fn_name] = fn

        self._started = True
        logger.info(
            "✅ [%s] WasmRuntime gestartet | exports=%s | wasm=%s Bytes",
            self.plugin_id,
            list(self._exports.keys()),
            self.wasm_path.stat().st_size,
        )

        # WASM init aufrufen falls initial_state gesetzt
        initial_count = self._initial_state.get("count", 0)
        initial_step  = self._initial_state.get("step", 1)
        await self.call("init", {"initial_value": initial_count, "step": initial_step})

    async def stop(self) -> None:
        self._started = False
        self._instance = None
        self._exports = {}
        logger.info("🛑 [%s] WasmRuntime stopped", self.plugin_id)

    async def call(self, function_name: str, data: dict[str, Any]) -> dict[str, Any]:
        if not self._started:
            raise PluginCallError(self.plugin_id, function_name, "Runtime not started")

        logger.debug("[%s] wasm.call(%s, %s)", self.plugin_id, function_name, data)
        result = await self._dispatch_wasm(function_name, data)
        logger.debug("[%s] wasm.call(%s) -> %s", self.plugin_id, function_name, result)

        return result

    async def on_event(self, event_name: str, data: dict[str, Any]) -> None:
        logger.debug("[%s] on_event(%s) — WASM hat keine Event-Exports", self.plugin_id, event_name)

    def _wasm_fn(self, name: str) -> Any:
        fn = self._exports.get(name)
        if fn is None:
            raise PluginCallError(
                self.plugin_id, name,
                f"WASM-Funktion '{name}' nicht exportiert. Verfügbar: {list(self._exports.keys())}"
            )
        return fn

    async def _dispatch_wasm(self, function_name: str, data: dict[str, Any]) -> dict[str, Any]:
        """Mapped die Plugin-API auf konkrete WASM i32-Funktionen."""

        if function_name == "init":
            initial = int(data.get("initial_value", 0))
            step    = int(data.get("step", 1))
            label   = data.get("label", "Counter")
            count   = self._wasm_fn("init")(self._store, initial, step)
            await self.host.notify_ui("plugin.initialized", {
                "plugin_id": self.plugin_id,
                "state": {"count": count, "step": step, "label": label},
            })
            return {"count": count, "step": step, "label": label}

        elif function_name == "increment":
            step_override = int(data.get("step", 0))  # 0 = nutze gespeicherten step
            count = self._wasm_fn("increment")(self._store, step_override)
            await self.host.notify_ui("counter.updated", {
                "plugin_id": self.plugin_id,
                "count": count,
            })
            return {"count": count}

        elif function_name == "reset":
            count = self._wasm_fn("reset")(self._store)
            await self.host.notify_ui("counter.updated", {
                "plugin_id": self.plugin_id,
                "count": count,
            })
            return {"count": count}

        elif function_name == "get_state":
            count = self._wasm_fn("get_count")(self._store)
            step  = self._wasm_fn("get_step")(self._store)
            return {"count": count, "step": step}

        elif function_name == "get_count":
            count = self._wasm_fn("get_count")(self._store)
            return {"count": count}

        elif function_name == "ping":
            return {"status": "ok", "plugin_id": self.plugin_id, "runtime": "wasm"}

        else:
            raise PluginCallError(
                self.plugin_id, function_name,
                f"Unbekannte Funktion '{function_name}'. "
                f"Unterstützt: init, increment, reset, get_state, get_count, ping"
            )


# ---------------------------------------------------------------------------
# Factory: wählt automatisch WasmRuntime oder StubRuntime
# ---------------------------------------------------------------------------

def create_plugin_runtime(
    plugin_id: str,
    wasm_path: str | Path,
    host: HostFunctions | None = None,
    initial_state: dict[str, Any] | None = None,
    prefer_stub: bool = False,
) -> "PluginRuntime":
    """
    Factory-Funktion: Gibt WasmRuntime zurück wenn wasmtime installiert ist
    und die .wasm-Datei existiert. Andernfalls StubRuntime.

    :param plugin_id:     Plugin-ID (z.B. "counter-widget")
    :param wasm_path:     Pfad zur logic.wasm Datei
    :param host:          Host-Funktionen-Implementierung (optional)
    :param initial_state: Startzustand ({"count": 0, "step": 1})
    :param prefer_stub:   Erzwinge StubRuntime auch wenn wasmtime verfügbar ist
    :returns:             WasmRuntime oder StubRuntime Instanz

    Beispiel::

        runtime = create_plugin_runtime(
            "counter-widget",
            "/nfs/plugins/pool/counter-widget/1.0.0/logic.wasm",
            initial_state={"count": 0, "step": 1},
        )
        await runtime.start()
        await runtime.call("increment", {})
    """
    wasm_path = Path(wasm_path)
    use_wasm = (
        not prefer_stub
        and _WASMTIME_AVAILABLE
        and wasm_path.exists()
        and wasm_path.stat().st_size > 8  # mehr als reiner Stub
    )

    if use_wasm:
        logger.info(
            "🔧 [%s] create_plugin_runtime → WasmRuntime (wasmtime=%s, wasm=%s Bytes)",
            plugin_id, _WASMTIME_AVAILABLE, wasm_path.stat().st_size
        )
        return WasmRuntime(plugin_id, wasm_path, host, initial_state)
    else:
        reason = (
            "prefer_stub=True" if prefer_stub
            else "wasmtime nicht installiert" if not _WASMTIME_AVAILABLE
            else f"wasm nicht gefunden: {wasm_path}"
        )
        logger.info("🔧 [%s] create_plugin_runtime → StubRuntime (%s)", plugin_id, reason)
        return StubRuntime(plugin_id, wasm_path, host, initial_state)


# ---------------------------------------------------------------------------
# StubRuntime — Pure-Python Fallback ohne WASM-Execution
# ---------------------------------------------------------------------------

class StubRuntime(PluginRuntime):
    """
    Python-Stub-Implementierung der PluginRuntime.

    Ersetzt Extism/WASM vollständig in Python — ermöglicht vollständige
    Integration und Tests ohne echte WASM-Ausführung.

    Die Stub-Logik deckt Standard-Szenarien ab:
      - counter-plugin:  "increment", "reset", "get_state"
      - Allgemein:       "get_state", "set_state", "ping"

    Für Plugin-spezifische Logik kann eine Unterklasse mit _dispatch() überschrieben werden.

    Phase 2: Diese Klasse wird durch ExtismRuntime ersetzt, welche die selbe API
    implementiert, aber die Funktionen im WASM-Modul ausführt.
    """

    def __init__(
        self,
        plugin_id: str,
        wasm_path: str | Path,
        host: HostFunctions | None = None,
        initial_state: dict[str, Any] | None = None,
    ) -> None:
        super().__init__(plugin_id, wasm_path, host)
        self._state: dict[str, Any] = initial_state.copy() if initial_state else {}

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    async def start(self) -> None:
        """
        Stub: Prüft nur ob die WASM-Datei existiert, lädt sie aber nicht.
        Gibt eine Warnung aus, dass WASM-Execution noch nicht aktiv ist.
        """
        if self._started:
            logger.warning("[%s] StubRuntime already started", self.plugin_id)
            return

        if not self.wasm_path.exists():
            logger.warning(
                "[%s] WASM-Datei nicht gefunden: %s — Stub läuft ohne WASM",
                self.plugin_id,
                self.wasm_path,
            )
        else:
            logger.info(
                "[%s] ⚠️  StubRuntime gestartet (WASM %s vorhanden aber nicht ausgeführt — Phase 2)",
                self.plugin_id,
                self.wasm_path.name,
            )

        self._started = True
        logger.info("✅ [%s] StubRuntime ready | state=%s", self.plugin_id, self._state)

    async def stop(self) -> None:
        self._started = False
        logger.info("🛑 [%s] StubRuntime stopped", self.plugin_id)

    # ------------------------------------------------------------------
    # Kernaufruf
    # ------------------------------------------------------------------

    async def call(self, function_name: str, data: dict[str, Any]) -> dict[str, Any]:
        """
        Führt eine Plugin-Funktion im Stub aus.

        Bekannte Funktionen haben eine eingebaute Python-Implementierung.
        Unbekannte Funktionen werfen PluginCallError.
        """
        if not self._started:
            raise PluginCallError(self.plugin_id, function_name, "Runtime not started")

        logger.debug("[%s] call(%s, %s)", self.plugin_id, function_name, data)

        result = await self._dispatch(function_name, data)
        logger.debug("[%s] call(%s) -> %s", self.plugin_id, function_name, result)
        return result

    async def on_event(self, event_name: str, data: dict[str, Any]) -> None:
        """Stub: Loggt eingehende Events, leitet sie nicht weiter (kein WASM)."""
        logger.debug("[%s] on_event(%s, %s) — Stub: ignoriert", self.plugin_id, event_name, data)

    # ------------------------------------------------------------------
    # Dispatch-Tabelle (überschreibbar für plugin-spezifische Stubs)
    # ------------------------------------------------------------------

    async def _dispatch(self, function_name: str, data: dict[str, Any]) -> dict[str, Any]:
        """
        Routing zu Stub-Implementierungen.
        Unterklassen können diese Methode erweitern oder ersetzen.
        """
        handlers: dict[str, Any] = {
            # Standard (alle Plugins)
            "ping":          self._fn_ping,
            "get_state":     self._fn_get_state,
            "set_state":     self._fn_set_state,
            # Counter-Widget spezifisch
            "init":          self._fn_init,
            "increment":     self._fn_increment,
            "reset":         self._fn_reset,
        }

        handler = handlers.get(function_name)
        if handler is None:
            raise PluginCallError(
                self.plugin_id,
                function_name,
                f"Unknown function '{function_name}'. Available in stub: {list(handlers.keys())}",
            )

        return await handler(data)

    # ------------------------------------------------------------------
    # Stub-Implementierungen
    # ------------------------------------------------------------------

    async def _fn_ping(self, _data: dict) -> dict:
        return {"status": "ok", "plugin_id": self.plugin_id, "runtime": "stub"}

    async def _fn_get_state(self, _data: dict) -> dict:
        return dict(self._state)

    async def _fn_set_state(self, data: dict) -> dict:
        self._state.update(data)
        return dict(self._state)

    async def _fn_init(self, data: dict) -> dict:
        """init: Initialisiert den Zähler mit optionalem initial_value aus config_overlay."""
        self._state.setdefault("count", data.get("initial_value", 0))
        self._state.setdefault("step", data.get("step", 1))
        self._state.setdefault("label", data.get("label", "Counter"))
        logger.info("✅ [%s] init | state=%s", self.plugin_id, self._state)

        # Host-Funktion: UI über Init informieren
        await self.host.notify_ui("plugin.initialized", {
            "plugin_id": self.plugin_id,
            "state": self._state,
        })
        return dict(self._state)

    async def _fn_increment(self, data: dict) -> dict:
        """increment: Erhöht den Zähler um 'step' (aus data oder state)."""
        step = data.get("step", self._state.get("step", 1))
        self._state["count"] = self._state.get("count", 0) + step
        logger.info("[%s] increment +%s -> %s", self.plugin_id, step, self._state["count"])

        # Host-Funktion: UI über neuen Wert informieren
        await self.host.notify_ui("counter.updated", {
            "plugin_id": self.plugin_id,
            "count":     self._state["count"],
        })
        return {"count": self._state["count"]}

    async def _fn_reset(self, _data: dict) -> dict:
        """reset: Setzt den Zähler auf 0 zurück."""
        self._state["count"] = 0
        logger.info("[%s] reset -> 0", self.plugin_id)

        await self.host.notify_ui("counter.updated", {
            "plugin_id": self.plugin_id,
            "count":     0,
        })
        return {"count": 0}
