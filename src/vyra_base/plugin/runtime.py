"""
vyra_base.plugin.runtime
=========================

Plugin-Runtime-Implementierungen.

Klassen:
    PluginCallError  — Basis-Exception für Fehler beim Plugin-Aufruf
    PluginRuntime    — Abstrakte Basis / WASM-Schnittstelle
    WasmRuntime      — Echter WASM-Executor via wasmtime (aktiv wenn verfügbar)
    StubRuntime      — Python-Stub ohne WASM (Fallback für Tests / Connectivity)

Die WasmRuntime führt beliebige WASM-Plugins aus. Welche Funktionen ein Plugin
anbietet, wird in seiner metadata.json unter ``exports[]`` beschrieben::

    {
        "exports": [
            {
                "name": "init",
                "args": [
                    {"name": "initial_value", "type": "i32"},
                    {"name": "step",          "type": "i32"}
                ]
            },
            {
                "name": "increment",
                "args": [{"name": "step", "type": "i32"}]
            },
            ...
        ]
    }

Die Runtime mappt die Keys eines ``data``-Dicts **in Reihenfolge der args-Definition**
auf i32-Parameter — kein hartkodiertes Wissen über einzelne Plugins nötig.

Nutze create_plugin_runtime() um automatisch die beste verfügbare Runtime zu erhalten.

Integrationsbeispiel (v2_modulemanager)::

    from vyra_base.plugin.runtime import create_plugin_runtime
    from mymodule.host_functions_impl import ModuleHostFunctions

    host_fns = ModuleHostFunctions(plugin_event_publisher, zenoh_session)
    runtime  = create_plugin_runtime(
        plugin_id   = "my-plugin",
        wasm_path   = "/opt/vyra/plugin_pool/my-plugin/1.0.0/logic.wasm",
        host        = host_fns,
        initial_state = {"initial_value": 0, "step": 1},
    )
    await runtime.start()

    result = await runtime.call("increment", {"step": 2})
    # → {"result": 2}   (WasmRuntime: echte WASM-Ausführung)
"""

from __future__ import annotations

import json
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

    Lädt das kompilierte logic.wasm und liest die zur WASM-Datei gehörende
    metadata.json. Aus ``metadata.json["exports"]`` werden Funktionssignaturen
    dynamisch gecacht — kein hartkodiertes Wissen über einzelne Plugin-Funktionen.

    Aufruf-Konvention (Metadata-driven i32):
      - ``metadata.json`` enthält ``exports[]`` mit Funktionsnamen und arg-Definitionen
      - ``call(function_name, data)`` mappt ``data``-Keys in Reihenfolge der args auf i32
      - Fehlende Keys → 0; überschüssige Keys → ignoriert
      - Rückgabe: WASM-Funktionen mit i32-Rückgabe → ``{"result": <int>}``
      - ``ping`` ist eingebaut (kein WASM-Export nötig)

    Nur verfügbar wenn ``wasmtime`` installiert ist.
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
        # fn_name → wasmtime callable
        self._exports: dict[str, Any] = {}
        # fn_name → list of {"name": str, "type": str} (aus metadata.json)
        self._exports_meta: dict[str, list[dict[str, str]]] = {}

    async def start(self) -> None:
        if self._started:
            logger.warning("[%s] WasmRuntime already started", self.plugin_id)
            return

        if not self.wasm_path.exists():
            raise FileNotFoundError(
                f"[{self.plugin_id}] WASM-Datei nicht gefunden: {self.wasm_path}"
            )

        # --- metadata.json laden -----------------------------------------------
        meta_path = self.wasm_path.parent / "metadata.json"
        if meta_path.exists():
            try:
                meta = json.loads(meta_path.read_text())
                for export in meta.get("exports", []):
                    fn_name = export.get("name", "")
                    if fn_name:
                        self._exports_meta[fn_name] = export.get("args", [])
                logger.info(
                    "📋 [%s] metadata.json geladen | exports=%s",
                    self.plugin_id, list(self._exports_meta.keys()),
                )
            except Exception as exc:
                logger.warning("[%s] metadata.json nicht lesbar: %s", self.plugin_id, exc)
        else:
            logger.warning(
                "[%s] Keine metadata.json neben WASM-Datei gefunden: %s",
                self.plugin_id, meta_path,
            )

        # --- WASM laden --------------------------------------------------------
        engine = Engine()
        self._store = Store(engine)
        wasm_module = WasmModule(engine, self.wasm_path.read_bytes())
        linker = Linker(engine)
        self._instance = linker.instantiate(self._store, wasm_module)

        # Alle aus metadata.json bekannten Funktionen cachen
        exports_obj = self._instance.exports(self._store)
        for fn_name in self._exports_meta.keys():
            fn = exports_obj.get(fn_name)
            if fn is not None:
                self._exports[fn_name] = fn
            else:
                logger.warning(
                    "[%s] WASM exportiert '%s' nicht (in metadata.json deklariert)",
                    self.plugin_id, fn_name,
                )

        self._started = True
        logger.info(
            "✅ [%s] WasmRuntime gestartet | exports=%s | wasm=%s Bytes",
            self.plugin_id,
            list(self._exports.keys()),
            self.wasm_path.stat().st_size,
        )

        # init aufrufen falls initial_state gesetzt und 'init' exportiert
        if self._initial_state and "init" in self._exports:
            await self.call("init", self._initial_state)

    async def stop(self) -> None:
        self._started = False
        self._instance = None
        self._exports = {}
        self._exports_meta = {}
        logger.info("🛑 [%s] WasmRuntime stopped", self.plugin_id)

    async def call(self, function_name: str, data: dict[str, Any]) -> dict[str, Any]:
        if not self._started:
            raise PluginCallError(self.plugin_id, function_name, "Runtime not started")

        logger.debug("[%s] wasm.call(%s, %s)", self.plugin_id, function_name, data)
        result = await self._dispatch_wasm(function_name, data)
        logger.debug("[%s] wasm.call(%s) -> %s", self.plugin_id, function_name, result)
        return result

    async def on_event(self, event_name: str, data: dict[str, Any]) -> None:
        logger.debug("[%s] on_event(%s) — kein WASM-Event-Handler", self.plugin_id, event_name)

    def _get_wasm_fn(self, name: str) -> Any:
        fn = self._exports.get(name)
        if fn is None:
            raise PluginCallError(
                self.plugin_id, name,
                f"WASM-Funktion '{name}' nicht verfügbar. "
                f"Bekannte Exports: {list(self._exports.keys())}"
            )
        return fn

    async def _dispatch_wasm(self, function_name: str, data: dict[str, Any]) -> dict[str, Any]:
        """
        Generischer Metadata-driven WASM-Dispatch.

        Mappt ``data``-Keys auf i32-Parameter in Reihenfolge der ``exports_meta``-Definition.
        Gibt ``{"result": <wasm_return>}`` zurück und publiziert generisch via notify_ui.
        """
        # ping ist eingebaut — kein WASM-Export nötig
        if function_name == "ping":
            return {"status": "ok", "plugin_id": self.plugin_id, "runtime": "wasm"}

        # Funktion in metadata bekannt?
        if function_name not in self._exports_meta:
            raise PluginCallError(
                self.plugin_id, function_name,
                f"Unbekannte Funktion '{function_name}'. "
                f"In metadata.json definierte Exports: {list(self._exports_meta.keys())}"
            )

        fn = self._get_wasm_fn(function_name)
        arg_defs = self._exports_meta[function_name]

        # i32-Argumente in Reihenfolge der Metadaten-Definition bauen
        args: list[int] = []
        for arg_def in arg_defs:
            arg_name = arg_def.get("name", "")
            args.append(int(data.get(arg_name, 0)))

        # WASM aufrufen
        raw_result = fn(self._store, *args)

        # Ergebnis normalisieren
        if isinstance(raw_result, int):
            result: dict[str, Any] = {"result": raw_result}
        elif isinstance(raw_result, (list, tuple)):
            result = {"result": list(raw_result)}
        elif raw_result is None:
            result = {}
        else:
            result = {"result": raw_result}

        # Generisches Event an UI/Frontend senden
        await self.host.notify_ui(f"plugin.{function_name}.result", {
            "plugin_id": self.plugin_id,
            "result":    result,
        })

        return result


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

    :param plugin_id:     Plugin-ID (z.B. "my-plugin")
    :param wasm_path:     Pfad zur logic.wasm Datei (neben ihr muss metadata.json liegen)
    :param host:          Host-Funktionen-Implementierung (optional, sonst NullHostFunctions)
    :param initial_state: Startzustand — Keys müssen zu den args des 'init'-Exports passen
    :param prefer_stub:   Erzwinge StubRuntime auch wenn wasmtime verfügbar ist
    :returns:             WasmRuntime oder StubRuntime Instanz

    Beispiel::

        runtime = create_plugin_runtime(
            plugin_id     = "my-plugin",
            wasm_path     = "/opt/vyra/plugin_pool/my-plugin/1.0.0/logic.wasm",
            host          = my_host_functions,
            initial_state = {"initial_value": 0, "step": 1},
        )
        await runtime.start()
        result = await runtime.call("increment", {"step": 2})
    """
    wasm_path = Path(wasm_path)
    use_wasm = (
        not prefer_stub
        and _WASMTIME_AVAILABLE
        and wasm_path.exists()
        and wasm_path.stat().st_size > 8
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

    Fallback für Umgebungen ohne wasmtime oder wenn die WASM-Datei fehlt.
    Eignet sich für Connectivity-Tests und Infrastruktur-Validierung ohne
    echte Plugin-Logik.

    Unterstützte Funktionen (generisch für alle Plugins):
      - ``ping``      — Lebenszeichen zurückgeben
      - ``get_state`` — Internen State zurückgeben
      - ``set_state`` — Internen State setzen

    Für alle anderen Funktionsnamen wird ein ``PluginCallError`` geworfen,
    da Plugin-spezifische Logik ausschließlich in der WasmRuntime ausgeführt wird.
    Unterklassen können ``_dispatch()`` erweitern um eigene Test-Stubs hinzuzufügen.
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
        if self._started:
            logger.warning("[%s] StubRuntime already started", self.plugin_id)
            return

        if not self.wasm_path.exists():
            logger.warning(
                "[%s] WASM-Datei nicht gefunden: %s — Stub läuft ohne WASM",
                self.plugin_id, self.wasm_path,
            )
        else:
            logger.info(
                "[%s] ⚠️  StubRuntime gestartet (WASM vorhanden aber nicht ausgeführt)",
                self.plugin_id,
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
        if not self._started:
            raise PluginCallError(self.plugin_id, function_name, "Runtime not started")

        logger.debug("[%s] call(%s, %s)", self.plugin_id, function_name, data)
        result = await self._dispatch(function_name, data)
        logger.debug("[%s] call(%s) -> %s", self.plugin_id, function_name, result)
        return result

    async def on_event(self, event_name: str, data: dict[str, Any]) -> None:
        logger.debug("[%s] on_event(%s, %s) — Stub: ignoriert", self.plugin_id, event_name, data)

    # ------------------------------------------------------------------
    # Dispatch-Tabelle (überschreibbar für plugin-spezifische Stubs)
    # ------------------------------------------------------------------

    async def _dispatch(self, function_name: str, data: dict[str, Any]) -> dict[str, Any]:
        """
        Routing zu generischen Stub-Implementierungen.
        Unterklassen können diese Methode erweitern oder ersetzen.
        Plugin-spezifische Funktionen werden nicht unterstützt — dafür WasmRuntime nutzen.
        """
        handlers: dict[str, Any] = {
            "ping":      self._fn_ping,
            "get_state": self._fn_get_state,
            "set_state": self._fn_set_state,
        }

        handler = handlers.get(function_name)
        if handler is None:
            raise PluginCallError(
                self.plugin_id,
                function_name,
                f"Funktion '{function_name}' wird im StubRuntime nicht unterstützt. "
                f"Generisch verfügbar: {list(handlers.keys())}. "
                f"Plugin-spezifische Logik erfordert WasmRuntime.",
            )

        return await handler(data)

    # ------------------------------------------------------------------
    # Generische Stub-Implementierungen
    # ------------------------------------------------------------------

    async def _fn_ping(self, _data: dict) -> dict:
        return {"status": "ok", "plugin_id": self.plugin_id, "runtime": "stub"}

    async def _fn_get_state(self, _data: dict) -> dict:
        return dict(self._state)

    async def _fn_set_state(self, data: dict) -> dict:
        self._state.update(data)
        return dict(self._state)
