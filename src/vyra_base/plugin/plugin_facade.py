"""
vyra_base.plugin.plugin_facade
================================

PluginFacade — Permission-Prüfung und Rate-Limiting für Plugin-Kommunikation.

Sitzt oberhalb von BaseHostFunctions und prüft vor jeder Kommunikationsoperation,
ob das Plugin die notwendige Berechtigung aus seiner metadata.json besitzt.

Berechtigungsformat in metadata.json::

    {
      "permissions": {
        "interfaces": [
          {
            "transport": "subscribe",
            "module_name": "v2_modulemanager",
            "module_id": null
          },
          {
            "transport": "publish",
            "module_name": "v2_dashboard",
            "module_id": null
          },
          {
            "transport": "client",
            "module_name": "v2_modulemanager",
            "module_id": null
          }
        ]
      }
    }

Unterstützte transport-Typen: "subscribe", "publish", "server", "client"
(keine actions — zu komplex für aktuellen Stand)

Rate-Limiting:
    Der Facade verfolgt Aufrufzeitpunkte und kann Plugins verlangsamen
    (throttle) oder vollständig sperren (block) wenn Limits überschritten werden.

Verwendung::

    facade = PluginFacade(
        plugin_id="counter-widget",
        host_functions=ModuleManagerHostFunctions(...),
        metadata={"permissions": {"interfaces": [...]}},
    )
    # Plugin-Code kommuniziert ausschließlich über den Facade:
    pub = await facade.create_publisher("my_topic", module_name="v2_dashboard")
"""

from __future__ import annotations

import asyncio
import logging
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Callable, Deque, Optional

logger = logging.getLogger(__name__)

# Standardlimits — können pro Plugin überschrieben werden
_DEFAULT_RATE_LIMIT_CALLS = 50        # Maximale Aufrufe im Zeitfenster
_DEFAULT_RATE_LIMIT_WINDOW_SEC = 10.0  # Zeitfenster in Sekunden
_DEFAULT_THROTTLE_DELAY_SEC = 0.1     # Verzögerung nach Rate-Limit-Überschreitung


@dataclass
class InterfacePermission:
    """
    Einzelne Interface-Berechtigung aus metadata.json.

    :param transport:   Transporttyp: "subscribe" | "publish" | "server" | "client"
    :param module_name: Ziel-/Quell-Modulname (z.B. "v2_modulemanager")
    :param module_id:   Optionale Modul-Instanz-ID (None = jede Instanz erlaubt)
    """
    transport:   str
    module_name: str
    module_id:   Optional[str] = None


@dataclass
class RateLimitConfig:
    """Konfiguration für Plugin-Ratebegrenzung."""
    max_calls:   int   = _DEFAULT_RATE_LIMIT_CALLS
    window_sec:  float = _DEFAULT_RATE_LIMIT_WINDOW_SEC
    throttle_sec: float = _DEFAULT_THROTTLE_DELAY_SEC
    blocked:     bool  = False  # Wenn True: Plugin permanent gesperrt


class PluginFacade:
    """
    Facade über BaseHostFunctions — prüft Berechtigungen und begrenzt Aufrufrate.

    Plugins dürfen nur Kommunikationsoperationen nutzen, die in der
    ``permissions.interfaces`` Liste der metadata.json definiert sind.

    Rate-Limiting:
        Wenn ein Plugin innerhalb eines Zeitfensters mehr Aufrufe tätigt als
        erlaubt, wird es automatisch verlangsamt (Throttle-Delay) oder bei
        wiederholter Überschreitung gesperrt.

    :param plugin_id:      Eindeutige Plugin-ID (z.B. 'counter-widget')
    :param host_functions: Konkrete HostFunctions-Implementierung des Moduls
    :param metadata:       Plugin-Metadaten aus metadata.json (enthält permissions)
    :param rate_config:    Optionale Rate-Limit-Konfiguration
    """

    def __init__(
        self,
        plugin_id:      str,
        host_functions: Any,   # BaseHostFunctions-Implementierung
        metadata:       dict[str, Any],
        rate_config:    Optional[RateLimitConfig] = None,
    ) -> None:
        self._plugin_id     = plugin_id
        self._host          = host_functions
        self._rate_config   = rate_config or RateLimitConfig()
        self._call_times:   Deque[float] = deque()
        self._block_count:  int = 0

        # Berechtigungen aus metadata.json parsen
        raw_interfaces = (
            metadata.get("permissions", {}).get("interfaces", [])
        )
        self._permissions: list[InterfacePermission] = [
            InterfacePermission(
                transport=entry.get("transport", ""),
                module_name=entry.get("module_name", ""),
                module_id=entry.get("module_id"),
            )
            for entry in raw_interfaces
            if entry.get("transport") and entry.get("module_name")
        ]

        logger.info(
            "✅ PluginFacade [%s]: %d erlaubte Interface-Berechtigungen",
            plugin_id, len(self._permissions),
        )

    # ------------------------------------------------------------------
    # Public API — delegiert nach Permission-Check
    # ------------------------------------------------------------------

    async def notify_ui(self, event_name: str, data: dict[str, Any]) -> None:
        """Weiterleitung ohne Berechtigung erforderlich (publish-to-self)."""
        await self._throttle()
        await self._host.notify_ui(event_name, data)

    async def create_publisher(
        self,
        name:        str,
        module_name: str,
        module_id:   Optional[str] = None,
        **kwargs:    Any,
    ) -> Any:
        """
        Erstellt einen Publisher — erfordert ``"publish"`` Permission für module_name.

        :raises PermissionError: Wenn keine passende Berechtigung vorhanden
        """
        self._check_permission("publish", module_name, module_id)
        await self._throttle()
        return await self._host.create_publisher(name, module_name, module_id, **kwargs)

    async def create_subscriber(
        self,
        name:        str,
        callback:    Callable,
        module_name: str,
        module_id:   Optional[str] = None,
        **kwargs:    Any,
    ) -> Any:
        """
        Erstellt einen Subscriber — erfordert ``"subscribe"`` Permission für module_name.

        :raises PermissionError: Wenn keine passende Berechtigung vorhanden
        """
        self._check_permission("subscribe", module_name, module_id)
        await self._throttle()
        return await self._host.create_subscriber(name, callback, module_name, module_id, **kwargs)

    async def create_server(
        self,
        name:     str,
        callback: Callable,
        **kwargs: Any,
    ) -> Any:
        """
        Erstellt einen Service-Server — erfordert ``"server"`` Permission.

        Der Server ist nicht an ein spezifisches Modul gebunden, daher wird
        nur der Transport-Typ geprüft.

        :raises PermissionError: Wenn keine ``"server"``-Berechtigung vorhanden
        """
        self._check_server_permission()
        await self._throttle()
        return await self._host.create_server(name, callback, **kwargs)

    async def create_client(
        self,
        name:        str,
        module_name: str,
        module_id:   Optional[str] = None,
        **kwargs:    Any,
    ) -> Any:
        """
        Erstellt einen Service-Client — erfordert ``"client"`` Permission für module_name.

        :raises PermissionError: Wenn keine passende Berechtigung vorhanden
        """
        self._check_permission("client", module_name, module_id)
        await self._throttle()
        return await self._host.create_client(name, module_name, module_id, **kwargs)

    async def log(self, level: str, message: str) -> None:
        """Logging-Durchleitung (keine Permission erforderlich)."""
        await self._host.log(level, message)

    # ------------------------------------------------------------------
    # Verwaltung: Block / Unblock
    # ------------------------------------------------------------------

    def block_plugin(self) -> None:
        """Sperrt das Plugin dauerhaft — alle Aufrufe werden mit PermissionError blockiert."""
        self._rate_config.blocked = True
        logger.warning("🚫 Plugin [%s] gesperrt (block_plugin)", self._plugin_id)

    def unblock_plugin(self) -> None:
        """Hebt die dauerhafte Sperre wieder auf."""
        self._rate_config.blocked = False
        self._block_count = 0
        self._call_times.clear()
        logger.info("✅ Plugin [%s] entsperrt", self._plugin_id)

    @property
    def is_blocked(self) -> bool:
        """True wenn das Plugin permanent gesperrt ist."""
        return self._rate_config.blocked

    # ------------------------------------------------------------------
    # Interne Hilfsmethoden
    # ------------------------------------------------------------------

    def _check_permission(
        self,
        transport:   str,
        module_name: str,
        module_id:   Optional[str],
    ) -> None:
        """
        Prüft ob eine passende Berechtigung existiert.

        Ein Wildcard-Eintrag (module_id=None) erlaubt alle Instanzen des Moduls.

        :raises PermissionError: Wenn keine Berechtigung gefunden
        """
        for perm in self._permissions:
            if perm.transport != transport:
                continue
            if perm.module_name != module_name:
                continue
            # module_id=None im Eintrag = alle Instanzen erlaubt
            if perm.module_id is None or perm.module_id == module_id:
                return

        raise PermissionError(
            f"Plugin [{self._plugin_id}] hat keine '{transport}'-Berechtigung "
            f"für Modul '{module_name}' (module_id={module_id!r}). "
            f"Erlaubte Berechtigungen: {self._permissions!r}"
        )

    def _check_server_permission(self) -> None:
        """Prüft ob eine ``"server"``-Berechtigung existiert (ohne Modulbindung)."""
        for perm in self._permissions:
            if perm.transport == "server":
                return
        raise PermissionError(
            f"Plugin [{self._plugin_id}] hat keine 'server'-Berechtigung. "
            f"Erlaubte Berechtigungen: {self._permissions!r}"
        )

    async def _throttle(self) -> None:
        """
        Rate-Limiting: Überwacht Aufruffrequenz und verlangsamt das Plugin
        bei Überschreitung. Bei wiederholter Überschreitung wird das Plugin
        dauerhaft gesperrt.

        :raises PermissionError: Wenn das Plugin gesperrt ist
        """
        if self._rate_config.blocked:
            raise PermissionError(
                f"Plugin [{self._plugin_id}] ist gesperrt und kann keine "
                "Kommunikationsoperationen durchführen."
            )

        now = time.monotonic()
        window = self._rate_config.window_sec

        # Alte Einträge außerhalb des Fensters entfernen
        while self._call_times and (now - self._call_times[0]) > window:
            self._call_times.popleft()

        self._call_times.append(now)

        if len(self._call_times) > self._rate_config.max_calls:
            self._block_count += 1
            delay = self._rate_config.throttle_sec * self._block_count
            logger.warning(
                "⚠️  Plugin [%s] überschreitet Rate-Limit (%d/%d in %.1fs) — "
                "Throttle %.2fs (Block-Count: %d)",
                self._plugin_id,
                len(self._call_times),
                self._rate_config.max_calls,
                window,
                delay,
                self._block_count,
            )

            # Nach 10 Überschreitungen dauerhaft sperren
            if self._block_count >= 10:
                self.block_plugin()
                raise PermissionError(
                    f"Plugin [{self._plugin_id}] nach wiederholter Rate-Limit-"
                    "Überschreitung dauerhaft gesperrt."
                )

            await asyncio.sleep(delay)
