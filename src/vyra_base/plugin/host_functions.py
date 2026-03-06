"""
vyra_base.plugin.host_functions
================================

Protokoll-Klasse: Definiert alle Host-Funktionen, die in die WASM-Sandbox
injiziert werden. Ein Modul, das Plugins hostet (z.B. v2_modulemanager), muss
eine konkrete Implementierung dieser Klasse bereitstellen.

WASM-Plugins können ausschließlich über diese Funktionen mit dem Host interagieren
(kein direktes Filesystem, kein direktes Network-Access aus dem WASM-Kontext).
"""

from __future__ import annotations

import logging
from typing import Any, Protocol, runtime_checkable

logger = logging.getLogger(__name__)


@runtime_checkable
class HostFunctions(Protocol):
    """
    Protokoll für Host-Funktionen einer Plugin-Runtime.

    Jede Methode entspricht einer Funktion, die in die WASM-Sandbox exportiert wird.
    Implementierungen müssen alle Methoden bereitstellen.

    Beispiel-Implementierung im v2_modulemanager::

        class ModuleManagerHostFunctions:
            def __init__(self, zenoh_session, redis_client):
                self._zenoh = zenoh_session
                self._redis = redis_client

            async def notify_ui(self, event_name: str, data: dict) -> None:
                await self._zenoh.put(f"vyra/ui/events/{event_name}", json.dumps(data))

            async def zenoh_get(self, key: str) -> Any:
                return await self._zenoh.get(key)

            async def zenoh_put(self, key: str, value: Any) -> None:
                await self._zenoh.put(key, value)

            async def log(self, level: str, message: str) -> None:
                getattr(logger, level, logger.info)(f"[plugin] {message}")
    """

    async def notify_ui(self, event_name: str, data: dict[str, Any]) -> None:
        """
        Sendet ein Event an das Dashboard-Frontend über Zenoh.

        Das Plugin nutzt diese Funktion, um UI-Updates auszulösen (z.B. Zähler-Update).

        :param event_name: Eindeutiger Event-Name (z.B. "counter.updated")
        :param data:       Beliebige JSON-serialisierbare Nutzdaten
        """
        ...

    async def zenoh_get(self, key: str) -> Any:
        """
        Liest einen Wert vom Zenoh-Bus. Scope wird vom Host geprüft.

        :param key: Zenoh-Key-Expression (muss im erlaubten Scope des Plugins liegen)
        :returns:   Gelesener Wert oder None
        """
        ...

    async def zenoh_put(self, key: str, value: Any) -> None:
        """
        Schreibt einen Wert auf den Zenoh-Bus. Scope wird vom Host geprüft.

        :param key:   Zenoh-Key-Expression
        :param value: Zu schreibender Wert (wird JSON-serialisiert)
        """
        ...

    async def log(self, level: str, message: str) -> None:
        """
        Logging aus dem Plugin heraus.

        :param level:   Log-Level ('debug', 'info', 'warning', 'error')
        :param message: Log-Nachricht
        """
        ...


class NullHostFunctions:
    """
    No-op Implementierung für Tests und Stub-Modus.
    Alle Operationen werden geloggt aber nicht ausgeführt.
    """

    async def notify_ui(self, event_name: str, data: dict[str, Any]) -> None:
        logger.debug("[NullHostFunctions] notify_ui(%s, %s)", event_name, data)

    async def zenoh_get(self, key: str) -> Any:
        logger.debug("[NullHostFunctions] zenoh_get(%s)", key)
        return None

    async def zenoh_put(self, key: str, value: Any) -> None:
        logger.debug("[NullHostFunctions] zenoh_put(%s, %s)", key, value)

    async def log(self, level: str, message: str) -> None:
        getattr(logger, level, logger.info)("[plugin] %s", message)
