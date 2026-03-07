"""
vyra_base.plugin.host_functions
================================

Protokoll-Klasse und abstrakte Basisklasse für Host-Funktionen, die in die
WASM-Sandbox injiziert werden. Ein Modul, das Plugins hostet (z.B. v2_modulemanager),
muss eine konkrete Implementierung bereitstellen.

WASM-Plugins können ausschließlich über diese Funktionen mit dem Host interagieren
(kein direktes Filesystem, kein direktes Network-Access aus dem WASM-Kontext).

Klassen:
    HostFunctions       — Protocol (structural typing, für isinstance-Checks)
    BaseHostFunctions   — Abstrakte Basisklasse für eigene Implementierungen;
                          implementiert log() konkret, rest bleibt abstrakt
    NullHostFunctions   — No-op Implementierung für Tests

Für ein Modul das Plugins hostet::

    from vyra_base.plugin.host_functions import BaseHostFunctions

    class MyModuleHostFunctions(BaseHostFunctions):
        def __init__(self, zenoh_publisher, zenoh_session):
            super().__init__()
            self._publisher = zenoh_publisher
            self._session   = zenoh_session

        async def notify_ui(self, event_name: str, data: dict) -> None:
            await self._publisher.publish({
                "event_name": event_name,
                "plugin_id":  self.plugin_id,
                "data":       data,
            })

        async def zenoh_get(self, key: str):
            return await self._session.get(key)

        async def zenoh_put(self, key: str, value) -> None:
            await self._session.put(key, value)
"""

from __future__ import annotations

import logging
from abc import ABC, abstractmethod
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


class BaseHostFunctions(ABC):
    """
    Abstrakte Basisklasse für modul-spezifische HostFunctions-Implementierungen.

    Implementiert `log()` konkret über Python-Logging.
    `notify_ui`, `zenoh_get` und `zenoh_put` müssen von der Unterklasse
    implementiert werden — typischerweise über einen VyraPublisher bzw.
    eine Zenoh-Session aus dem Modul-Kontext.

    Zenoh-Interface-Referenz:
        Der `plugin_event`-Publisher wird in
        ``vyra_base/interfaces/config/vyra_plugin.meta.json`` (type: message,
        namespace: plugin, functionname: plugin_event) beschrieben.
        Modulspezifische Services werden in der
        ``{module}_interfaces/config/{module}_plugin.meta.json`` definiert.

    Beispiel::

        class MyModuleHostFunctions(BaseHostFunctions):
            def __init__(self, plugin_event_publisher, zenoh_session):
                super().__init__()
                self._pub     = plugin_event_publisher
                self._session = zenoh_session

            async def notify_ui(self, event_name: str, data: dict) -> None:
                await self._pub.publish({
                    "event_name": event_name,
                    "plugin_id":  "<plugin_id>",
                    "data":       data,
                })

            async def zenoh_get(self, key: str):
                return await self._session.get(key)

            async def zenoh_put(self, key: str, value) -> None:
                await self._session.put(key, value)
    """

    @abstractmethod
    async def notify_ui(self, event_name: str, data: dict[str, Any]) -> None:
        """
        Sendet ein Event an das Frontend über Zenoh (plugin_event Publisher).

        :param event_name: Generischer Event-Name, z.B. 'plugin.increment.result'
        :param data:       JSON-serialisierbare Nutzdaten
        """
        ...

    @abstractmethod
    async def zenoh_get(self, key: str) -> Any:
        """
        Liest einen Wert vom Zenoh-Bus. Scope-Validierung liegt beim Aufrufer.

        :param key: Zenoh-Key-Expression
        :returns:   Gelesener Wert oder None
        """
        ...

    @abstractmethod
    async def zenoh_put(self, key: str, value: Any) -> None:
        """
        Schreibt einen Wert auf den Zenoh-Bus. Scope-Validierung liegt beim Aufrufer.

        :param key:   Zenoh-Key-Expression
        :param value: Zu schreibender Wert (wird JSON-serialisiert)
        """
        ...

    async def log(self, level: str, message: str) -> None:
        """
        Logging aus dem Plugin heraus — konkret implementiert via Python-Logger.

        :param level:   Log-Level ('debug', 'info', 'warning', 'error')
        :param message: Log-Nachricht
        """
        getattr(logger, level, logger.info)("[plugin] %s", message)
