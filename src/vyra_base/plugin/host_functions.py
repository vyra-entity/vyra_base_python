"""
vyra_base.plugin.host_functions
================================

Protokoll-Klasse und abstrakte Basisklasse für Host-Funktionen, die in die
WASM-Sandbox injiziert werden. Ein Modul, das Plugins hostet (z.B. v2_modulemanager),
muss eine konkrete Implementierung bereitstellen.

WASM-Plugins können ausschließlich über diese Funktionen mit dem Host interagieren
(kein direktes Filesystem, kein direktes Network-Access aus dem WASM-Kontext).

Kommunikation läuft ausschließlich über die InterfaceFactory (create_publisher,
create_subscriber, create_server, create_client) — kein direkter Zenoh-Zugriff.
Der PluginFacade sitzt oberhalb und prüft Berechtigungen aus metadata.json.

Klassen:
    HostFunctions       — Protocol (structural typing, für isinstance-Checks)
    BaseHostFunctions   — Abstrakte Basisklasse für eigene Implementierungen;
                          implementiert log() konkret, rest bleibt abstrakt
    NullHostFunctions   — No-op Implementierung für Tests

Für ein Modul das Plugins hostet::

    from vyra_base.plugin.host_functions import BaseHostFunctions
    from vyra_base.com.core.factory import InterfaceFactory

    class MyModuleHostFunctions(BaseHostFunctions):
        def __init__(self, plugin_event_publisher):
            super().__init__()
            self._publisher = plugin_event_publisher

        async def notify_ui(self, event_name: str, data: dict) -> None:
            await self._publisher.publish({
                "event_name": event_name,
                "plugin_id":  self.plugin_id,
                "data":       data,
            })

        async def create_publisher(self, name, module_name, module_id=None, **kwargs):
            return await InterfaceFactory.create_publisher(name, module_id=module_id,
                                                           module_name=module_name, **kwargs)

        async def create_subscriber(self, name, callback, module_name, module_id=None, **kwargs):
            return await InterfaceFactory.create_subscriber(name, subscriber_callback=callback,
                                                            module_id=module_id,
                                                            module_name=module_name, **kwargs)

        async def create_server(self, name, callback, **kwargs):
            return await InterfaceFactory.create_server(name, response_callback=callback, **kwargs)

        async def create_client(self, name, module_name, module_id=None, **kwargs):
            return await InterfaceFactory.create_client(name, module_id=module_id,
                                                        module_name=module_name, **kwargs)
"""

from __future__ import annotations

import logging
from abc import ABC, abstractmethod
from typing import Any, Callable, Optional, Protocol, runtime_checkable

logger = logging.getLogger(__name__)


@runtime_checkable
class HostFunctions(Protocol):
    """
    Protokoll für Host-Funktionen einer Plugin-Runtime.

    Jede Methode entspricht einer Funktion, die in die WASM-Sandbox exportiert wird.
    Implementierungen müssen alle Methoden bereitstellen.

    Kommunikation erfolgt über InterfaceFactory-Methoden (kein direkter Zenoh-Zugriff).
    Der PluginFacade prüft Berechtigungen vor Delegierung an BaseHostFunctions.
    """

    async def notify_ui(self, event_name: str, data: dict[str, Any]) -> None:
        """
        Sendet ein Event an das Modul-Frontend über Vyra-Interface.

        :param event_name: Eindeutiger Event-Name (z.B. "counter.updated")
        :param data:       Beliebige JSON-serialisierbare Nutzdaten
        """
        ...

    async def create_publisher(
        self,
        name: str,
        module_name: str,
        module_id: Optional[str] = None,
        **kwargs: Any,
    ) -> Any:
        """
        Erstellt einen Publisher über InterfaceFactory.

        :param name:        Interface-Name
        :param module_name: Ziel-Modulname
        :param module_id:   Optionale Modul-ID (spezifische Instanz)
        """
        ...

    async def create_subscriber(
        self,
        name: str,
        callback: Callable,
        module_name: str,
        module_id: Optional[str] = None,
        **kwargs: Any,
    ) -> Any:
        """
        Erstellt einen Subscriber über InterfaceFactory.

        :param name:        Interface-Name
        :param callback:    Async-Callback für eingehende Nachrichten
        :param module_name: Quell-Modulname
        :param module_id:   Optionale Modul-ID
        """
        ...

    async def create_server(
        self,
        name: str,
        callback: Callable,
        **kwargs: Any,
    ) -> Any:
        """
        Erstellt einen Service-Server über InterfaceFactory.

        :param name:     Service-Name
        :param callback: Async-Callback für eingehende Requests
        """
        ...

    async def create_client(
        self,
        name: str,
        module_name: str,
        module_id: Optional[str] = None,
        **kwargs: Any,
    ) -> Any:
        """
        Erstellt einen Service-Client über InterfaceFactory.

        :param name:        Service-Name
        :param module_name: Ziel-Modulname
        :param module_id:   Optionale Modul-ID
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

    async def create_publisher(self, name: str, module_name: str, module_id: Optional[str] = None, **kwargs: Any) -> None:
        logger.debug("[NullHostFunctions] create_publisher(%s, %s)", name, module_name)

    async def create_subscriber(self, name: str, callback: Callable, module_name: str, module_id: Optional[str] = None, **kwargs: Any) -> None:
        logger.debug("[NullHostFunctions] create_subscriber(%s, %s)", name, module_name)

    async def create_server(self, name: str, callback: Callable, **kwargs: Any) -> None:
        logger.debug("[NullHostFunctions] create_server(%s)", name)

    async def create_client(self, name: str, module_name: str, module_id: Optional[str] = None, **kwargs: Any) -> None:
        logger.debug("[NullHostFunctions] create_client(%s, %s)", name, module_name)

    async def log(self, level: str, message: str) -> None:
        getattr(logger, level, logger.info)("[plugin] %s", message)


class BaseHostFunctions(ABC):
    """
    Abstrakte Basisklasse für modul-spezifische HostFunctions-Implementierungen.

    Implementiert `log()` konkret über Python-Logging.
    `notify_ui`, `create_publisher`, `create_subscriber`, `create_server`,
    `create_client` müssen von der Unterklasse implementiert werden.

    Kommunikation läuft ausschließlich über die InterfaceFactory — kein direkter
    Zenoh-Zugriff. Der PluginFacade (vyra_base.plugin.plugin_facade) sitzt
    oberhalb und prüft Berechtigungen aus metadata.json vor jeder Operation.

    Interface-Referenz:
        Der `plugin_event`-Publisher wird in
        ``vyra_base/interfaces/config/vyra_plugin.meta.json`` beschrieben.
        Modulspezifische Services stehen in
        ``{module}_interfaces/config/{module}_plugin.meta.json``.

    Beispiel::

        class MyModuleHostFunctions(BaseHostFunctions):
            def __init__(self, plugin_event_publisher):
                super().__init__()
                self._pub = plugin_event_publisher

            async def notify_ui(self, event_name: str, data: dict) -> None:
                await self._pub.publish({"event_name": event_name, "data": data})

            async def create_publisher(self, name, module_name, module_id=None, **kw):
                return await InterfaceFactory.create_publisher(
                    name, module_id=module_id, module_name=module_name, **kw)

            async def create_subscriber(self, name, callback, module_name, module_id=None, **kw):
                return await InterfaceFactory.create_subscriber(
                    name, subscriber_callback=callback,
                    module_id=module_id, module_name=module_name, **kw)

            async def create_server(self, name, callback, **kw):
                return await InterfaceFactory.create_server(
                    name, response_callback=callback, **kw)

            async def create_client(self, name, module_name, module_id=None, **kw):
                return await InterfaceFactory.create_client(
                    name, module_id=module_id, module_name=module_name, **kw)
    """

    @abstractmethod
    async def notify_ui(self, event_name: str, data: dict[str, Any]) -> None:
        """
        Sendet ein Event an das Frontend über den plugin_event Publisher.

        :param event_name: Generischer Event-Name, z.B. 'plugin.increment.result'
        :param data:       JSON-serialisierbare Nutzdaten
        """
        ...

    @abstractmethod
    async def create_publisher(
        self,
        name: str,
        module_name: str,
        module_id: Optional[str] = None,
        **kwargs: Any,
    ) -> Any:
        """Erstellt einen Publisher via InterfaceFactory."""
        ...

    @abstractmethod
    async def create_subscriber(
        self,
        name: str,
        callback: Callable,
        module_name: str,
        module_id: Optional[str] = None,
        **kwargs: Any,
    ) -> Any:
        """Erstellt einen Subscriber via InterfaceFactory."""
        ...

    @abstractmethod
    async def create_server(
        self,
        name: str,
        callback: Callable,
        **kwargs: Any,
    ) -> Any:
        """Erstellt einen Service-Server via InterfaceFactory."""
        ...

    @abstractmethod
    async def create_client(
        self,
        name: str,
        module_name: str,
        module_id: Optional[str] = None,
        **kwargs: Any,
    ) -> Any:
        """Erstellt einen Service-Client via InterfaceFactory.

        :param name:        Service-Name
        :param module_name: Ziel-Modulname
        :param module_id:   Optionale Modul-ID
        """
        ...

    async def log(self, level: str, message: str) -> None:
        """
        Logging aus dem Plugin heraus — konkret implementiert via Python-Logger.

        :param level:   Log-Level ('debug', 'info', 'warning', 'error')
        :param message: Log-Nachricht
        """
        getattr(logger, level, logger.info)("[plugin] %s", message)
