"""
Feeder registry for VYRA custom feeders.

``FeederRegistry`` is a thread-safe singleton that keeps track of all
registered :class:`~vyra_base.com.feeder.interfaces.IFeeder` subclasses.
Use the :func:`register_feeder` decorator to register a feeder class so the
framework (and downstream tooling) can discover it by name.

Usage::

    from vyra_base.com.feeder.registry import register_feeder, FeederRegistry
    from vyra_base.com.feeder.custom_feeder import CustomBaseFeeder

    @register_feeder("TemperatureFeed")
    class TemperatureFeeder(CustomBaseFeeder):
        ...

    # Discover & instantiate
    cls = FeederRegistry.get("TemperatureFeed")
    feeder = cls(...)
"""

from __future__ import annotations

import logging
import threading
from typing import Optional, Type

from vyra_base.com.feeder.interfaces import IFeeder

logger = logging.getLogger(__name__)

_registry_lock = threading.Lock()


class FeederRegistry:
    """Thread-safe registry of :class:`~vyra_base.com.feeder.interfaces.IFeeder`
    subclasses.

    All built-in feeders (StateFeeder, NewsFeeder, ErrorFeeder) are NOT
    pre-registered here — the registry is exclusively for user-defined
    custom feeders added via :func:`register_feeder`.
    """

    _instance: dict[str, Type[IFeeder]] = {}

    @classmethod
    def register(cls, name: str, feeder_class: Type[IFeeder]) -> None:
        """Register *feeder_class* under *name*.

        :param name: Unique key (normally the ``functionname`` from the
            interface config, e.g. ``"TemperatureFeed"``).
        :type name: str
        :param feeder_class: The feeder class to register.  Must be a
            subclass of :class:`~vyra_base.com.feeder.interfaces.IFeeder`.
        :type feeder_class: Type[IFeeder]
        :raises TypeError: If *feeder_class* does not subclass ``IFeeder``.
        :raises ValueError: If *name* is already registered (use
            ``override=True`` to replace).
        """
        if not (isinstance(feeder_class, type) and issubclass(feeder_class, IFeeder)):
            raise TypeError(
                f"FeederRegistry.register: '{feeder_class}' must be a subclass of IFeeder."
            )
        with _registry_lock:
            if name in cls._instance:
                logger.warning(
                    "FeederRegistry: '%s' is already registered (%s). Overwriting.",
                    name, cls._instance[name].__name__
                )
            cls._instance[name] = feeder_class
            logger.debug("FeederRegistry: registered '%s' → %s", name, feeder_class.__name__)

    @classmethod
    def get(cls, name: str) -> Optional[Type[IFeeder]]:
        """Return the feeder class registered under *name*, or ``None``.

        :param name: Feeder name to look up.
        :type name: str
        :rtype: Optional[Type[IFeeder]]
        """
        return cls._instance.get(name)

    @classmethod
    def list_feeders(cls) -> list[str]:
        """Return a sorted list of all registered feeder names.

        :rtype: list[str]
        """
        with _registry_lock:
            return sorted(cls._instance.keys())

    @classmethod
    def unregister(cls, name: str) -> bool:
        """Remove the feeder registered under *name*.

        :param name: Feeder name to remove.
        :type name: str
        :return: ``True`` if the entry existed and was removed.
        :rtype: bool
        """
        with _registry_lock:
            if name in cls._instance:
                del cls._instance[name]
                logger.debug("FeederRegistry: unregistered '%s'.", name)
                return True
        return False


def register_feeder(name: str):
    """Class decorator that registers a feeder in :class:`FeederRegistry`.

    The decorated class must be a subclass of
    :class:`~vyra_base.com.feeder.interfaces.IFeeder` (or
    :class:`~vyra_base.com.feeder.custom_feeder.CustomBaseFeeder`).

    :param name: The name under which this feeder is registered.  Should
        match the ``functionname`` in the module's interface config JSON so
        that the framework can select the correct protocol automatically.
    :type name: str
    :raises TypeError: If the decorated class is not an ``IFeeder`` subclass.

    Example::

        @register_feeder("TemperatureFeed")
        class TemperatureFeeder(CustomBaseFeeder):
            pass
    """
    def decorator(cls: Type[IFeeder]) -> Type[IFeeder]:
        FeederRegistry.register(name, cls)
        return cls
    return decorator
