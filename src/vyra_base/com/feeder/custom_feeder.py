"""
Custom feeder base class for VYRA application developers.

``CustomBaseFeeder`` is the recommended starting point when you need to
publish proprietary domain data (e.g. sensor readings, machine parameters,
alarm setpoints) over the VYRA transport layer.

Quick start::

    from vyra_base.com.feeder.custom_feeder import CustomBaseFeeder
    from vyra_base.com.feeder.registry import register_feeder
    from vyra_base.defaults.entries import ModuleEntry

    @register_feeder("TemperatureFeed")
    class TemperatureFeeder(CustomBaseFeeder):
        \"\"\"Publishes temperature readings from a PLC.\"\"\"

        def _build_message(self, raw: float) -> dict:
            return {"value": raw, "unit": "°C", "sensor": self._sensor_id}

        def _validate(self, raw: float) -> bool:
            return -50.0 <= raw <= 300.0

    # Usage in your module application:
    feeder = TemperatureFeeder(
        feeder_name="TemperatureFeed",
        module_entity=my_module_entity,
    )
    await feeder.start()
    await feeder.feed(87.3)     # publishes {"value": 87.3, "unit": "°C", "sensor": "PT-01"}

The protocol is resolved automatically from the interface config JSON — add
a ``"type": "publisher", "functionname": "TemperatureFeed"`` entry with the
appropriate ``tags`` (e.g. ``["zenoh"]``).
"""

from __future__ import annotations

import logging
from typing import Any, Optional

from vyra_base.com.feeder.feeder import BaseFeeder
from vyra_base.defaults.entries import ModuleEntry
from vyra_base.defaults.exceptions import FeederException
from vyra_base.com.core.interface_path_registry import InterfacePathRegistry, get_interface_registry

logger = logging.getLogger(__name__)


class CustomBaseFeeder(BaseFeeder):
    """Base class for user-defined VYRA feeders.

    Extends :class:`~vyra_base.com.feeder.feeder.BaseFeeder` with two
    hooks that subclasses can override to implement custom message mapping
    and validation without touching the transport layer.

    :param feeder_name: The feeder's name.  **Must** match the
        ``functionname`` in the module's interface config JSON so that the
        protocol resolver can find the correct transport entry.
    :type feeder_name: str
    :param module_entity: Module configuration (name, uuid, …).
    :type module_entity: ModuleEntry
    :param message_type: Optional message class forwarded to the publisher
        (e.g. a protobuf class).  Pass ``None`` for dict-based protocols.
    :type message_type: Any, optional
    :param node: ROS2 node (required only when using the ROS2 transport).
    :type node: Any, optional
    :param loggingOn: If ``True``, every :meth:`feed` call is also logged
        via the standard Python logger.
    :type loggingOn: bool, optional
    """

    def __init__(
        self,
        feeder_name: str,
        module_entity: ModuleEntry,
        message_type: Any = None,
        node: Optional[Any] = None,
        loggingOn: bool = False,
    ):
        super().__init__()
        self._feederName: str = feeder_name
        self._module_entity: ModuleEntry = module_entity
        self._type: Any = message_type
        self._node: Optional[Any] = node
        self._loggingOn: bool = loggingOn

    # ------------------------------------------------------------------
    # Hooks — override in subclasses
    # ------------------------------------------------------------------

    def _build_message(self, raw: Any) -> Any:
        """Transform *raw* into the domain object expected by the publisher.

        The default implementation returns *raw* unchanged.  Override to
        map your input type to e.g. a protobuf message, a dataclass, or a
        dict.

        :param raw: The raw value passed to :meth:`feed`.
        :type raw: Any
        :return: The domain object to publish.
        :rtype: Any
        """
        return raw

    def _validate(self, raw: Any) -> bool:
        """Validate *raw* before publishing.

        The default implementation always returns ``True``.  Override to add
        range checks, type checks, or business-rule validation.  If this
        method returns ``False`` the message is **not** published and a
        warning is logged.

        :param raw: The raw value passed to :meth:`feed`.
        :type raw: Any
        :return: ``True`` if the value should be published.
        :rtype: bool
        """
        return True

    # ------------------------------------------------------------------
    # IFeeder.feed override — applies hooks before publishing
    # ------------------------------------------------------------------

    async def feed(self, raw: Any) -> None:  # type: ignore[override]
        """Validate, transform, and publish *raw*.

        1. Call :meth:`_validate` — skip on ``False``.
        2. Call :meth:`_build_message` — transform to domain object.
        3. Delegate to :meth:`~vyra_base.com.feeder.feeder.BaseFeeder.feed`.

        :param raw: The raw value to validate, transform, and publish.
        :type raw: Any
        """
        if not self._validate(raw):
            logger.warning(
                "⚠️ %s: validation failed for message '%s', skipping.",
                self._feederName, raw
            )
            return

        try:
            message = self._build_message(raw)
        except Exception as exc:
            logger.error(
                "❌ %s: _build_message failed: %s", self._feederName, exc
            )
            self._error_count += 1
            return

        await super().feed(message)

    # ------------------------------------------------------------------
    # start() — resolve interface paths from registry
    # ------------------------------------------------------------------

    async def start(self) -> None:
        """Resolve protocol from interface config and start the feeder."""
        paths = get_interface_registry().get_interface_paths()
        if paths:
            self.set_interface_paths(paths)
        await self.create(loggingOn=self._loggingOn)
