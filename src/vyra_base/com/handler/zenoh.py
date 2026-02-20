"""
Zenoh transport handler for VYRA feeders.

Publishers are created via
:func:`~vyra_base.com.core.factory.InterfaceFactory.create_publisher` using
:attr:`~vyra_base.com.core.types.ProtocolType.ZENOH`.  This keeps the
Zenoh session management inside the CAL layer (``t_zenoh`` provider) and
decoupled from the handler.
"""

from __future__ import annotations

import logging
from logging import LogRecord
from typing import Any

from vyra_base.com.handler.communication import CommunicationHandler
from vyra_base.com.core.types import VyraPublisher
from vyra_base.helper.error_handler import ErrorTraceback

logger = logging.getLogger(__name__)


class ZenohHandler(CommunicationHandler):
    """Feeder handler that publishes messages over the Zenoh protocol.

    The internal ``VyraPublisher`` is created externally (by
    :class:`~vyra_base.com.handler.factory.HandlerFactory`) using
    :func:`~vyra_base.com.core.factory.InterfaceFactory.create_publisher`
    with ``protocols=[ProtocolType.ZENOH]``.  This handler only wraps the
    publisher to bridge the feeder / logging interface.

    :cvar __handlerName__: Identifies this handler as ``"ZenohHandler"``.
    :param initiator: Name of the feeder that owns this handler.
    :type initiator: str
    :param publisher: Pre-created Zenoh ``VyraPublisher``.
    :type publisher: VyraPublisher
    :param type: Expected message type (used for logging / validation).
    :type type: Any
    """

    __handlerName__: str = 'ZenohHandler'
    __doc__: str = 'Zenoh transport handler'

    def __init__(self, initiator: str, publisher: VyraPublisher, type: Any):
        self._initiator = initiator
        self._publisher: VyraPublisher = publisher
        self._type: Any = type
        super().__init__()

    # ------------------------------------------------------------------
    # IFeederHandler implementation
    # ------------------------------------------------------------------

    def get_protocol(self) -> str:
        """Return ``"zenoh"``."""
        return "zenoh"

    def is_available(self) -> bool:
        """Return ``True`` if the publisher is set and connected."""
        return self._publisher is not None

    async def dispatch(self, message: Any) -> None:
        """Publish *message* via the Zenoh CAL publisher.

        :param message: Domain object to publish.  Must be compatible with
            the message type configured for the topic.
        :type message: Any
        """
        try:
            logger.debug(
                "%s → %s publishing via Zenoh: %s",
                self._initiator,
                ZenohHandler.__handlerName__,
                message,
            )
            await self._publisher.publish(message)
        finally:
            ErrorTraceback.check_error_exist()

    # ------------------------------------------------------------------
    # logging.Handler override — emit record.msg directly
    # ------------------------------------------------------------------

    async def emit(self, record: LogRecord) -> None:  # type: ignore[override]
        """Publish the raw ``record.msg`` object (not the formatted string).

        Zenoh topics carry typed domain objects, not plain log strings.

        :param record: Python log record whose ``msg`` is a domain object.
        :type record: LogRecord
        """
        await self.dispatch(record.msg)
