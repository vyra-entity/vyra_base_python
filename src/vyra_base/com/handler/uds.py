"""
Unix Domain Socket (UDS) transport handler for VYRA feeders.

Publishers are created via
:func:`~vyra_base.com.core.factory.InterfaceFactory.create_publisher` using
:attr:`~vyra_base.com.core.types.ProtocolType.UDS`.

UDS is ideal for high-throughput, low-latency intra-host communication
between VYRA modules running in the same Docker network namespace or on
the same physical host.
"""

from __future__ import annotations

import logging
from logging import LogRecord
from typing import Any

from vyra_base.com.handler.communication import CommunicationHandler
from vyra_base.com.core.types import VyraPublisher
from vyra_base.helper.error_handler import ErrorTraceback

logger = logging.getLogger(__name__)


class UDSHandler(CommunicationHandler):
    """Feeder handler that publishes messages over Unix Domain Sockets.

    Uses the ``t_uds`` CAL provider via a pre-created ``VyraPublisher``
    (created by :class:`~vyra_base.com.handler.factory.HandlerFactory`).
    UDS is a good fit for safety-critical, latency-sensitive events that
    must not leave the host boundary.

    :cvar __handlerName__: Identifies this handler as ``"UDSHandler"``.
    :param initiator: Name of the owning feeder.
    :type initiator: str
    :param publisher: Pre-created UDS ``VyraPublisher``.
    :type publisher: VyraPublisher
    :param type: Expected message type.
    :type type: Any
    """

    __handlerName__: str = 'UDSHandler'
    __doc__: str = 'Unix Domain Socket transport handler'

    def __init__(self, initiator: str, publisher: VyraPublisher, type: Any):
        self._initiator = initiator
        self._publisher: VyraPublisher = publisher
        self._type: Any = type
        super().__init__()

    # ------------------------------------------------------------------
    # IFeederHandler implementation
    # ------------------------------------------------------------------

    def get_protocol(self) -> str:
        """Return ``"uds"``."""
        return "uds"

    def is_available(self) -> bool:
        """Return ``True`` if the publisher is set."""
        return self._publisher is not None

    async def dispatch(self, message: Any) -> None:
        """Publish *message* via the UDS CAL publisher.

        :param message: Domain object to publish.
        :type message: Any
        """
        try:
            logger.debug(
                "%s → %s publishing via UDS: %s",
                self._initiator,
                UDSHandler.__handlerName__,
                message,
            )
            await self._publisher.publish(message)
        finally:
            ErrorTraceback.check_error_exist()

    async def emit(self, record: LogRecord) -> None:  # type: ignore[override]
        """Publish the raw ``record.msg`` domain object.

        :param record: Python log record whose ``msg`` is a domain object.
        :type record: LogRecord
        """
        await self.dispatch(record.msg)
