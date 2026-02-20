"""
Abstract interfaces for VYRA feeder handlers.

This module defines the ``IFeederHandler`` abstract base class that all
communication handlers must implement.  Transport handlers (ROS2, Zenoh,
Redis, UDS) create their internal publisher via
:func:`~vyra_base.com.core.factory.InterfaceFactory.create_publisher`,
keeping the CAL layer fully decoupled from handler logic.

Design decisions
----------------
* **Hybrid logging.Handler + async dispatch**: Handlers extend Python's
  :class:`logging.Handler` so they can still be attached to Python loggers
  (feeder logging pipeline) *and* offer a proper ``async dispatch()`` for
  direct message transport — no breaking change for existing code.
* **Protocol identification**: Every handler declares its
  :class:`~vyra_base.com.core.types.ProtocolType`, enabling
  :class:`~vyra_base.com.handler.factory.HandlerFactory` to select the
  correct implementation at runtime.
* **Availability check**: ``is_available()`` lets the feeder probe whether a
  handler's transport backend is reachable before flushing buffered messages.
"""

from __future__ import annotations

import abc
import logging
from typing import Any, Optional


class IFeederHandler(logging.Handler, abc.ABC):
    """Abstract base class for all VYRA feeder handlers.

    Every concrete handler must implement :meth:`dispatch` (async direct
    transport) and :meth:`get_protocol`.  The :meth:`emit` method bridges the
    Python logging pipeline to :meth:`dispatch` so that handlers can be
    attached directly to a Python :class:`logging.Logger`.

    Subclasses **must not** change the ``emit`` → ``dispatch`` delegation
    unless there is a specific reason (e.g. ``DBCommunicationHandler`` logs
    the formatted string rather than the raw message object).

    :cvar __handlerName__: Human-readable handler identifier.  Set in each
        subclass.
    :cvar __doc__: One-line description of what this handler transports.
    """

    __handlerName__: str = "AbstractHandler"

    # ------------------------------------------------------------------
    # Abstract interface — every handler must implement these
    # ------------------------------------------------------------------

    @abc.abstractmethod
    async def dispatch(self, message: Any) -> None:
        """Transport *message* over the backing protocol.

        This is the primary entry-point for the feeder.  The feeder calls
        ``dispatch`` directly (bypassing the logging pipeline) when it already
        has a fully-formed domain object (e.g. ``StateEntry``).

        :param message: Domain object or raw value to transport.
        :type message: Any
        :raises HandlerDispatchError: If the transport operation fails
            unrecoverably.
        """

    @abc.abstractmethod
    def get_protocol(self) -> str:
        """Return the :class:`~vyra_base.com.core.types.ProtocolType` value
        this handler uses (e.g. ``"ros2"``, ``"zenoh"``, …).

        Using ``str`` as return type keeps the module importable without
        the full ``vyra_base.com.core.types`` dependency tree.

        :rtype: str
        """

    # ------------------------------------------------------------------
    # Optional overrides — sensible defaults provided
    # ------------------------------------------------------------------

    def get_handler_name(self) -> str:
        """Return the handler's human-readable name.

        Defaults to :attr:`__handlerName__`.

        :rtype: str
        """
        return self.__handlerName__

    def is_available(self) -> bool:
        """Check whether the backing transport is currently reachable.

        The default implementation always returns ``True``.  Override in
        transport handlers to probe the actual protocol connection.

        :rtype: bool
        """
        return True

    # ------------------------------------------------------------------
    # logging.Handler bridge — delegates emit → dispatch
    # ------------------------------------------------------------------

    def emit(self, record: logging.LogRecord) -> None:
        """Bridge Python logging to :meth:`dispatch`.

        Called by the Python logging framework.  Schedules :meth:`dispatch`
        via :func:`asyncio.get_event_loop` so that the synchronous logging
        call does not block the event loop.

        Subclasses that need a different bridging strategy (e.g. formatting
        the record as a plain string for a database) should override this
        method directly.

        :param record: Python :class:`~logging.LogRecord` to emit.
        :type record: logging.LogRecord
        """
        import asyncio

        try:
            loop = asyncio.get_event_loop()
            if loop.is_running():
                loop.create_task(self.dispatch(record.msg))
            else:
                loop.run_until_complete(self.dispatch(record.msg))
        except Exception:
            self.handleError(record)
