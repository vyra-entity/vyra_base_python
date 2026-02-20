"""
Abstract interface for VYRA feeders.

``IFeeder`` is the contracts all feeders must satisfy.  It is kept minimal
so that both built-in feeders (StateFeeder, NewsFeeder, ErrorFeeder) and
user-defined custom feeders can implement it without inheriting a large base
class.
"""

from __future__ import annotations

import abc
from collections import deque
from typing import Any, Optional


class IFeeder(abc.ABC):
    """Abstract base class for all VYRA feeders.

    A feeder is a *publisher* that continuously pushes domain-typed data
    (state changes, news messages, errors, …) over a transport protocol
    (Zenoh, ROS2, Redis, UDS, …).

    The protocol is resolved automatically from the module's interface config
    JSON via :class:`~vyra_base.com.feeder.config_resolver.FeederConfigResolver`.

    Lifecycle::

        feeder = MyFeeder(...)
        await feeder.start()          # resolve protocol + create publisher
        feeder.feed(my_data)          # async-safe publish
        alive = feeder.is_alive()     # health check
        feeder.feed_count             # read metrics

    """

    # ------------------------------------------------------------------
    # Mandatory interface
    # ------------------------------------------------------------------

    @abc.abstractmethod
    async def start(self) -> None:
        """Initialise the feeder: resolve the transport protocol, create the
        publisher, and flush any buffered messages.

        Must be awaited before the first :meth:`feed` call.
        """

    @abc.abstractmethod
    def feed(self, message: Any) -> None:
        """Publish *message* immediately.

        If called before :meth:`start` the message is buffered and flushed
        once the publisher is ready.

        :param message: Domain object to publish.
        :type message: Any
        """

    @abc.abstractmethod
    def get_feeder_name(self) -> str:
        """Return the feeder's name (= ``functionname`` in interface config).

        :rtype: str
        """

    # ------------------------------------------------------------------
    # Optional interface — default implementations provided
    # ------------------------------------------------------------------

    def get_protocol(self) -> Optional[str]:
        """Return the resolved transport protocol string, or ``None`` if the
        feeder has not been started yet.

        :rtype: Optional[str]
        """
        return None

    def is_alive(self) -> bool:
        """Return ``True`` if the feeder's publisher is ready and the backing
        transport is reachable.

        Feeders that do not implement a liveness check always return ``True``.

        :rtype: bool
        """
        return True

    def is_ready(self) -> bool:
        """Return ``True`` after :meth:`start` has completed successfully.

        :rtype: bool
        """
        return False

    def get_buffer(self) -> deque:
        """Return the internal pre-start message buffer.

        :rtype: collections.deque
        """
        return deque()

    @property
    def feed_count(self) -> int:
        """Number of messages successfully published since the feeder started.

        :rtype: int
        """
        return 0

    @property
    def error_count(self) -> int:
        """Number of publish errors since the feeder started.

        :rtype: int
        """
        return 0
