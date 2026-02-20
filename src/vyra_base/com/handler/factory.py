"""
Factory for creating VYRA feeder handlers.

``HandlerFactory`` is the single entry-point for constructing any
:class:`~vyra_base.com.handler.interfaces.IFeederHandler` instance.

Transport handlers (ROS2, Zenoh, Redis, UDS)
---------------------------------------------
Their internal :class:`~vyra_base.com.core.types.VyraPublisher` is **always**
created through
:func:`~vyra_base.com.core.factory.InterfaceFactory.create_publisher`,
keeping the CAL transport layer fully encapsulated.  The handler itself is
only responsible for bridging the feeder / logging interface to that
publisher.

Non-transport handlers (Database)
-----------------------------------
Receive dedicated configuration objects (e.g. a database connection object)
rather than a ``VyraPublisher``.

Usage::

    from vyra_base.com.handler.factory import HandlerFactory
    from vyra_base.com.core.types import ProtocolType

    handler = await HandlerFactory.create(
        protocol=ProtocolType.ZENOH,
        initiator="StateFeeder",
        feeder_name="StateFeeder",
        message_type=my_proto_type,
    )
"""

from __future__ import annotations

import logging
from typing import Any, Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from vyra_base.com.handler.interfaces import IFeederHandler

logger = logging.getLogger(__name__)


class HandlerFactory:
    """Factory that creates :class:`~vyra_base.com.handler.interfaces.IFeederHandler`
    instances for a given :class:`~vyra_base.com.core.types.ProtocolType`.

    All transport handlers use
    :func:`~vyra_base.com.core.factory.InterfaceFactory.create_publisher`
    to obtain their ``VyraPublisher`` — the factory does not touch provider
    internals directly.
    """

    @staticmethod
    async def create(
        protocol: Any,  # ProtocolType (str enum) — avoid hard import cycle
        initiator: str,
        feeder_name: str,
        message_type: Any,
        *,
        node: Optional[Any] = None,
        qos_profile: Optional[Any] = None,
        database: Optional[Any] = None,
        extra_kwargs: Optional[dict] = None,
    ) -> "IFeederHandler":
        """Create a handler for the given *protocol*.

        Transport handler creation flow:

        .. code-block:: text

            HandlerFactory.create(ProtocolType.ZENOH, ...)
              └─► InterfaceFactory.create_publisher(protocols=[ZENOH], ...)
                    └─► ZenohProvider.create_publisher(...)   ← t_zenoh/provider.py
                          └─► ZenohPublisherImpl(...)         ← t_zenoh/vyra_models/
              └─► ZenohHandler(initiator, publisher, message_type)

        :param protocol: Transport protocol to use.  One of
            :class:`~vyra_base.com.core.types.ProtocolType`.
        :type protocol: ProtocolType
        :param initiator: Name of the feeder owning this handler (used in
            log messages).
        :type initiator: str
        :param feeder_name: Topic / service name forwarded to
            ``InterfaceFactory.create_publisher``.
        :type feeder_name: str
        :param message_type: Message type for the publisher (e.g.
            ROS2 msg class, protobuf class, or ``None`` for dict-based
            protocols).
        :type message_type: Any
        :param node: ROS2 node (required only for ``ProtocolType.ROS2``).
        :type node: Any, optional
        :param qos_profile: ROS2 QoS profile (optional, ROS2 only).
        :type qos_profile: Any, optional
        :param database: Database connection/handler (required only for
            ``ProtocolType.DATABASE`` / DB handlers).
        :type database: Any, optional
        :param extra_kwargs: Extra keyword arguments forwarded to
            ``InterfaceFactory.create_publisher``.
        :type extra_kwargs: dict, optional
        :return: A fully initialised handler instance ready to be added
            to a feeder.
        :rtype: IFeederHandler
        :raises ValueError: If an unsupported *protocol* is given.
        :raises RuntimeError: If the publisher cannot be created.
        """
        from vyra_base.com.core.factory import InterfaceFactory
        from vyra_base.com.core.types import ProtocolType

        extra_kwargs = extra_kwargs or {}
        protocol_str = str(protocol.value) if hasattr(protocol, "value") else str(protocol)

        # ----------------------------------------------------------------
        # Transport handlers — publisher via InterfaceFactory
        # ----------------------------------------------------------------
        if protocol_str in (ProtocolType.ROS2.value, ProtocolType.ZENOH.value,
                            ProtocolType.REDIS.value, ProtocolType.UDS.value):

            pub_kwargs: dict[str, Any] = {
                "name": feeder_name,
                "protocols": [protocol],
                "message_type": message_type,
                **extra_kwargs,
            }
            if protocol_str == ProtocolType.ROS2.value and node is not None:
                pub_kwargs["node"] = node
                if qos_profile is not None:
                    pub_kwargs["qos_profile"] = qos_profile

            try:
                publisher = await InterfaceFactory.create_publisher(**pub_kwargs)
            except Exception as exc:
                raise RuntimeError(
                    f"HandlerFactory: could not create publisher for "
                    f"feeder='{feeder_name}' protocol='{protocol_str}': {exc}"
                ) from exc

            if protocol_str == ProtocolType.ROS2.value:
                from vyra_base.com.handler.ros2 import ROS2Handler
                return ROS2Handler(initiator=initiator, publisher=publisher, type=message_type)

            if protocol_str == ProtocolType.ZENOH.value:
                from vyra_base.com.handler.zenoh import ZenohHandler
                return ZenohHandler(initiator=initiator, publisher=publisher, type=message_type)

            if protocol_str == ProtocolType.REDIS.value:
                from vyra_base.com.handler.redis import RedisHandler
                return RedisHandler(initiator=initiator, publisher=publisher, type=message_type)

            if protocol_str == ProtocolType.UDS.value:
                from vyra_base.com.handler.uds import UDSHandler
                return UDSHandler(initiator=initiator, publisher=publisher, type=message_type)

        # ----------------------------------------------------------------
        # Database handler — no publisher
        # ----------------------------------------------------------------
        if protocol_str == "database":
            from vyra_base.com.handler.database import DBCommunicationHandler
            if database is None:
                raise ValueError(
                    "HandlerFactory: 'database' must be provided for protocol='database'."
                )
            return DBCommunicationHandler(database=database)

        raise ValueError(
            f"HandlerFactory: unsupported protocol '{protocol_str}'. "
            f"Supported: ros2, zenoh, redis, uds, database."
        )
