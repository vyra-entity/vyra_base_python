from __future__ import annotations

import asyncio
import datetime
import logging
from collections import deque
from typing import Any, Type, Optional, Sequence
from pathlib import Path

# Check ROS2 availability
try:
    import rclpy
    _ROS2_AVAILABLE = True
except ImportError:
    _ROS2_AVAILABLE = False

if _ROS2_AVAILABLE:
    from rclpy.qos import (
        QoSProfile,
        QoSHistoryPolicy,
        QoSReliabilityPolicy,
        QoSDurabilityPolicy,
    )
    from vyra_base.com import InterfaceFactory, ProtocolType

from vyra_base.com.feeder.interfaces import IFeeder
from vyra_base.com.handler.communication import CommunicationHandler
from vyra_base.defaults.exceptions import FeederException
from vyra_base.helper.logging_config import VyraLoggingConfig
from vyra_base.com.feeder.config_resolver import FeederConfigResolver
from vyra_base.com.core.types import ProtocolType as PT
from vyra_base.com.core.types import VyraPublisher
from vyra_base.com.core.interface_loader import InterfaceLoader
from vyra_base.com.feeder.message_mapper import MessageMapper

logger = logging.getLogger(__name__)


class BaseFeeder(IFeeder):
    """
    Concrete base class for all VYRA feeders.

    Implements :class:`~vyra_base.com.feeder.interfaces.IFeeder` and provides:

    * **Protocol auto-resolution** via
      :class:`~vyra_base.com.feeder.config_resolver.FeederConfigResolver` —
      the transport protocol is read from the module's interface config JSON
      (``functionname`` matched against ``feeder_name``).
    * **Pre-start buffer** — messages fed before :meth:`start` are queued and
      flushed automatically.
    * **Metrics** — ``feed_count``, ``error_count``, ``last_feed_at``.
    * **Health check** — :meth:`is_alive` probes the backing publisher.
    * **Retry policy** — configurable ``max_retries`` and ``retry_delay``.

    Abstract class — subclasses set ``_feederName``, ``_type``, optionally
    ``_interface_paths``.
    """

    def __init__(self) -> None:
        """
        Initialize the BaseFeeder.
        """
        # QoS configuration (only if ROS2 available)
        if _ROS2_AVAILABLE:
            self._qos = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
            )
        else:
            self._qos = None

        self._feedBaseName: str = 'vyraFeeder'
        self._feederName: str = 'AbstractFeeder'
        self._doc: str = 'Abstract class for all feeder classes.'
        self._level: int = logging.INFO
        self._ros2_available: bool = _ROS2_AVAILABLE

        # Interface paths for protocol resolution (set by subclasses or entity)
        self._interface_paths: list[str] = []

        self._handler_classes: list[Type[CommunicationHandler]] = []
        self._handler: list[Type[CommunicationHandler] | CommunicationHandler] = []
        # self._feeder: logging.Logger
        self._loggingOn: bool = False
        self._node: Optional[Any] = None
        self._type: Any = None
        # Resolved message/interface type (loaded from InterfaceLoader or set from self._type)
        self._msg_type: Optional[Any] = None
        self._publisher: Optional[VyraPublisher] = None
        self._feedbuffer: deque = deque(maxlen=20)

        # Resolved protocol (set after start())
        self._resolved_protocol: Optional[str] = None
        self._is_ready: bool = False

        # Metrics
        self._feed_count: int = 0
        self._error_count: int = 0
        self._last_feed_at: Optional[datetime.datetime] = None

        # Retry policy
        self._max_retries: int = 3
        self._retry_delay: float = 1.0

    # ------------------------------------------------------------------
    # IFeeder implementation
    # ------------------------------------------------------------------

    def get_feeder_name(self) -> str:
        """Return the feeder name (= ``functionname`` in interface config)."""
        return self._feederName

    def get_protocol(self) -> Optional[str]:
        """Return the resolved transport protocol, or ``None`` before start."""
        return self._resolved_protocol

    def is_alive(self) -> bool:
        """Return ``True`` if the publisher is set and available."""
        if not self._publisher:
            return False
        if hasattr(self._publisher, 'is_connected'):
            return bool(self._publisher.is_connected())
        return True

    def is_ready(self) -> bool:
        """Return ``True`` after :meth:`start` has completed successfully."""
        return self._is_ready

    def get_buffer(self) -> deque:
        """Return the pre-start message buffer."""
        return self._feedbuffer

    @property
    def feed_count(self) -> int:
        """Number of successfully published messages."""
        return self._feed_count

    @property
    def error_count(self) -> int:
        """Number of publish errors."""
        return self._error_count

    @property
    def last_feed_at(self) -> Optional[datetime.datetime]:
        """Timestamp of the last successful :meth:`feed` call."""
        return self._last_feed_at

    # ------------------------------------------------------------------
    # Initialisation helpers
    # ------------------------------------------------------------------

    def set_interface_paths(self, paths: Sequence[str]|list[Path]) -> None:
        """Override the interface paths used for protocol resolution.

        Called by a :class:`~vyra_base.core.entity.VyraEntity` after
        constructing the feeder to provide module-specific config paths.

        :param paths: List of directory or JSON file paths.
        :type paths: Sequence[str] | list[Path]
        """
        if isinstance(paths, Sequence):
            self._interface_paths = [str(p) for p in paths]
        else:             
            self._interface_paths = [str(paths)]

    async def create(self, loggingOn: bool = False) -> None:
        """Create the publisher using protocol resolved from interface config.

        Resolution order:

        1. ``FeederConfigResolver.resolve(feeder_name, interface_paths)`` —
           reads the module's JSON config, maps ``tags`` to a protocol.
        2. If no config found (or ``interface_paths`` empty): fall back to
           ``InterfaceFactory.create_publisher`` with the default chain
           ``[ZENOH, ROS2, REDIS, UDS]``.

        :raises FeederException: If no protocol is available at all.
        :param loggingOn: Emit feeder messages also to the base logger.
        :type loggingOn: bool
        """
        self._loggingOn = loggingOn

        # ── Protocol resolution via interface config ──────────────────
        resolved_protocols = None
        if self._interface_paths:
            try:
                result = FeederConfigResolver.resolve(
                    feeder_name=self._feederName,
                    interface_paths=self._interface_paths,
                )
                if result is not None:
                    self._resolved_protocol = result.protocol
                    try:
                        resolved_protocols = [PT(result.protocol)]
                    except ValueError:
                        logger.warning(
                            "⚠️ Protocol '%s' from config not a valid ProtocolType, "
                            "using fallback chain.", result.protocol
                        )
            except Exception as exc:
                logger.warning("⚠️ FeederConfigResolver error for '%s': %s — using fallback.",
                               self._feederName, exc)

        # ── Publisher creation ────────────────────────────────────────
        try:
            pub_kwargs: dict[str, Any] = {
                "name": self._feederName,
                "message_type": self._type,
                "namespace": "feeder",
            }
            if resolved_protocols:
                pub_kwargs["protocols"] = resolved_protocols
            else:
                # Default fallback chain
                if self._ros2_available:
                    pub_kwargs["protocols"] = [PT.ROS2, PT.ZENOH, PT.REDIS, PT.UDS]
                else:
                    pub_kwargs["protocols"] = [PT.ZENOH, PT.REDIS, PT.UDS]

            if self._ros2_available and self._node:
                pub_kwargs["node"] = self._node
                if self._qos:
                    pub_kwargs["qos_profile"] = self._qos

            self._publisher = await InterfaceFactory.create_publisher(**pub_kwargs)

            if self._resolved_protocol is None and hasattr(self._publisher, 'protocol'):
                self._resolved_protocol = getattr(self._publisher.protocol, 'value',
                                                   str(self._publisher.protocol))

            logger.info("✅ %s using protocol '%s'", self._feederName, self._resolved_protocol)

        except Exception as exc_primary:
            # Last-resort legacy retry
            logger.warning("⚠️ create_publisher failed (%s), retrying with legacy kwargs.",
                           exc_primary)
            try:
                fallback = [PT.ROS2, PT.REDIS] if self._ros2_available else [PT.REDIS]
                self._publisher = await InterfaceFactory.create_publisher(
                    name=self._feederName,
                    protocols=fallback,
                    message_type=self._type,
                    namespace="feeder",
                    is_publisher=True,
                    **({'node': self._node, 'qos_profile': self._qos}
                       if self._ros2_available and self._node else {})
                )
            except Exception as exc_fallback:
                logger.error("❌ Failed to create feeder publisher for '%s': %s",
                             self._feederName, exc_fallback)
                raise FeederException(
                    f"Could not create publisher for {self._feederName}. "
                    f"Install ROS2 or configure Zenoh/Redis."
                ) from exc_fallback

        if self._publisher is None:
            raise FeederException(
                f"No communication protocol available for {self._feederName}. "
                f"Install ROS2, Zenoh, or Redis."
            )

        # Auto-load interface type via InterfaceLoader (after protocol is known)
        self._msg_type = await self._load_msg_type()

        # DEPRECATED
        # self.create_feeder()

        # Attach ROS2 handlers only for ROS2-backed publishers (legacy support)
        if self._ros2_available and hasattr(self._publisher, 'publisher_server'):
            for handler_class in self._handler_classes:
                if not isinstance(handler_class, type) or not issubclass(
                        handler_class, CommunicationHandler):
                    raise TypeError("Handler class must be a subclass of CommunicationHandler")
                handler = handler_class(
                    initiator=self._feederName,
                    publisher=self._publisher,
                    type=self._publisher.message_type
                )
                
                self.add_handler(handler)
        else:
            logger.debug("⚠️ Skipping ROS2 handlers for %s (non-ROS2 publisher)",
                         self._feederName)

        # Flush buffered messages
        self._is_ready = True
        await self._flush_buffer()

    # ------------------------------------------------------------------
    # Interface type loading
    # ------------------------------------------------------------------

    async def _load_msg_type(self) -> Optional[Any]:
        """
        Resolve the transport message type for this feeder.

        Priority:
        1. Use :class:`~vyra_base.com.core.interface_loader.InterfaceLoader`
           to look up the interface for ``self._feederName`` with the
           resolved protocol.
        2. Fall back to ``self._type`` (explicit type passed in ``__init__``).

        Returns:
            ROS2 message class, proto ``*_pb2`` module, or ``None``.
        """
        protocol = self._resolved_protocol or ""

        # Try InterfaceLoader first
        if self._interface_paths or self._feederName:
            try:
                
                loader = InterfaceLoader(
                    interface_paths=[Path(p) for p in self._interface_paths]
                    if self._interface_paths else None
                )
                loaded = loader.get_interface_for_function(
                    self._feederName, protocol=protocol
                )
                if loaded is not None:
                    logger.info(
                        f"✅ InterfaceLoader resolved msg type for '{self._feederName}' "
                        f"via protocol '{protocol}': {loaded}"
                    )
                    return loaded
            except Exception as exc:
                logger.debug(
                    f"⚠️ InterfaceLoader failed for '{self._feederName}': {exc}"
                )

        # Fallback: use explicit type from __init__
        if self._type is not None:
            logger.debug(
                f"Using explicit _type for '{self._feederName}': {self._type}"
            )
        return self._type

    # ------------------------------------------------------------------
    # Conversion hook (overridden by concrete feeders)
    # ------------------------------------------------------------------

    def _prepare_entry_for_publish(self, entry: Any) -> Any:
        """
        Convert a Python entry object to a wire-ready dict (or pass through).

        Concrete feeders override this to apply feeder-specific field
        name mappings and value transformations.  The returned value is
        then further converted by :class:`~vyra_base.com.feeder.message_mapper.MessageMapper`
        into the protocol-native message type inside :meth:`_publish`.

        The default implementation returns *entry* unchanged.
        """
        return entry

    # ------------------------------------------------------------------
    # Buffer flush
    # ------------------------------------------------------------------

    async def _flush_buffer(self) -> None:
        """Publish all messages that arrived before :meth:`start`."""
        flushed = 0
        while self._feedbuffer:
            msg = self._feedbuffer.popleft()
            try:
                await self._publish(msg)
                flushed += 1
            except Exception as exc:
                logger.warning("⚠️ Buffer flush failed for message in %s: %s",
                               self._feederName, exc)
        if flushed:
            logger.debug("🔁 %s flushed %d buffered message(s).", self._feederName, flushed)

    async def start(self) -> None:
        """Start the feeder (implements :class:`IFeeder`).

        Subclasses may override to add extra initialisation before calling
        ``await super().start()``.
        """
        await self.create(loggingOn=self._loggingOn)

    # ------------------------------------------------------------------
    # Feed / publish path
    # ------------------------------------------------------------------

    async def feed(self, msg: Any) -> None:
        """Enqueue *msg* for publishing (implements :class:`IFeeder`).

        If the feeder is not yet ready the message is buffered.  Otherwise
        the publish path is executed synchronously (event-loop aware).

        :param msg: Message to publish.
        :type msg: Any
        """
        if not self._is_ready:
            self._feedbuffer.append(msg)
            logger.debug("⏳ %s buffering message (not started yet).", self._feederName)
            return

        # if hasattr(self, '_feeder'):
        #     self._feeder.log(self._level, msg)

        if self._loggingOn:
            logger.info("🗞 Feeder %s fed: %s", self._feederName, msg)

        if not self._publisher:
            logger.error("❌ No publisher for feeder %s", self._feederName)
            self._error_count += 1
            return

        try:
            await self._publish(msg)

        except Exception as exc:
            logger.error("❌ Feed failed in %s: %s", self._feederName, exc)
            self._error_count += 1

    def feed_sync(self, msg: Any) -> None:
        """Sync version of :meth:`feed` for use in sync contexts."""
        if not self._is_ready:
            self._feedbuffer.append(msg)
            logger.debug("⏳ %s buffering message (not started yet).", self._feederName)
            return

        # if hasattr(self, '_feeder'):
        #     self._feeder.log(self._level, msg)

        if self._loggingOn:
            logger.info("🗞 Feeder %s fed: %s", self._feederName, msg)

        if not self._publisher:
            logger.error("❌ No publisher for feeder %s", self._feederName)
            self._error_count += 1
            return

        loop = asyncio.get_event_loop()
        if loop.is_running():
            loop.create_task(self._publish(msg))
        else:
            loop.run_until_complete(self._publish(msg))

    async def _publish(self, msg: Any) -> None:
        """
        Internal coroutine — convert entry, then publish with retry.

        Conversion order:
        1. ``_prepare_entry_for_publish(msg)``  — feeder-specific field mapping
        2. :meth:`~vyra_base.com.feeder.message_mapper.MessageMapper.to_transport_msg`
           — route to ROS2 / Protobuf / passthrough based on resolved protocol
        """
        last_exc: Optional[Exception] = None

        if not self._publisher:
            raise FeederException(f"No publisher available for {self._feederName}")

        # Step 1: feeder-specific pre-processing
        wire_data = self._prepare_entry_for_publish(msg)

        # Step 2: convert to transport-native message
        try:
            transport_msg = MessageMapper.to_transport_msg(
                wire_data if isinstance(wire_data, dict) else wire_data,
                protocol=self._resolved_protocol or "",
                msg_type=self._msg_type,
            )
        except Exception as exc:
            logger.debug(
                f"⚠️ MessageMapper conversion skipped for '{self._feederName}': {exc}"
            )
            transport_msg = wire_data

        for attempt in range(1, self._max_retries + 1):
            try:
                if hasattr(self._publisher, 'publish'):
                    await self._publisher.publish(transport_msg)
                else:
                    raise FeederException(
                        f"Publisher has no publish() method: {type(self._publisher)}"
                    )
                self._feed_count += 1
                self._last_feed_at = datetime.datetime.utcnow()
                return
            except Exception as exc:
                last_exc = exc
                if attempt < self._max_retries:
                    await asyncio.sleep(self._retry_delay)

        self._error_count += 1
        logger.error("❌ %s publish failed after %d attempt(s): %s",
                     self._feederName, self._max_retries, last_exc)

    # ------------------------------------------------------------------
    # Feeder logger setup
    # ------------------------------------------------------------------

    # def create_feeder(self) -> None:
    #     """Set up the Python logger for this feeder."""
    #     feed_logger_name = f"{self._feedBaseName}.{self._feederName}"
    #     self._feeder = logging.getLogger(feed_logger_name)
    #     self._feeder.setLevel(self._level)

    #     if self._loggingOn:
    #         vyra_logger = logging.getLogger("vyra_base")
    #         for handler in vyra_logger.handlers:
    #             if handler not in self._feeder.handlers:
    #                 self._feeder.addHandler(handler)

    def add_handler(self, handler: CommunicationHandler) -> bool:
        """Attach a :class:`~vyra_base.com.handler.communication.CommunicationHandler`.

        :param handler: Handler instance to attach.
        :type handler: CommunicationHandler
        :return: ``True`` if added, ``False`` if already present.
        :rtype: bool
        """
        if not isinstance(handler, CommunicationHandler):
            raise TypeError(f"Expected CommunicationHandler, got {type(handler)}")
        if handler in self._handler:
            return False

        # DEPRECATED
        # self._feeder.addHandler(handler)
        self._handler.append(handler)
        return True

    def add_handler_class(self, handler_class: Type[CommunicationHandler]) -> None:
        """Register a handler class to be instantiated during :meth:`create`.

        :param handler_class: Handler class (must subclass CommunicationHandler).
        :type handler_class: Type[CommunicationHandler]
        """
        if not isinstance(handler_class, type) or not issubclass(
                handler_class, CommunicationHandler):
            raise TypeError("Handler class must be a subclass of CommunicationHandler")
        if handler_class not in self._handler_classes:
            self._handler_classes.append(handler_class)
