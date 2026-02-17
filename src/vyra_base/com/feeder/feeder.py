from __future__ import annotations

import asyncio
import logging
from collections import deque
from typing import Any, Type, Optional

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

from vyra_base.com.handler.communication import CommunicationHandler
from vyra_base.defaults.exceptions import FeederException
from vyra_base.helper.logging_config import VyraLoggingConfig

logger = logging.getLogger(__name__)


class BaseFeeder:
    """
    Abstract class.

    Provides the required interface for the ``deque`` method all inheriting deque objects require to work.
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

        self._handler_classes: list[Type[CommunicationHandler]] = []  # Store handler classes
        self._handler: list[Type[CommunicationHandler] | CommunicationHandler] = []  # Store handler instances
        self._feeder: logging.Logger
        self._loggingOn: bool  # If true, the feeder will log messages in the base logger
        self._node: Optional[Any] = None  # VyraNode or None
        self._type: Any
        self._publisher: Optional[Any] = None  # VyraPublisher or legacy VyraSpeaker
        self._feedbuffer = deque(maxlen=20)  

    async def create(self, loggingOn: bool = False) -> None:
        """
        Create the feeder and its communication handlers.
        Uses new unified transport layer (Publisher) with protocol fallback.
        
        :raises FeederException: If the publisher could not be created.
        :raises TypeError: If a handler class is not a subclass of CommunicationHandler.
        :param loggingOn: If True, enables logging in the base logger.
        :type loggingOn: bool
        """
        self._loggingOn: bool = loggingOn
        
        # Use new unified create_publisher (with ROS2 fallback to legacy if needed)
        try:
            # New unified API: create_publisher with protocol fallback
            self._publisher = await InterfaceFactory.create_publisher(
                name=self._feederName,
                protocols=[ProtocolType.ROS2, ProtocolType.ZENOH, ProtocolType.REDIS] if self._ros2_available else [ProtocolType.ZENOH, ProtocolType.REDIS],
                message_type=self._type,
                topic_builder=getattr(self, '_topic_builder', None),  # Optional TopicBuilder
                **({'node': self._node, 'qos_profile': self._qos} if self._ros2_available and self._node else {})
            )
            
            logger.info(f"✅ {self._feederName} using {self._publisher.protocol.value} protocol")
            
        except Exception as e:
            # Fallback to legacy create_publisher if create_publisher not available yet
            logger.warning(f"⚠️ create_publisher failed, trying legacy create_publisher: {e}")
            try:
                self._publisher = await InterfaceFactory.create_publisher(
                    name=self._feederName,
                    protocols=[ProtocolType.ROS2, ProtocolType.REDIS] if self._ros2_available else [ProtocolType.REDIS],
                    message_type=self._type,
                    is_publisher=True,
                    **({'node': self._node, 'qos_profile': self._qos} if self._ros2_available and self._node else {})
                )
                logger.info(f"✅ {self._feederName} using legacy publisher with {self._publisher.protocol.value}")
            except Exception as e2:
                logger.error(f"❌ Failed to create feeder publisher: {e2}")
                raise FeederException(
                    f"Could not create publisher for {self._feederName}. "
                    f"Install ROS2 or configure Zenoh/Redis."
                )
        
        if self._publisher is None:
            raise FeederException(
                f"No communication protocol available for {self._feederName}. "
                f"Install ROS2, Zenoh, or Redis."
            )

        self.create_feeder()

        # Create handlers only for ROS2 publishers (legacy support)
        if self._ros2_available and hasattr(self._publisher, 'publisher_server'):
            for handler_class in self._handler_classes:
                if not isinstance(handler_class, type) or not issubclass(handler_class, CommunicationHandler):
                    raise TypeError("Handler class must be a subclass of CommunicationHandler")

                handler = handler_class(
                    initiator=self._feederName,
                    publisher=self._publisher,  # Pass publisher as 'publisher' for legacy compatibility
                    type=self._publisher.publisher_server.publisher_info.type
                )
                self.add_handler(handler)
        else:
            logger.debug(f"⚠️ Skipping ROS2 handlers for {self._feederName} (non-ROS2 protocol or unified Publisher)")

    def feed(self, msg: Any) -> None:
        """
        Feed a message to the feeder.
        Uses Publisher.publish() for new unified transport layer.

        :param msg: The message to feed.
        :type msg: Any
        """
        if not hasattr(self, '_feeder'):
            self._feedbuffer.append(msg)
            logger.info(f"⏳ Feeder {self._feederName} not yet created. Buffering message: {msg}")
            return
        
        self._feeder.log(self._level, msg)

        if self._loggingOn:
            logger.info(
                f"Feeder {self._feederName} fed with message: {msg}"
            )
        
        if not self._publisher:
            logger.error(f"❌ No publisher available for feeder {self._feederName}")
            return
        
        # Use unified Publisher.publish() or legacy Publisher.shout()
        try:
            loop = asyncio.get_event_loop()
            if hasattr(self._publisher, 'publish'):
                loop.run_until_complete(self._publisher.publish(msg))
            elif hasattr(self._publisher, 'shout'):  # Legacy fallback
                loop.run_until_complete(self._publisher.shout(msg))
            else:
                logger.error(f"❌ Publisher has no publish() or shout() method: {type(self._publisher)}")
        except Exception as e:
            logger.error(f"❌ Failed to feed message via {self._publisher.protocol}: {e}")

    def create_feeder(self):
        """
        Set the logger for the feeder.
        """
        feed_logger_name: str = f"{self._feedBaseName}.{self._feederName}"
        self._feeder = logging.getLogger(feed_logger_name)
        self._feeder.setLevel(self._level)
        # self._feeder.propagate = False

        if self._loggingOn:
            # Add handlers from vyra_base logger to the feeder logger
            vyra_logger = logging.getLogger("vyra_base")
            for handler in vyra_logger.handlers:
                if handler not in self._feeder.handlers:
                    self._feeder.addHandler(handler)

    def add_handler(self, handler: CommunicationHandler) -> bool:
        """
        Add a communication handler to the feeder.

        :param handler: The communication handler to add.
        :type handler: CommunicationHandler
        :raises TypeError: If the handler is not a CommunicationHandler.
        :return: True if the handler was added, False if it was already present.
        :rtype: bool
        """
        if not isinstance(handler, CommunicationHandler):
            raise TypeError(
                f"Expected a CommunicationHandler, got {type(handler)}"
            )
        
        if handler in self._handler:
            return False
        
        self._feeder.addHandler(handler)
        self._handler.append(handler)  # Store the instance

        return True

    def add_handler_class(self, handler_class: Type[CommunicationHandler]) -> None:
        """
        Add a handler class to the feeder.

        :param handler_class: The handler class to add.
        :type handler_class: Type[CommunicationHandler]
        :raises TypeError: If handler_class is not a subclass of CommunicationHandler.
        """
        if not isinstance(handler_class, type) or not issubclass(handler_class, CommunicationHandler):
            raise TypeError("Handler class must be a subclass of CommunicationHandler")
        if handler_class not in self._handler_classes:
            self._handler_classes.append(handler_class)