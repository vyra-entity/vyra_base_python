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
    from vyra_base.com.transport.ros2.node import VyraNode
    from vyra_base.com.core.types import VyraSpeaker

from vyra_base.com.handler.communication import CommunicationHandler
from vyra_base.defaults.exceptions import FeederException
from vyra_base.helper.logger import Logger


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
        self._speaker: Optional[Any] = None  # VyraSpeaker or new protocol speaker
        self._feedbuffer = deque(maxlen=20)  

    async def create(self, loggingOn: bool = False) -> None:
        """
        Create the feeder and its communication handlers.
        Supports ROS2 (if available) or fallback to Redis/MQTT.
        
        :raises FeederException: If the speaker could not be created.
        :raises TypeError: If a handler class is not a subclass of CommunicationHandler.
        :param loggingOn: If True, enables logging in the base logger.
        :type loggingOn: bool
        """
        self._loggingOn: bool = loggingOn
        
        # Try ROS2 first if available
        if self._ros2_available and self._node is not None:
            try:
                self._speaker = await InterfaceFactory.create_speaker(
                    name=self._feederName,
                    protocols=[ProtocolType.ROS2],
                    message_type=self._type,
                    node=self._node,
                    is_publisher=True,
                    qos_profile=self._qos
                )
                
                if self._speaker.publisher_server is None:
                    raise FeederException(
                        f"Could not create ROS2 speaker for {self._feederName}."
                    )
                
                Logger.info(f"✅ {self._feederName} using ROS2 protocol")
                
            except Exception as e:
                Logger.warn(f"⚠️ ROS2 speaker creation failed: {e}. Falling back to Redis/MQTT.")
                self._speaker = None
        
        # Fallback to new protocol speakers (Redis, MQTT)
        if self._speaker is None:
            try:
                # Use InterfaceFactory for protocol fallback
                self._speaker = await InterfaceFactory.create_speaker(
                    name=self._feederName,
                    protocols=[ProtocolType.REDIS, ProtocolType.MQTT]
                )
                
                Logger.info(f"✅ {self._feederName} using {self._speaker.protocol.value} protocol")
                
            except Exception as e:
                Logger.error(f"❌ Failed to create feeder speaker: {e}")
                raise FeederException(
                    f"Could not create speaker for {self._feederName}. "
                    f"Install ROS2 or configure Redis/MQTT."
                )
        
        if self._speaker is None:
            raise FeederException(
                f"No communication protocol available for {self._feederName}. "
                f"Install ROS2, Redis, or MQTT."
            )

        self.create_feeder()

        # Create handlers only for ROS2 speakers
        if self._ros2_available and hasattr(self._speaker, 'publisher_server'):
            for handler_class in self._handler_classes:
                if not isinstance(handler_class, type) or not issubclass(handler_class, CommunicationHandler):
                    raise TypeError("Handler class must be a subclass of CommunicationHandler")

                handler = handler_class(
                    initiator=self._feederName,
                    speaker=self._speaker,
                    type=self._speaker.publisher_server.publisher_info.type
                )
                self.add_handler(handler)
        else:
            Logger.debug(f"⚠️ Skipping ROS2 handlers for {self._feederName} (non-ROS2 protocol)")

    def feed(self, msg: Any) -> None:
        """
        Feed a message to the feeder.
        Supports ROS2 handlers or direct speaker.shout() for new protocols.

        :param msg: The message to feed.
        :type msg: Any
        """
        if not hasattr(self, '_feeder'):
            self._feedbuffer.append(msg)
            Logger.info(f"⏳ Feeder {self._feederName} not yet created. Buffering message: {msg}")
            return
        
        self._feeder.log(self._level, msg)

        if self._loggingOn:
            Logger.log(
                f"Feeder {self._feederName} fed with message: {msg}"
            )
        
        if not self._speaker:
            Logger.error(f"❌ No speaker available for feeder {self._feederName}")
            return
        
        # If using new protocols (Redis/MQTT), publish directly
        if not self._ros2_available or not hasattr(self._speaker, 'publisher_server'):
            try:
                loop = asyncio.get_event_loop()
                loop.run_until_complete(self._speaker.shout(msg))
            except Exception as e:
                Logger.error(f"❌ Failed to feed message via {self._speaker.protocol}: {e}")

    def create_feeder(self):
        """
        Set the logger for the feeder.
        """
        feed_logger_name: str = f"{self._feedBaseName}.{self._feederName}"
        self._feeder = logging.getLogger(feed_logger_name)
        self._feeder.setLevel(self._level)
        # self._feeder.propagate = False

        if self._loggingOn:
            Logger.add_external(feed_logger_name)

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