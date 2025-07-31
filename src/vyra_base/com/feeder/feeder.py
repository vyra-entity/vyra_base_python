from __future__ import annotations

import logging
from typing import Any, Type

from rclpy.qos import (
    QoSProfile,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
)

from vyra_base.com.communication_handler import CommunicationHandler
from vyra_base.com.datalayer.interface_factory import create_vyra_speaker
from vyra_base.com.datalayer.node import VyraNode
from vyra_base.com.datalayer.speaker import VyraSpeaker
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
        self._qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self._feedBaseName: str = 'vyraFeeder'
        self._feederName: str = 'AbstractFeeder'
        self._doc: str = 'Abstract class for all feeder classes.'
        self._level: int = logging.INFO

        self._handler_classes: list[Type[CommunicationHandler]] = []  # Store handler classes
        self._handler: list[Type[CommunicationHandler] | CommunicationHandler] = []  # Store handler instances
        self._feeder: logging.Logger
        self._loggingOn: bool  # If true, the feeder will log messages in the base logger
        self._node: VyraNode
        self._type: Any
        self._speaker: VyraSpeaker

    def create(self, loggingOn: bool = False) -> None:
        """
        Create the feeder and its communication handlers.
        :raises FeederException: If the speaker could not be created.
        :raises TypeError: If a handler class is not a subclass of CommunicationHandler.
        :param loggingOn: If True, enables logging in the base logger.
        :type loggingOn: bool
        :raises FeederException: If the speaker could not be created.
        :raises TypeError: If a handler class is not a subclass of CommunicationHandler.
        """
        self._speaker: VyraSpeaker = create_vyra_speaker(
            type=self._type,
            node=self._node,
            description=self._doc,
            qos_profile=self._qos,
            domain_name=self._feederName,
        )
        self._loggingOn: bool = loggingOn

        if self._speaker.publisher_server is None:
            raise FeederException(
                f"Could not create speaker for {self._feederName} with type {type}."
            )        

        self.create_feeder()

        for handler_class in self._handler_classes:
            if not isinstance(handler_class, type) or not issubclass(handler_class, CommunicationHandler):
                raise TypeError("Handler class must be a subclass of CommunicationHandler")

            handler = handler_class(
                initiator=self._feederName,
                speaker=self._speaker,
                type=self._speaker.publisher_server.publisher_info.type
            )
            self.add_handler(handler)

    def feed(self, msg: Any) -> None:
        """
        Feed a message to the feeder.

        :param msg: The message to feed.
        :type msg: Any
        """
        self._feeder.log(self._level, msg)

        if self._loggingOn:
            Logger.log(
                f"Feeder {self._feederName} fed with message: {msg}"
            )

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