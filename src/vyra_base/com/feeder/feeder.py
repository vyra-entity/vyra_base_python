from __future__ import annotations
import logging
from typing import Any
from typing import Type

from vyra_base.com.communication_handler import CommunicationHandler
from vyra_base.com.datalayer.interface_factory import create_vyra_speaker
from vyra_base.com.datalayer.speaker import VyraSpeaker
from vyra_base.com.datalayer.node import VyraNode
from vyra_base.defaults.exceptions import FeederException
from vyra_base.helper.logger import Logger


class BaseFeeder:
    """ Abstract class 

        Abstraction that provides the required interface for deuque method all
        inheriting deque objects require to work.
    """
    _feedBaseName: str = 'vyraFeeder'
    _feederName: str = 'AbstractFeeder'
    _doc: str = 'Abstract class for all feeder classes.'
    _level: int = logging.DEBUG

    _handler: list[Type[CommunicationHandler]] = []
    _feeder: logging.Logger
    _loggingOn: bool  # If true, the feeder will log messages in the base logger
    _node: VyraNode
    _type: Any

    def __init__(self, loggingOn: bool = False) -> None:
        speaker: VyraSpeaker = create_vyra_speaker(
            name=self._feederName,
            node=self._node,
            type=self._type,
            description=self._doc
        )
        self._loggingOn: bool = loggingOn

        if speaker.publisher_server is None:
            raise FeederException(
                f"Could not create speaker for {self._feederName} with type {type}."
            )        

        self.create_feeder()

        for handler_class in self._handler:
            if not isinstance(handler_class, type) or not issubclass(handler_class, CommunicationHandler):
                raise TypeError("Handler class must be a subclass of CommunicationHandler")

            handler = handler_class(
                publisher=speaker.publisher_server,
                type=speaker.publisher_server.publisher_info.type
            )
            self.add_handler(handler)

    def feed(self, msg: Any) -> None:
        """ """
        Logger.log("Log to Handler")
        self._feeder.log(self._level, msg)

        if self._loggingOn:
            Logger.log(
                f"Feeder {self._feederName} fed with message: {msg}",
                level=self._level
            )

    def create_feeder(self):
        """ Set the logger for the feeder. """
        feed_logger_name: str = f"{self._feedBaseName}.{self._feederName}"
        self._feeder = logging.getLogger(feed_logger_name)
        self._feeder.setLevel(self._level)
        # self._feeder.propagate = False

        if self._loggingOn:
            Logger.add_external(feed_logger_name)

    def add_handler(self, handler: CommunicationHandler) -> bool:
        """ Add a communication handler to the feeder. """
        if not isinstance(handler, CommunicationHandler):
            raise TypeError(
                f"Expected a CommunicationHandler, got {type(handler)}"
            )
        
        if handler in self._handler:
            return False
        
        self._feeder.addHandler(handler)

        return True