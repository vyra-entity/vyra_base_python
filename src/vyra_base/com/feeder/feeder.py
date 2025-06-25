from __future__ import annotations

import logging

from collections import deque

from typing import Any

from vyra_base.com.communication_handler import CommunicationHandler
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

    _handler: list[CommunicationHandler] = []
    _feeder: logging.Logger
    _loggingOn: bool  # If true, the feeder will log messages in the base logger
    
    def feed(self, msg: Any) -> None:
        """ """
        Logger.log("Log to ROS2 Handler")
        self._feeder.log(self._level, msg)

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
        self._handler.append(handler)

        return True