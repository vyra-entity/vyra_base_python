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
    _level: int = logging.INFO

    _handler: list[CommunicationHandler] = []
    _logger: logging.Logger
    _loggingOn: bool  # If true, the feeder will log messages in the base logger
    
    def feed(self, msg: Any) -> None:
        """ """
        self._logger.log(self._level, msg)

    def add_logger(self):
        """ Set the logger for the feeder. """
        feed_logger_name: str = f"{self._feedBaseName}.{self._feederName}"
        self._logger = logging.getLogger(feed_logger_name)
        self._logger.setLevel(self._level)
        self._logger.propagate = False

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
        
        self._handler.append(handler)

        return True