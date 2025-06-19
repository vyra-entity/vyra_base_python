from collections import deque

import logging

from typing import Any

from vyra_base.com import ros2_handler

from .feeder import BaseFeeder
from vyra_base.defaults.constants import FeederConstants
from vyra_base.defaults.entries import ErrorEntry
from vyra_base.defaults.exceptions import FeederException
from vyra_base.com.ros2_handler import ROS2Handler
from vyra_base.com.datalayer.speaker import VyraSpeaker

from vyra_base.com.datalayer.interface_factory import create_vyra_speaker
from vyra_base.com.datalayer.node import VyraNode

class ErrorFeeder(BaseFeeder):
    """ Collection of the error messages """

    def __init__(
            self, 
            type: Any,
            node: VyraNode,
            loggingOn: bool = False
            ):
        
        self._feederName: str = 'ErrorFeeder'
        self._doc: str = 'Collect error messages of this module.'
        self._level: int = logging.INFO
        self._loggingOn: bool = loggingOn

        speaker: VyraSpeaker = create_vyra_speaker(
            name=self._feederName,
            node=node,
            type=type,
            description=self._doc
        )
        if speaker.publisher_server is None:
            raise FeederException(
                f"Could not create speaker for {self._feederName} with type {type}."
            )        

        ros2_handler = ROS2Handler(
            speaker.publisher_server,
            type=speaker.publisher_server.publisher_info.type
        )

        self.add_logger()
        
        self.add_handler(ros2_handler)

    async def feed(self, errorElement: ErrorEntry) -> None:
        """Adds value to the logger and the remote handler"""
        if isinstance(errorElement, ErrorEntry):
            super().feed(errorElement)
        else:
            raise FeederException(f"Wrong Type. Expect: ErrorEntry, got {type(value)}")


