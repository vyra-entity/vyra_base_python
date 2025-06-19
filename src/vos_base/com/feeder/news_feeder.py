
import logging
import uuid
from typing import Any
import re

from vos_base.com import ros2_handler

from datetime import datetime

from .feeder import BaseFeeder
from vos_base.defaults.entries import NewsFeedEntry
from vos_base.defaults.exceptions import FeederException
from vos_base.com.ros2_handler import ROS2Handler
from vos_base.com.datalayer.speaker import VOSSpeaker

from vos_base.com.datalayer.interface_factory import create_vos_speaker
from vos_base.com.datalayer.node import VOSNode

class NewsFeeder(BaseFeeder):
    """ Collection of the news messages """

    def __init__(
            self, 
            type: Any,
            node: VOSNode,
            loggingOn: bool = False
            ):
        
        self._feederName: str = 'NewsFeeder'
        self._doc: str = 'Collect news messages of this module.'
        self._level: int = logging.INFO
        self._loggingOn: bool = loggingOn

        speaker: VOSSpeaker = create_vos_speaker(
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

    async def feed(self, newsElement: NewsFeedEntry) -> None:
        """Adds value to the logger and the remote handler"""
        if isinstance(newsElement, NewsFeedEntry):
            super().feed(newsElement)
        else:
            raise FeederException(f"Wrong Type. Expect: NewsFeedEntry, got {type(value)}")


async def build_newsfeed(
        self, 
        *args: Any,
        module_name: str = "N/A",
        module_template: str = "N/A"):
        """Building a well structured newsfeed entry from plain text and module information"""

        if not isinstance(type, NewsFeedEntry.MESSAGE_TYPE):
            raise FeederException(
                f"Invalid type for newsfeed entry: {type}. "
                f"Expected one of {NewsFeedEntry.MESSAGE_TYPE.__members__}."
            )
        
        message: str = (''.join(map(str, args))).split('|')[-1]
        ansi_escape = re.compile(
            r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')
        message = ansi_escape.sub('', message)
        message = message.replace('__call__', 'Aufruf')
        message = message.replace('__init__', 'Initialisierung')
        
        message_type = extract_enum_from_string(message)

        if message_type is None or message_type not in NewsFeedEntry.MESSAGE_TYPE:
            raise FeederException(
                f"Invalid type for newsfeed entry: {type}. "
                f"Expected one of {NewsFeedEntry.MESSAGE_TYPE.__members__}."
            )

        newsfeed_entry = NewsFeedEntry(
            level= message_type,
            message= message,
            timestamp= datetime.now(),
            uuid= uuid.uuid4(),
            module_name= module_name,
            module_template= module_template
        )


def extract_enum_from_string(s: str) -> NewsFeedEntry.MESSAGE_TYPE | None:
    match = re.search(r"<([^<>]+)>", s)
    if match:
        value = match.group(1)
        try:
            return NewsFeedEntry.MESSAGE_TYPE(value)
        except ValueError:
            return None
    return None