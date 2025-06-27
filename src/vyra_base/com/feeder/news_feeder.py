import logging
import re
import uuid
from datetime import datetime
from typing import Any, Union

from builtin_interfaces.msg import Time as BuiltinTime

from .feeder import BaseFeeder
from vyra_base.com.datalayer.node import VyraNode
from vyra_base.com.datalayer.typeconverter import Ros2TypeConverter
from vyra_base.com.ros2_handler import ROS2Handler
from vyra_base.defaults.entries import ModuleEntry, NewsEntry
from vyra_base.defaults.exceptions import FeederException
from vyra_base.helper.logger import Logger


class NewsFeeder(BaseFeeder):
    """Collection of the news messages.

    :param type: The ros2-msg type for the feeder.
    :type type: Any
    :param node: The VyraNode instance associated with this feeder (ROS2 Node).
    :type node: VyraNode
    :param module_config: Module configuration entry.
    :type module_config: ModuleEntry
    :param loggingOn: Flag to enable or disable logging next to feeding. Defaults to False.
    :type loggingOn: bool, optional
    :raises FeederException: If the VyraSpeaker cannot be created with the given type.
    """

    def __init__(
            self, 
            type: Any,
            node: VyraNode,
            module_config: ModuleEntry,
            loggingOn: bool = False
        ):
        super().__init__()

        self._feederName: str = f'NewsFeeder'
        self._doc: str = 'Collect news messages of this module.'
        self._level: int = logging.INFO
        self._type: Any = type
        self._node: VyraNode = node
        self._module_config: ModuleEntry = module_config

        self._handler.append(ROS2Handler)

        self.create(loggingOn=loggingOn)

    def feed(self, newsElement: Union[NewsEntry, str, list]) -> None:
        """Feed a news entry to the feeder.

        The content can either be a string which will be processed into a NewsEntry,
        or a NewsEntry object itself. Use the method ``build_newsfeed`` to create a
        NewsEntry from a string or list.

        :param newsElement: The news entry to be fed. Can be a string or list of strings
            to be processed into a NewsEntry, or a NewsEntry object.
        :type newsElement: Union[NewsEntry, str, list]
        :raises FeederException: If the type of newsElement is not supported.
        """

        if isinstance(newsElement, str) or isinstance(newsElement, list):
            newsfeed_entry = self.build_newsfeed(newsElement)
        elif isinstance(newsElement, NewsEntry):
            newsfeed_entry = newsElement
        else:
            raise FeederException(f"Wrong Type. Expect: NewsEntry, got {type(newsElement)}")

        newsfeed_entry.timestamp = Ros2TypeConverter.time_to_ros2buildintime(
                newsfeed_entry.timestamp if newsfeed_entry.timestamp else datetime.now()
        )
        
        newsfeed_entry.uuid = Ros2TypeConverter.uuid_to_ros2uuid(
            newsfeed_entry.uuid if newsfeed_entry.uuid else uuid.uuid4()
        )

        newsfeed_entry.module_id = Ros2TypeConverter.uuid_to_ros2uuid(
            self._module_config.uuid
        )

        super().feed(newsfeed_entry)

    def build_newsfeed(self, *args: Any) -> NewsEntry:
        """Build a well structured newsfeed entry from plain text and module information.

        :param args: The arguments to be processed into a news entry. Can be a string or list of strings.
        :type args: Any
        :return: A structured news entry containing the message, level, timestamp, UUID, module name, module ID, module template, and type.
        :rtype: NewsEntry
        :raises FeederException: If the type of the message level is not valid.
        """
       
        message: str = (''.join(map(str, args))).split('|')[-1]
        ansi_escape = re.compile(
            r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')
        message = ansi_escape.sub('', message)
        message = message.replace('__call__', 'Call')
        message = message.replace('__init__', 'Initialize')
        
        message_level = extract_level_from_msg(message)

        if message_level is None:
            message_level = NewsEntry.MESSAGE_LEVEL.INFO

        if message_level not in NewsEntry.MESSAGE_LEVEL:
            raise FeederException(
                f"Invalid type for newsfeed entry: {type}. "
                f"Expected one of {NewsEntry.MESSAGE_LEVEL.__members__}."
            )

        newsfeed_entry: NewsEntry = NewsEntry(
            level= message_level.value,
            message= message,
            timestamp= datetime.now(),
            uuid= uuid.uuid4(),
            module_name= self._module_config.name,
            module_id= self._module_config.uuid,
            _type= self._type

        )
        return newsfeed_entry


def extract_level_from_msg(message: str) -> Union[NewsEntry.MESSAGE_LEVEL, None]:
    """Extract the message level from a given message string.

    :param message: The message string from which to extract the level.
    :type message: str
    :return: The extracted message level if found, otherwise None.
    :rtype: Union[NewsEntry.MESSAGE_LEVEL, None]
    """
    match = re.search(r"<([^<>]+)>", message)
    if match:
        value = match.group(1)
        try:
            return NewsEntry.MESSAGE_LEVEL(value)
        except ValueError:
            return None
    return None
