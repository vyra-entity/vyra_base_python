import logging
import uuid
from collections import deque
from datetime import datetime
from typing import Any, Union

from vyra_base.com import ros2_handler
from vyra_base.com.datalayer.node import VyraNode
from vyra_base.com.datalayer.typeconverter import Ros2TypeConverter
from vyra_base.com.ros2_handler import ROS2Handler
from vyra_base.defaults.entries import ErrorEntry, ModuleEntry
from vyra_base.defaults.exceptions import FeederException
from vyra_base.helper.logger import Logger

from .feeder import BaseFeeder

class ErrorFeeder(BaseFeeder):
    """
    Collection of the error messages.

    :param type: The ros2-msg type for the feeder.
    :type type: Any
    :param node: The VyraNode instance associated with this feeder (ROS2 Node).
    :type node: VyraNode
    :param module_config: The module configuration entry.
    :type module_config: ModuleEntry
    :param loggingOn: Flag to enable or disable logging next to feeding. Defaults to False.
    :type loggingOn: bool, Optional
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

        self._feederName: str = f'ErrorFeeder'
        self._doc: str = 'Collect error messages of this module.'
        self._level: int = logging.ERROR
        self._type: Any = type
        self._node: VyraNode = node
        self._module_config: ModuleEntry = module_config

        self._handler_classes.append(ROS2Handler)

        self.create(loggingOn=loggingOn)

    def feed(self, errorElement: Union[ErrorEntry, dict]) -> None:
        """
        Feed a news entry to the feeder.

        The content can either be a dictionary which will be processed into an
        :class:`vyra_base.defaults.entries.ErrorEntry`, or an :class:`vyra_base.defaults.entries.ErrorEntry` object itself. Use the method
        :meth:`build_errorfeed` to create an :class:`vyra_base.defaults.entries.ErrorEntry` from a dict.

        :param errorElement: The error entry to be fed. Can be a dictionary with error details or an :class:`vyra_base.defaults.entries.ErrorEntry` object.
        :type errorElement: Union[ErrorEntry, dict]
        :raises FeederException: If the type of errorElement is neither a dict nor an :class:`vyra_base.defaults.entries.ErrorEntry`.
        """
        if isinstance(errorElement, dict):
            errorfeed_entry = self.build_errorfeed(errorElement)
        elif isinstance(errorElement, ErrorEntry):
            errorfeed_entry = errorElement
        else:
            raise FeederException(f"Wrong Type. Expect: NewsEntry, got {type(errorElement)}")

        errorfeed_entry.timestamp = Ros2TypeConverter.time_to_ros2buildintime(
                errorfeed_entry.timestamp if errorfeed_entry.timestamp else datetime.now()
        )
        
        errorfeed_entry.uuid = Ros2TypeConverter.uuid_to_ros2uuid(
            errorfeed_entry.uuid if errorfeed_entry.uuid else uuid.uuid4()
        )

        super().feed(errorfeed_entry)

    def build_errorfeed(self, errorDict: dict) -> ErrorEntry:
        """
        Build an error entry from the given keyword arguments.

        :param errorDict: A dictionary containing error details. Keys are:

            - ``code``: int16 - Error code (default: 0x00000000)
            - ``uuid``: UUID - Unique identifier for the error (default: a new UUID)
            - ``description``: str - Description of the error (default: '')
            - ``solution``: str - Suggested solution for the error (default: '')
            - ``miscellaneous``: str - Additional information (default: '')
            - ``level``: ErrorEntry.ERROR_LEVEL - Level of the error (default: ErrorEntry.ERROR_LEVEL.MINOR_FAULT)

        :type errorDict: dict
        :return: An instance of :class:`vyra_base.defaults.entries.ErrorEntry` populated with the provided details.
        :rtype: ErrorEntry
        """
        errorfeed_entry = ErrorEntry(
            _type=self._type,
            code=errorDict.get('error_code', 0x00000000),
            module_id=self._module_config.uuid,
            module_name=self._module_config.name,
            uuid=errorDict.get('uuid', uuid.uuid4()),
            timestamp=datetime.now(),
            description=errorDict.get('description', ''),
            solution=errorDict.get('solution', ''),
            miscellaneous=errorDict.get('miscellaneous', ''),
            level=errorDict.get('level', ErrorEntry.ERROR_LEVEL.MINOR_FAULT).value
        )
        return errorfeed_entry
