from collections import deque

import logging

from datetime import datetime
from typing import Any
from typing import Union

import uuid

from vyra_base.com import ros2_handler

from .feeder import BaseFeeder
from vyra_base.defaults.entries import ErrorEntry
from vyra_base.defaults.entries import ModuleEntry
from vyra_base.defaults.exceptions import FeederException
from vyra_base.com.datalayer.typeconverter import Ros2TypeConverter
from vyra_base.com.ros2_handler import ROS2Handler
from vyra_base.com.datalayer.node import VyraNode

class ErrorFeeder(BaseFeeder):
    """ Collection of the error messages """
    def __init__(
            self, 
            type: Any,
            node: VyraNode,
            module_config: ModuleEntry,
            loggingOn: bool = False
        ):
        """
        Initializes a ErrorFeeder instance for collecting error messages of a module.
        Parameters:
            type (Any): The ros2-msg type for the feeder.
            node (VyraNode): The VyraNode instance associated with this feeder (ROS2 Node).
            loggingOn (bool, optional): Flag to enable or disable logging next to feeding. Defaults to False.
            module_name (str, optional): Name of the module using this feeder. Defaults to 'N/A'.
            module_template (str, optional): Template identifier for the module. Defaults to 'N/A'.
        Raises:
            FeederException: If the VyraSpeaker cannot be created with the given type.
        """
        

        self._feederName: str = f'ErrorFeeder'
        self._doc: str = 'Collect error messages of this module.'
        self._level: int = logging.ERROR
        self._type: Any = type
        self._node: VyraNode = node
        self._module_config: ModuleEntry = module_config

        self._handler.append(ROS2Handler)

        super().__init__(loggingOn=loggingOn)

    def feed(self, errorElement: Union[ErrorEntry, dict]) -> None:
        """Feed a news entry to the feeder. The content can either be a string which
        will be processed into a NewsEntry, or a NewsEntry object itself."""

        if isinstance(errorElement, dict):
            # If a string is passed, we assume it is a message to be logged
            errorfeed_entry = self.build_errorfeed(errorElement)
        elif isinstance(errorElement, ErrorEntry):
            # If a NewsEntry object is passed, we use it directly
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
        """Builds a error entry from the given keyword arguments."""
        errorfeed_entry = ErrorEntry(
            type=self._type,
            code=errorDict.get('error_code', 0x00000000),
            module_id=self._module_config.uuid,
            module_name=self._module_config.name,
            uuid=errorDict.get('uuid', uuid.uuid4()),
            timestamp=datetime.now(),
            description=errorDict.get('description', ''),
            solution=errorDict.get('solution', ''),
            miscellaneous=errorDict.get('miscellaneous', ''),
            level=errorDict.get('level', ErrorEntry.ERROR_LEVEL.MINOR_FAULT)
        )
        return errorfeed_entry
