import logging
from collections import deque
from datetime import datetime
from typing import Any, Union
import uuid

from vyra_base.com import ros2_handler
from vyra_base.com.datalayer.node import VyraNode
from vyra_base.com.datalayer.typeconverter import Ros2TypeConverter
from vyra_base.com.ros2_handler import ROS2Handler
from vyra_base.defaults.entries import ErrorEntry, ModuleEntry
from vyra_base.defaults.exceptions import FeederException
from vyra_base.helper.logger import Logger

from .feeder import BaseFeeder

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
        super().__init__()

        self._feederName: str = f'ErrorFeeder'
        self._doc: str = 'Collect error messages of this module.'
        self._level: int = logging.ERROR
        self._type: Any = type
        self._node: VyraNode = node
        self._module_config: ModuleEntry = module_config

        self._handler.append(ROS2Handler)

        self.create(loggingOn=loggingOn)

    def feed(self, errorElement: Union[ErrorEntry, dict]) -> None:
        """Feed a news entry to the feeder. The content can either be a string which
        will be processed into an ErrorEntry if a dict is given, or a ErrorEntry object 
        itself. Use the method `build_errorfeed` to create an ErrorEntry from a dict. 
        Args:
            errorElement (Union[ErrorEntry, dict]): The error entry to be fed.
                Can be a dictionary with error details or an ErrorEntry object.
        Raises:
            FeederException: If the type of errorElement is neither a dict nor an ErrorEntry.
        """

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

        errorfeed_entry.module_id = Ros2TypeConverter.uuid_to_ros2uuid(
            self._module_config.uuid
        )

        super().feed(errorfeed_entry)

    def build_errorfeed(self, errorDict: dict) -> ErrorEntry:
        """Builds a error entry from the given keyword arguments.
        Args:
            errorDict (dict): A dictionary containing error details.
                Keywords are:
                - 'code': int16 - Error code (default: 0x00000000)
                - 'uuid': UUID - Unique identifier for the error (default: a new UUID)
                - 'description': str - Description of the error (default: '')
                - 'solution': str - Suggested solution for the error (default: '')
                - 'miscellaneous': str - Additional information (default: '')
                - 'level': ErrorEntry.ERROR_LEVEL - Level of the error 
                           (default: ErrorEntry.ERROR_LEVEL.MINOR_FAULT)
        Returns:
            ErrorEntry: An instance of ErrorEntry populated with the provided details.
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
