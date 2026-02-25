from __future__ import annotations
import logging
import uuid
from datetime import datetime
from typing import Any, Union, Optional

# Check ROS2 availability
try:
    import rclpy
    _ROS2_AVAILABLE = True
except ImportError:
    _ROS2_AVAILABLE = False

if _ROS2_AVAILABLE:
    from vyra_base.com.transport.t_ros2 import Ros2TypeConverter
    from vyra_base.com.handler.ros2 import ROS2Handler

from vyra_base.defaults.entries import ErrorEntry, ModuleEntry
from vyra_base.defaults.exceptions import FeederException
from vyra_base.com.core.interface_path_registry import get_interface_registry

from .feeder import BaseFeeder


class ErrorFeeder(BaseFeeder):
    """
    Collection of the error messages.

    :param type: The ros2-msg type for the feeder.
    :type type: Any
    :param node: The VyraNode instance associated with this feeder (ROS2 Node).
    :type node: VyraNode
    :param module_entity: The module configuration entry.
    :type module_entity: ModuleEntry
    :param loggingOn: Flag to enable or disable logging next to feeding. Defaults to False.
    :type loggingOn: bool, Optional
    :raises FeederException: If the VyraSpeaker cannot be created with the given type.
    """
    def __init__(
            self, 
            type: Any,
            node: Optional[Any],
            module_entity: ModuleEntry,
            loggingOn: bool = True
        ):
        super().__init__()

        # _feederName must match the "functionname" in the interface config JSON
        self._feederName: str = 'ErrorFeed'
        self._doc: str = 'Collect error messages of this module.'
        self._level: int = logging.ERROR
        self._type: Any = type
        self._node: Optional[Any] = node
        self._module_entity: ModuleEntry = module_entity
        self._ros2_available: bool = _ROS2_AVAILABLE and node is not None
        self._loggingOn: bool = loggingOn

        if self._ros2_available and ROS2Handler is not None:
            self._handler_classes.append(ROS2Handler)

    async def start(self) -> None:
        """Starts the feeder by initializing handlers.

        Automatically resolves the transport protocol from the module's
        interface config (reads ``ErrorFeed`` entry, picks ``tags``).
        """
        paths = get_interface_registry().get_interface_paths()

        if paths:
            self.set_interface_paths(paths)
        await self.create(loggingOn=self._loggingOn)
        
    async def feed(self, errorElement: Union[ErrorEntry, dict]) -> None:
        """
        Feed an error entry to the feeder.

        Normalises the input to an :class:`~vyra_base.defaults.entries.ErrorEntry`,
        then delegates to :meth:`~BaseFeeder.feed` which calls
        :meth:`_prepare_entry_for_publish` followed by
        :class:`~vyra_base.com.feeder.message_mapper.MessageMapper` for
        protocol-aware conversion.

        :param errorElement: The error entry to be fed. Can be a dictionary with error
            details or an :class:`~vyra_base.defaults.entries.ErrorEntry` object.
        :type errorElement: Union[ErrorEntry, dict]
        :raises FeederException: If the type of errorElement is neither a dict nor an
            :class:`~vyra_base.defaults.entries.ErrorEntry`.
        """
        if isinstance(errorElement, dict):
            errorfeed_entry = self.build_errorfeed(errorElement)
        elif isinstance(errorElement, ErrorEntry):
            errorfeed_entry = errorElement
        else:
            raise FeederException(f"Wrong Type. Expect: ErrorEntry, got {type(errorElement)}")

        # Ensure timestamp and uuid are set
        if errorfeed_entry.timestamp is None:
            errorfeed_entry.timestamp = datetime.now()
        if errorfeed_entry.uuid is None:
            errorfeed_entry.uuid = uuid.uuid4()

        await super().feed(errorfeed_entry)

    def _prepare_entry_for_publish(self, entry: ErrorEntry) -> dict:  # type: ignore[override]
        """
        Convert an :class:`~vyra_base.defaults.entries.ErrorEntry` to a
        wire-ready dict whose keys match the transport interface field names
        (``VBASEErrorFeed``).
        """
        return {
            'error_code': int(entry.code or 0),
            'module_id': str(entry.module_id) if entry.module_id else '',
            'description': str(entry.description or ''),
            'solution': str(entry.solution or ''),
            'miscellaneous': str(entry.miscellaneous or ''),
            'timestamp': entry.timestamp or datetime.now(),
        }

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
            module_id=self._module_entity.uuid,
            module_name=self._module_entity.name,
            uuid=errorDict.get('uuid', uuid.uuid4()),
            timestamp=datetime.now(),
            description=errorDict.get('description', ''),
            solution=errorDict.get('solution', ''),
            miscellaneous=errorDict.get('miscellaneous', ''),
            level=errorDict.get('level', ErrorEntry.ERROR_LEVEL.MINOR_FAULT).value
        )
        return errorfeed_entry
