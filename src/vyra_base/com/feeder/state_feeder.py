import logging
from typing import Any, Optional

# Check ROS2 availability
try:
    import rclpy
    _ROS2_AVAILABLE = True
except ImportError:
    _ROS2_AVAILABLE = False

if _ROS2_AVAILABLE:
    from vyra_base.com.transport.t_ros2 import Ros2TypeConverter
    from vyra_base.com.handler.ros2 import ROS2Handler
else:
    Ros2TypeConverter = None
    ROS2Handler = None

from .feeder import BaseFeeder
from vyra_base.defaults.entries import ModuleEntry
from vyra_base.defaults.entries import StateEntry
from vyra_base.defaults.exceptions import FeederException
from vyra_base.helper.logger import Logger

class StateFeeder(BaseFeeder):
    """
    Responsible for loading a ROS2 Handler and feeding StateEntry elements to this handler.

    :param type: The ros2-msg type for the feeder.
    :type type: Any
    :param node: The VyraNode instance associated with this feeder (ROS2 Node).
    :type node: VyraNode
    :param module_entity: Module configuration entry.
    :type module_entity: ModuleEntry
    :param loggingOn: Flag to enable or disable logging next to feeding. Defaults to False.
    :type loggingOn: bool, Optional

    :raises FeederException: If the VyraSpeaker cannot be created with the given type.

    :ivar _feederName: Name of the feeder.
    :ivar _doc: Documentation string for the feeder.
    :ivar _level: Logging level.
    :ivar _type: ROS2 message type.
    :ivar _node: VyraNode instance.
    :ivar _module_entity: Module configuration.
    """

    def __init__(
            self, 
            type: Any,
            node: Optional[Any],
            module_entity: ModuleEntry,
            loggingOn: bool = False,
            ):
        """
        Initializes a StateFeeder instance for sending changes in state of a module.

        :param type: The ros2-msg type for the feeder.
        :type type: Any
        :param node: The VyraNode instance (optional, None if ROS2 unavailable).
        :type node: Optional[Any]
        :param module_entity: Module configuration entry.
        :type module_entity: ModuleEntry
        :param loggingOn: Flag to enable or disable logging next to feeding. Defaults to False.
        :type loggingOn: bool, Optional

        :raises FeederException: If the VyraSpeaker cannot be created with the given type.
        """
        super().__init__()

        self._feederName: str = 'StateFeeder'
        self._doc: str = 'Collect states from this module.'
        self._level: int = logging.INFO
        self._type: Any = type
        self._node: Optional[Any] = node
        self._module_entity: ModuleEntry = module_entity
        self._ros2_available: bool = _ROS2_AVAILABLE and node is not None
        self._loggingOn: bool = loggingOn

        if self._ros2_available and ROS2Handler:
            self._handler_classes.append(ROS2Handler)
    
    async def start(self) -> None:
        """
        Starts the feeder by initializing handlers.
        """
        await self.create(loggingOn=self._loggingOn)
        
    def feed(self, stateElement: StateEntry) -> None:
        """
        Adds value to the logger and the remote handler.

        :param stateElement: The state entry to feed.
        :type stateElement: StateEntry

        :raises FeederException: If the provided element is not of type StateEntry.
        """
        if isinstance(stateElement, StateEntry):
            # Convert to ROS2 types only if ROS2 available
            if self._ros2_available and Ros2TypeConverter:
                stateElement.timestamp = Ros2TypeConverter.time_to_ros2buildintime(
                    stateElement.timestamp)
            # For non-ROS2, timestamp stays as datetime
            super().feed(stateElement)
        else:
            raise FeederException(f"Wrong Type. Expect: StateEntry, got {type(stateElement)}")
