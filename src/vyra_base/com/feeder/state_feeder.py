import logging
from typing import Any

from .feeder import BaseFeeder
from vyra_base.com.datalayer.node import VyraNode
from vyra_base.com.datalayer.typeconverter import Ros2TypeConverter
from vyra_base.com.ros2_handler import ROS2Handler
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
    :param module_config: Module configuration entry.
    :type module_config: ModuleEntry
    :param loggingOn: Flag to enable or disable logging next to feeding. Defaults to False.
    :type loggingOn: bool, optional

    :raises FeederException: If the VyraSpeaker cannot be created with the given type.

    :ivar _feederName: Name of the feeder.
    :ivar _doc: Documentation string for the feeder.
    :ivar _level: Logging level.
    :ivar _type: ROS2 message type.
    :ivar _node: VyraNode instance.
    :ivar _module_config: Module configuration.
    """

    def __init__(
            self, 
            type: Any,
            node: VyraNode,
            module_config: ModuleEntry,
            loggingOn: bool = False,
            ):
        """
        Initializes a StateFeeder instance for sending changes in state of a module.

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
        super().__init__()

        self._feederName: str = 'StateFeeder'
        self._doc: str = 'Collect states from this module.'
        self._level: int = logging.INFO
        self._type: Any = type
        self._node: VyraNode = node
        self._module_config: ModuleEntry = module_config

        self._handler.append(ROS2Handler)

        self.create(loggingOn=loggingOn)
        
    def feed(self, stateElement: StateEntry) -> None:
        """
        Adds value to the logger and the remote handler.

        :param stateElement: The state entry to feed.
        :type stateElement: StateEntry

        :raises FeederException: If the provided element is not of type StateEntry.
        """
        if isinstance(stateElement, StateEntry):
            stateElement.timestamp = Ros2TypeConverter.time_to_ros2buildintime(
                stateElement.timestamp)
            stateElement.module_id = Ros2TypeConverter.uuid_to_ros2uuid(
                self._module_config.uuid)
            super().feed(stateElement)
        else:
            raise FeederException(f"Wrong Type. Expect: StateEntry, got {type(stateElement)}")
