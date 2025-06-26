import logging

from typing import Any

from .feeder import BaseFeeder
from vyra_base.com.datalayer.node import VyraNode
from vyra_base.com.datalayer.typeconverter import Ros2TypeConverter
from vyra_base.defaults.entries import ModuleEntry
from vyra_base.defaults.entries import StateEntry
from vyra_base.defaults.exceptions import FeederException
from vyra_base.com.ros2_handler import ROS2Handler
from vyra_base.helper.logger import Logger

class StateFeeder(BaseFeeder):
    """ Resposible for loading a ROS2 Handler and feeding StateEntry elements to
    this handler. """

    def __init__(
            self, 
            type: Any,
            node: VyraNode,
            module_config: ModuleEntry,
            loggingOn: bool = False,
            ):
        
        """
        Initializes a StateFeeder instance for sending changes in state of a module.
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

        self._feederName: str = 'StateFeeder'
        self._doc: str = 'Collect states from this module.'
        self._level: int = logging.INFO
        self._type: Any = type
        self._node: VyraNode = node
        self._module_config: ModuleEntry = module_config

        self._handler.append(ROS2Handler)

        self.create(loggingOn=loggingOn)
        
    def feed(self, stateElement: StateEntry) -> None:
        """Adds value to the logger and the remote handler"""

        if isinstance(stateElement, StateEntry):
            
            stateElement.timestamp = Ros2TypeConverter.time_to_ros2buildintime(
                stateElement.timestamp)
            
            stateElement.module_id = Ros2TypeConverter.uuid_to_ros2uuid(
                self._module_config.uuid)

            super().feed(stateElement)
        else:
            raise FeederException(f"Wrong Type. Expect: StateEntry, got {type(stateElement)}")


