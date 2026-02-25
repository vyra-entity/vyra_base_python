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
from vyra_base.com.core.interface_path_registry import get_interface_registry

logger = logging.getLogger(__name__)


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

        # _feederName must match the "functionname" in the interface config JSON
        self._feederName: str = 'StateFeed'
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
        """Starts the feeder by initializing handlers.

        Automatically resolves the transport protocol from the module's
        interface config (reads ``StateFeed`` entry, picks ``tags``).
        """
        # Provide interface paths from module entity if available
        paths = get_interface_registry().get_interface_paths()
        if paths:
            self.set_interface_paths(paths)
        await self.create(loggingOn=self._loggingOn)
        
    async def feed(self, stateElement: StateEntry) -> None:
        """
        Feed a state entry to the feeder.

        Validates the input type, then delegates to :meth:`~BaseFeeder.feed`
        which calls :meth:`_prepare_entry_for_publish` followed by
        :class:`~vyra_base.com.feeder.message_mapper.MessageMapper` for
        protocol-aware conversion.

        :param stateElement: The state entry to feed.
        :type stateElement: StateEntry
        :raises FeederException: If the provided element is not of type StateEntry.
        """
        if not isinstance(stateElement, StateEntry):
            raise FeederException(f"Wrong Type. Expect: StateEntry, got {type(stateElement)}")
        await super().feed(stateElement)

    def feed_sync(self, msg: Any) -> None:
        return super().feed_sync(msg)

    def _prepare_entry_for_publish(self, entry: StateEntry) -> dict:  # type: ignore[override]
        """
        Convert a :class:`~vyra_base.defaults.entries.StateEntry` to a
        wire-ready dict whose keys match the transport interface field names
        (``VBASEStateFeed``).
        """
        from datetime import datetime as _dt
        return {
            'previous': str(entry.previous) if entry.previous else '',
            'current': str(entry.current) if entry.current else '',
            'timestamp': entry.timestamp if entry.timestamp else _dt.now(),
        }
