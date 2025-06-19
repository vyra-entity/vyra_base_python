import asyncua

from collections import deque

from .feeder import BaseFeeder
from varioboticos_base.defaults.constants import FeederConstants
from varioboticos_base.defaults.entries import AvailableModuleEntry
from varioboticos_base.defaults.exceptions import FeederException


class AvailableModuleFeeder(BaseFeeder):
    """Collection if all available modules in the domain."""

    def __init__(self, queue_len=FeederConstants.MAX_DEFAULT_LENGTH.value):
        self.__feederName__: str = 'NewsFeeder'
        self.__doc__: str = 'Collect news, warning, hints, ... from this module.'
        self._feeds: deque = deque(maxlen=queue_len)

    async def load_data(self, value: AvailableModuleEntry) -> None:
        """Adds value to the local deque and the remote opcua server."""
        if isinstance(value, AvailableModuleEntry):
            await super().load_data(value)
        else:
            raise FeederException(f"Wrong Type. Expect: AvailableModuleFeeder, got {type(value)}")
