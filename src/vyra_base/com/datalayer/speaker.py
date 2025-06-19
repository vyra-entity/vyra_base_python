from __future__ import annotations

from dataclasses import dataclass
from typing import Any
from typing import Union
from typing import Callable

from .publisher import VyraPublisher

@dataclass
class VyraSpeaker:
    """
    Class to represent a speaker in the system.
    """
    description: str = ""
    publisher_server: Union[VyraPublisher, None] = None
    last_send: Any = None

    def merge(self, other: VyraSpeaker) -> VyraSpeaker:
        """
        Merge another Speaker into this one, combining their attributes.
        :param other: Another Speaker instance to merge with.
        :return: A new Speaker instance with merged attributes.
        """
        self.description = other.description or self.description
        self.publisher_server = other.publisher_server or self.publisher_server
        self.last_send = other.last_send or self.last_send

        return self