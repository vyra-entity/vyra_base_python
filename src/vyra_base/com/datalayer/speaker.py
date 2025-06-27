from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Union

from .publisher import VyraPublisher

@dataclass
class VyraSpeaker:
    """
    Represents a speaker in the system.

    :param description: Description of the speaker.
    :type description: str
    :param publisher_server: The publisher server associated with the speaker.
    :type publisher_server: VyraPublisher or None
    :param last_send: The last sent message or data.
    :type last_send: Any
    """
    description: str = ""
    publisher_server: Union[VyraPublisher, None] = None
    last_send: Any = None

    def merge(self, other: VyraSpeaker) -> VyraSpeaker:
        """
        Merges another speaker into this one, combining their attributes.

        :param other: Another speaker instance to merge with.
        :type other: VyraSpeaker
        :return: This speaker instance with merged attributes.
        :rtype: VyraSpeaker
        """
        self.description = other.description or self.description
        self.publisher_server = other.publisher_server or self.publisher_server
        self.last_send = other.last_send or self.last_send

        return self