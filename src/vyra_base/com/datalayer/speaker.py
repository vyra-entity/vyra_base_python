from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Union

from vyra_base.helper.logger import Logger

from .publisher import VyraPublisher

@dataclass
class VyraSpeaker:
    """
    Represents a speaker in the system.
    A speaker is responsible for sending messages or data to a topic using a publisher server.
    :param name: Name of the speaker.
    :type name: str
    :param type: Type of the speaker, typically a ROS2 message type.
    :type type: Any
    :param description: Description of the speaker.
    :type description: str
    :param publisher_server: The publisher server associated with the speaker.
    :type publisher_server: VyraPublisher or None
    :param last_send: The last sent message or data.
    :type last_send: Any
    """
    name: str = "vyra_speaker"
    type: Any = None
    description: str = ""
    publisher_server: Union[VyraPublisher, None] = None
    last_send: Any = None

    def __del__(self):
        """
        Destructor to clean up the speaker.
        If the speaker has a publisher server, it will be destroyed.
        """
        if self.publisher_server:
            self.publisher_server.destroy()
            self.publisher_server = None
            Logger.info(f"VyraSpeaker '{self.name}' destroyed.")

    def merge(self, other: VyraSpeaker) -> VyraSpeaker:
        """
        Merges another speaker into this one, combining their attributes.

        :param other: Another speaker instance to merge with.
        :type other: VyraSpeaker
        :return: This speaker instance with merged attributes.
        :rtype: VyraSpeaker
        """
        self.name = other.name or self.name
        self.type = other.type or self.type
        self.description = other.description or self.description
        self.publisher_server = other.publisher_server or self.publisher_server
        self.last_send = other.last_send or self.last_send

        return self
    
    def shout(self, message: Any) -> None:
        """
        Sends a message using the speaker's publisher server.

        :param message: The message to send.
        :type message: Any
        """
        if self.publisher_server:
            self.publisher_server.publish(message)
            self.last_send = message
        else:
            raise ValueError("No publisher server available to send the message.")