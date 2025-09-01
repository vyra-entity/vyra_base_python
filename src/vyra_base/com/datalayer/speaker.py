from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Union

from vyra_base.helper.logger import Logger

from .publisher import VyraPublisher
from .subscriber import VyraSubscriber

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
            Logger.debug(f"{self.name} is shouting: {message}")
            self.last_send = message
        else:
            raise ValueError("No publisher server available to send the message.")


@dataclass
class VyraSpeakerListener:
    """
    Represents a speaker listener in the system.
    A speaker listener is responsible for receiving messages or data from a topic using a subscriber server.
    :param name: Name of the speaker listener.
    :type name: str
    :param type: Type of the speaker listener, typically a ROS2 message type.
    :type type: Any
    :param description: Description of the speaker listener.
    :type description: str
    :param publisher_server: The publisher server associated with the speaker listener.
    :type publisher_server: VyraPublisher or None
    :param last_send: The last sent message or data.
    :type last_send: Any
    """
    name: str = "vyra_speaker"
    type: Any = None
    description: str = ""
    subscriber_server: Union[VyraSubscriber, None] = None
    last_send: Any = None

    def __del__(self):
        """
        Destructor to clean up the speaker.
        If the speaker has a publisher server, it will be destroyed.
        """
        if self.subscriber_server:
            self.subscriber_server.remove_subscription()
            self.subscriber_server = None

    def merge(self, other: VyraSpeakerListener) -> VyraSpeakerListener:
        """
        Merges another speaker listener into this one, combining their attributes.

        :param other: Another speaker listener instance to merge with.
        :type other: VyraSpeakerListener
        :return: This speaker listener instance with merged attributes.
        :rtype: VyraSpeakerListener
        """
        self.name = other.name or self.name
        self.type = other.type or self.type
        self.description = other.description or self.description
        self.subscriber_server = other.subscriber_server or self.subscriber_server
        self.last_send = other.last_send or self.last_send

        return self

    def listen(self) -> Any:
        """
        Listens for a message from the speaker's subscriber server.

        :param timeout: The maximum time to wait for a message, in seconds.
        :type timeout: float
        :return: The received message, or None if no message was received within the timeout.
        :rtype: Any
        """
        if self.subscriber_server:
            self.subscriber_server.create_subscription()
        else:
            raise ValueError("No subscriber server available to listen for messages.")