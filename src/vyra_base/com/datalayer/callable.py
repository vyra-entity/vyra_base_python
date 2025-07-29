from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Union

from vyra_base.com.datalayer.service_server import VyraServiceServer
from vyra_base.helper.logger import Logger

@dataclass
class VyraCallable:
    """
    Represents and stores all information about a Vyra callable.

    A callable is a function that can be invoked with arguments and returns a result
    within a specified time limit. It is used to define quick responses to requests in Vyra.

    :param name: Name of the callable.
    :type name: str
    :param type: Type of the callable, typically a ROS2 service type.
    :type type: Any
    :param description: Description of the callable.
    :type description: str
    :param last_return: The last return value of the callable.
    :type last_return: Any
    :param service_server: The associated VyraServiceServer instance.
    :type service_server: VyraServiceServer or None
    :param connected_callback: Callback function when connected.
    :type connected_callback: Callable or None
    """
    name: str = ""
    type: Any = None
    description: str = ""
    last_return: Any = None
    service_server: Union[VyraServiceServer, None] = None
    connected_callback: Union[Callable, None] = None

    def __repr__(self) -> str:
        """
        Returns the string representation of the VyraCallable.

        :return: The name of the callable.
        :rtype: str
        """
        return self.name
    
    def merge(self, other: Any) -> VyraCallable:
        """
        Merge another Callable into this one, combining their attributes.

        :param other: Another Callable instance to merge with.
        :type other: Any
        :return: A new Callable instance with merged attributes.
        :rtype: VyraCallable
        """
        self.name = other.name or self.name
        self.type = other.type or self.type
        self.description = other.description or self.description
        self.last_return = other.last_return or self.last_return
        self.service_server = other.service_server or self.service_server
        self.connected_callback = other.connected_callback or self.connected_callback

        return self
    
    def __del__(self):
        """
        Destructor to clean up the callable.
        If the callable has a service server, it will be destroyed.
        """
        if self.service_server:
            self.service_server.destroy_service()
            self.service_server = None