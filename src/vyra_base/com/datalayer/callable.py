from __future__ import annotations

from dataclasses import dataclass
from typing import Any  
from typing import Callable
from typing import Union

from vyra_base.com.datalayer.service_server import VyraServiceServer

@dataclass
class VyraCallable:
    """
    A class that represents and stores all information about a VOS callable.
    A callable is a function that can be invoked with arguments and returns a result
    within a specified time limit. It is used to define quick responses
    to requests in the VOS.
    :param name: Name of the callable.
    :param type: Type of the callable, typically a ROS2 service type.
    :param 
    """
    name: str = ""
    type: Any = None
    description: str = ""
    last_return: Any = None
    service_server: Union[VyraServiceServer, None] = None
    connected_callback: Union[Callable, None] = None

    def __repr__(self) -> str:
        return self.name
    
    def merge(self, other: Any) -> VyraCallable:
        """
        Merge another Callable into this one, combining their attributes.
        :param other: Another Callable instance to merge with.
        :return: A new Callable instance with merged attributes.
        """
        self.name = other.name or self.name
        self.type = other.type or self.type
        self.description = other.description or self.description
        self.last_return = other.last_return or self.last_return
        self.service_server = other.service_server or self.service_server
        self.connected_callback = other.connected_callback or self.connected_callback

        return self
    
