from __future__ import annotations

from typing import Any
from typing import Callable
from typing import Union
from datetime import datetime
from dataclasses import dataclass, field
from uuid import UUID

from rclpy.qos import QoSProfile

from enum import Enum
from re import DEBUG

class AvailableModuleEntry:
    pass


@dataclass(slots=True)
class ErrorEntry:
    """ Contains the database responses.

        Primitive Python object that stores the object values.
    """
    error_code: str
    module_id: str
    date: datetime
    type: Any
    description: str=''
    solution: str=''
    miscellaneous: str=''


class FunctionConfigParamTypes(Enum):
        # Primitive Types
    BOOL = "bool"
    BYTE = "byte"
    CHAR = "char"
    INT8 = "int8"
    UINT8 = "uint8"
    INT16 = "int16"
    UINT16 = "uint16"
    INT32 = "int32"
    UINT32 = "uint32"
    INT64 = "int64"
    UINT64 = "uint64"
    FLOAT32 = "float32"
    FLOAT64 = "float64"
    STRING = "string"

    # Time Types
    TIME = "time"
    DURATION = "duration"

    # Any Types (Other user defined types)
    ANY = "any"

    # Special Types
    NONE = "none"

    # Collection Types
    ARRAY = "array"


class FunctionConfigBaseTypes(Enum):
    """ Enum for the function configuration base types. """
    speaker = 'speaker'
    callable = 'callable'
    job = 'job'


class FunctionConfigBaseParams(Enum):
    """ Enum for the function configuration base parameters. """
    datatype: FunctionConfigParamTypes
    displayname: str
    description: str


class FunctionConfigBaseReturn(Enum):
    """ Enum for the function configuration base parameters. """
    datatype: FunctionConfigParamTypes
    displayname: str
    description: str


class FunctionConfigDisplaystyle(Enum):
    """ Enum for the function configuration display style. """
    visible: bool
    published: bool

@dataclass(slots=True)
class FunctionConfigPeriodicSpeaker:
    """ Enum for the function configuration periodic speaker. """
    caller: Callable  # Interval in seconds
    interval: float = 1.0  # Time interval in seconds

    def __post_init__(self):
        if not (0.01 <= self.interval <= 10.0):
            raise ValueError(f"speed {self.interval} out of bounds (0.01–10.0)")

@dataclass(slots=True)
class FunctionConfigEntry:
    """ Contains the function configuration.

        Primitive Python object that stores the function settings.
        This object is used to make funciton configuration typesafe
        and available for adding dds communication nodes for other 
        modules.
        Makes use of the memory saving __slots__.

        Attributes:
            tags (list[str]): Tags for the function.
            type (FunctionConfigBaseTypes): Type of the function.
            ros2type (Any): ROS2 type of the function.
            functionname (str): Name of the function.
            displayname (str): Display name of the function.
            description (str): Description of the function.
            displaystyle (FunctionConfigDisplaystyle): Display style of the function.
            params (list[FunctionConfigBaseParams]): Parameters of the function.
            returns (list[FunctionConfigBaseReturn]): Return values of the function.
            qosprofile (Union[int, QoSProfile]) [default=10]: Quality of Service profile 
                for the function.
            callback (Callable, optional): Only !vos-callable!. Function to be called 
                when the function is invoked.
            periodic (FunctionConfigPeriodicSpeaker, optional): Only !vos-speaker!. 
                Function to be called periodically.
    """
    tags: list[str]
    type: FunctionConfigBaseTypes
    ros2type: Any
    functionname: str
    displayname: str
    description: str
    displaystyle: FunctionConfigDisplaystyle
    params: list[FunctionConfigBaseParams] = field(default_factory=list)
    returns: list[FunctionConfigBaseReturn] = field(default_factory=list)
    qosprofile: Union[int, QoSProfile] = 10
    callback: Union[Callable, None] = None
    periodic: Union[FunctionConfigPeriodicSpeaker, None] = None


class NewsFeedEntry(dict):
    """ News feed entry object. 
       
        Object that writes informational data to the graphical user interface.
        Makes use of the memory saving __slots__. 
    """
    __slots__ = (
        'level', 
        'message', 
        'timestamp', 
        'uuid', 
        'module_name', 
        'module_template'
        'type'
    )
    def __init__(
            self, 
            level: NewsFeedEntry.MESSAGE_TYPE, 
            message: str, 
            timestamp: datetime, 
            uuid: UUID, 
            module_name: str,
            module_template: str,
            type: Any
        ) -> None:

        self.message: NewsFeedEntry.Body = self.Body(level, message)
        self.timestamp: datetime = timestamp
        self.uuid: UUID = uuid
        self.module_name: str = module_name
        self.module_template: str = module_template
        self.type: Any = type


    class MESSAGE_TYPE(Enum):
        """ Enum for the message type. """
        ACTION = 'action'
        INFO = 'info'
        DEBUG = 'debug'
        WARNING = 'warning'
        HINT = 'hint'
    
    class Body:
        """ Message body object. """
        __slots__ = ('level', 'message')
        def __init__(self, level: NewsFeedEntry.MESSAGE_TYPE, message: str) -> None:
            self.level: NewsFeedEntry.MESSAGE_TYPE = level
            self.message: str = message


@dataclass(slots=True)
class PullRequestEntry:
    """ Contains the database responses.

        Primitive Python object that stores the object values.
    """
    uuid: str
    ack_by_user: str
    ack_on_date: str 
    request_structure: str
    request_on_date: str 
    request_description: str 
    response: str
    request_action:str 
    request_action_args: list 
    module_id: str
    color:int
    type: Any


@dataclass(repr=True, slots=True)
class StateEntry:
    """ Contains the database responses.

        Primitive Python object that stores the object values.
        Makes use of the memory saving __slots__.
    """
    previous: str
    current: str 
    module_uuid: UUID
    module_name: str
    timestamp: datetime
    type: Any


@dataclass(slots=True)
class ModuleEntry:
    """ Contains the module information.

        Primitive Python object that stores the object values.
        Makes use of the memory saving __slots__.
    """
    uuid: UUID
    name: str
    template: str
    description: str
    version: str
    