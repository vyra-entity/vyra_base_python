from __future__ import annotations
import uuid

from typing import Any
from typing import Any
from typing import Callable
from typing import Union
from datetime import datetime
from dataclasses import dataclass, field
from uuid import UUID

from enum import Enum
from enum import IntEnum
from re import DEBUG

from rclpy.qos import QoSProfile


class FunctionConfigParamTypes(Enum):
    """ Enum for the function configuration parameter types.
        Represents the types of parameters that can be used in a function configuration.
    """

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
    """ Enum for the function configuration base types. 
        Represents the types of a function configuration.
        Attributes:
            speaker (str): Represents a speaker function. 
                           Is a simple publisher
            callable (str): Represents a callable function. 
                            Is a request -reply pattern.
            job (str): Represents a job function. 
                       Is a request - feedback - reply pattern
                       as longrunner. Could be run infinitely
                       or until a certain condition is met.
        Values:
            speaker: Represents a speaker function.
            callable: Represents a callable function.
            job: Represents a job function.
        Usage:
            You can use this enum to define the type of a function configuration.
            For example, you can create a function configuration with type FunctionConfigBaseTypes.speaker.
    """
    speaker = 'speaker'
    callable = 'callable'
    job = 'job'


class FunctionConfigBaseParams(Enum):
    """ Enum for the function configuration base parameters. 
        Represents the parameters of a function configuration.
        Attributes:
            datatype (FunctionConfigParamTypes): The data type of the parameter.
            displayname (str): The display name of the parameter.
            description (str): A description of the parameter.
    """
    datatype: FunctionConfigParamTypes
    displayname: str
    description: str


class FunctionConfigBaseReturn(Enum):
    """ Enum for the function configuration base parameters. 
        Represents the return values of a function configuration.
        Attributes:
            datatype (FunctionConfigParamTypes): The data type of the return value.
            displayname (str): The display name of the return value.
            description (str): A description of the return value.
    """
    datatype: FunctionConfigParamTypes
    displayname: str
    description: str


class FunctionConfigDisplaystyle(Enum):
    """ Enum for the function configuration display style. 
        Defines how the function is displayed in the GUI.
        Attributes:
            visible (bool): If True, the function is visible in the GUI.
            published (bool): If True, the function is published in the GUI.
    """
    visible: bool
    published: bool

@dataclass(slots=True)
class FunctionConfigPeriodicSpeaker:
    """ Enum for the function configuration periodic speaker. 
        Primitive Python object that stores the periodic speaker settings.
        
        Attributes:
            caller (Callable): The function to be called periodically.
            interval (float): The time interval in seconds for the periodic call.
                              Must be between 0.01 and 10.0 seconds.
                              If not set, defaults to 1.0 seconds.
        Raises:
            ValueError: If the interval is not between 0.01 and 10.0 seconds.
    """
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
            callback (Callable, optional): Only !vyra-callable!. Function to be called 
                when the function is invoked.
            periodic (FunctionConfigPeriodicSpeaker, optional): Only !vyra-speaker!. 
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


@dataclass(slots=True)
class ErrorEntry:
    """ Container for the error entry.
         
          Primitive Python object that stores the error details.
          Makes use of the memory saving __slots__.
          
          Attributes:
                _type (Any): The ros2 type of the error entry.
                level (Union[ErrorEntry.ERROR_LEVEL, str]): The level of the error.
                code (int): The error code. If not set, defaults to 0x00000000.
                            If set, it should be a 32-bit integer.
                            It is used to identify the error type.
                            It can be used to categorize errors into different types.
                module_name (str): Name of the module where the error occurred.
                module_id (Union[UUID, str]): Unique identifier for the module.
                uuid (UUID): Unique identifier for the error entry.
                timestamp (Any): Timestamp of when the error occurred.
                description (str): Description of the error.
                solution (str): Suggested solution for the error.
                miscellaneous (str): Additional information about the error.
    """
    _type: Any
    level: Union[ERROR_LEVEL, int] = 0  # Default level is MINOR_FAULT(0)
    code: int= 0x00000000
    uuid: UUID= field(default_factory=uuid.uuid4)
    timestamp: Any= ''
    description: str=''
    solution: str=''
    miscellaneous: str=''
    module_name: str= 'N/A'
    module_id: Union[UUID, str]= 'N/A'

    class ERROR_LEVEL(IntEnum):
        """ Enum for the error level. """

        MINOR_FAULT = 0
        MAJOR_FAULT = 1
        CRITICAL_FAULT = 2
        EMERGENCY_FAULT = 3


class NewsEntry(dict):
    """   News feed entry object. 

        Object that writes informational data to the graphical user interface.
        Makes use of the memory saving __slots__. 

        Attributes:
            _type (Any): The type of the news entry.
            level (Union[NewsEntry.MESSAGE_LEVEL, str]): The level of the message.
            message (str): The message content.
            timestamp (Any): Timestamp of when the message was created.
            uuid (UUID): Unique identifier for the news entry.
            module_name (str): Name of the module that created the news entry.
            module_id (UUID): Unique identifier for the module.
    """
    __slots__ = (
        '_type',
        'level',
        'message',
        'timestamp',
        'uuid',
        'module_name',
        'module_id'
    )
    def __init__(
            self, 
            _type: Any,
            level: Union[NewsEntry.MESSAGE_LEVEL, int]= 2,  # Default level is INFO(2) 
            message: str= '', 
            timestamp: Any= datetime.now(), 
            uuid: UUID= uuid.uuid4(), 
            module_name: str = 'N/A',
            module_id: UUID = uuid.uuid4(),
        ) -> None:

        self.message: str = message
        self.level: Union[NewsEntry.MESSAGE_LEVEL, int] = level
        self.timestamp: datetime = timestamp
        self.uuid: UUID = uuid
        self.module_name: str = module_name
        self.module_id: UUID = module_id
        self._type: Any = _type


    class MESSAGE_LEVEL(IntEnum):
        """ Enum for the message type. """
        ACTION = 0
        DEBUG = 1
        INFO = 2
        HINT = 3
        WARNING = 4


@dataclass(slots=True)
class PullRequestEntry:
    """   Contains the pull request entry.

        Primitive Python object that stores the pull request details.
        Makes use of the memory saving __slots__.
        
        Attributes:
            _type (Any): The ros2 type of the pull request entry.
            uuid (str): Unique identifier for the pull request.
            ack_by_user (str): User who acknowledged the pull request.
            ack_on_date (str): Date when the pull request was acknowledged.
            request_structure (str): Structure of the request.
            request_on_date (str): Date when the request was made.
            request_description (str): Description of the request.
            response (str): Response to the request.
            request_action (str): Action requested in the pull request.
            request_action_args (list): Arguments for the requested action.
            module_id (str): Unique identifier for the module associated with the pull request.
            color (int): Color code for visual representation in GUI.
    """
    _type: Any
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


@dataclass(repr=True, slots=True)
class StateEntry:
    """   Contains the state entry.
        Primitive Python object that stores the state details.
        Makes use of the memory saving __slots__. For life cycle 
        overview of the module, read the documentation.

        Attributes:
            _type (Any): The ros2 type of the state entry.
            current (str): Current state of the module.
            trigger (str): Trigger that caused the state change.
                           This is usually a function call or an event.
            module_id (UUID): Unique identifier for the module.
            module_name (str): Name of the module.
            timestamp (Any): Timestamp of when the state was recorded.
            previous (str): Previous state of the module, defaults to 'N/A'.
    """
    _type: Any
    current: str 
    trigger: str
    module_id: UUID
    module_name: str
    timestamp: Any
    previous: str = 'N/A'

@dataclass(slots=True)
class ModuleEntry:
    """    Contains the module entry.
        Primitive Python object that stores the module details.
        Makes use of the memory saving __slots__.

        Attributes:
            uuid (UUID): Unique identifier for the module.
            name (str): Name of the module.
            template (str): Template identifier for the module.
            description (str): Description of the module.
            version (str): Version of the module. Details see VERSIONING.md.
                           Using semantic versioning, e.g. '1.0.0'.
    """
    uuid: UUID
    name: str
    template: str
    description: str
    version: str

# tbd: This class is not used in the current implementation.
@dataclass(slots=True)
class AvailableModuleEntry:
    pass
