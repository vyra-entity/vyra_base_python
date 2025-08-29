from __future__ import annotations

import uuid
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum, IntEnum
from re import DEBUG
from typing import Any, Callable, Union
from uuid import UUID

from rclpy.qos import QoSProfile


class FunctionConfigParamTypes(Enum):
    """
    Enum for the function configuration parameter types.

    Represents the types of parameters that can be used in a function configuration.

    :cvar BOOL: Boolean type
    :cvar BYTE: Byte type
    :cvar CHAR: Char type
    :cvar INT8: 8-bit integer
    :cvar UINT8: 8-bit unsigned integer
    :cvar INT16: 16-bit integer
    :cvar UINT16: 16-bit unsigned integer
    :cvar INT32: 32-bit integer
    :cvar UINT32: 32-bit unsigned integer
    :cvar INT64: 64-bit integer
    :cvar UINT64: 64-bit unsigned integer
    :cvar FLOAT32: 32-bit float
    :cvar FLOAT64: 64-bit float
    :cvar STRING: String type
    :cvar TIME: Time type
    :cvar DURATION: Duration type
    :cvar ANY: Any type (user defined)
    :cvar NONE: None type
    :cvar ARRAY: Array type
    """
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
    TIME = "time"
    DURATION = "duration"
    ANY = "any"
    NONE = "none"
    ARRAY = "array"


class FunctionConfigBaseTypes(Enum):
    """
    Enum for the function configuration base types.

    Represents the types of a function configuration.

    :cvar speaker: Represents a speaker function (simple publisher)
    :cvar callable: Represents a callable function (request-reply pattern)
    :cvar job: Represents a job function (request-feedback-reply pattern)
    """
    speaker = 'speaker'
    callable = 'callable'
    job = 'job'


@dataclass(slots=True)
class FunctionConfigBaseParams:
    """
    Enum for the function configuration base parameters.

    Represents the parameters of a function configuration.

    :ivar datatype: The data type of the parameter (:class:`FunctionConfigParamTypes`)
    :ivar displayname: The display name of the parameter
    :ivar description: A description of the parameter
    """
    datatype: FunctionConfigParamTypes
    displayname: str
    description: str

    def __dict__(self) -> dict:
        return {
            "datatype": dict(self.datatype),
            "displayname": self.displayname,
            "description": self.description
        }


@dataclass(slots=True)
class FunctionConfigBaseReturn:
    """
    Enum for the function configuration base return values.

    Represents the return values of a function configuration.

    :ivar datatype: The data type of the return value (:class:`FunctionConfigParamTypes`)
    :ivar displayname: The display name of the return value
    :ivar description: A description of the return value
    """
    datatype: FunctionConfigParamTypes
    displayname: str
    description: str

    def __dict__(self) -> dict:
        return {
            "datatype": dict(self.datatype),
            "displayname": self.displayname,
            "description": self.description
        }


@dataclass(slots=True)
class FunctionConfigDisplaystyle:
    """
    Enum for the function configuration display style.

    Defines how the function is displayed in the GUI.

    :ivar visible: If True, the function is visible in the GUI
    :ivar published: If True, the function is published in the GUI
    """
    visible: bool = False
    published: bool = False

    def __dict__(self) -> dict:
        return {
            "visible": self.visible,
            "published": self.published
        }


@dataclass(slots=True)
class FunctionConfigPeriodicSpeaker:
    """
    Stores the periodic speaker settings.

    :param caller: The function to be called periodically
    :type caller: Callable
    :param interval: The time interval in seconds for the periodic call (default: 1.0, must be between 0.01 and 10.0)
    :type interval: float

    :raises ValueError: If the interval is not between 0.01 and 10.0 seconds
    """
    caller: Callable
    interval: float = 1.0

    def __post_init__(self):
        if not (0.01 <= self.interval <= 10.0):
            raise ValueError(f"speed {self.interval} out of bounds (0.01–10.0)")

    def __dict__(self) -> dict:
        return {
            "caller": self.caller.__name__,
            "interval": self.interval
        }

@dataclass(slots=True)
class FunctionConfigEntry:
    """
    Contains the function configuration.

    Stores the function settings for type safety and DDS communication node configuration.

    :param tags: Tags for the function
    :type tags: list[str]
    :param type: Type of the function
    :type type: FunctionConfigBaseTypes
    :param ros2type: ROS2 type of the function
    :type ros2type: Any
    :param functionname: Name of the function
    :type functionname: str
    :param displayname: Display name of the function
    :type displayname: str
    :param description: Description of the function
    :type description: str
    :param displaystyle: Display style of the function
    :type displaystyle: FunctionConfigDisplaystyle
    :param params: Parameters of the function
    :type params: list[FunctionConfigBaseParams]
    :param returns: Return values of the function
    :type returns: list[FunctionConfigBaseReturn]
    :param qosprofile: Quality of Service profile (default: 10)
    :type qosprofile: Union[int, QoSProfile]
    :param callback: Function to be called when the function is invoked (only for callable)
    :type callback: Callable, Optional
    :param periodic: Function to be called periodically (only for speaker)
    :type periodic: FunctionConfigPeriodicSpeaker, Optional
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

    def __dict__(self) -> dict:
        return {
            "tags": self.tags,
            "type": dict(self.type),
            "ros2type": self.ros2type,
            "functionname": self.functionname,
            "displayname": self.displayname,
            "description": self.description,
            "displaystyle": dict(self.displaystyle),
            "params": [dict(p) for p in self.params],
            "returns": [dict(r) for r in self.returns],
            "qosprofile": self.qosprofile,
            "callback": self.callback.__name__ if self.callback else "N/A",
            "periodic": dict(self.periodic) if self.periodic else "N/A",
        }


@dataclass(slots=True)
class ErrorEntry:
    """
    Container for the error entry.

    Stores the error details.

    :param _type: The ros2 type of the error entry
    :type _type: Any
    :param level: The level of the error (default: 0, MINOR_FAULT)
    :type level: Union[ErrorEntry.ERROR_LEVEL, int]
    :param code: The error code (default: 0x00000000)
    :type code: int
    :param uuid: Unique identifier for the error entry
    :type uuid: Union[UUID, Any]
    :param timestamp: Timestamp of when the error occurred
    :type timestamp: Any
    :param description: Description of the error
    :type description: str
    :param solution: Suggested solution for the error
    :type solution: str
    :param miscellaneous: Additional information about the error
    :type miscellaneous: str
    :param module_name: Name of the module where the error occurred
    :type module_name: str
    :param module_id: Unique identifier for the module
    :type module_id: Union[UUID, Any]
    """
    _type: Any
    level: Union["ErrorEntry.ERROR_LEVEL", int] = 0
    code: int = 0x00000000
    uuid: Union[UUID, Any] = field(default_factory=uuid.uuid4)
    timestamp: Any = ''
    description: str = ''
    solution: str = ''
    miscellaneous: str = ''
    module_name: str = 'N/A'
    module_id: Union[UUID, Any] = 'N/A'

    def __repr__(self) -> str:
        return (
            f"ErrorEntry(level={self.level}, code={self.code}, "
            f"description={self.description}, solution={self.solution}"
        )

    class ERROR_LEVEL(IntEnum):
        """
        Enum for the error level.

        :cvar MINOR_FAULT: Minor fault (0)
        :cvar MAJOR_FAULT: Major fault (1)
        :cvar CRITICAL_FAULT: Critical fault (2)
        :cvar EMERGENCY_FAULT: Emergency fault (3)
        """
        MINOR_FAULT = 0
        MAJOR_FAULT = 1
        CRITICAL_FAULT = 2
        EMERGENCY_FAULT = 3


class NewsEntry(dict):
    """
    News feed entry object.

    Writes informational data to the graphical user interface.

    :param _type: The type of the news entry
    :type _type: Any
    :param level: The level of the message (default: 2, INFO)
    :type level: Union[NewsEntry.MESSAGE_LEVEL, int]
    :param message: The message content
    :type message: str
    :param timestamp: Timestamp of when the message was created
    :type timestamp: Any
    :param uuid: Unique identifier for the news entry
    :type uuid: Union[UUID, Any]
    :param module_name: Name of the module that created the news entry
    :type module_name: str
    :param module_id: Unique identifier for the module
    :type module_id: Union[UUID, Any]
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
        level: Union["NewsEntry.MESSAGE_LEVEL", int] = 2,
        message: str = '',
        timestamp: Any = datetime.now(),
        uuid: Union[UUID, Any] = uuid.uuid4(),
        module_name: str = 'N/A',
        module_id: Union[UUID, Any] = uuid.uuid4(),
    ) -> None:
        self.message: str = message
        self.level: Union[NewsEntry.MESSAGE_LEVEL, int] = level
        self.timestamp: Union[datetime, Any] = timestamp
        self.uuid: Union[UUID, Any] = uuid
        self.module_name: str = module_name
        self.module_id: Union[UUID, Any] = module_id
        self._type: Any = _type

    def __repr__(self) -> str:
        return (
            f"NewsEntry(level={self.level}, message={self.message}"
        )

    class MESSAGE_LEVEL(IntEnum):
        """
        Enum for the message type.

        :cvar ACTION: Action message (0)
        :cvar DEBUG: Debug message (1)
        :cvar INFO: Info message (2)
        :cvar HINT: Hint message (3)
        :cvar WARNING: Warning message (4)
        """
        ACTION = 0
        DEBUG = 1
        INFO = 2
        HINT = 3
        WARNING = 4


@dataclass(slots=True)
class PullRequestEntry:
    """
    Contains the pull request entry.

    Stores the pull request details.

    :param _type: The ros2 type of the pull request entry
    :type _type: Any
    :param uuid: Unique identifier for the pull request
    :type uuid: str
    :param ack_by_user: User who acknowledged the pull request
    :type ack_by_user: str
    :param ack_on_date: Date when the pull request was acknowledged
    :type ack_on_date: str
    :param request_structure: Structure of the request
    :type request_structure: str
    :param request_on_date: Date when the request was made
    :type request_on_date: str
    :param request_description: Description of the request
    :type request_description: str
    :param response: Response to the request
    :type response: str
    :param request_action: Action requested in the pull request
    :type request_action: str
    :param request_action_args: Arguments for the requested action
    :type request_action_args: list
    :param module_id: Unique identifier for the module associated with the pull request
    :type module_id: str
    :param color: Color code for visual representation in GUI
    :type color: int
    """
    _type: Any
    uuid: str
    ack_by_user: str
    ack_on_date: str
    request_structure: str
    request_on_date: str
    request_description: str
    response: str
    request_action: str
    request_action_args: list
    module_id: str
    color: int

    def __repr__(self) -> str:
        return (
            f"PullRequestEntry(uuid={self.uuid}, ack_by_user={self.ack_by_user}, "
            f"ack_on_date={self.ack_on_date}, request_structure={self.request_structure}, "
            f"request_on_date={self.request_on_date}, request_description={self.request_description}, "
            f"response={self.response}, request_action={self.request_action}, "
            f"request_action_args={self.request_action_args}, module_id={self.module_id}, "
            f"color={self.color})"
        )

@dataclass(repr=True, slots=True)
class StateEntry:
    """
    Contains the state entry.

    Stores the state details for life cycle overview of the module.

    :param _type: The ros2 type of the state entry
    :type _type: Any
    :param current: Current state of the module
    :type current: str
    :param trigger: Trigger that caused the state change
    :type trigger: str
    :param module_id: Unique identifier for the module
    :type module_id: Union[UUID, Any]
    :param module_name: Name of the module
    :type module_name: str
    :param timestamp: Timestamp of when the state was recorded
    :type timestamp: Any
    :param previous: Previous state of the module (default: 'N/A')
    :type previous: str
    """
    _type: Any
    current: str
    trigger: str
    module_id: Union[UUID, Any]
    module_name: str
    timestamp: Any
    previous: str = 'N/A'

    def __repr__(self) -> str:
        return (
            f"StateEntry(current={self.current}, trigger={self.trigger}, "
            f"previous={self.previous})"
        )

@dataclass(slots=True)
class ModuleEntry:
    """
    Contains the module entry.

    Stores the module details.

    :param uuid: Unique identifier for the module
    :type uuid: UUID
    :param name: Name of the module
    :type name: str
    :param template: Template identifier for the module
    :type template: str
    :param description: Description of the module
    :type description: str
    :param version: Version of the module (semantic versioning, e.g. '1.0.0')
    :type version: str
    """
    uuid: str
    name: str
    template: str
    description: str
    version: str

    def __repr__(self) -> str:
        return (
            f"ModuleEntry(uuid={self.uuid}, name={self.name}, "
            f"template={self.template}, description={self.description}, "
            f"version={self.version})"
        )
    
    def to_dict(self) -> dict:
        """
        Returns the dictionary representation of the ModuleEntry.

        :return: Dictionary containing the module details.
        :rtype: dict
        """
        return {
            'uuid': str(self.uuid),
            'name': self.name,
            'template': self.template,
            'description': self.description,
            'version': self.version
        }

    @staticmethod
    def gen_uuid() -> str:
        """Generates a new UUID for the module entry in hex format"""
        return uuid.uuid4().hex

    def restructure_uuid(self) -> UUID:
        """
        Converts the UUID string to a UUID object.

        :return: UUID object of the module entry.
        :rtype: UUID
        """
        return uuid.UUID(hex=self.uuid)


@dataclass(slots=True)
class AvailableModuleEntry:
    """
    Not used in the current implementation.
    """
    pass
