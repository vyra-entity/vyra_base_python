from logging import LogRecord
from typing import Any

from rclpy.node import Node

from vyra_base.com.communication_handler import CommunicationHandler
from vyra_base.com.datalayer.speaker import VyraSpeaker
from vyra_base.helper.logger import Logger
from vyra_base.helper.logger import LogEntry
from vyra_base.helper.error_handler import ErrorTraceback


class ROS2Handler(CommunicationHandler):
    """
    Abstract class for all DDS communication handlers.

    :cvar __handlerName__: Name of the handler.
    :type __handlerName__: str
    :cvar __doc__: Documentation string for the handler.
    :type __doc__: str
    :param initiator: The initiator of the handler.
    :type initiator: str
    :param speaker: The speaker instance to use.
    :type speaker: Vyraspeaker
    :param type: The ROS2 message type.
    :type type: Any
    """

    __handlerName__: str = 'ROS2Handler'
    __doc__: str = 'ROS2 communication handler'

    def __init__(self, initiator: str, speaker: VyraSpeaker, type: Any):
        """
        Initialize the ROS2Handler.

        :param initiator: The initiator of the handler.
        :type initiator: str
        :param speaker: The speaker instance to use.
        :type speaker: VyraSpeaker
        :param type: The ROS2 message type.
        :type type: Any
        """
        self._initiator = initiator
        self._speaker: VyraSpeaker = speaker
        self._type: Any = type
        super().__init__()

    def emit(self, record: LogRecord):
        try:
            Logger.debug(
                f"{self._initiator} instruct {ROS2Handler.__handlerName__} "
                f"publish -> {record.msg}"
            )

            record_msg: Any = record.msg
            ros_msg: Any = self._type()

            record_fields = [f.lstrip('_') for f in record_msg.__slots__]
            # type_fields = [f.lstrip('_') for f in self._type.__slots__]
            type_fields = list(self._type.get_fields_and_field_types().keys())

            for field in type_fields:
                value = getattr(record_msg, field, '__not_found__')
                if value == '__not_found__':
                    Logger.warn(
                        f"Ros2-Field '{field}' not found in msg-type "
                        f"{record_msg.__class__.__name__}. "
                        f"Needed fields are: {type_fields}. "
                        f"Provided message fields are: {record_fields}. "
                        "Abort publishing this message."
                    )
                    return None

                setattr(ros_msg, field, value)

            self._speaker.shout(ros_msg)
        finally:
            ErrorTraceback.check_error_exist()