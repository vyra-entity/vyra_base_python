from logging import LogRecord
from typing import Any

from rclpy.node import Node

from vyra_base.com.communication_handler import CommunicationHandler
from vyra_base.com.datalayer.publisher import VyraPublisher
from vyra_base.helper.logger import Logger
from vyra_base.helper.logger import LogEntry
from vyra_base.helper.error_handler import ErrorTraceback


class ROS2Handler(CommunicationHandler):
    """
    Abstract class for all DDS communication handlers.

    This class provides the required interface for all DDS communication handlers
    to work with the Feeder class.

    :cvar __handlerName__: Name of the handler.
    :cvar __doc__: Description of the handler.
    """

    __handlerName__: str = 'ROS2Handler'
    __doc__: str = 'ROS2 communication handler'

    def __init__(self, initiator: str, publisher: VyraPublisher, type: Any):
        """
        Initialize the ROS2Handler.

        :param initiator: The initiator of the handler.
        :type initiator: str
        :param publisher: The publisher instance to use.
        :type publisher: VyraPublisher
        :param type: The ROS2 message type.
        :type type: Any
        """
        self._initiator = initiator
        self._publisher: VyraPublisher = publisher
        self._type: Any = type
        super().__init__()

    def emit(self, record: LogRecord):
        """
        Publish a log record as a ROS2 message.

        :param record: The log record to publish.
        :type record: logging.LogRecord
        :return: None
        """
        try:
            Logger.debug(
                f"{self._initiator} instruct {ROS2Handler.__handlerName__} "
                f"publish -> {record.msg}"
            )

            record_msg: Any = record.msg
            ros_msg: Any = self._type()

            record_fields = [f.lstrip('_') for f in record_msg.__slots__]
            type_fields = [f.lstrip('_') for f in self._type.__slots__]

            for field in type_fields:
                value = getattr(record_msg, field, '__not_found__')
                if value == '__not_found__':
                    Logger.log(
                        LogEntry(
                            f"Ros2-Field '{field}' not found in msg-type "
                            f"{record_msg.__class__.__name__}. "
                            "Abort publishing this message."
                        ).warn()
                    )
                    return None

                setattr(ros_msg, field, value)

            self._publisher.publish(ros_msg)
        finally:
            ErrorTraceback.check_error_exist()