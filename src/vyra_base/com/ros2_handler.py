from rclpy.node import Node

from typing import Any
from logging import LogRecord

from vyra_base.com.communication_handler import CommunicationHandler
from vyra_base.com.datalayer.publisher import VyraPublisher
from vyra_base.helper.logger import Logger
from vyra_base.helper.logger import LogEntry
from vyra_base.helper.error_handler import ErrorTraceback

class ROS2Handler(CommunicationHandler):
    """ Abstract class for all DDS communication handlers.

        This class provides the required interface for all DDS communication handlers
        to work with the Feeder class.
    """
    __handlerName__: str = 'ROS2Handler'
    __doc__: str = 'ROS2 communication handler'

    def __init__(self, publisher: VyraPublisher, type: Any):
        self._publisher: VyraPublisher = publisher
        self._type: Any = type
        super().__init__()

    def emit(self, record: LogRecord):
        try:
            Logger.log("Emitting log record to ROS2 Handler")
            Logger.log(type(record.msg))
            record_msg = record.msg
            ros_msg: Any = self._type()

            record_fields = [f.lstrip('_') for f in record_msg.__slots__]
            type_fields = [f.lstrip('_') for f in self._type.__slots__]


            for field in type_fields:
                value = getattr(record_msg, field, '__not_found__')
                if value == '__not_found__':
                    Logger.log(
                        LogEntry(
                            f"Ros2-Field '{field}' not found in msg-type {record_msg.__class__.__name__}. "
                            "Abort publishing this message."
                        ).warn()
                    )
                    return None
                
                setattr(ros_msg, field, value)
           
            self._publisher.publish(ros_msg)
        finally:
            ErrorTraceback.check_error_exist()