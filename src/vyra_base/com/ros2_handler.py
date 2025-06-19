from rclpy.node import Node

from typing import Any

from vyra_base.com.communication_handler import CommunicationHandler
from vyra_base.com.datalayer.publisher import VyraPublisher

class ROS2Handler(CommunicationHandler):
    """ Abstract class for all DDS communication handlers.

        This class provides the required interface for all DDS communication handlers
        to work with the Feeder class.
    """
    __handlerName__: str = 'DDSHandler'
    __doc__: str = 'DDS communication handler'


    def __init__(self, publisher: VyraPublisher, type: Any):
        self._publisher: VyraPublisher = publisher
        self._type: Any = type
        super().__init__()

    def emit(self, record: Any):
        try:
            log_msg: Any = self.format(record)
            ros_msg: Any = self._type()
            ros_msg.data = log_msg
            self._publisher.publish(ros_msg)
        except Exception:
            self.handleError(record)