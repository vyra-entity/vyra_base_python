
from http import client
import rclpy

from dataclasses import dataclass
from rclpy.client import Client

from typing import Any
from typing import Callable
from typing import NoReturn
from typing import Union

from vyra_base.com.datalayer.node import VyraNode

def _base_callback(*args, **kwargs) -> NoReturn:
    raise NotImplementedError("No callback provided for service client.")

def _base_request(*args, **kwargs) -> NoReturn:
    raise NotImplementedError("No request function provided for service client.")


@dataclass
class ServiceInfo:
    name: str = 'vyra_service_server'
    type: Any = None
    request: Callable = _base_request
    callback: Callable = _base_callback
    client: Union[Client, None] = None

class VyraServiceClient:
    """
    Base class for ROS2 services.
    This class will be factory created to implement specific service functionality.
    """

    def __init__(self, serviceInfo: ServiceInfo, node: VyraNode) -> None:
        self._service_info: ServiceInfo = serviceInfo
        self._node: VyraNode = node
        self.TIMEOUT_SEC = 1.0  # Timeout for service calls in seconds
    
    def create_service(self) -> None:
        """
        Create a service in the ROS2 node.
        This method should be called to register the service with the ROS2 node.
        """
        self._node.get_logger().info(f"Creating service: {self._service_info.name}")
        if not self._service_info.type:
            raise ValueError("Service type must be provided.")
        
        if not self._service_info.name:
            raise ValueError("Service name must be provided.")
        
        self._service_info.client = self._node.create_client(
            self._service_info.type, 
            self._service_info.name
        )

        while not self._service_info.client.wait_for_service(timeout_sec=self.TIMEOUT_SEC):
            self._node.get_logger().info('service not available, waiting again...')

        self._service_info.request = self._service_info.type.Request()

    def send(self, **kwargs: dict) -> Any:
        for key, value in kwargs.items():
            setattr(self._service_info.request, key, value)


    def callback(self, request, response)-> None:
        """
        Add a callback to the service.
        This method should be overridden in subclasses to provide specific functionality.
        """
        self._node.get_logger().info(f"Received request on {self._service_info.name}")

        self._service_info.callback(request, response)
    