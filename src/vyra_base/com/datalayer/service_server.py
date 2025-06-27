import rclpy
from dataclasses import dataclass
from inspect import iscoroutine
from typing import Any, Callable, NoReturn, Union

from rclpy.service import Service as rclService

from vyra_base.com.datalayer.node import VyraNode
from vyra_base.helper.logger import Logger
from vyra_base.helper.error_handler import ErrorTraceback

def _dummy_callback(*args, **kwargs) -> NoReturn:
    """
    Dummy callback that raises a NotImplementedError.

    :raises NotImplementedError: Always raised to indicate no callback is provided.
    """
    raise NotImplementedError("_dummy_callback >> No execute callback provided for service server.")


@dataclass
class ServiceInfo:
    """
    Data class for storing service information.

    :param name: Name of the service.
    :type name: str
    :param type: Type of the service.
    :type type: Any
    :param callback: Callback function for the service.
    :type callback: Callable
    :param service: The ROS2 service instance.
    :type service: Union[rclService, None]
    """
    name: str = 'vyra_service_server'
    type: Any = None
    callback: Callable = _dummy_callback
    service: Union[rclService, None] = None

class VyraServiceServer:
    """
    Base class for ROS2 services.

    This class is intended to be factory-created to implement specific service functionality.
    """

    def __init__(self, serviceInfo: ServiceInfo, node: VyraNode, async_loop = None) -> None:
        """
        Initialize the VyraServiceServer.

        :param serviceInfo: Information about the service.
        :type serviceInfo: ServiceInfo
        :param node: The ROS2 node to attach the service to.
        :type node: VyraNode
        :param async_loop: Optional asyncio event loop.
        :type async_loop: Any, Optional
        """
        self.service_info: ServiceInfo = serviceInfo
        self._node: VyraNode = node
        self._async_loop = async_loop
    
    def create_service(self, callback: Callable, async_loop = None) -> None:
        """
        Create and register a service in the ROS2 node.

        :param callback: The callback function to handle service requests.
        :type callback: Callable
        :param async_loop: Optional asyncio event loop.
        :type async_loop: Any, Optional
        :raises TypeError: If the callback is not callable.
        :raises ValueError: If the service type or name is not provided.
        """
        self.service_info.callback = callback
        
        if not callable(callback):
            raise TypeError("Callback must be a callable function.")

        if async_loop is not None:
            self._async_loop = async_loop

        self._node.get_logger().info(f"Creating service: {self.service_info.name}")
        
        if not self.service_info.type:
            raise ValueError("Service type must be provided.")
        
        if not self.service_info.name:
            raise ValueError("Service name must be provided.")
        
        self.service_info.service = self._node.create_service(
            self.service_info.type, 
            self.service_info.name, 
            self.callback
        )

        self._node.get_logger().info(
            f"Service {self.service_info.name} created with "
            f"type {self.service_info.type} and "
            f"callback {callback}.")

    async def callback(self, request, response) -> None:
        """
        Handle incoming service requests.

        This method should be overridden in subclasses to provide specific functionality.

        :param request: The service request.
        :type request: Any
        :param response: The service response.
        :type response: Any
        :return: The response object.
        :rtype: Any
        """
        self._node.get_logger().info(f"Received request on {self.service_info.name}")

        try:
            await self.service_info.callback(request=request, response=response)
        
            print(f">>>>>>{response}")
           
        finally:
            if ErrorTraceback.check_error_exist():
                self._node.get_logger().error(
                    f"Error in service callback for {self.service_info.name}")
                print("Error in service callback:")

            print(self.service_info.callback)
        
        return response