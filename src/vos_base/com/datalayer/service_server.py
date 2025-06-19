
import asyncio
import rclpy
from dataclasses import dataclass

from rclpy.service import Service as rclService
from typing import Any
from typing import Callable
from typing import NoReturn
from typing import Union

from inspect import iscoroutine

from vos_base.com.datalayer.node import VOSNode
from vos_base.helper.logger import Logger

from vos_base.helper.error_handler import ErrorTraceback

def _dummy_callback(*args, **kwargs) -> NoReturn:
    raise NotImplementedError("_dummy_callback >> No execute callback provided for service server.")


@dataclass
class ServiceInfo:
    name: str = 'vos_service_server'
    type: Any = None
    callback: Callable = _dummy_callback
    service: Union[rclService, None] = None

class VOSServiceServer:
    """
    Base class for ROS2 services.
    This class will be factory created to implement specific service functionality.
    """

    def __init__(self, serviceInfo: ServiceInfo, node: VOSNode, async_loop = None) -> None:
        self.service_info: ServiceInfo = serviceInfo
        self._node: VOSNode = node
        self._async_loop = async_loop
    
    def create_service(self, callback: Callable, async_loop = None) -> None:
        """
        Create a service in the ROS2 node.
        This method should be called to register the service with the ROS2 node.
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

    async def callback(self, request, response)-> None:
        """
        Add a callback to the service.
        This method should be overridden in subclasses to provide specific functionality.
        """
        self._node.get_logger().info(f"Received request on {self.service_info.name}")

        try:
            await self.service_info.callback(request=request, response=response)
        
            print(f">>>>>>{response}")
           
        finally:
            Logger.log("TEST")
            if ErrorTraceback.check_error_exist():
                self._node.get_logger().error(
                    f"Error in service callback for {self.service_info.name}")
                print("Error in service callback:")

            print(self.service_info.callback)
        
        return response