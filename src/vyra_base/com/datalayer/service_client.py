from dataclasses import dataclass, field
from http import client
from datetime import datetime
from socket import timeout
from typing import Any, Callable, NoReturn, Union

import rclpy
from rclpy.client import Client

from vyra_base.com.datalayer.node import VyraNode
from vyra_base.helper.error_handler import ErrorTraceback
from vyra_base.helper.logger import Logger

def _base_request(*args, **kwargs) -> NoReturn:
    """
    Raises
    ------
    NotImplementedError
        If no request function is provided for the service client.
    """
    raise NotImplementedError("No request function provided for service client.")


@dataclass
class ServiceClientInfo:
    """
    Data class for storing service information.

    Attributes
    ----------
    name : str
        Name of the service.
    type : Any
        Type of the service.
    request : Callable
        Function to create a service request.
    callback : Callable
        Callback function for the service.
    client : Union[Client, None]
        ROS2 client instance or None.
    timeout : int | None
        Optional timeout for the service client.
    calls_at_timeout : int
        Number of calls to attempt at timeout before giving up.
    """
    name: str = 'vyra_service_client'
    type: Any = None
    request: Callable = _base_request
    client: Union[Client, None] = None
    timeout: float | None = None
    last_responses: list = field(default_factory=list)

class VyraServiceClient:
    """
    Base class for ROS2 services.

    This class is intended to be factory-created to implement specific service functionality.
    """

    def __init__(self, serviceInfo: ServiceClientInfo, node: VyraNode) -> None:
        """
        Initialize the VyraServiceClient.

        Parameters
        ----------
        serviceInfo : ServiceInfo
            Information about the service.
        node : VyraNode
            ROS2 node instance.
        """
        self._service_info: ServiceClientInfo = serviceInfo
        self._node: VyraNode = node
    
    def create_service_caller(self) -> None:
        """
        Create a service caller in the ROS2 node.

        This method should be called to register the service caller with the ROS2 node.

        Raises
        ------
        ValueError
            If the service type or name is not provided.
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

        while not self._service_info.client.wait_for_service(
            timeout_sec=self._service_info.timeout):
    
            Logger.error(
                f'service <{self._service_info.name}> not available, timeout...')
            
            raise TimeoutError(
                f'Service <{self._service_info.name}> not available, timeout...') 

        self._service_info.request = self._service_info.type.Request()

    @ErrorTraceback.w_check_error_exist
    async def send(self, **kwargs: dict) -> Any:
        """
        Set attributes on the service request.

        Parameters
        ----------
        **kwargs : dict
            Key-value pairs to set as attributes on the service request.
        """
        for key, value in kwargs.items():
            setattr(self._service_info.request, key, value)

        if self._service_info.client is None:
            raise RuntimeError("Service client is not created. Call create_service() first.")
        
        result = await self._service_info.client.call_async(
            self._service_info.request,
        )

        self._service_info.last_responses.append((datetime.now(), result))

        return result