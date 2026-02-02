from dataclasses import dataclass, field
from http import client
from datetime import datetime
from socket import timeout
from typing import Any, Callable, NoReturn, Union
import asyncio

import rclpy
from rclpy.client import Client

from vyra_base.com.transport.ros2.node import VyraNode
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
    
    async def create_service_caller(self) -> None:
        """
        Create a service caller in the ROS2 node.

        This method should be called to register the service caller with the ROS2 node.

        Raises
        ------
        ValueError
            If the service type or name is not provided.
        TimeoutError
            If the service is not available within the timeout period.
        """
        self._node.get_logger().info(f"Creating service client: {self._service_info.name}")
        if not self._service_info.type:
            raise ValueError("Service type must be provided.")
        
        if not self._service_info.name:
            raise ValueError("Service name must be provided.")
        
        self._service_info.client = self._node.create_client(
            self._service_info.type, 
            self._service_info.name
        )

        Logger.debug(f"🔍 Waiting for service <{self._service_info.name}> to become available (timeout: {self._service_info.timeout}s)...")
        
        # Use async loop to avoid blocking the event loop
        start_time = asyncio.get_event_loop().time()
        check_interval = 0.1  # Check every 100ms
        
        while True:
            # Non-blocking check with short timeout
            if self._service_info.client.wait_for_service(timeout_sec=0):
                Logger.info(f'✅ Service <{self._service_info.name}> is now available!')
                break
            
            # Check if total timeout exceeded
            elapsed = asyncio.get_event_loop().time() - start_time
            if self._service_info.timeout and elapsed >= self._service_info.timeout:
                Logger.error(
                    f'❌ Service <{self._service_info.name}> not available after {elapsed:.1f}s timeout')
                raise TimeoutError(
                    f'Service <{self._service_info.name}> not available, timeout after {elapsed:.1f}s')
            
            # Yield control to event loop to allow spinner to run
            Logger.debug(f"⏳ Service <{self._service_info.name}> not ready yet (elapsed: {elapsed:.1f}s), yielding to event loop...")
            await asyncio.sleep(check_interval)

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