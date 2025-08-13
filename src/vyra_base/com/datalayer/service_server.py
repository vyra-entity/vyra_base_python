from asyncio import AbstractEventLoop
import rclpy
from dataclasses import dataclass
from inspect import iscoroutinefunction
from typing import Any, Callable, NoReturn, Union
import threading
import concurrent.futures

from rclpy.service import Service as rclService

from vyra_base.com.datalayer.node import VyraNode
from vyra_base.helper.logger import Logger
from vyra_base.helper.error_handler import ErrorTraceback
import asyncio

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
    :param callback_timeout: Optional timeout for the callback.
    :type callback_timeout: int | None
    """
    name: str = 'vyra_service_server'
    type: Any = None
    callback: Callable = _dummy_callback
    service: Union[rclService, None] = None
    callback_timeout: int | None = None

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
        self._async_loop: AbstractEventLoop | None = async_loop
        self.callback_task = None

    def create_service(
            self, 
            callback: Callable, 
            async_loop: AbstractEventLoop = None) -> None:
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
            self._service_callback
        )

        self._node.get_logger().info(
            f"Service {self.service_info.name} created with "
            f"type {self.service_info.type} and "
            f"callback {callback}.")

    def destroy_service(self) -> None:
        """
        Destroy the service in the ROS2 node.

        This method will remove the service if it exists.
        """
        if self.service_info.service:
            self._node.destroy_service(self.service_info.service)
            self.service_info.service = None
            self._node.get_logger().info(f"Service '{self.service_info.name}' destroyed.")

    @ErrorTraceback.w_check_error_exist
    def _service_callback(self, request, response) -> None:
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
        
        Logger.info(f"Service callback triggered for: {self.service_info.name}")

        # Check if it's an async function or async method
        is_async = (iscoroutinefunction(self.service_info.callback) or 
                   (hasattr(self.service_info.callback, '__func__') and 
                    iscoroutinefunction(self.service_info.callback.__func__)))

        if self._async_loop is not None and is_async:
            Logger.debug(
                f"Scheduling async callback for service {self.service_info.name}."
            )

            result = self.run_async_in_thread(
                self.service_info.callback(request, response),
                self.service_info.callback_timeout
            )
            Logger.debug(
                f"Async callback for {self.service_info.name} completed: {result}"
            )
        
        elif callable(self.service_info.callback):
            Logger.log(f"Executing sync callback for service {self.service_info.name}.")
            # Fallback: just call the callback directly (sync)
            self.service_info.callback(request=request, response=response)

        return response
    
    def run_async_in_thread(self, coro, timeout=None):
        def target():
            # Prüfe ob bereits ein Loop im aktuellen Thread existiert
            try:
                existing_loop = asyncio.get_running_loop()
                Logger.debug(
                    f"Warning: Thread already has running loop: {existing_loop}"
                )
                # Das sollte in einem neuen Thread nie passieren
            except RuntimeError:
                # Das ist normal - neuer Thread hat keinen Loop
                pass
            
            # Erstelle neuen Loop für diesen Thread
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            
            try:
                Logger.debug(
                    f"Created new event loop in thread: {threading.current_thread().name}"
                )
                if timeout is not None:
                    return loop.run_until_complete(asyncio.wait_for(coro, timeout))
                else:
                    return loop.run_until_complete(coro)
            finally:
                # WICHTIG: Loop schließen um Ressourcen freizugeben
                loop.close()
                # Optional: Loop aus Thread entfernen
                asyncio.set_event_loop(None)
        
        with concurrent.futures.ThreadPoolExecutor() as executor:
            future = executor.submit(target)
            return future.result()