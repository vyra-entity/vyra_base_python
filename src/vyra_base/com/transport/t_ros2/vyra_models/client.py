"""
ROS2 Client Implementation

Async-first service client for ROS2.
"""
import asyncio
import logging
from typing import Any, Callable, Optional

import rclpy

from vyra_base.com.core.types import VyraClient, ProtocolType
from vyra_base.com.core.topic_builder import TopicBuilder
from vyra_base.com.core.exceptions import InterfaceError, TimeoutError
from vyra_base.com.transport.t_ros2.communication.service_client import (
    ROS2ServiceClient,
    ServiceClientInfo,
)

logger = logging.getLogger(__name__)


class VyraClientImpl(VyraClient):
    """
    Vyra Client implementation.
    
    Wraps Vyra service client for async request-response.
    """
    
    def __init__(
        self,
        name: str,
        topic_builder: TopicBuilder,
        node: Any,
        service_type: Any,
        request_callback: Optional[Callable] = None,
        qos_profile: Optional[Any] = None,
        **kwargs
    ):
        super().__init__(name, topic_builder, request_callback, ProtocolType.ROS2, **kwargs)
        self.node = node
        self.service_type = service_type
        self.qos_profile = qos_profile
        self._ros2_client: Optional[ROS2ServiceClient] = None
        
    async def initialize(self) -> bool:
        """Initialize ROS2 service client."""
        try:
            service_name = self.topic_builder.build(self.name, namespace=self.namespace, subsection=self.subsection)
            
            # Create service client via VyraServiceClient
            service_info = ServiceClientInfo(
                name=service_name,
                type=self.service_type
            )
            
            self._ros2_client = ROS2ServiceClient(
                serviceInfo=service_info,
                node=self.node
            )
            await self._ros2_client.create_service_caller()
            
            self._transport_handle = self._ros2_client
            self._initialized = True
            
            logger.info(f"✅ ROS2 Client '{self.name}' initialized for service '{service_name}'")
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to initialize ROS2 Client '{self.name}': {e}")
            raise InterfaceError(f"Client initialization failed: {e}")
    
    async def shutdown(self) -> None:
        """Shutdown ROS2 client."""
        if self._ros2_client:
            self._ros2_client.destroy_service_caller()
            self._ros2_client = None
        self._initialized = False
        self._transport_handle = None
        
    async def call(self, request: Any, timeout: float = 5.0) -> Any:
        """
        Call service and await response (async).

        Accepts either a pre-built ROS2 request message **or** a plain Python
        dict.  When a dict is passed it is used to populate a freshly
        instantiated ``<ServiceType>.Request`` object field-by-field (with
        case-insensitive key matching so callers can use either ``t1`` or
        ``T1``).  The raw ROS2 response is automatically converted to an
        ordered dict before being returned so that higher layers (e.g. the
        proxy router) can treat all protocol responses uniformly.

        Args:
            request: ROS2 request message *or* dict of request fields.
            timeout: Timeout in seconds.

        Returns:
            dict: Response fields as a Python dict.

        Raises:
            InterfaceError: If not initialized or call fails.
            TimeoutError: If the service is not reachable within *timeout*.
        """
        if not self._initialized or not self._ros2_client:
            raise InterfaceError(f"Client '{self.name}' not initialized")

        try:
            loop = asyncio.get_event_loop()

            if not self._ros2_client.service_info.client:
                raise InterfaceError(f"Client '{self.name}' has no ROS2 client instance")

            service_ready = await loop.run_in_executor(
                None,
                self._ros2_client.service_info.client.wait_for_service,
                timeout
            )

            if not service_ready:
                raise TimeoutError(
                    f"Service '{self.name}' not available after {timeout}s",
                    timeout=timeout)

            # Convert dict → ROS2 request object when needed
            ros2_request = request
            if isinstance(request, dict) and self.service_type is not None:
                ros2_request = self.service_type.Request()
                for key, value in request.items():
                    if hasattr(ros2_request, key):
                        setattr(ros2_request, key, value)
                    elif hasattr(ros2_request, key.lower()):
                        setattr(ros2_request, key.lower(), value)

            # Send the request asynchronously then spin the node ourselves
            # inside the poll loop.  The main ros_spinner_runner also calls
            # spin_once(node) on the same global executor, but because each
            # spin_once temporarily adds and immediately removes the node the
            # two callers never truly conflict — they just alternate ownership.
            # Without client-side spinning the DDS wait-set built by the
            # spinner's brief 1 ms timeout often misses the incoming response,
            # causing a 5 s timeout.  Spinning here with a 50 ms timeout gives
            # the executor enough time to process the client response entity.
            ros2_future = self._ros2_client.service_info.client.call_async(ros2_request)
            deadline = loop.time() + (timeout or 30.0)
            while not ros2_future.done():
                if loop.time() > deadline:
                    raise TimeoutError(
                        f"Service '{self.name}' did not respond within {timeout}s",
                        timeout=timeout)
                try:
                    await loop.run_in_executor(
                        None,
                        lambda: rclpy.spin_once(self.node, timeout_sec=0.05),
                    )
                except Exception:
                    pass
                await asyncio.sleep(0.001)
            if ros2_future.exception():
                raise ros2_future.exception()
            response = ros2_future.result()

            if self.request_callback and response:
                if asyncio.iscoroutinefunction(self.request_callback):
                    await self.request_callback(response)
                else:
                    self.request_callback(response)

            # Convert ROS2 response → dict so all protocols return the same type
            if response is not None and hasattr(response, 'get_fields_and_field_types'):
                return {
                    field: getattr(response, field)
                    for field in response.get_fields_and_field_types()
                }

            return response

        except Exception as e:
            logger.error(f"❌ Failed to call service '{self.name}': {e}", exc_info=True)
            raise InterfaceError(f"Service call failed: {e}")
