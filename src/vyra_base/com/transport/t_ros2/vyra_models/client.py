"""
ROS2 Client Implementation

Async-first service client for ROS2.
"""
import asyncio
import logging
from typing import Any, Callable, Optional

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
            service_name = self.topic_builder.build(self.name)
            
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
        
        Args:
            request: Request message
            timeout: Timeout in seconds
            
        Returns:
            Response message
            
        Raises:
            InterfaceError: If not initialized or call fails
            TimeoutError: If call times out
        """
        if not self._initialized or not self._ros2_client:
            raise InterfaceError(f"Client '{self.name}' not initialized")
        
        try:
            # Wait for service to be available
            loop = asyncio.get_event_loop()
            
            # Access client via service_info
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
            
            # Call service (sync, wrap in executor)
            response = await loop.run_in_executor(
                None,
                self._ros2_client.service_info.client.call,
                request
            )
            
            # Optional: call request_callback with response
            if self.request_callback and response:
                if asyncio.iscoroutinefunction(self.request_callback):
                    await self.request_callback(response)
                else:
                    self.request_callback(response)
            
            return response
            
        except Exception as e:
            logger.error(f"❌ Failed to call service '{self.name}': {e}")
            raise InterfaceError(f"Service call failed: {e}")
