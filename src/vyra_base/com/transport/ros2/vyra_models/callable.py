"""
ROS2 Callable Implementation

Concrete implementation of VyraCallable for ROS2 Service communication.
"""
import asyncio
import logging
from typing import Any, Callable, Optional

from vyra_base.com.core.types import VyraCallable, ProtocolType
from vyra_base.com.core.exceptions import InterfaceError
from vyra_base.com.transport.ros2.node import VyraNode
from vyra_base.com.transport.ros2.communication.service_client import ServiceClientInfo
from vyra_base.com.transport.ros2.communication.service_server import ServiceServerInfo
from vyra_base.com.transport.ros2.communication import VyraServiceServer, VyraServiceClient

logger = logging.getLogger(__name__)


class ROS2Callable(VyraCallable):
    """
    ROS2-specific implementation of VyraCallable using ROS2 Services.
    
    Wraps VyraServiceServer for server-side and VyraServiceClient for client-side.
    """
    
    def __init__(
        self,
        name: str,
        callback: Optional[Callable] = None,
        node: Optional[VyraNode] = None,
        service_type: Optional[Any] = None,
        **kwargs
    ):
        super().__init__(name, callback, ProtocolType.ROS2, **kwargs)
        self.node = node
        self.service_type = service_type
        self._service_server: Optional[VyraServiceServer] = None
        self._service_client: Optional[VyraServiceClient] = None
        self._last_response: Any = None
    
    async def initialize(self) -> bool:
        """
        Initialize ROS2 service.
        
        Creates either service_server (if callback provided) or service_client.
        """
        if self._initialized:
            logger.warning(f"ROS2Callable '{self.name}' already initialized")
            return True
        
        if not self.node:
            raise InterfaceError("Node is required for ROS2Callable")
        
        if not self.service_type:
            raise InterfaceError("service_type is required for ROS2Callable")
        
        try:
            if self.callback:
                # Server-side: create service server
                logger.info(f"🔧 Creating ROS2 service server: {self.name}")
                
                service_info = ServiceServerInfo(
                    name=self.name,
                    type=self.service_type,
                    callback=self.callback
                )

                self._service_server = VyraServiceServer(
                    serviceInfo=service_info,
                    node=self.node
                )
                logger.info(f"✅ ROS2 service server created: {self.name}")
            else:
                # Client-side: create service client
                logger.info(f"🔧 Creating ROS2 service client: {self.name}")
                
                service_info = ServiceClientInfo(
                    name=self.name,
                    type=self.service_type
                )
                
                self._service_client = VyraServiceClient(
                    serviceInfo=service_info,
                    node=self.node
                )
                logger.info(f"✅ ROS2 service client created: {self.name}")
            
            self._initialized = True
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to initialize ROS2Callable '{self.name}': {e}")
            raise InterfaceError(f"Failed to initialize ROS2Callable: {e}")
    
    async def shutdown(self) -> None:
        """Shutdown and cleanup ROS2 service resources."""
        if not self._initialized:
            return
        
        if not self.node:
            raise InterfaceError("Node is required for ROS2Callable shutdown")
        
        logger.info(f"🛑 Shutting down ROS2Callable: {self.name}")
        
        # Cleanup service server/client
        if self._service_server:
            self.node.destroy_service(self._service_server._service_info.service)
            self._service_server = None
        
        if self._service_client:
            self.node.destroy_client(self._service_client.service_info.client)
            self._service_client = None
        
        self._initialized = False
        logger.info(f"✅ ROS2Callable '{self.name}' shutdown complete")
    
    async def call(self, request: Any, timeout: float = 5.0) -> Any:
        """
        Call the ROS2 service.
        
        Args:
            request: Service request data
            timeout: Timeout in seconds
            
        Returns:
            Service response
            
        Raises:
            InterfaceError: If not initialized or client-side call fails
        """
        if not self._initialized:
            raise InterfaceError(f"ROS2Callable '{self.name}' not initialized")
        
        if not self._service_client or not self._service_client.service_info.client:
            raise InterfaceError(
                f"Cannot call ROS2Callable '{self.name}': no service client. "
                "This is likely a server-side callable."
            )
        
        try:
            logger.debug(f"📞 Calling ROS2 service: {self.name}")
            
            # Call service via VyraServiceClient
            response = await asyncio.wait_for(
                self._service_client.service_info.client.call_async(
                    request=request,
                ),
                timeout=timeout
            )
            
            self._last_response = response
            logger.debug(f"✅ ROS2 service call succeeded: {self.name}")
            return response
        
        except asyncio.TimeoutError:
            logger.error(f"❌ ROS2 service call timed out '{self.name}' after {timeout}s")
            raise InterfaceError(f"Service call timed out after {timeout} seconds")
        except Exception as e:
            logger.error(f"❌ ROS2 service call failed '{self.name}': {e}")
            raise InterfaceError(f"Service call failed: {e}")
    
    def get_service_server(self) -> Optional[VyraServiceServer]:
        """Get the underlying ROS2 service server (server-side only)."""
        return self._service_server
    
    def get_service_client(self) -> Optional[VyraServiceClient]:
        """Get the underlying ROS2 service client (client-side only)."""
        return self._service_client
