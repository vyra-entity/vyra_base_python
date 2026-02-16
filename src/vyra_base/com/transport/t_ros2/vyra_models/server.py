"""
ROS2 Server Implementation

Async-first service server for ROS2.
"""
import asyncio
import logging
from typing import Any, Callable, Optional

from vyra_base.com.core.types import VyraServer, ProtocolType
from vyra_base.com.core.topic_builder import TopicBuilder
from vyra_base.com.core.exceptions import InterfaceError
from vyra_base.com.transport.t_ros2.communication.service_server import (
    ROS2ServiceServer,
    ServiceServerInfo
)

logger = logging.getLogger(__name__)


class VyraServerImpl(VyraServer):
    """
    Vyra Server implementation.
    
    Wraps Vyra service server with async callback support.
    """
    
    def __init__(
        self,
        name: str,
        topic_builder: TopicBuilder,
        node: Any,
        service_type: Any,
        response_callback: Optional[Callable] = None,
        qos_profile: Optional[Any] = None,
        **kwargs
    ):
        super().__init__(name, topic_builder, response_callback, ProtocolType.ROS2, **kwargs)
        self.node = node
        self.service_type = service_type
        self.qos_profile = qos_profile
        self._ros2_server: Optional[ROS2ServiceServer] = None
        
    async def initialize(self) -> bool:
        """Initialize ROS2 service server."""
        try:
            service_name = self.topic_builder.build(self.name)
            
            # Wrap async callback for ROS2 sync context
            def sync_callback(request, response):
                try:
                    loop = asyncio.get_event_loop()
                    if loop.is_closed():
                        loop = asyncio.new_event_loop()
                        asyncio.set_event_loop(loop)
                    
                    if self.response_callback:
                        if asyncio.iscoroutinefunction(self.response_callback):
                            result = loop.run_until_complete(self.response_callback(request))
                        else:
                            result = self.response_callback(request)
                        
                        # Copy result fields to response
                        if result:
                            for field in result.__slots__:
                                setattr(response, field, getattr(result, field))
                    
                    return response
                except Exception as e:
                    logger.error(f"❌ Server callback error: {e}")
                    return response
            
            # Create service server via VyraServiceServer
            service_info = ServiceServerInfo(
                name=service_name,
                type=self.service_type,
                callback=sync_callback if self.response_callback is not None else lambda req, resp: resp
            )
            
            self._ros2_server = ROS2ServiceServer(
                serviceInfo=service_info,
                node=self.node
            )
            self._ros2_server.create_service(
                callback=sync_callback if self.response_callback is not None else lambda req, resp: resp
            )
            
            self._transport_handle = self._ros2_server
            self._initialized = True
            
            logger.info(f"✅ ROS2 Server '{self.name}' initialized on service '{service_name}'")
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to initialize ROS2 Server '{self.name}': {e}")
            raise InterfaceError(f"Server initialization failed: {e}")
    
    async def shutdown(self) -> None:
        """Shutdown ROS2 server."""
        if self._ros2_server:
            self._ros2_server.destroy_service()
            self._ros2_server = None
        self._initialized = False
        self._transport_handle = None
        
    async def serve(self) -> None:
        """
        Start serving (already active after initialize for ROS2).
        
        Note: ROS2 servers are automatically active after creation.
        """
        if not self._initialized:
            raise InterfaceError(f"Server '{self.name}' not initialized")
        
        logger.debug(f"Server '{self.name}' is serving")
