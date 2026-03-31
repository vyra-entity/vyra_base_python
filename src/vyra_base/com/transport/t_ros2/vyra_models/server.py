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
            service_name = self.topic_builder.build(self.name, namespace=self.namespace, subsection=self.subsection)
            
            # Capture the running event loop now (initialize() is called from the
            # main asyncio context).  The sync_callback runs in a ROS2 executor
            # thread, so we must use run_coroutine_threadsafe instead of
            # run_until_complete (which would deadlock on an already-running loop).
            main_loop = asyncio.get_event_loop()

            def sync_callback(request, response):
                """Bridge between ROS2 sync service handler and Vyra's async callbacks."""
                try:
                    if self.response_callback:
                        if asyncio.iscoroutinefunction(self.response_callback):
                            future = asyncio.run_coroutine_threadsafe(
                                self.response_callback(request), main_loop
                            )
                            result = future.result(timeout=30.0)
                        else:
                            result = self.response_callback(request)

                        # Copy result fields to response.
                        # Callbacks may return a dict (most common in Vyra applications)
                        # or a ROS2 response object with __slots__.
                        if result:
                            if isinstance(result, dict):
                                for key, value in result.items():
                                    if hasattr(response, key):
                                        setattr(response, key, value)
                                    elif hasattr(response, key.lower()):
                                        setattr(response, key.lower(), value)
                                    else:
                                        logger.debug(
                                            f"⚠️ Response field '{key}' not found on "
                                            f"{type(response).__name__} — skipping"
                                        )
                            elif hasattr(result, '__slots__'):
                                for field in result.__slots__:
                                    setattr(response, field, getattr(result, field))
                            else:
                                for field, value in vars(result).items():
                                    if not field.startswith('_') and hasattr(response, field):
                                        setattr(response, field, value)
                    
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
