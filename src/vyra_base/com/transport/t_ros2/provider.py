"""
ROS2 Protocol Provider

Implements AbstractProtocolProvider for ROS2 transport.
Provides distributed communication via DDS middleware.
"""
from __future__ import annotations

import asyncio
from ctypes import ArgumentError
import logging
from typing import Any, Callable, Optional, Dict, TYPE_CHECKING

from vyra_base.com.core.topic_builder import TopicBuilder
from vyra_base.com.core.types import (
    ProtocolType,
    # New unified types
    VyraPublisher,
    VyraSubscriber,
    VyraServer,
    VyraClient,
    VyraActionServer,
    VyraActionClient
)
from vyra_base.com.core.exceptions import (
    ProtocolUnavailableError,
    ProviderError,
)
from vyra_base.com.transport.t_ros2.node import NodeSettings, VyraNode
from vyra_base.com.transport.t_ros2.vyra_models import (
    # New unified
    VyraPublisherImpl,
    VyraSubscriberImpl,
    VyraServerImpl,
    VyraClientImpl,
    VyraActionServerImpl,
    VyraActionClientImpl
)
from vyra_base.com.providers.protocol_provider import AbstractProtocolProvider
from vyra_base.com.core.callback_registry import CallbackRegistry

logger = logging.getLogger(__name__)

# Check if rclpy is available
try:
    import rclpy
    from rclpy.node import Node
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    logger.warning(
        "⚠️ rclpy not available. ROS2 transport disabled. "
        "Install ROS2 and source setup.bash"
    )


class ROS2Provider(AbstractProtocolProvider):
    """
    Protocol provider for ROS2 transport.
    
    Features:
    - Distributed communication via DDS
    - Quality of Service (QoS) policies
    - Discovery and introspection
    - SROS2 security support
    
    Requirements:
    - ROS2 installation
    - rclpy package
    - Source /opt/ros/<distro>/setup.bash
    
    Example:
        >>> # Initialize provider
        >>> provider = ROS2Provider(ProtocolType.ROS2)
        >>> 
        >>> if await provider.check_availability():
        ...     await provider.initialize(config={
        ...         "node_name": "my_module",
        ...         "namespace": "/vyra"
        ...     })
        ...     
        ...     # Create server (service server)
        ...     async def handle_request(req):
        ...         return {"result": req.value * 2}
        ...     
        ...     server = await provider.create_server(
        ...         "calculate",
        ...         handle_request,
        ...         service_type="example_interfaces/srv/AddTwoInts"
        ...     )
        ...     
        ...     # Create publisher (topic publisher)
        ...     publisher = await provider.create_publisher(
        ...         "sensor_data",
        ...         message_type="std_msgs/msg/String"
        ...     )
    """
    
    def __init__(
        self,
        module_name: str,
        module_id: str,
        protocol: ProtocolType = ProtocolType.ROS2,
    ):
        """
        Initialize ROS2 provider.
        
        Args:
            protocol: Protocol type (must be ROS2)
            node_name: Default node name
        """
        super().__init__(protocol)
        self.node_name = f"{module_name}_{module_id}"
        self._node: Optional[VyraNode] = None
        self._available = ROS2_AVAILABLE  # Set availability based on rclpy presence
        
        # Default configuration
        self._config = {
            "node_name": self.node_name,
            "namespace": "",
            "use_sim_time": False,
            "enable_rosout": True,
            "parameter_overrides": None,
        }
        self._topic_builder = TopicBuilder(module_name, module_id)
    
    async def check_availability(self) -> bool:
        """
        Check if ROS2 is available.
        
        Returns:
            bool: True if rclpy is installed and importable
        """
        self._available = ROS2_AVAILABLE
        
        if not self._available:
            logger.warning(
                "⚠️ ROS2 transport not available. "
                "Install ROS2 and source setup.bash"
            )
        else:
            logger.info("✅ ROS2 transport available")
        
        return self._available
    
    async def initialize(self, config: Optional[Dict[str, Any]] = None) -> bool:
        """
        Initialize ROS2 provider.
        
        Args:
            config: Optional configuration
                - node_name: ROS2 node name
                - namespace: Node namespace
                - use_sim_time: Use simulation time
                - enable_rosout: Enable rosout logging
                - parameter_overrides: Node parameter overrides
                
        Returns:
            bool: True if initialization successful
        """
        available = await self.check_availability()
        if not available:
            raise ProtocolUnavailableError(
                "ROS2 transport not available. "
                "Install ROS2 and source setup.bash"
            )
        
        if self._initialized:
            logger.warning("⚠️ Provider already initialized")
            return True
        
        # Update configuration
        if config:
            self._config.update(config)
        
        logger.info(
            f"🚀 Initializing ROS2 provider: {self._config['node_name']}"
        )
        
        try:
            # Initialize rclpy if not already done
            if not rclpy.ok():
                rclpy.init()
                logger.debug("✅ rclpy initialized")
            
            # Create node (we'll use the existing VyraNode or create a basic one)
            # Note: In actual usage, this will be integrated with existing VyraNode
            
            
            node_settings = NodeSettings(
                name=self._config["node_name"],
                parameters=self._config.get("parameter_overrides", {})
            )
            
            self._node = VyraNode(node_settings)
            
            self._initialized = True
            logger.info(
                f"✅ ROS2 provider initialized: {self._config['node_name']}"
            )
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to initialize ROS2 provider: {e}")
            return False
    
    async def shutdown(self) -> None:
        """Shutdown the provider and cleanup resources."""
        if not self._initialized:
            return
        
        logger.info(f"🛑 Shutting down ROS2 provider: {self.node_name}")
        
        # Destroy node
        if self._node:
            self._node.destroy_node()
            self._node = None
        
        # Shutdown rclpy
        if rclpy.ok():
            rclpy.shutdown()
            logger.debug("✅ rclpy shutdown")
        
        self._initialized = False
        logger.info("✅ ROS2 provider shutdown complete")
    

    
    # ========================================================================
    # UNIFIED INTERFACE CREATION METHODS
    # ========================================================================
    
    async def create_publisher(
        self,
        name: str,
        **kwargs
    ) -> VyraPublisher:
        """Create ROS2 publisher."""
        self.require_initialization()
        
        message_type = kwargs.pop("message_type", None)
        if not message_type:
            message_type = self._topic_builder.load_interface_type(name, self.protocol)
        if not message_type:
            raise ArgumentError("message_type is required for ROS2 publisher")
        
        node = kwargs.pop("node", None)
        qos_profile = kwargs.pop("qos_profile", None)
        
        logger.info(f"🔧 Creating ROS2 publisher: {name} (type: {message_type})")
        
        publisher = VyraPublisherImpl(
            name=name,
            topic_builder=self._topic_builder,
            node=node or self._node,
            message_type=message_type,
            qos_profile=qos_profile,
            **kwargs
        )
        
        await publisher.initialize()
        logger.info(f"✅ ROS2 publisher created: {name}")
        return publisher
    
    async def create_subscriber(
        self,
        name: str,
        subscriber_callback: Optional[Callable],
        **kwargs
    ) -> VyraSubscriber:
        """Create ROS2 subscriber."""
        self.require_initialization()
        
        message_type = kwargs.pop("message_type", None)
        if not message_type:
            message_type = self._topic_builder.load_interface_type(name, self.protocol)
        if not message_type:
            raise ArgumentError("message_type is required for ROS2 subscriber")
        
        if subscriber_callback is None:
            _bp = CallbackRegistry.get_blueprint(name)
            if _bp and _bp.is_bound():
                subscriber_callback = _bp.callback
            else:
                raise ProviderError(
                    f"No subscriber_callback provided for '{name}' and no bound blueprint found in CallbackRegistry"
                )
        
        node = kwargs.pop("node", None)
        qos_profile = kwargs.pop("qos_profile", None)
        
        logger.info(f"🔧 Creating ROS2 subscriber: {name} (type: {message_type})")
        
        subscriber = VyraSubscriberImpl(
            name=name,
            topic_builder=self._topic_builder,
            node=node or self._node,
            message_type=message_type,
            subscriber_callback=subscriber_callback,
            qos_profile=qos_profile,
            **kwargs
        )
        
        await subscriber.initialize()
        logger.info(f"✅ ROS2 subscriber created: {name}")
        return subscriber
    
    async def create_server(
        self,
        name: str,
        response_callback: Optional[Callable],
        **kwargs
    ) -> VyraServer:
        """Create ROS2 service server."""
        self.require_initialization()
        
        service_type = kwargs.pop("service_type", None)
        if not service_type:
            service_type = self._topic_builder.load_interface_type(name, self.protocol)
        if not service_type:
            raise ArgumentError("service_type is required for ROS2 server")
        
        if response_callback is None:
            _bp = CallbackRegistry.get_blueprint(name)
            if _bp and _bp.is_bound():
                response_callback = _bp.callback
            else:
                raise ProviderError(
                    f"No response_callback provided for server '{name}' and no bound blueprint found in CallbackRegistry"
                )
        
        node = kwargs.pop("node", None)
        qos_profile = kwargs.pop("qos_profile", None)
        
        logger.info(f"🔧 Creating ROS2 server: {name} (type: {service_type})")
        
        server = VyraServerImpl(
            name=name,
            topic_builder=self._topic_builder,
            node=node or self._node,
            service_type=service_type,
            response_callback=response_callback,
            qos_profile=qos_profile,
            **kwargs
        )
        
        await server.initialize()
        logger.info(f"✅ ROS2 server created: {name}")
        return server
    
    async def create_client(
        self,
        name: str,
        **kwargs
    ) -> VyraClient:
        """Create ROS2 service client."""
        self.require_initialization()
        
        service_type = kwargs.pop("service_type", None)
        if not service_type:
            service_type = self._topic_builder.load_interface_type(name, self.protocol)
        if not service_type:
            raise ArgumentError("service_type is required for ROS2 client")
        
        node = kwargs.pop("node", None)
        qos_profile = kwargs.pop("qos_profile", None)
        request_callback = kwargs.pop("request_callback", None)
        
        logger.info(f"🔧 Creating ROS2 client: {name} (type: {service_type})")
        
        client = VyraClientImpl(
            name=name,
            topic_builder=self._topic_builder,
            node=node or self._node,
            service_type=service_type,
            request_callback=request_callback,
            qos_profile=qos_profile,
            **kwargs
        )
        
        await client.initialize()
        logger.info(f"✅ ROS2 client created: {name}")
        return client
    
    async def create_action_server(
        self,
        name: str,
        handle_goal_request: Optional[Callable] = None,
        handle_cancel_request: Optional[Callable] = None,
        execution_callback: Optional[Callable] = None,
        **kwargs
    ) -> VyraActionServer:
        """Create ROS2 action server."""
        self.require_initialization()
        
        action_type = kwargs.pop("action_type", None)
        if not action_type:
            action_type = self._topic_builder.load_interface_type(name, self.protocol)
        if not action_type:
            raise ArgumentError("action_type is required for ROS2 action server")
        
        if execution_callback is None:
            _bp = CallbackRegistry.get_blueprint(name)
            if _bp and _bp.is_bound():
                execution_callback = _bp.callback
            else:
                raise ProviderError(
                    f"No execution_callback provided for action server '{name}' and no bound blueprint found in CallbackRegistry"
                )
        if handle_goal_request is None:
            _bp = CallbackRegistry.get_blueprint(name)
            if _bp is not None:
                handle_goal_request = getattr(_bp, 'get_callback', lambda x: None)('on_goal')
        if handle_cancel_request is None:
            _bp = CallbackRegistry.get_blueprint(name)
            if _bp is not None:
                handle_cancel_request = getattr(_bp, 'get_callback', lambda x: None)('on_cancel')
        
        node = kwargs.pop("node", None)
        
        logger.info(f"🔧 Creating ROS2 action server: {name} (type: {action_type})")
        
        action_server = VyraActionServerImpl(
            name=name,
            topic_builder=self._topic_builder,
            node=node or self._node,
            action_type=action_type,
            handle_goal_request=handle_goal_request,
            handle_cancel_request=handle_cancel_request,
            execution_callback=execution_callback,
            **kwargs
        )
        
        await action_server.initialize()
        logger.info(f"✅ ROS2 action server created: {name}")
        return action_server
    
    async def create_action_client(
        self,
        name: str,
        direct_response_callback: Optional[Callable] = None,
        feedback_callback: Optional[Callable] = None,
        goal_callback: Optional[Callable] = None,
        **kwargs
    ) -> VyraActionClient:
        """Create ROS2 action client."""
        self.require_initialization()
        
        action_type = kwargs.pop("action_type", None)
        if not action_type:
            action_type = self._topic_builder.load_interface_type(name, self.protocol)
        if not action_type:
            raise ArgumentError("action_type is required for ROS2 action client")
        
        node = kwargs.pop("node", None)
        
        logger.info(f"🔧 Creating ROS2 action client: {name} (type: {action_type})")
        
        action_client = VyraActionClientImpl(
            name=name,
            topic_builder=self._topic_builder,
            node=node or self._node,
            action_type=action_type,
            direct_response_callback=direct_response_callback,
            feedback_callback=feedback_callback,
            goal_callback=goal_callback,
            **kwargs
        )
        
        await action_client.initialize()
        logger.info(f"✅ ROS2 action client created: {name}")
        return action_client
    
    def get_node(self) -> Node:
        """
        Get the ROS2 node.
        
        Returns:
            rclpy Node instance
        """
        if not self._initialized or not self._node:
            raise ProviderError("Provider not initialized or node not available")
        
        return self._node
