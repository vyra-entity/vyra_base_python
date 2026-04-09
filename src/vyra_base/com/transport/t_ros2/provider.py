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
            
            # Use the node passed from VyraEntity if available; otherwise create a new one.
            # Sharing the entity's node ensures the background spinner (which spins entity.node)
            # also processes ActionServer/ActionClient callbacks.
            if self._config.get("node") is not None:
                self._node = self._config["node"]
                logger.debug(f"✅ ROS2 provider reusing entity node: {self._node.get_name()}")
            else:
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
        
        # Destroy node only if we own it (not shared with entity)
        if self._node and self._config.get("node") is None:
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
        
        topic_builder = kwargs.pop("topic_builder", None) or self._topic_builder
        
        message_type = self._resolve_ros2_type(kwargs.pop("message_type", None))
        # Only accept message_type if it's a valid ROS2 type (has _TYPE_SUPPORT).
        # Generic Python types like dict are not valid for ROS2; fall through
        # to topic_builder resolution in that case.
        if not message_type or not hasattr(message_type, '_TYPE_SUPPORT'):
            message_type = topic_builder.load_interface_type(name, self.protocol)
        if not message_type:
            raise ArgumentError("message_type is required for ROS2 publisher")
        
        node = kwargs.pop("node", None)
        qos_profile = kwargs.pop("qos_profile", None)
        
        logger.info(f"🔧 Creating ROS2 publisher: {name} (type: {message_type})")
        
        publisher = VyraPublisherImpl(
            name=name,
            topic_builder=topic_builder,
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
        
        topic_builder = kwargs.pop("topic_builder", None) or self._topic_builder
        
        message_type = self._resolve_ros2_type(kwargs.pop("message_type", None))
        if not message_type or not hasattr(message_type, '_TYPE_SUPPORT'):
            message_type = topic_builder.load_interface_type(name, self.protocol)
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
            topic_builder=topic_builder,
            node=node or self._node,
            message_type=message_type,
            subscriber_callback=subscriber_callback,
            qos_profile=qos_profile,
            **kwargs
        )
        
        await subscriber.initialize()
        logger.info(f"✅ ROS2 subscriber created: {name}")
        return subscriber
    
    @staticmethod
    def _resolve_ros2_type(service_type: Any) -> Optional[type]:
        """Extract the actual ROS2 class from a mixed list or return the type as-is.

        ``interfacetypes`` passed from the interface builder is a list that may
        contain both string file names (e.g. ``"ServiceTest.proto"``) and the
        resolved Python class for ROS2 (e.g.
        ``<class 'v2_modulemanager_interfaces.srv._service_test.ServiceTest'>``).
        ROS2 needs the bare class; all non-class entries are ignored.
        """
        if service_type is None:
            return None
        if isinstance(service_type, list):
            for item in service_type:
                if not isinstance(item, str):
                    return item
            return None  # list contained only strings (proto names) → no ROS2 type
        return service_type  # already a class or module

    async def create_server(
        self,
        name: str,
        response_callback: Optional[Callable],
        **kwargs
    ) -> VyraServer:
        """Create ROS2 service server."""
        self.require_initialization()
        
        topic_builder = kwargs.pop("topic_builder", None) or self._topic_builder
        
        service_type = self._resolve_ros2_type(kwargs.pop("service_type", None))
        if not service_type:
            service_type = topic_builder.load_interface_type(name, self.protocol)
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
            topic_builder=topic_builder,
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
        topic_builder: Optional[TopicBuilder] = None,
        service_type: Optional[type] = None,
        request_callback: Optional[Callable] = None,
        **kwargs
    ) -> VyraClient:
        """
        Create ROS2 service client.

        Args:
            name: Service name
            topic_builder: TopicBuilder instance. When provided (e.g. injected by
                the factory for cross-module proxy calls), it is used for both
                interface-type discovery and topic-name building, so that the
                correct module-specific interface paths are consulted. Falls back
                to the provider's own ``_topic_builder`` when omitted.
            service_type: Explicit ROS2 service type class.  Resolved via
                ``topic_builder.load_interface_type`` when not supplied.
            request_callback: Optional async callback for responses.
            **kwargs: Additional parameters forwarded to ``VyraClientImpl``
                (node, qos_profile, …).
        """
        self.require_initialization()

        # Use injected topic_builder first so cross-module proxy calls resolve
        # interface types from the target module's interface package rather than
        # the provider's default (which only covers the local vyra_base interfaces).
        effective_topic_builder = topic_builder or self._topic_builder

        # service_type may be a mixed list from interface_builder; extract the class
        service_type = self._resolve_ros2_type(service_type)

        if not service_type:
            service_type = effective_topic_builder.load_interface_type(name, self.protocol)
        if not service_type:
            raise ArgumentError("service_type is required for ROS2 client")

        node = kwargs.pop("node", None)
        qos_profile = kwargs.pop("qos_profile", None)

        logger.info(f"🔧 Creating ROS2 client: {name} (type: {service_type})")

        client = VyraClientImpl(
            name=name,
            topic_builder=effective_topic_builder,
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
        
        topic_builder = kwargs.pop("topic_builder", None) or self._topic_builder
        
        action_type = self._resolve_ros2_type(kwargs.pop("action_type", None))
        if not action_type:
            action_type = topic_builder.load_interface_type(name, self.protocol)
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
            topic_builder=topic_builder,
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
        
        topic_builder = kwargs.pop("topic_builder", None) or self._topic_builder
        action_type = self._resolve_ros2_type(kwargs.pop("action_type", None))
        if not action_type:
            action_type = topic_builder.load_interface_type(name, self.protocol)
        if not action_type:
            raise ArgumentError("action_type is required for ROS2 action client")
        
        node = kwargs.pop("node", None)
        
        logger.info(f"🔧 Creating ROS2 action client: {name} (type: {action_type})")
        
        action_client = VyraActionClientImpl(
            name=name,
            topic_builder=topic_builder,
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
