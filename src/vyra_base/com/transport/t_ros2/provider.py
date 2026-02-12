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
    VyraCallable,
    VyraSpeaker,
    VyraJob,
)
from vyra_base.com.core.exceptions import (
    ProtocolUnavailableError,
    ProviderError,
)
from vyra_base.com.transport.t_ros2.node import NodeSettings, VyraNode
from vyra_base.com.transport.t_ros2.vyra_models import ROS2Speaker, ROS2Callable, ROS2Job
from vyra_base.com.providers.protocol_provider import AbstractProtocolProvider

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
        ...     # Create callable (service server)
        ...     async def handle_request(req):
        ...         return {"result": req.value * 2}
        ...     
        ...     callable = await provider.create_callable(
        ...         "calculate",
        ...         handle_request,
        ...         service_type="example_interfaces/srv/AddTwoInts"
        ...     )
        ...     
        ...     # Create speaker (topic publisher)
        ...     speaker = await provider.create_speaker(
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
    
    async def create_callable(
        self,
        name: str,
        callback: Optional[Callable],
        **kwargs
    ) -> VyraCallable:
        """
        Create a ROS2 callable (service server or client).
        
        Args:
            name: Service name
            callback: Server-side callback function (None for client)
            **kwargs: Additional parameters
                - service_type: ROS2 service type (required)
                - qos_profile: QoS profile
                - is_callable: True for server, False for client (auto-detected from callback if not provided)
                
        Returns:
            VyraCallable instance
            
        Raises:
            ProviderError: If provider not initialized
            ValueError: If service_type not provided
        """
        self.require_initialization()
        
        service_type = kwargs.pop("service_type", None)
        if not service_type:
            raise ArgumentError("service_type is required for ROS2 callable")
        
        node = kwargs.pop("node", None)
        
        # Check is_callable flag (defaults to True if callback provided, False otherwise)
        is_callable = kwargs.pop("is_callable", callback is not None)
        
        role = "server" if is_callable else "client"

        logger.info(
            f"🔧 Creating ROS2 service {role}: {name} (type: {service_type})"
        )
        
        # Ensure callback matches is_callable flag
        if is_callable and callback is None:
            raise ArgumentError("Callback required for service server (is_callable=True)")
        if not is_callable and callback is not None:
            logger.debug("Ignoring callback for service client (is_callable=False)")
            callback = None
        
        # Create ROS2 callable
        callable_instance = ROS2Callable(
            name=name,
            topic_builder=self._topic_builder,
            callback=callback,
            node=node or self._node,
            service_type=service_type,
            **kwargs
        )
        
        await callable_instance.initialize()
        
        logger.info(f"✅ ROS2 service {role} created: {name}")
        return callable_instance
    
    async def create_speaker(
        self,
        name: str,
        **kwargs
    ) -> VyraSpeaker:
        """
        Create a ROS2 speaker (topic publisher/subscriber).
        
        Args:
            name: Topic name
            **kwargs: Additional parameters
                - message_type: ROS2 message type (required)
                - qos_profile: QoS profile
                - is_publisher: True for publisher, False for subscriber (default: True)
                
        Returns:
            VyraSpeaker instance
            
        Raises:
            ProviderError: If provider not initialized
            ArgumentError: If message_type not provided
        """
        self.require_initialization()
        
        message_type = kwargs.pop("message_type", None)
        if not message_type:
            raise ArgumentError("message_type is required for ROS2 speaker")

        node = kwargs.pop("node", None)

        is_publisher = kwargs.get("is_publisher", True)
        kwargs.pop("is_publisher", None)
        role = "publisher" if is_publisher else "subscriber"
        
        logger.info(
            f"🔧 Creating ROS2 {role}: {name} (type: {message_type})"
        )
                
        # Create ROS2 speaker
        speaker_instance = ROS2Speaker(
            name=name,
            topic_builder=self._topic_builder,
            node=node or self._node,
            message_type=message_type,
            is_publisher=is_publisher,
            **kwargs
        )
        
        await speaker_instance.initialize()
        
        logger.info(f"✅ ROS2 {role} created: {name}")
        return speaker_instance
    
    async def create_job(
        self,
        name: str,
        callback: Callable,
        **kwargs
    ) -> VyraJob:
        """
        Create a ROS2 job (action server or client).
        
        Args:
            name: Action name
            callback: Action execution callback (for server) or None (for client)
            **kwargs: Additional parameters
                - action_type: ROS2 action type (required)
                - is_job: True for server, False for client (auto-detected from callback if not provided)
                - feedback_callback: Optional feedback callback (client-side)
                
        Returns:
            VyraJob instance
            
        Raises:
            ProviderError: If provider not initialized
            ArgumentError: If action_type not provided
        """
        self.require_initialization()
        
        action_type = kwargs.pop("action_type", None)
        if not action_type:
            raise ArgumentError("action_type is required for ROS2 job")
        
        node = kwargs.pop("node", None)
        
        # Check is_job flag (defaults to True if callback provided, False otherwise)
        is_job = kwargs.pop("is_job", callback is not None)
        
        role = "server" if is_job else "client"

        logger.info(
            f"🔧 Creating ROS2 action {role}: {name} (type: {action_type})"
        )
        
        # Ensure callback matches is_job flag
        if is_job and callback is None:
            raise ArgumentError("Callback required for action server (is_job=True)")
        if not is_job and callback is not None:
            logger.debug("Ignoring callback for action client (is_job=False)")
            callback = None
        
        # Extract feedback and result callbacks for compatibility
        feedback_callback = kwargs.pop("feedback_callback", None)
        result_callback = callback if is_job else None
        
        # Create ROS2 job
        job_instance = ROS2Job(
            name=name,
            topic_builder=self._topic_builder,
            result_callback=result_callback,
            feedback_callback=feedback_callback,
            node=node or self._node,
            action_type=action_type,
            **kwargs
        )
        
        await job_instance.initialize()
        
        logger.info(f"✅ ROS2 action {role} created: {name}")
        return job_instance
    
    def get_node(self) -> Node:
        """
        Get the ROS2 node.
        
        Returns:
            rclpy Node instance
        """
        if not self._initialized or not self._node:
            raise ProviderError("Provider not initialized or node not available")
        
        return self._node
