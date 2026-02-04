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
from vyra_base.com.transport.ros2.node import NodeSettings, VyraNode
from vyra_base.com.transport.ros2.vyra_models import ROS2Speaker, ROS2Callable, ROS2Job
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
        protocol: ProtocolType = ProtocolType.ROS2,
        node_name: str = "vyra_node"
    ):
        """
        Initialize ROS2 provider.
        
        Args:
            protocol: Protocol type (must be ROS2)
            node_name: Default node name
        """
        super().__init__(protocol)
        self.node_name = node_name
        self._node: Optional[VyraNode] = None
        
        # Default configuration
        self._config = {
            "node_name": node_name,
            "namespace": "",
            "use_sim_time": False,
            "enable_rosout": True,
            "parameter_overrides": None,
        }
    
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
        Create a ROS2 callable (service server).
        
        Args:
            name: Service name
            callback: Server-side callback function
            **kwargs: Additional parameters
                - service_type: ROS2 service type (required)
                - qos_profile: QoS profile
                
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

        logger.info(
            f"🔧 Creating ROS2 service: {name} (type: {service_type})"
        )
        
        # Create ROS2 callable
        callable_instance = ROS2Callable(
            name=name,
            callback=callback,
            node=node or self._node,
            service_type=service_type,
            **kwargs
        )
        
        await callable_instance.initialize()
        
        logger.info(f"✅ ROS2 service created: {name}")
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
        Create a ROS2 job (action server/client).
        
        Args:
            name: Action name
            callback: Action execution callback
            **kwargs: Additional parameters
                - action_type: ROS2 action type (required)
                
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

        logger.info(
            f"🔧 Creating ROS2 action: {name} (type: {action_type})"
        )
                
        # Create ROS2 job
        job_instance = ROS2Job(
            name=name,
            callback=callback,
            node=node or self._node,
            action_type=action_type,
            **kwargs
        )
        
        await job_instance.initialize()
        
        logger.info(f"✅ ROS2 action created: {name}")
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
