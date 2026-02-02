"""
ROS2 Protocol Provider

Implements AbstractProtocolProvider for ROS2 transport.
Provides distributed communication via DDS middleware.
"""
import logging
from typing import Any, Callable, Optional, Dict

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
        self._node: Optional[Node] = None
        
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
        if not self._available:
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
            from vyra_base.com.transport.ros2.node import NodeSettings, VyraNode
            
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
        callback: Callable,
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
        
        service_type = kwargs.get("service_type")
        if not service_type:
            raise ValueError("service_type is required for ROS2 callable")
        
        logger.info(
            f"🔧 Creating ROS2 service: {name} (type: {service_type})"
        )
        
        # Import ROS2-specific implementation
        from vyra_base.com.transport.ros2.ros2_callable import ROS2Callable
        
        # Create ROS2 callable
        callable_instance = ROS2Callable(
            name=name,
            callback=callback,
            node=self._node,
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
            ValueError: If message_type not provided
        """
        self.require_initialization()
        
        message_type = kwargs.get("message_type")
        if not message_type:
            raise ValueError("message_type is required for ROS2 speaker")
        
        is_publisher = kwargs.get("is_publisher", True)
        role = "publisher" if is_publisher else "subscriber"
        
        logger.info(
            f"🔧 Creating ROS2 {role}: {name} (type: {message_type})"
        )
        
        # Import ROS2-specific implementation
        from vyra_base.com.transport.ros2.ros2_speaker import ROS2Speaker
        
        # Create ROS2 speaker
        speaker_instance = ROS2Speaker(
            name=name,
            node=self._node,
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
            ValueError: If action_type not provided
        """
        self.require_initialization()
        
        action_type = kwargs.get("action_type")
        if not action_type:
            raise ValueError("action_type is required for ROS2 job")
        
        logger.info(
            f"🔧 Creating ROS2 action: {name} (type: {action_type})"
        )
        
        # Import ROS2-specific implementation
        from vyra_base.com.transport.ros2.ros2_job import ROS2Job
        
        # Create ROS2 job
        job_instance = ROS2Job(
            name=name,
            callback=callback,
            node=self._node,
            action_type=action_type,
            **kwargs
        )
        
        await job_instance.initialize()
        
        logger.info(f"✅ ROS2 action created: {name}")
        return job_instance
    
    def get_node(self) -> Optional[Node]:
        """
        Get the ROS2 node.
        
        Returns:
            rclpy Node instance or None if not initialized
        """
        return self._node
    
    def spin_once(self, timeout_sec: float = 0.0) -> None:
        """
        Spin the node once.
        
        Args:
            timeout_sec: Timeout in seconds
        """
        if self._node:
            rclpy.spin_once(self._node, timeout_sec=timeout_sec)
    
    async def spin_async(self) -> None:
        """Spin the node asynchronously (use in async context)."""
        if self._node:
            # Run spin in executor to avoid blocking
            loop = asyncio.get_event_loop()
            await loop.run_in_executor(None, rclpy.spin, self._node)
