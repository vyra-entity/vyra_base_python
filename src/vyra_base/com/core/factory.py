"""
Interface Factory

Unified interface creation with automatic protocol selection and fallback.
"""
import logging

from typing import Any, Callable, Optional, List, Union
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
    InterfaceError,
)
from vyra_base.com.providers.protocol_provider import AbstractProtocolProvider
from vyra_base.com.providers.provider_registry import ProviderRegistry

logger = logging.getLogger(__name__)


class InterfaceFactory:
    """
    Factory for creating communication interfaces with protocol fallback.
    
    Features:
    - Automatic protocol selection
    - Fallback chain (ROS2 → SharedMemory → UDS → Redis → gRPC)
    - Protocol availability checking
    - Graceful degradation
    
    Example:
        >>> # Auto-select best available protocol
        >>> callable = await InterfaceFactory.create_callable(
        ...     "my_service",
        ...     callback=handle_request
        ... )
        >>> 
        >>> # Explicit protocol with fallback
        >>> callable = await InterfaceFactory.create_callable(
        ...     "my_service",
        ...     protocols=[ProtocolType.ROS2, ProtocolType.SHARED_MEMORY],
        ...     callback=handle_request
        ... )
        >>> 
        >>> # Speaker for pub/sub
        >>> speaker = await InterfaceFactory.create_speaker(
        ...     "temperature",
        ...     protocols=[ProtocolType.REDIS, ProtocolType.MQTT]
        ... )
    """
    
    # Default fallback chain for each interface type
    # Zenoh is prioritized as the default for best performance
    
    
    # New unified fallback chains
    PUBLISHER_FALLBACK = [
        ProtocolType.ZENOH,
        ProtocolType.ROS2,
        ProtocolType.REDIS,
        ProtocolType.UDS
    ]
    
    SUBSCRIBER_FALLBACK = [
        ProtocolType.ZENOH,
        ProtocolType.ROS2,
        ProtocolType.REDIS,
        ProtocolType.UDS
    ]
    
    SERVER_FALLBACK = [
        ProtocolType.ZENOH,
        ProtocolType.ROS2,
        ProtocolType.REDIS,
        ProtocolType.UDS
    ]
    
    CLIENT_FALLBACK = [
        ProtocolType.ZENOH,
        ProtocolType.ROS2,
        ProtocolType.REDIS,
        ProtocolType.UDS
    ]
    
    ACTION_SERVER_FALLBACK = [
        ProtocolType.ZENOH,
        ProtocolType.ROS2,
        ProtocolType.REDIS,
        ProtocolType.UDS
    ]
    
    ACTION_CLIENT_FALLBACK = [
        ProtocolType.ZENOH,
        ProtocolType.ROS2,
        ProtocolType.REDIS,
        ProtocolType.UDS
    ]
    
    @staticmethod
    def register_provider(provider: Union[AbstractProtocolProvider, list]) -> None:
        """
        Register one or more protocol providers.
        
        Args:
            provider: Single provider instance or list of providers
        """
        registry = ProviderRegistry()
        
        if isinstance(provider, list):
            for prov in provider:
                registry.register_provider(prov)
        else:
            registry.register_provider(provider)
    
    @staticmethod
    def unregister_provider(protocol: ProtocolType) -> None:
        """
        Unregister a protocol provider.
        
        Args:
            protocol: Protocol type to unregister
        """
        registry = ProviderRegistry()
        registry.unregister_provider(protocol)
    
    # ========================================================================
    # NEW UNIFIED INTERFACE CREATION METHODS
    # ========================================================================

    @staticmethod
    async def create_publisher(
        name: str,
        protocols: Optional[List[ProtocolType]] = None,
        **kwargs
    ) -> VyraPublisher:
        """
        Create Publisher with automatic protocol selection.
        
        Args:
            name: Topic/channel name
            protocols: Preferred protocols (fallback if not specified)
            **kwargs: Additional parameters (message_type, qos, etc.)
            
        Returns:
            VyraPublisher: Initialized publisher
            
        Raises:
            InterfaceError: If no protocol available
        """
        protocols = protocols or InterfaceFactory.PUBLISHER_FALLBACK
        registry = ProviderRegistry()
        
        for protocol in protocols:
            try:
                provider = registry.get_provider(protocol)
                
                if not provider:
                    logger.debug(f"⚠️ No provider for {protocol.value}")
                    continue
                
                if not await provider.check_availability():
                    logger.debug(f"⚠️ Protocol {protocol.value} not available")
                    continue
                
                publisher = await provider.create_publisher(
                    name=name,
                    **kwargs
                )
                
                logger.info(f"✅ Publisher '{name}' created via {protocol.value}")
                return publisher
                
            except NotImplementedError:
                logger.debug(f"⚠️ {protocol.value} doesn't support publisher")
                continue
            except Exception as e:
                logger.warning(f"⚠️ Failed to create publisher with {protocol.value}: {e}")
                continue
        
        raise InterfaceError(
            f"Cannot create publisher '{name}'. "
            f"Tried protocols: {[p.value for p in protocols]}"
        )

    @staticmethod
    async def create_subscriber(
        name: str,
        subscriber_callback: Callable,
        protocols: Optional[List[ProtocolType]] = None,
        **kwargs
    ) -> VyraSubscriber:
        """
        Create Subscriber with automatic protocol selection.
        
        Args:
            name: Topic/channel name
            subscriber_callback: Async callback for received messages
            protocols: Preferred protocols (fallback if not specified)
            **kwargs: Additional parameters (message_type, qos, etc.)
            
        Returns:
            VyraSubscriber: Initialized subscriber
            
        Raises:
            InterfaceError: If no protocol available
        """
        protocols = protocols or InterfaceFactory.SUBSCRIBER_FALLBACK
        registry = ProviderRegistry()
        
        for protocol in protocols:
            try:
                provider = registry.get_provider(protocol)
                
                if not provider:
                    logger.debug(f"⚠️ No provider for {protocol.value}")
                    continue
                
                if not await provider.check_availability():
                    logger.debug(f"⚠️ Protocol {protocol.value} not available")
                    continue
                
                subscriber = await provider.create_subscriber(
                    name=name,
                    subscriber_callback=subscriber_callback,
                    **kwargs
                )
                
                logger.info(f"✅ Subscriber '{name}' created via {protocol.value}")
                return subscriber
                
            except NotImplementedError:
                logger.debug(f"⚠️ {protocol.value} doesn't support subscriber")
                continue
            except Exception as e:
                logger.warning(f"⚠️ Failed to create subscriber with {protocol.value}: {e}")
                continue
        
        raise InterfaceError(
            f"Cannot create subscriber '{name}'. "
            f"Tried protocols: {[p.value for p in protocols]}"
        )

    @staticmethod
    async def create_server(
        name: str,
        response_callback: Optional[Callable],
        protocols: Optional[List[ProtocolType]] = None,
        **kwargs
    ) -> VyraServer:
        """
        Create Server with automatic protocol selection.
        
        Args:
            name: Service name
            response_callback: Async callback for requests
            protocols: Preferred protocols (fallback if not specified)
            **kwargs: Additional parameters (service_type, qos, etc.)
            
        Returns:
            VyraServer: Initialized server
            
        Raises:
            InterfaceError: If no protocol available
        """
        protocols = protocols or InterfaceFactory.SERVER_FALLBACK
        registry = ProviderRegistry()
        
        for protocol in protocols:
            try:
                provider = registry.get_provider(protocol)
                
                if not provider:
                    logger.debug(f"⚠️ No provider for {protocol.value}")
                    continue
                
                if not await provider.check_availability():
                    logger.debug(f"⚠️ Protocol {protocol.value} not available")
                    continue
                
                server = await provider.create_server(
                    name=name,
                    response_callback=response_callback,
                    **kwargs
                )
                
                logger.info(f"✅ Server '{name}' created via {protocol.value}")
                return server
                
            except NotImplementedError:
                logger.debug(f"⚠️ {protocol.value} doesn't support server")
                continue
            except Exception as e:
                logger.warning(f"⚠️ Failed to create server with {protocol.value}: {e}")
                continue
        
        raise InterfaceError(
            f"Cannot create server '{name}'. "
            f"Tried protocols: {[p.value for p in protocols]}"
        )

    @staticmethod
    async def create_client(
        name: str,
        protocols: Optional[List[ProtocolType]] = None,
        **kwargs
    ) -> VyraClient:
        """
        Create Client with automatic protocol selection.
        
        Args:
            name: Service name
            protocols: Preferred protocols (fallback if not specified)
            **kwargs: Additional parameters (service_type, qos, etc.)
            
        Returns:
            VyraClient: Initialized client
            
        Raises:
            InterfaceError: If no protocol available
        """
        protocols = protocols or InterfaceFactory.CLIENT_FALLBACK
        registry = ProviderRegistry()
        
        for protocol in protocols:
            try:
                provider = registry.get_provider(protocol)
                
                if not provider:
                    logger.debug(f"⚠️ No provider for {protocol.value}")
                    continue
                
                if not await provider.check_availability():
                    logger.debug(f"⚠️ Protocol {protocol.value} not available")
                    continue
                
                client = await provider.create_client(
                    name=name,
                    **kwargs
                )
                
                logger.info(f"✅ Client '{name}' created via {protocol.value}")
                return client
                
            except NotImplementedError:
                logger.debug(f"⚠️ {protocol.value} doesn't support client")
                continue
            except Exception as e:
                logger.warning(f"⚠️ Failed to create client with {protocol.value}: {e}")
                continue
        
        raise InterfaceError(
            f"Cannot create client '{name}'. "
            f"Tried protocols: {[p.value for p in protocols]}"
        )

    @staticmethod
    async def create_action_server(
        name: str,
        handle_goal_request: Callable,
        handle_cancel_request: Callable,
        execution_callback: Callable,
        protocols: Optional[List[ProtocolType]] = None,
        **kwargs
    ) -> VyraActionServer:
        """
        Create Action Server with automatic protocol selection.
        
        Args:
            name: Action name
            handle_goal_request: Async callback to accept/reject goals
            handle_cancel_request: Async callback for cancellations
            execution_callback: Async callback for goal execution
            protocols: Preferred protocols (fallback if not specified)
            **kwargs: Additional parameters (action_type, qos, etc.)
            
        Returns:
            VyraActionServer: Initialized action server
            
        Raises:
            InterfaceError: If no protocol available
        """
        protocols = protocols or InterfaceFactory.ACTION_SERVER_FALLBACK
        registry = ProviderRegistry()
        
        for protocol in protocols:
            try:
                provider = registry.get_provider(protocol)
                
                if not provider:
                    logger.debug(f"⚠️ No provider for {protocol.value}")
                    continue
                
                if not await provider.check_availability():
                    logger.debug(f"⚠️ Protocol {protocol.value} not available")
                    continue
                
                action_server = await provider.create_action_server(
                    name=name,
                    handle_goal_request=handle_goal_request,
                    handle_cancel_request=handle_cancel_request,
                    execution_callback=execution_callback,
                    **kwargs
                )
                
                logger.info(f"✅ ActionServer '{name}' created via {protocol.value}")
                return action_server
                
            except NotImplementedError:
                logger.debug(f"⚠️ {protocol.value} doesn't support action server")
                continue
            except Exception as e:
                logger.warning(f"⚠️ Failed to create action server with {protocol.value}: {e}")
                continue
        
        raise InterfaceError(
            f"Cannot create action server '{name}'. "
            f"Tried protocols: {[p.value for p in protocols]}"
        )

    @staticmethod
    async def create_action_client(
        name: str,
        direct_response_callback: Optional[Callable] = None,
        feedback_callback: Optional[Callable] = None,
        goal_callback: Optional[Callable] = None,
        protocols: Optional[List[ProtocolType]] = None,
        **kwargs
    ) -> VyraActionClient:
        """
        Create Action Client with automatic protocol selection.
        
        Args:
            name: Action name
            direct_response_callback: Async callback for goal acceptance
            feedback_callback: Async callback for feedback
            goal_callback: Async callback for final result
            protocols: Preferred protocols (fallback if not specified)
            **kwargs: Additional parameters (action_type, qos, etc.)
            
        Returns:
            VyraActionClient: Initialized action client
            
        Raises:
            InterfaceError: If no protocol available
        """
        protocols = protocols or InterfaceFactory.ACTION_CLIENT_FALLBACK
        registry = ProviderRegistry()
        
        for protocol in protocols:
            try:
                provider = registry.get_provider(protocol)
                
                if not provider:
                    logger.debug(f"⚠️ No provider for {protocol.value}")
                    continue
                
                if not await provider.check_availability():
                    logger.debug(f"⚠️ Protocol {protocol.value} not available")
                    continue
                
                action_client = await provider.create_action_client(
                    name=name,
                    direct_response_callback=direct_response_callback,
                    feedback_callback=feedback_callback,
                    goal_callback=goal_callback,
                    **kwargs
                )
                
                logger.info(f"✅ ActionClient '{name}' created via {protocol.value}")
                return action_client
                
            except NotImplementedError:
                logger.debug(f"⚠️ {protocol.value} doesn't support action client")
                continue
            except Exception as e:
                logger.warning(f"⚠️ Failed to create action client with {protocol.value}: {e}")
                continue
        
        raise InterfaceError(
            f"Cannot create action client '{name}'. "
            f"Tried protocols: {[p.value for p in protocols]}"
        )

    # ========================================================================
    # UTILITY METHODS
    # ========================================================================
    
    @staticmethod
    def get_available_protocols(
        interface_type: str,
        protocols: List[ProtocolType]
    ) -> List[ProtocolType]:
        """
        Customize fallback chain for interface type.
        
        Args:
            interface_type: "callable", "speaker", or "job"
            protocols: Ordered list of protocols to try
            
        Example:
            >>> # Prioritize SharedMemory over ROS2
            >>> InterfaceFactory.set_fallback_chain(
            ...     "callable",
            ...     [ProtocolType.SHARED_MEMORY, ProtocolType.ROS2, ProtocolType.UDS]
            ... )
        """
        if interface_type == "callable":
            InterfaceFactory.CALLABLE_FALLBACK = protocols
        elif interface_type == "speaker":
            InterfaceFactory.SPEAKER_FALLBACK = protocols
        elif interface_type == "job":
            InterfaceFactory.JOB_FALLBACK = protocols
        else:
            raise ValueError(f"Invalid interface_type: {interface_type}")
        
        logger.info(f"✅ Fallback chain for {interface_type} updated")
        return protocols