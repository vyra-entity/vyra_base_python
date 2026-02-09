"""
Interface Factory

Unified interface creation with automatic protocol selection and fallback.
"""
from typing import Any, Callable, Optional, List, Union
from vyra_base.com.core.types import (
    ProtocolType,
    VyraCallable,
    VyraSpeaker,
    VyraJob,
)
from vyra_base.com.core.exceptions import (
    ProtocolUnavailableError,
    InterfaceError,
)
from vyra_base.com.providers.protocol_provider import AbstractProtocolProvider
from vyra_base.com.providers.provider_registry import ProviderRegistry
from vyra_base.helper.logger import Logger


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
    CALLABLE_FALLBACK = [
        ProtocolType.ZENOH,
        ProtocolType.ROS2,
        ProtocolType.REDIS,
        ProtocolType.UDS
    ]
    
    SPEAKER_FALLBACK = [
        ProtocolType.ZENOH,
        ProtocolType.ROS2,
        ProtocolType.REDIS,
        ProtocolType.UDS
    ]
    
    JOB_FALLBACK = [
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

    @staticmethod
    async def create_callable(
        name: str,
        callback: Optional[Callable] = None,
        protocols: Optional[List[ProtocolType]] = None,
        **kwargs
    ) -> VyraCallable:
        """
        Create Callable with automatic protocol selection.
        
        Args:
            name: Callable name
            callback: Optional callback for server-side
            protocols: Preferred protocols (fallback if not specified)
            **kwargs: Additional parameters passed to provider
            
        Returns:
            VyraCallable: Initialized callable
            
        Raises:
            InterfaceError: If no protocol available
            
        Example:
            >>> # Auto-select (uses CALLABLE_FALLBACK)
            >>> callable = await InterfaceFactory.create_callable(
            ...     "calculate",
            ...     callback=lambda req: {"result": req["x"] + req["y"]}
            ... )
            >>> 
            >>> # Explicit protocol preference
            >>> callable = await InterfaceFactory.create_callable(
            ...     "calculate",
            ...     protocols=[ProtocolType.ROS2, ProtocolType.UDS],
            ...     callback=handle_calc
            ... )
        """
        if callback is None:
            Logger.info("callback-object is None")
        
        protocols = protocols or InterfaceFactory.CALLABLE_FALLBACK
        registry = ProviderRegistry()
        
        for protocol in protocols:
            try:
                provider = registry.get_provider(protocol)
                
                if not provider:
                    Logger.debug(f"⚠️ No provider for {protocol.value}")
                    continue
                
                if not await provider.check_availability():
                    Logger.debug(f"⚠️ Protocol {protocol.value} not available")
                    continue
                
                callable_instance = await provider.create_callable(
                    name=name,
                    callback=callback,
                    **kwargs
                )
                
                Logger.info(f"✅ Callable '{name}' created via {protocol.value}")
                return callable_instance
                
            except Exception as e:
                Logger.warning(f"⚠️ Failed to create callable with {protocol.value}: {e}")
                continue
        
        # No protocol worked
        raise InterfaceError(
            f"Cannot create callable '{name}'. "
            f"Tried protocols: {[p.value for p in protocols]}"
        )
    
    @staticmethod
    async def create_speaker(
        name: str,
        callback: Optional[Callable] = None,
        protocols: Optional[List[ProtocolType]] = None,
        **kwargs
    ) -> VyraSpeaker:
        """
        Create Speaker with automatic protocol selection.
        
        Args:
            name: Topic/channel name
            callback: Optional callback for subscriptions
            protocols: Preferred protocols (fallback if not specified)
            **kwargs: Additional parameters (qos, retain, etc.)
            
        Returns:
            VyraSpeaker: Initialized speaker
            
        Raises:
            InterfaceError: If no protocol available
            
        Example:
            >>> # Auto-select (uses SPEAKER_FALLBACK)
            >>> speaker = await InterfaceFactory.create_speaker(
            ...     "temperature",
            ...     callback=lambda msg: print(f"Temp: {msg}")
            ... )
            >>> await speaker.shout({"value": 23.5})
            >>> 
            >>> # Explicit MQTT with QoS 2
            >>> speaker = await InterfaceFactory.create_speaker(
            ...     "sensor/humidity",
            ...     protocols=[ProtocolType.MQTT],
            ...     qos=2,
            ...     retain=True
            ... )
        """
        protocols = protocols or InterfaceFactory.SPEAKER_FALLBACK
        registry = ProviderRegistry()
        
        for protocol in protocols:
            try:
                provider = registry.get_provider(protocol)
                
                if not provider:
                    Logger.debug(f"⚠️ No provider for {protocol.value}")
                    continue
                
                if not await provider.check_availability():
                    Logger.debug(f"⚠️ Protocol {protocol.value} not available")
                    continue
                
                speaker = await provider.create_speaker(
                    name=name,
                    callback=callback,
                    **kwargs
                )
                
                Logger.info(f"✅ Speaker '{name}' created via {protocol.value}")
                return speaker
                
            except NotImplementedError:
                # Some providers don't support speakers
                Logger.debug(f"⚠️ {protocol.value} doesn't support speakers")
                continue
            except Exception as e:
                Logger.warning(f"⚠️ Failed to create speaker with {protocol.value}: {e}")
                continue
        
        # No protocol worked
        raise InterfaceError(
            f"Cannot create speaker '{name}'. "
            f"Tried protocols: {[p.value for p in protocols]}"
        )

    @staticmethod
    async def create_listener(
        name: str,
        callback: Callable[[Any], None],
        protocols: Optional[List[ProtocolType]] = None,
        **kwargs
    ) -> VyraSpeaker:
        """
        Create Speaker and start listening immediately. Used for better usability.
        
        Args:
            name: Topic/channel name
            callback: Callback for received messages
            protocols: Preferred protocols (fallback if not specified)
            **kwargs: Additional parameters
        Returns:
            VyraSpeaker: Initialized speaker with active subscription
        Raises:
            InterfaceError: If no protocol available
        """
        if "is_publisher" in kwargs and kwargs["is_publisher"]:
            raise ValueError("create_listener is for subscribers only. Remove 'is_publisher' flag.")
        
        speaker = await InterfaceFactory.create_speaker(
            name=name,
            callback=callback,
            protocols=protocols,
            **kwargs
        )
        
        await speaker.listen(callback)
        return speaker
    
    @staticmethod
    async def create_job(
        name: str,
        callback: Optional[Callable] = None,
        protocols: Optional[List[ProtocolType]] = None,
        **kwargs
    ) -> VyraJob:
        """
        Create Job with automatic protocol selection.
        
        Args:
            name: Job name
            callback: Optional callback for job execution
            protocols: Preferred protocols (fallback if not specified)
            **kwargs: Additional parameters
            
        Returns:
            VyraJob: Initialized job
            
        Raises:
            InterfaceError: If no protocol available
            
        Example:
            >>> # Auto-select (uses JOB_FALLBACK)
            >>> job = await InterfaceFactory.create_job(
            ...     "process_images",
            ...     callback=process_batch
            ... )
            >>> await job.enqueue({"images": ["img1.jpg", "img2.jpg"]})
        """
        protocols = protocols or InterfaceFactory.JOB_FALLBACK
        registry = ProviderRegistry()
        
        if callback is None:
                Logger.warning("callback-object is None")
                raise TypeError
        
        for protocol in protocols:
            try:
                provider = registry.get_provider(protocol)
                
                if not provider:
                    Logger.debug(f"⚠️ No provider for {protocol.value}")
                    continue
                
                if not await provider.check_availability():
                    Logger.debug(f"⚠️ Protocol {protocol.value} not available")
                    continue
                
                job = await provider.create_job(
                    name=name,
                    callback=callback,
                    **kwargs
                )
                
                Logger.info(f"✅ Job '{name}' created via {protocol.value}")
                return job
                
            except NotImplementedError:
                # Some providers don't support jobs
                Logger.debug(f"⚠️ {protocol.value} doesn't support jobs")
                continue
            except Exception as e:
                Logger.warning(f"⚠️ Failed to create job with {protocol.value}: {e}")
                continue
        
        # No protocol worked
        raise InterfaceError(
            f"Cannot create job '{name}'. "
            f"Tried protocols: {[p.value for p in protocols]}"
        )
    
    @staticmethod
    def get_available_protocols() -> List[ProtocolType]:
        """
        Get list of currently available protocols.
        
        Returns:
            List[ProtocolType]: Available protocols
            
        Example:
            >>> protocols = InterfaceFactory.get_available_protocols()
            >>> if ProtocolType.ROS2 in protocols:
            ...     print("ROS2 is available")
        """
        registry = ProviderRegistry()
        available = []
        
        for protocol in ProtocolType:
            provider = registry.get_provider(protocol)
            if provider:
                try:
                    # Sync check (providers should cache availability)
                    available.append(protocol)
                except Exception:
                    pass
        
        return available
    
    @staticmethod
    def set_fallback_chain(
        interface_type: str,
        protocols: List[ProtocolType]
    ) -> None:
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
        
        Logger.info(f"✅ Fallback chain for {interface_type} updated")
