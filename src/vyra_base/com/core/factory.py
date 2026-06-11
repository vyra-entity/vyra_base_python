"""
Interface Factory

Unified interface creation with automatic protocol selection and fallback.
"""
import asyncio
import functools
import logging

from typing import Any, Callable, Dict, Optional, List, Union

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
from vyra_base.com.core.topic_builder import TopicBuilder
from vyra_base.com.transport.registry import (
    add_publisher as add_publisher_registry,
    add_subscriber as add_subscriber_registry,
    add_server as add_server_registry,
    add_client as add_client_registry,
    add_action_server as add_action_server_registry,
    add_action_client as add_action_client_registry
)

logger = logging.getLogger(__name__)


class TransportProviderFactory:
    """
    Factory for creating communication interfaces with protocol fallback.
    
    Features:
    - Automatic protocol selection
    - Fallback chain (ROS2 → SharedMemory → UDS → Redis → gRPC)
    - Protocol availability checking
    - Graceful degradation
    
    Example:
        >>> # Auto-select best available protocol
        >>> server = await TransportProviderFactory.create_server(
        ...     "my_service",
        ...     callback=handle_request
        ... )
        >>> 
        >>> # Explicit protocol with fallback
        >>> server = await TransportProviderFactory.create_server(
        ...     "my_service",
        ...     protocols=[ProtocolType.ROS2, ProtocolType.SHARED_MEMORY],
        ...     callback=handle_request
        ... )
        >>> 
        >>> # Publisher for pub/sub
        >>> publisher = await TransportProviderFactory.create_publisher(
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
    def _debug_wrap(
        callback: Optional[Callable],
        name: str,
        iface_type: str,
        protocol: str,
    ) -> Optional[Callable]:
        """Wrap a callback with a DEBUG log entry on every invocation.

        Logs the interface type, topic/service name, protocol and the raw
        arguments that were received from the transport layer.  The wrapper
        is transparent – it forwards all positional and keyword arguments
        unchanged and returns whatever the original callback returns.

        Args:
            callback:   The original callback to wrap. If *None* the method
                        returns *None* unchanged.
            name:       Topic / service name of the interface.
            iface_type: Human-readable interface category used in the log
                        message (e.g. ``"subscribe"``, ``"call"``,
                        ``"actionCall.execute"``).
            protocol:   Protocol name as reported by
                        :class:`~vyra_base.com.core.types.ProtocolType`
                        (e.g. ``"ros2"``, ``"zenoh"``).

        Returns:
            Wrapped callback (async-safe) or *None* if *callback* was *None*.
        """
        if callback is None:
            return None

        if asyncio.iscoroutinefunction(callback):
            @functools.wraps(callback)
            async def _async_wrapper(*args: Any, **kwargs: Any) -> Any:
                logger.debug(
                    "[TRANSPORT IN] type=%s  name='%s'  protocol=%s | args=%r  kwargs=%r",
                    iface_type, name, protocol, args, kwargs,
                )
                return await callback(*args, **kwargs)
            return _async_wrapper
        else:
            @functools.wraps(callback)
            def _sync_wrapper(*args: Any, **kwargs: Any) -> Any:
                logger.debug(
                    "[TRANSPORT IN] type=%s  name='%s'  protocol=%s | args=%r  kwargs=%r",
                    iface_type, name, protocol, args, kwargs,
                )
                return callback(*args, **kwargs)
            return _sync_wrapper

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

        Only the own (default) provider is used.

        Args:
            name: Topic/channel name
            protocols: Preferred protocols (fallback if not specified)
            **kwargs: Additional parameters (message_type, qos, etc.)

        Returns:
            VyraPublisher: Initialized publisher

        Raises:
            InterfaceError: If no protocol available
        """
        protocols = protocols or TransportProviderFactory.PUBLISHER_FALLBACK
        provider_registry = ProviderRegistry()

        for protocol in protocols:
            try:
                # Publishers always use own provider (module_id=None)
                provider = provider_registry.get_provider(protocol, module_id=None)
                
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

                add_publisher_registry(publisher)
                
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
        module_id: Optional[str] = None,
        module_name: Optional[str] = None,
        **kwargs
    ) -> Optional[VyraSubscriber]:
        """
        Create Subscriber with automatic protocol selection.

        Args:
            name: Topic/channel name
            subscriber_callback: Async callback for received messages
            protocols: Preferred protocols (fallback if not specified)
            module_id: Optional target module ID. When provided the registry
                       looks up (or creates) a provider scoped to that module.
            module_name: Human-readable name of the target module (required
                         when module_id is supplied).
            **kwargs: Additional parameters (message_type, qos, etc.)

        Returns:
            Optional[VyraSubscriber]: Initialized subscriber or None if pending

        Raises:
            InterfaceError: If no protocol available
        """
        protocols = protocols or TransportProviderFactory.SUBSCRIBER_FALLBACK
        provider_registry = ProviderRegistry()

        # Inject a remote TopicBuilder when targeting a foreign module
        if module_id and module_name:
            kwargs.setdefault("topic_builder", TopicBuilder(module_name, module_id))

        if not subscriber_callback:
            logger.warning(
                "create_subscriber: '%s' has no subscriber_callback — "
                "the EndpointOrchestrator will activate it once a callback is bound.",
                name,
            )
            return None

        for protocol in protocols:
            try:
                if module_id and module_name:
                    provider = provider_registry.get_or_create_provider_for_module(
                        protocol, module_name, module_id
                    )
                else:
                    provider = provider_registry.get_provider(protocol, module_id=None)
                
                if not provider:
                    logger.debug(f"⚠️ No provider for {protocol.value}")
                    continue
                
                if not await provider.check_availability():
                    logger.debug(f"⚠️ Protocol {protocol.value} not available")
                    continue
                
                subscriber = await provider.create_subscriber(
                    name=name,
                    subscriber_callback=TransportProviderFactory._debug_wrap(
                        subscriber_callback, name, "subscribe", protocol.value
                    ),
                    **kwargs
                )
                
                add_subscriber_registry(subscriber)

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
    ) -> Optional[VyraServer]:
        """
        Create Server with automatic protocol selection.

        Only the own (default) provider is used. module_id / module_name are
        determined by the provider that was registered without a module_id.

        Args:
            name: Service name
            response_callback: Async callback for requests
            protocols: Preferred protocols (fallback if not specified)
            **kwargs: Additional parameters (service_type, qos, etc.)

        Returns:
            Optional[VyraServer]: Initialized server or None if pending

        Raises:
            InterfaceError: If no protocol available
        """
        protocols = protocols or TransportProviderFactory.SERVER_FALLBACK
        provider_registry = ProviderRegistry()

        if not response_callback:
            logger.warning(
                "create_server: '%s' has no response_callback — "
                "the EndpointOrchestrator will activate it once a callback is bound.",
                name,
            )
            return None

        for protocol in protocols:
            try:
                # Servers always use own provider (module_id=None)
                provider = provider_registry.get_provider(protocol, module_id=None)
                
                if not provider:
                    logger.debug(f"⚠️ No provider for {protocol.value}")
                    continue
                
                if not await provider.check_availability():
                    logger.debug(f"⚠️ Protocol {protocol.value} not available")
                    continue
                
                server = await provider.create_server(
                    name=name,
                    response_callback=TransportProviderFactory._debug_wrap(
                        response_callback, name, "call", protocol.value
                    ),
                    **kwargs
                )
                
                add_server_registry(server)
                
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
        module_id: Optional[str] = None,
        module_name: Optional[str] = None,
        **kwargs
    ) -> VyraClient:
        """
        Create Client with automatic protocol selection.

        Args:
            name: Service name
            protocols: Preferred protocols (fallback if not specified)
            module_id: Optional target module ID. When provided the registry
                       looks up (or creates) a provider scoped to that module.
            module_name: Human-readable name of the target module (required
                         when module_id is supplied).
            **kwargs: Additional parameters (service_type, qos, etc.)

        Returns:
            VyraClient: Initialized client

        Raises:
            InterfaceError: If no protocol available
        """
        protocols = protocols or TransportProviderFactory.CLIENT_FALLBACK
        provider_registry = ProviderRegistry()

        # Inject a remote TopicBuilder when targeting a foreign module
        if module_id and module_name:
            kwargs.setdefault("topic_builder", TopicBuilder(module_name, module_id))

        for protocol in protocols:
            try:
                if module_id and module_name:
                    provider = provider_registry.get_or_create_provider_for_module(
                        protocol, module_name, module_id
                    )
                else:
                    provider = provider_registry.get_provider(protocol, module_id=None)
                
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
                
                add_client_registry(client)

                # Wrap client.call() so every outbound call and its inbound
                # response are logged at DEBUG level.
                _proto = protocol.value
                _name = name
                _original_call = client.call
                async def _logged_call(request: Any, timeout: float = 5.0) -> Any:
                    logger.debug(
                        "[TRANSPORT OUT] type=call  name='%s'  protocol=%s | request=%r  timeout=%r | module_name=%s  module_id=%s",
                        _name, _proto, request, timeout, kwargs.get("module_name"), kwargs.get("module_id")
                    )
                    result = await _original_call(request, timeout=timeout)
                    logger.debug(
                        "[TRANSPORT IN]  type=call  name='%s'  protocol=%s | result=%r | module_name=%s  module_id=%s",
                        _name, _proto, result, kwargs.get("module_name"), kwargs.get("module_id")
                    )
                    return result
                client.call = _logged_call

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
        handle_goal_request: Callable | None,
        handle_cancel_request: Callable | None,
        execution_callback: Callable | None,
        protocols: Optional[List[ProtocolType]] = None,
        **kwargs
    ) -> Optional[VyraActionServer]:
        """
        Create Action Server with automatic protocol selection.

        Only the own (default) provider is used.

        Args:
            name: Action name
            handle_goal_request: Async callback to accept/reject goals
            handle_cancel_request: Async callback for cancellations
            execution_callback: Async callback for goal execution
            protocols: Preferred protocols (fallback if not specified)
            **kwargs: Additional parameters (action_type, qos, etc.)

        Returns:
            Optional[VyraActionServer]: Initialized action server or None if pending

        Raises:
            InterfaceError: If no protocol available
        """
        protocols = protocols or TransportProviderFactory.ACTION_SERVER_FALLBACK
        provider_registry = ProviderRegistry()

        if not handle_goal_request or not handle_cancel_request or not execution_callback:
            logger.warning(
                "create_action_server: '%s' is missing callbacks "
                "(goal=%s, cancel=%s, execute=%s) — "
                "the EndpointOrchestrator will activate it once all callbacks are bound.",
                name,
                bool(handle_goal_request),
                bool(handle_cancel_request),
                bool(execution_callback),
            )
            return None

        for protocol in protocols:
            try:
                # Action servers always use own provider (module_id=None)
                provider = provider_registry.get_provider(protocol, module_id=None)
                
                if not provider:
                    logger.debug(f"⚠️ No provider for {protocol.value}")
                    continue
                
                if not await provider.check_availability():
                    logger.debug(f"⚠️ Protocol {protocol.value} not available")
                    continue

                action_server = await provider.create_action_server(
                    name=name,
                    handle_goal_request=TransportProviderFactory._debug_wrap(
                        handle_goal_request, name, "actionCall.goal", protocol.value
                    ),
                    handle_cancel_request=TransportProviderFactory._debug_wrap(
                        handle_cancel_request, name, "actionCall.cancel", protocol.value
                    ),
                    execution_callback=TransportProviderFactory._debug_wrap(
                        execution_callback, name, "actionCall.execute", protocol.value
                    ),
                    **kwargs
                )
                
                add_action_server_registry(action_server)
                
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
        module_id: Optional[str] = None,
        module_name: Optional[str] = None,
        **kwargs
    ) -> Optional[VyraActionClient]:
        """
        Create Action Client with automatic protocol selection.

        Args:
            name: Action name
            direct_response_callback: Async callback for goal acceptance
            feedback_callback: Async callback for feedback
            goal_callback: Async callback for final result
            protocols: Preferred protocols (fallback if not specified)
            module_id: Optional target module ID. When provided the registry
                       looks up (or creates) a provider scoped to that module.
            module_name: Human-readable name of the target module (required
                         when module_id is supplied).
            **kwargs: Additional parameters (action_type, qos, etc.)

        Returns:
            Optional[VyraActionClient]: Initialized action client or None if pending

        Raises:
            InterfaceError: If no protocol available
        """
        protocols = protocols or TransportProviderFactory.ACTION_CLIENT_FALLBACK
        provider_registry = ProviderRegistry()

        # Inject a remote TopicBuilder when targeting a foreign module
        if module_id and module_name:
            kwargs.setdefault("topic_builder", TopicBuilder(module_name, module_id))

        for protocol in protocols:
            try:
                if module_id and module_name:
                    provider = provider_registry.get_or_create_provider_for_module(
                        protocol, module_name, module_id
                    )
                else:
                    provider = provider_registry.get_provider(protocol, module_id=None)
                
                if not provider:
                    logger.debug(f"⚠️ No provider for {protocol.value}")
                    continue
                
                if not await provider.check_availability():
                    logger.debug(f"⚠️ Protocol {protocol.value} not available")
                    continue
                
                action_client = await provider.create_action_client(
                    name=name,
                    direct_response_callback=TransportProviderFactory._debug_wrap(
                        direct_response_callback, name, "actionCall.response", protocol.value
                    ),
                    feedback_callback=TransportProviderFactory._debug_wrap(
                        feedback_callback, name, "actionCall.feedback", protocol.value
                    ),
                    goal_callback=TransportProviderFactory._debug_wrap(
                        goal_callback, name, "actionCall.result", protocol.value
                    ),
                    **kwargs
                )
                
                add_action_client_registry(action_client)

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
            interface_type: "server", "publisher", or "actionServer"
            protocols: Ordered list of protocols to try
            
        Example:
            >>> # Prioritize SharedMemory over ROS2
            >>> TransportProviderFactory.set_fallback_chain(
            ...     "server",
            ...     [ProtocolType.SHARED_MEMORY, ProtocolType.ROS2, ProtocolType.UDS]
            ... )
        """
        if interface_type == "server":
            TransportProviderFactory.SERVER_FALLBACK = protocols
        elif interface_type == "publisher":
            TransportProviderFactory.PUBLISHER_FALLBACK = protocols
        elif interface_type == "actionServer":
            TransportProviderFactory.ACTION_SERVER_FALLBACK = protocols
        else:
            raise ValueError(f"Invalid interface_type: {interface_type}")
        
        logger.info(f"✅ Fallback chain for {interface_type} updated")
        return protocols
    
    # ========================================================================
    