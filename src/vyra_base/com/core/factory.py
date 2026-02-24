"""
Interface Factory

Unified interface creation with automatic protocol selection and fallback.
"""
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
from vyra_base.com.core.blueprints import (
    HandlerBlueprint,
    ServiceBlueprint,
    PublisherBlueprint,
    SubscriberBlueprint,
    ActionBlueprint,
    InterfaceType as BlueprintInterfaceType
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
        >>> server = await InterfaceFactory.create_server(
        ...     "my_service",
        ...     callback=handle_request
        ... )
        >>> 
        >>> # Explicit protocol with fallback
        >>> server = await InterfaceFactory.create_server(
        ...     "my_service",
        ...     protocols=[ProtocolType.ROS2, ProtocolType.SHARED_MEMORY],
        ...     callback=handle_request
        ... )
        >>> 
        >>> # Publisher for pub/sub
        >>> publisher = await InterfaceFactory.create_publisher(
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
    
    _pending_interface: Dict[str, Any] = {}
    
    @staticmethod
    def loop_check_pending():
        """
        Check pending interfaces and initialize if callbacks are now available.
        Should be called periodically (e.g. in main loop) to handle late callback registration.
        """
        for name, info in list(InterfaceFactory._pending_interface.items()):
            provider_func = info["provider"]
            kwargs = info["kwargs"]
            
            # Check if required callbacks are now available based on interface type
            # A server needs "response_callback", a subscriber needs "subscriber_callback",
            # an action server needs all three: handle_goal_request, handle_cancel_request, execution_callback
            ready = False
            if "response_callback" in kwargs and kwargs.get("response_callback"):
                ready = True  # Server: response_callback now available
            elif "subscriber_callback" in kwargs and kwargs.get("subscriber_callback"):
                ready = True  # Subscriber: subscriber_callback now available
            elif (
                kwargs.get("handle_goal_request") and
                kwargs.get("handle_cancel_request") and
                kwargs.get("execution_callback")
            ):
                ready = True  # ActionServer: all three callbacks now available
            
            if ready:
                try:
                    # Create the interface using the provider function
                    interface = provider_func(**kwargs)
                    
                    # Add to registry based on type
                    if "subscriber_callback" in kwargs:
                        add_subscriber_registry(interface)
                    elif "response_callback" in kwargs:
                        add_server_registry(interface)
                    elif "handle_goal_request" in kwargs and "execution_callback" in kwargs:
                        add_action_server_registry(interface)
                    
                    logger.info(f"✅ Pending interface '{name}' initialized and registered")
                    del InterfaceFactory._pending_interface[name]
                except Exception as e:
                    logger.error(f"❌ Failed to initialize pending interface '{name}': {e}")

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
        protocols = protocols or InterfaceFactory.PUBLISHER_FALLBACK
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
        protocols = protocols or InterfaceFactory.SUBSCRIBER_FALLBACK
        provider_registry = ProviderRegistry()

        # Inject a remote TopicBuilder when targeting a foreign module
        if module_id and module_name:
            kwargs.setdefault("topic_builder", TopicBuilder(module_name, module_id))

        PENDING = not subscriber_callback

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
                
                if PENDING:
                    logger.info(
                        f"⚠️ Creating subscriber '{name}' with protocol {protocol.value} "
                        f"but missing subscriber callback"
                    )
                    InterfaceFactory._pending_interface[name] = {
                        "provider": provider.create_subscriber,
                        "kwargs": {
                            "subscriber_callback": subscriber_callback,
                            **kwargs
                        }
                    }
                    logger.info(f"✅ Subscriber '{name}' registered as pending with {protocol.value}")
                    return None  # Return None for pending interface
                
                subscriber = await provider.create_subscriber(
                    name=name,
                    subscriber_callback=subscriber_callback,
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
        protocols = protocols or InterfaceFactory.SERVER_FALLBACK
        provider_registry = ProviderRegistry()

        PENDING = not response_callback

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
                
                if PENDING:
                    logger.info(
                        f"⚠️ Creating server '{name}' with protocol {protocol.value} "
                        f"but missing response callback"
                    )
                    InterfaceFactory._pending_interface[name] = {
                        "provider": provider.create_server,
                        "kwargs": {
                            "response_callback": response_callback,
                            **kwargs
                        }
                    }
                    logger.info(f"✅ Server '{name}' registered as pending with {protocol.value}")
                    return None  # Return None for pending interface
                
                server = await provider.create_server(
                    name=name,
                    response_callback=response_callback,
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
        protocols = protocols or InterfaceFactory.CLIENT_FALLBACK
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
        protocols = protocols or InterfaceFactory.ACTION_SERVER_FALLBACK
        provider_registry = ProviderRegistry()

        PENDING = (
            not handle_goal_request or
            not execution_callback or
            not handle_cancel_request
        )

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
                
                if PENDING:
                    logger.info(
                        f"⚠️ Creating action server '{name}' with protocol {protocol.value} "
                        f"but missing callbacks (goal: {bool(handle_goal_request)}, "
                        f"cancel: {bool(handle_cancel_request)}, "
                        f"execution: {bool(execution_callback)})"
                    )
                    InterfaceFactory._pending_interface[name] = {
                        "provider": provider.create_action_server,
                        "kwargs": {
                            "handle_goal_request": handle_goal_request,
                            "handle_cancel_request": handle_cancel_request,
                            "execution_callback": execution_callback,
                            **kwargs
                        }
                    }
                    logger.info(f"✅ Action server '{name}' registered as pending with {protocol.value}")
                    return None  # Return None for pending interface

                action_server = await provider.create_action_server(
                    name=name,
                    handle_goal_request=handle_goal_request,
                    handle_cancel_request=handle_cancel_request,
                    execution_callback=execution_callback,
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
        protocols = protocols or InterfaceFactory.ACTION_CLIENT_FALLBACK
        provider_registry = ProviderRegistry()

        # Inject a remote TopicBuilder when targeting a foreign module
        if module_id and module_name:
            kwargs.setdefault("topic_builder", TopicBuilder(module_name, module_id))

        PENDING = (
            not direct_response_callback or
            not feedback_callback or
            not goal_callback
        )

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
                
                if PENDING:
                    logger.info(
                        f"⚠️ Creating action client '{name}' with protocol {protocol.value} "
                        f"but missing callbacks (direct: {bool(direct_response_callback)}, "
                        f"feedback: {bool(feedback_callback)}, "
                        f"goal: {bool(goal_callback)})"
                    )
                    InterfaceFactory._pending_interface[name] = {
                        "provider": provider.create_action_client,
                        "kwargs": {
                            "direct_response_callback": direct_response_callback,
                            "feedback_callback": feedback_callback,
                            "goal_callback": goal_callback,
                            **kwargs
                        }
                    }
                    logger.info(f"✅ Action client '{name}' registered as pending with {protocol.value}")
                    return None  # Return None for pending interface
                
                action_client = await provider.create_action_client(
                    name=name,
                    direct_response_callback=direct_response_callback,
                    feedback_callback=feedback_callback,
                    goal_callback=goal_callback,
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
            >>> InterfaceFactory.set_fallback_chain(
            ...     "server",
            ...     [ProtocolType.SHARED_MEMORY, ProtocolType.ROS2, ProtocolType.UDS]
            ... )
        """
        if interface_type == "server":
            InterfaceFactory.SERVER_FALLBACK = protocols
        elif interface_type == "publisher":
            InterfaceFactory.PUBLISHER_FALLBACK = protocols
        elif interface_type == "actionServer":
            InterfaceFactory.ACTION_SERVER_FALLBACK = protocols
        else:
            raise ValueError(f"Invalid interface_type: {interface_type}")
        
        logger.info(f"✅ Fallback chain for {interface_type} updated")
        return protocols
    
    # ========================================================================
    # BLUEPRINT-BASED INTERFACE CREATION
    # ========================================================================
    
    @staticmethod
    async def create_from_blueprint(
        blueprint: HandlerBlueprint,
        **override_kwargs
    ) -> Optional[Union[VyraServer, VyraPublisher, VyraSubscriber, VyraActionServer]]:
        """
        Create interface from a HandlerBlueprint.
        
        This is the primary method for two-phase initialization:
        1. Blueprint created during decoration/configuration
        2. Interface created when blueprint is bound with callback
        
        Args:
            blueprint: HandlerBlueprint instance
            **override_kwargs: Override blueprint metadata
            
        Returns:
            Created interface or None if pending (callback not bound yet)
            
        Example:
            >>> from vyra_base.com.core.blueprints import ServiceBlueprint
            >>> blueprint = ServiceBlueprint(name="my_service")
            >>> # ... later when callback available ...
            >>> blueprint.bind_callback(my_callback)
            >>> server = await InterfaceFactory.create_from_blueprint(blueprint)
        """
        
        
        # Merge blueprint metadata with overrides
        kwargs = {**blueprint.metadata, **override_kwargs}
        protocols = blueprint.protocols if blueprint.protocols else None
        
        # Dispatch to appropriate creation method
        if isinstance(blueprint, ServiceBlueprint):
            return await InterfaceFactory.create_server(
                name=blueprint.name,
                response_callback=blueprint.callback,
                protocols=protocols,
                service_type=blueprint.service_type,
                **kwargs
            )
        
        elif isinstance(blueprint, PublisherBlueprint):
            return await InterfaceFactory.create_publisher(
                name=blueprint.name,
                protocols=protocols,
                message_type=blueprint.message_type,
                **kwargs
            )
        
        elif isinstance(blueprint, SubscriberBlueprint):
            if not blueprint.callback:
                logger.warning(
                    f"⚠️ Subscriber blueprint '{blueprint.name}' missing callback, "
                    f"registering as pending"
                )
                InterfaceFactory._pending_interface[blueprint.name] = {
                    "provider": InterfaceFactory.create_subscriber,
                    "kwargs": {**kwargs, "subscriber_callback": blueprint.callback}
                }
                return None  # Return None for pending interface
            
            return await InterfaceFactory.create_subscriber(
                name=blueprint.name,
                subscriber_callback=blueprint.callback,
                protocols=protocols,
                message_type=blueprint.message_type,
                **kwargs
            )
        
        elif isinstance(blueprint, ActionBlueprint):
            # Extract all three callbacks from ActionBlueprint
            return await InterfaceFactory.create_action_server(
                name=blueprint.name,
                handle_goal_request=blueprint.get_callback('on_goal'),
                handle_cancel_request=blueprint.get_callback('on_cancel'),
                execution_callback=blueprint.get_callback('execute'),
                protocols=protocols,
                action_type=blueprint.action_type,
                **kwargs
            )
        
        else:
            raise InterfaceError(f"Unknown blueprint type: {type(blueprint)}")
    
    @staticmethod
    async def bind_pending_callback(
        name: str,
        callback: Optional[Callable] = None,
        handle_goal_request: Optional[Callable] = None,
        handle_cancel_request: Optional[Callable] = None,
        execution_callback: Optional[Callable] = None,
    ) -> Optional[Union[VyraServer, VyraSubscriber, VyraActionServer]]:
        """
        Bind one or more callbacks to a pending interface and initialize it.
        
        For servers/subscribers, use the ``callback`` parameter.
        For action servers, use the dedicated ``handle_goal_request``,
        ``handle_cancel_request`` and ``execution_callback`` parameters.
        If a parameter is ``None``, the previously stored value is kept.
        
        Args:
            name: Interface name (must match pending registration)
            callback: Callback for server (response_callback) or
                      subscriber (subscriber_callback)
            handle_goal_request: Action server goal-accept callback
            handle_cancel_request: Action server cancel callback
            execution_callback: Action server execution callback
            
        Returns:
            Initialized interface or None if not found in pending
            
        Example:
            >>> # Phase 1: Create server without callback
            >>> await InterfaceFactory.create_server("my_service", response_callback=None)
            >>> # Phase 2: Bind callback later
            >>> server = await InterfaceFactory.bind_pending_callback(
            ...     "my_service", callback=my_callback_function
            ... )
            >>>
            >>> # Action server – bind all three callbacks
            >>> action_srv = await InterfaceFactory.bind_pending_callback(
            ...     "my_action",
            ...     handle_goal_request=on_goal,
            ...     handle_cancel_request=on_cancel,
            ...     execution_callback=execute,
            ... )
        """
        if name not in InterfaceFactory._pending_interface:
            logger.warning(f"⚠️  No pending interface found for '{name}'")
            return None
        
        info = InterfaceFactory._pending_interface[name]
        provider_func = info["provider"]
        kwargs = info["kwargs"]
        
        # Update kwargs with the provided callbacks (only override non-None values)
        if "subscriber" in provider_func.__name__:
            if callback is not None:
                kwargs["subscriber_callback"] = callback
        elif "action" in provider_func.__name__:
            if handle_goal_request is not None:
                kwargs["handle_goal_request"] = handle_goal_request
            if handle_cancel_request is not None:
                kwargs["handle_cancel_request"] = handle_cancel_request
            if execution_callback is not None:
                kwargs["execution_callback"] = execution_callback
        elif "server" in provider_func.__name__:
            if callback is not None:
                kwargs["response_callback"] = callback
        else:
            logger.error(f"❌ Unknown provider type for pending interface '{name}'")
            return None
        
        try:
            # Create the interface with bound callback
            interface = await provider_func(**kwargs)
            
            # Add to registry based on type
            if "subscriber_callback" in kwargs:
                add_subscriber_registry(interface)
            elif "response_callback" in kwargs:
                add_server_registry(interface)
            elif "execution_callback" in kwargs:
                add_action_server_registry(interface)
            
            # Remove from pending
            del InterfaceFactory._pending_interface[name]
            
            logger.info(f"✅ Bound callback and initialized pending interface '{name}'")
            return interface
            
        except Exception as e:
            logger.error(f"❌ Failed to initialize pending interface '{name}': {e}")
            return None
    
    @staticmethod
    async def process_pending_interfaces() -> Dict[str, bool]:
        """
        Process all pending interfaces, attempting to initialize those with callbacks.
        
        This should be called periodically (e.g., from entity event loop) or after
        binding callbacks via CallbackRegistry.
        
        Returns:
            Dictionary mapping interface names to initialization success status
            
        Example:
            >>> # In entity event loop
            >>> while running:
            ...     results = await InterfaceFactory.process_pending_interfaces()
            ...     await asyncio.sleep(1.0)
        """
        results = {}
        
        for name in list(InterfaceFactory._pending_interface.keys()):
            info = InterfaceFactory._pending_interface[name]
            kwargs = info["kwargs"]
            
            # Check if all required callbacks are now available.
            # Action servers require all three callbacks; servers/subscribers one each.
            is_action = (
                "handle_goal_request" in kwargs or
                "handle_cancel_request" in kwargs or
                "execution_callback" in kwargs
            )
            if is_action:
                has_callback = (
                    kwargs.get("handle_goal_request") and
                    kwargs.get("handle_cancel_request") and
                    kwargs.get("execution_callback")
                )
            else:
                has_callback = (
                    kwargs.get("subscriber_callback") or
                    kwargs.get("response_callback")
                )
            
            if has_callback:
                # All required callbacks are already stored in kwargs –
                # call bind_pending_callback without overriding anything.
                interface = await InterfaceFactory.bind_pending_callback(name)
                results[name] = interface is not None
        
        if results:
            success_count = sum(results.values())
            logger.info(f"📊 Processed {len(results)} pending interfaces: {success_count} successful")
        
        return results
    
    @staticmethod
    def get_pending_count() -> int:
        """Get count of pending interfaces awaiting callback binding."""
        return len(InterfaceFactory._pending_interface)
    
    @staticmethod
    def list_pending() -> List[str]:
        """Get list of pending interface names."""
        return list(InterfaceFactory._pending_interface.keys())
    
    @staticmethod
    def has_pending(name: str) -> bool:
        """Check if an interface is pending callback binding."""
        return name in InterfaceFactory._pending_interface