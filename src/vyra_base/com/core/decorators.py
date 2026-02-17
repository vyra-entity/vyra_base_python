"""
Communication Decorators

Modern decorators for multi-protocol communication with late-binding support.

The decorator system uses a two-phase initialization pattern:
1. Phase 1 (Decoration): Create HandlerBlueprint, register in CallbackRegistry
2. Phase 2 (Binding): Bind decorated method to blueprint during component initialization

This decouples interface definition from implementation, enabling:
- Dynamic interface registration
- Late binding of callbacks
- Better testing (mock callbacks easily)
- Clear separation of concerns

Example:
    >>> class MyComponent:
    ...     @remote_service
    ...     async def calculate(self, request, response=None):
    ...         return {"result": request["x"] + request["y"]}
    
    The decorator creates a ServiceBlueprint, but binding happens when
    the component is registered with an entity.
"""
import asyncio
import functools
import logging
from typing import Callable, Optional, List, Any, Dict

from vyra_base.com.core.types import ProtocolType
from vyra_base.com.core.blueprints import (
    ServiceBlueprint,
    PublisherBlueprint,
    ActionBlueprint,
    SubscriberBlueprint,
    AnyBlueprint
)
from vyra_base.com.core.callback_registry import CallbackRegistry

logger = logging.getLogger(__name__)


def remote_service(
    name: Optional[str] = None,
    protocols: Optional[List[ProtocolType]] = None,
    auto_register: bool = True,
    namespace: Optional[str] = None,
    **kwargs
):
    """
    Decorator for request/response communication (ROS2 Service, gRPC, etc.)
    
    Creates a ServiceBlueprint and registers it in CallbackRegistry.
    The decorated method will be bound to the blueprint during component initialization.
    
    Args:
        name: Service name (defaults to function name)
        protocols: Preferred protocols (will use factory fallback if None)
        auto_register: Whether to auto-register blueprint (default True)
        namespace: Optional module namespace for blueprint registration
        **kwargs: Additional metadata (service_type, qos, etc.)
        
    Example:
        >>> class MyComponent(OperationalStateMachine):
        ...     @remote_service
        ...     async def calculate(self, request, response=None):
        ...         result = request["x"] + request["y"]
        ...         return {"result": result}
        ...     
        ...     @remote_service(protocols=[ProtocolType.ROS2, ProtocolType.ZENOH])
        ...     async def process_data(self, request, response=None):
        ...         # This method prioritizes ROS2, falls back to Zenoh
        ...         return {"status": "processed"}
        
    Note:
        - Methods must accept (request, response=None) signature
        - Response parameter maintained for ROS2 compatibility
        - Async methods recommended (sync methods wrapped automatically)
        - Blueprint created during decoration, callback bound during initialization
    """
    def decorator(func: Callable) -> Callable:
        # Create blueprint (without callback yet)
        blueprint_name = name or func.__name__
        blueprint = ServiceBlueprint(
            name=blueprint_name,
            protocols=protocols or [],
            metadata=kwargs,
            service_type=kwargs.get('service_type')
        )
        
        # Register blueprint if auto_register is True
        if auto_register:
            try:
                CallbackRegistry.register_blueprint(blueprint, namespace=namespace)
                logger.debug(f"📝 Registered service blueprint '{blueprint_name}'")
            except ValueError as e:
                logger.warning(f"⚠️  Blueprint '{blueprint_name}' already registered: {e}")
        
        # Store metadata on function for backward compatibility and discovery
        func._vyra_remote_server = True
        func._vyra_server_name = blueprint_name
        func._vyra_protocols = protocols
        func._vyra_auto_register = auto_register
        func._vyra_kwargs = kwargs
        func._vyra_blueprint = blueprint  # NEW: Reference to blueprint
        func._vyra_namespace = namespace
        
        # Wrap async functions
        if asyncio.iscoroutinefunction(func):
            @functools.wraps(func)
            async def async_wrapper(*args, **wrapper_kwargs):
                return await func(*args, **wrapper_kwargs)
            
            wrapper = async_wrapper
        else:
            # Wrap sync functions with async wrapper
            @functools.wraps(func)
            async def async_wrapper(*args, **wrapper_kwargs):
                return func(*args, **wrapper_kwargs)
            
            wrapper = async_wrapper
        
        # Transfer metadata to wrapper
        setattr(wrapper, "_vyra_remote_server", True)
        setattr(wrapper, "_vyra_server_name", blueprint_name)
        setattr(wrapper, "_vyra_protocols", protocols)
        setattr(wrapper, "_vyra_auto_register", auto_register)
        setattr(wrapper, "_vyra_kwargs", kwargs)
        setattr(wrapper, "_vyra_blueprint", blueprint)
        setattr(wrapper, "_vyra_namespace", namespace)
        
        return wrapper
    
    return decorator


def remote_publisher(
    name: Optional[str] = None,
    protocols: Optional[List[ProtocolType]] = None,
    auto_register: bool = True,
    namespace: Optional[str] = None,
    **kwargs
):
    """
    Decorator for pub/sub communication (publisher side).
    
    Creates a PublisherBlueprint and registers it in CallbackRegistry.
    The decorated method becomes a publisher interface.
    
    Args:
        name: Topic/channel name (defaults to function name)
        protocols: Preferred protocols (will use factory fallback if None)
        auto_register: Whether to auto-register blueprint
        namespace: Optional module namespace
        **kwargs: Additional metadata (message_type, qos, retain, etc.)
        
    Example:
        >>> class Component:
        ...     @remote_publisher(protocols=[ProtocolType.REDIS])
        ...     async def publish_status(self, message):
        ...         '''This method will become a publisher'''
        ...         pass
        ...     
        ...     async def some_task(self):
        ...         # Publish status update
        ...         await self.publish_status({"state": "running"})
        
    Note:
        - Publisher creation happens via InterfaceFactory during binding
        - The decorated method is replaced with actual publishing logic
    """
    def decorator(func: Callable) -> Callable:
        # Create blueprint
        blueprint_name = name or func.__name__
        blueprint = PublisherBlueprint(
            name=blueprint_name,
            protocols=protocols or [],
            metadata=kwargs,
            message_type=kwargs.get('message_type')
        )
        
        # Register blueprint if auto_register is True
        if auto_register:
            try:
                CallbackRegistry.register_blueprint(blueprint, namespace=namespace)
                logger.debug(f"📝 Registered publisher blueprint '{blueprint_name}'")
            except ValueError as e:
                logger.warning(f"⚠️  Blueprint '{blueprint_name}' already registered: {e}")
        
        # Store metadata on function
        func._vyra_remote_publisher = True
        func._vyra_publisher_name = blueprint_name
        func._vyra_protocols = protocols
        func._vyra_auto_register = auto_register
        func._vyra_kwargs = kwargs
        func._vyra_blueprint = blueprint
        func._vyra_namespace = namespace
        
        # Create wrapper that will publish when called
        @functools.wraps(func)
        async def wrapper(self_obj, message: Any):
            # Get or create publisher on first use
            publisher_attr = f"_vyra_publisher_{func.__name__}"
            
            if not hasattr(self_obj, publisher_attr):
                logger.warning(
                    f"⚠️  Publisher '{blueprint_name}' not initialized. "
                    f"This should be created via InterfaceFactory during binding."
                )
                # Lazy creation as fallback (not ideal, but prevents crashes)
                from vyra_base.com.core.factory import InterfaceFactory
                publisher = await InterfaceFactory.create_publisher(
                    name=blueprint_name,
                    protocols=protocols or [],
                    **kwargs
                )
                setattr(self_obj, publisher_attr, publisher)
            
            publisher = getattr(self_obj, publisher_attr)
            return await publisher.shout(message)
        
        # Transfer metadata to wrapper
        setattr(wrapper, "_vyra_remote_publisher", True)
        setattr(wrapper, "_vyra_publisher_name", blueprint_name)
        setattr(wrapper, "_vyra_protocols", protocols)
        setattr(wrapper, "_vyra_auto_register", auto_register)
        setattr(wrapper, "_vyra_kwargs", kwargs)
        setattr(wrapper, "_vyra_blueprint", blueprint)
        setattr(wrapper, "_vyra_namespace", namespace)
        
        return wrapper
    
    return decorator


def remote_actionServer(
    name: Optional[str] = None,
    protocols: Optional[List[ProtocolType]] = None,
    auto_register: bool = True,
    namespace: Optional[str] = None,
    **kwargs
):
    """
    Decorator for long-running task communication (ROS2 Action, job queues, etc.)
    
    Creates an ActionBlueprint and registers it in CallbackRegistry.
    The decorated method will handle action goals with feedback and cancellation support.
    
    Args:
        name: Action name (defaults to function name)
        protocols: Preferred protocols (will use factory fallback if None)
        auto_register: Whether to auto-register blueprint
        namespace: Optional module namespace
        **kwargs: Additional metadata (action_type, timeout, etc.)
        
    Example:
        >>> class Component:
        ...     @remote_actionServer(protocols=[ProtocolType.ROS2])
        ...     async def process_batch(self, goal_handle):
        ...         # Long-running job with feedback
        ...         for i, item in enumerate(goal_handle.goal.items):
        ...             if goal_handle.is_cancel_requested():
        ...                 return {"cancelled": True}
        ...             process(item)
        ...             goal_handle.publish_feedback({"progress": i+1})
        ...         return {"processed": len(goal_handle.goal.items)}
        
    Note:
        - The goal_handle provides: goal, publish_feedback(), is_cancel_requested()
        - Async methods recommended for long-running operations
    """
    def decorator(func: Callable) -> Callable:
        # Create blueprint
        blueprint_name = name or func.__name__
        blueprint = ActionBlueprint(
            name=blueprint_name,
            protocols=protocols or [],
            metadata=kwargs,
            action_type=kwargs.get('action_type')
        )
        
        # Register blueprint if auto_register is True
        if auto_register:
            try:
                CallbackRegistry.register_blueprint(blueprint, namespace=namespace)
                logger.debug(f"📝 Registered action blueprint '{blueprint_name}'")
            except ValueError as e:
                logger.warning(f"⚠️  Blueprint '{blueprint_name}' already registered: {e}")
        
        # Store metadata on function
        func._vyra_remote_action = True
        func._vyra_action_name = blueprint_name
        func._vyra_protocols = protocols
        func._vyra_auto_register = auto_register
        func._vyra_kwargs = kwargs
        func._vyra_blueprint = blueprint
        func._vyra_namespace = namespace
        
        # Wrap async functions
        if asyncio.iscoroutinefunction(func):
            @functools.wraps(func)
            async def async_wrapper(*args, **wrapper_kwargs):
                return await func(*args, **wrapper_kwargs)
            
            wrapper = async_wrapper
        else:
            # Wrap sync functions with async wrapper
            @functools.wraps(func)
            async def async_wrapper(*args, **wrapper_kwargs):
                return func(*args, **wrapper_kwargs)
            
            wrapper = async_wrapper
        
        # Transfer metadata to wrapper
        setattr(wrapper, "_vyra_remote_action", True)
        setattr(wrapper, "_vyra_action_name", blueprint_name)
        setattr(wrapper, "_vyra_protocols", protocols)
        setattr(wrapper, "_vyra_auto_register", auto_register)
        setattr(wrapper, "_vyra_kwargs", kwargs)
        setattr(wrapper, "_vyra_blueprint", blueprint)
        setattr(wrapper, "_vyra_namespace", namespace)
        
        return wrapper
    
    return decorator


def remote_subscriber(
    name: Optional[str] = None,
    protocols: Optional[List[ProtocolType]] = None,
    auto_register: bool = True,
    namespace: Optional[str] = None,
    **kwargs
):
    """
    Decorator for subscription callbacks (subscriber side of pub/sub).
    
    Creates a SubscriberBlueprint and registers it in CallbackRegistry.
    The decorated method will be called when messages are received on the topic.
    
    Args:
        name: Topic/channel name (defaults to function name)
        protocols: Preferred protocols (will use factory fallback if None)
        auto_register: Whether to auto-register blueprint
        namespace: Optional module namespace
        **kwargs: Additional metadata (message_type, qos, etc.)
        
    Example:
        >>> class Component:
        ...     @remote_subscriber(topic="/status")
        ...     async def on_status_update(self, message):
        ...         logger.info(f"Received status: {message}")
        
    Note:
        - Subscriber is automatically created and bound during registration
        - The decorated method is called for each received message
    """
    def decorator(func: Callable) -> Callable:
        # Create blueprint
        blueprint_name = name or func.__name__
        blueprint = SubscriberBlueprint(
            name=blueprint_name,
            protocols=protocols or [],
            metadata=kwargs,
            message_type=kwargs.get('message_type')
        )
        
        # Register blueprint if auto_register is True
        if auto_register:
            try:
                CallbackRegistry.register_blueprint(blueprint, namespace=namespace)
                logger.debug(f"📝 Registered subscriber blueprint '{blueprint_name}'")
            except ValueError as e:
                logger.warning(f"⚠️  Blueprint '{blueprint_name}' already registered: {e}")
        
        # Store metadata on function
        func._vyra_remote_subscriber = True
        func._vyra_subscriber_name = blueprint_name
        func._vyra_protocols = protocols
        func._vyra_auto_register = auto_register
        func._vyra_kwargs = kwargs
        func._vyra_blueprint = blueprint
        func._vyra_namespace = namespace
        
        # Wrap async functions
        if asyncio.iscoroutinefunction(func):
            @functools.wraps(func)
            async def async_wrapper(*args, **wrapper_kwargs):
                return await func(*args, **wrapper_kwargs)
            
            wrapper = async_wrapper
        else:
            # Wrap sync functions with async wrapper
            @functools.wraps(func)
            async def async_wrapper(*args, **wrapper_kwargs):
                return func(*args, **wrapper_kwargs)
            
            wrapper = async_wrapper
        
        # Transfer metadata to wrapper
        setattr(wrapper, "_vyra_remote_subscriber", True)
        setattr(wrapper, "_vyra_subscriber_name", blueprint_name)
        setattr(wrapper, "_vyra_protocols", protocols)
        setattr(wrapper, "_vyra_auto_register", auto_register)
        setattr(wrapper, "_vyra_kwargs", kwargs)
        setattr(wrapper, "_vyra_blueprint", blueprint)
        setattr(wrapper, "_vyra_namespace", namespace)
        
        return wrapper
    
    return decorator


def get_decorated_methods(obj: Any) -> dict:
    """
    Get all methods decorated with communication decorators.
    
    Scans an object for decorated methods and returns them organized by type.
    Also retrieves associated blueprints from CallbackRegistry.
    
    Args:
        obj: Object to inspect (typically a component instance)
        
    Returns:
        dict: {
            "servers": [...],       # @remote_service methods
            "publishers": [...],    # @remote_publisher methods
            "subscribers": [...],   # @remote_subscriber methods
            "actions": [...]        # @remote_actionServer methods
        }
        
        Each entry contains: {
            "name": str,             # Interface name
            "method": Callable,      # Bound method
            "protocols": List,       # Protocol preferences
            "kwargs": dict,          # Additional metadata
            "blueprint": Blueprint   # Associated blueprint (if registered)
        }
        
    Example:
        >>> component = MyComponent()
        >>> methods = get_decorated_methods(component)
        >>> print(f"Found {len(methods['callables'])} services")
        >>> for svc in methods['callables']:
        ...     if svc['blueprint'] and not svc['blueprint'].is_bound():
        ...         svc['blueprint'].bind_callback(svc['method'])
    """
    result = {
        "servers": [],
        "publishers": [],
        "subscribers": [],
        "actions": [],
    }
    
    for attr_name in dir(obj):
        try:
            attr = getattr(obj, attr_name)
            
            # Service (server)
            if hasattr(attr, "_vyra_remote_server"):
                blueprint = getattr(attr, "_vyra_blueprint", None)
                result["servers"].append({
                    "name": getattr(attr, "_vyra_server_name"),
                    "method": attr,
                    "protocols": attr._vyra_protocols,
                    "kwargs": attr._vyra_kwargs,
                    "blueprint": blueprint,
                    "namespace": getattr(attr, "_vyra_namespace", None),
                })
            
            # Publisher
            if hasattr(attr, "_vyra_remote_publisher"):
                blueprint = getattr(attr, "_vyra_blueprint", None)
                result["publishers"].append({
                    "name": attr._vyra_publisher_name,
                    "method": attr,
                    "protocols": attr._vyra_protocols,
                    "kwargs": attr._vyra_kwargs,
                    "blueprint": blueprint,
                    "namespace": getattr(attr, "_vyra_namespace", None),
                })
            
            # Subscriber
            if hasattr(attr, "_vyra_remote_subscriber"):
                blueprint = getattr(attr, "_vyra_blueprint", None)
                result["subscribers"].append({
                    "name": attr._vyra_subscriber_name,
                    "method": attr,
                    "protocols": attr._vyra_protocols,
                    "kwargs": attr._vyra_kwargs,
                    "blueprint": blueprint,
                    "namespace": getattr(attr, "_vyra_namespace", None),
                })
            
            # Action
            if hasattr(attr, "_vyra_remote_action"):
                blueprint = getattr(attr, "_vyra_blueprint", None)
                result["actions"].append({
                    "name": attr._vyra_action_name,
                    "method": attr,
                    "protocols": attr._vyra_protocols,
                    "kwargs": attr._vyra_kwargs,
                    "blueprint": blueprint,
                    "namespace": getattr(attr, "_vyra_namespace", None),
                })
        
        except Exception as e:
            logger.debug(f"Skipping attribute '{attr_name}': {e}")
            continue
    
    return result


def bind_decorated_callbacks(
    component: Any,
    namespace: Optional[str] = None,
    force_rebind: bool = False
) -> Dict[str, bool]:
    """
    Bind all decorated methods from a component to their registered blueprints.
    
    This is the Phase 2 of the two-phase initialization, typically called during
    component initialization (e.g., in Component.__init__ or set_interfaces()).
    
    Args:
        component: Component instance with decorated methods
        namespace: Optional namespace for blueprint lookup
        force_rebind: If True, unbind existing callbacks before binding
        
    Returns:
        Dictionary mapping interface names to binding success status
        
    Example:
        >>> component = MyComponent()
        >>> results = bind_decorated_callbacks(component, namespace="v2_modulemanager")
        >>> print(f"Successfully bound {sum(results.values())} interfaces")
        >>> failed = [name for name, success in results.items() if not success]
        >>> if failed:
        ...     logger.warning(f"Failed to bind: {failed}")
    """
    decorated = get_decorated_methods(component)
    results = {}
    
    # Bind services
    for item in decorated["servers"]:
        blueprint = item.get("blueprint")
        if blueprint:
            try:
                if force_rebind:
                    blueprint.unbind_callback()
                blueprint.bind_callback(item["method"])
                results[item["name"]] = True
                logger.debug(f"✅ Bound service callback: {item['name']}")
            except Exception as e:
                results[item["name"]] = False
                logger.error(f"❌ Failed to bind service {item['name']}: {e}")
        else:
            # Try to find blueprint in registry
            bp = CallbackRegistry.get_blueprint(item["name"], namespace=namespace)
            if bp:
                try:
                    if force_rebind:
                        bp.unbind_callback()
                    bp.bind_callback(item["method"])
                    results[item["name"]] = True
                    logger.debug(f"✅ Bound service callback from registry: {item['name']}")
                except Exception as e:
                    results[item["name"]] = False
                    logger.error(f"❌ Failed to bind service {item['name']}: {e}")
            else:
                results[item["name"]] = False
                logger.warning(f"⚠️  No blueprint found for service: {item['name']}")
    
    # Bind publishers
    for item in decorated["publishers"]:
        blueprint = item.get("blueprint")
        if blueprint:
            try:
                if force_rebind:
                    blueprint.unbind_callback()
                # Publishers use the wrapper function, not the original method
                blueprint.bind_callback(item["method"])
                results[item["name"]] = True
                logger.debug(f"✅ Bound publisher callback: {item['name']}")
            except Exception as e:
                results[item["name"]] = False
                logger.error(f"❌ Failed to bind publisher {item['name']}: {e}")
    
    # Bind subscribers
    for item in decorated["subscribers"]:
        blueprint = item.get("blueprint")
        if blueprint:
            try:
                if force_rebind:
                    blueprint.unbind_callback()
                blueprint.bind_callback(item["method"])
                results[item["name"]] = True
                logger.debug(f"✅ Bound subscriber callback: {item['name']}")
            except Exception as e:
                results[item["name"]] = False
                logger.error(f"❌ Failed to bind subscriber {item['name']}: {e}")
    
    # Bind actions
    for item in decorated["actions"]:
        blueprint = item.get("blueprint")
        if blueprint:
            try:
                if force_rebind:
                    blueprint.unbind_callback()
                blueprint.bind_callback(item["method"])
                results[item["name"]] = True
                logger.debug(f"✅ Bound action callback: {item['name']}")
            except Exception as e:
                results[item["name"]] = False
                logger.error(f"❌ Failed to bind action {item['name']}: {e}")
    
    # Summary
    total = len(results)
    success = sum(results.values())
    logger.info(f"📊 Callback binding complete: {success}/{total} successful")
    
    return results


# Backward compatibility aliases (OLD NAMES - DEPRECATED)
# These maintain compatibility with existing code but should be migrated
remote_callable = remote_service        # DEPRECATED: Use remote_service
remote_callable_ros2 = remote_service  # DEPRECATED: Use remote_service
remote_speaker = remote_publisher      # DEPRECATED: Use remote_publisher
remote_listener = remote_subscriber    # DEPRECATED: Use remote_subscriber
remote_job = remote_actionServer       # DEPRECATED: Use remote_actionServer
