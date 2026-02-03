"""
Communication Decorators

Modern decorators for multi-protocol communication.
"""
import asyncio
import functools
import logging
from typing import Callable, Optional, List, Any
from vyra_base.com.core.types import ProtocolType
from vyra_base.com.core.factory import InterfaceFactory

logger = logging.getLogger(__name__)


def remote_callable(
    name: Optional[str] = None,
    protocols: Optional[List[ProtocolType]] = None,
    auto_register: bool = True,
    **kwargs
):
    """
    Decorator for methods that should be accessible as remote callables.
    
    Now supports multiple protocols with automatic fallback.
    
    Args:
        name: Callable name (defaults to function name)
        protocols: Preferred protocols (uses InterfaceFactory.CALLABLE_FALLBACK if None)
        auto_register: Whether to auto-register with entity
        **kwargs: Additional parameters passed to create_callable
        
    Example:
        >>> class MyComponent(OperationalStateMachine):
        ...     @remote_callable
        ...     async def calculate(self, request, response=None):
        ...         result = request["x"] + request["y"]
        ...         return {"result": result}
        ...     
        ...     @remote_callable(protocols=[ProtocolType.ROS2, ProtocolType.SHARED_MEMORY])
        ...     async def process_data(self, request, response=None):
        ...         # This method prioritizes ROS2, falls back to SharedMemory
        ...         return {"status": "processed"}
        
    Note:
        - Methods must accept (request, response=None) signature
        - Response parameter maintained for ROS2 compatibility
        - Async methods recommended (sync methods wrapped automatically)
    """
    def decorator(func: Callable) -> Callable:
        # Store metadata on function
        func._vyra_remote_callable = True
        func._vyra_callable_name = name or func.__name__
        func._vyra_protocols = protocols
        func._vyra_auto_register = auto_register
        func._vyra_kwargs = kwargs
        
        # Wrap async functions directly
        if asyncio.iscoroutinefunction(func):
            @functools.wraps(func)
            async def async_wrapper(*args, **wrapper_kwargs):
                return await func(*args, **wrapper_kwargs)
            
            # Use setattr to dynamically assign attributes to the wrapper function
            setattr(async_wrapper, "_vyra_remote_callable", True)
            setattr(async_wrapper, "_vyra_callable_name", func._vyra_callable_name)
            setattr(async_wrapper, "_vyra_protocols", protocols)
            setattr(async_wrapper, "_vyra_auto_register", auto_register)
            setattr(async_wrapper, "_vyra_kwargs", kwargs)
            
            return async_wrapper
        
        # Wrap sync functions with async wrapper
        else:
            @functools.wraps(func)
            async def async_wrapper(*args, **wrapper_kwargs):
                return func(*args, **wrapper_kwargs)
            
            setattr(async_wrapper, "_vyra_remote_callable", True)
            setattr(async_wrapper, "_vyra_callable_name", func._vyra_callable_name)
            setattr(async_wrapper, "_vyra_protocols", protocols)
            setattr(async_wrapper, "_vyra_auto_register", auto_register)
            setattr(async_wrapper, "_vyra_kwargs", kwargs)
            
            return async_wrapper
    
    return decorator


def remote_speaker(
    name: Optional[str] = None,
    protocols: Optional[List[ProtocolType]] = None,
    auto_register: bool = True,
    **kwargs
):
    """
    Decorator for pub/sub communication.
    
    Args:
        name: Topic/channel name (defaults to function name)
        protocols: Preferred protocols (uses InterfaceFactory.SPEAKER_FALLBACK if None)
        auto_register: Whether to auto-register with entity
        **kwargs: Additional parameters (qos, retain, etc.)
        
    Example:
        >>> class Component:
        ...     @remote_speaker(protocols=[ProtocolType.REDIS])
        ...     async def publish_status(self, message):
        ...         '''This method will become a speaker'''
        ...         pass
        ...     
        ...     async def some_task(self):
        ...         # Publish status update
        ...         await self.publish_status({"state": "running"})
    """
    def decorator(func: Callable) -> Callable:
        func._vyra_remote_speaker = True
        func._vyra_speaker_name = name or func.__name__
        func._vyra_protocols = protocols
        func._vyra_auto_register = auto_register
        func._vyra_kwargs = kwargs
        
        @functools.wraps(func)
        async def wrapper(self_obj, message: Any):
            # Get or create speaker
            speaker_attr = f"_vyra_speaker_{func.__name__}"
            
            if not hasattr(self_obj, speaker_attr):
                # Create speaker on first use
                speaker = await InterfaceFactory.create_speaker(
                    name=func._vyra_speaker_name,
                    protocols=protocols,
                    **kwargs
                )
                setattr(self_obj, speaker_attr, speaker)
            
            speaker = getattr(self_obj, speaker_attr)
            return await speaker.shout(message)
        
        setattr(wrapper, "_vyra_remote_speaker", True)
        setattr(wrapper, "_vyra_speaker_name", func._vyra_speaker_name)
        setattr(wrapper, "_vyra_protocols", protocols)
        setattr(wrapper, "_vyra_auto_register", auto_register)
        setattr(wrapper, "_vyra_kwargs", kwargs)
        
        return wrapper
    
    return decorator


def remote_job(
    name: Optional[str] = None,
    protocols: Optional[List[ProtocolType]] = None,
    auto_register: bool = True,
    **kwargs
):
    """
    Decorator for job/task-based communication.
    
    Args:
        name: Job name (defaults to function name)
        protocols: Preferred protocols (uses InterfaceFactory.JOB_FALLBACK if None)
        auto_register: Whether to auto-register with entity
        **kwargs: Additional parameters
        
    Example:
        >>> class Component:
        ...     @remote_job(protocols=[ProtocolType.REDIS])
        ...     async def process_batch(self, job_data):
        ...         # Long-running job
        ...         for item in job_data["items"]:
        ...             process(item)
        ...         return {"processed": len(job_data["items"])}
    """
    def decorator(func: Callable) -> Callable:
        func._vyra_remote_job = True
        func._vyra_job_name = name or func.__name__
        func._vyra_protocols = protocols
        func._vyra_auto_register = auto_register
        func._vyra_kwargs = kwargs
        
        if asyncio.iscoroutinefunction(func):
            @functools.wraps(func)
            async def async_wrapper(*args, **wrapper_kwargs):
                return await func(*args, **wrapper_kwargs)
            
            setattr(async_wrapper, "_vyra_remote_job", True)
            setattr(async_wrapper, "_vyra_job_name", func._vyra_job_name)
            setattr(async_wrapper, "_vyra_protocols", protocols)
            setattr(async_wrapper, "_vyra_auto_register", auto_register)
            setattr(async_wrapper, "_vyra_kwargs", kwargs)
            
            return async_wrapper
        else:
            @functools.wraps(func)
            async def async_wrapper(*args, **wrapper_kwargs):
                return func(*args, **wrapper_kwargs)
            
            setattr(async_wrapper, "_vyra_remote_job", True)
            setattr(async_wrapper, "_vyra_job_name", func._vyra_job_name)
            setattr(async_wrapper, "_vyra_protocols", protocols)
            setattr(async_wrapper, "_vyra_auto_register", auto_register)
            setattr(async_wrapper, "_vyra_kwargs", kwargs)
            
            return async_wrapper
    
    return decorator


def get_decorated_methods(obj: Any) -> dict:
    """
    Get all methods decorated with @remote_callable, @remote_speaker, @remote_job.
    
    Args:
        obj: Object to inspect
        
    Returns:
        dict: {"callables": [...], "speakers": [...], "jobs": [...]}
        
    Example:
        >>> component = MyComponent()
        >>> methods = get_decorated_methods(component)
        >>> print(f"Found {len(methods['callables'])} callables")
    """
    result = {
        "callables": [],
        "speakers": [],
        "jobs": [],
    }
    
    for attr_name in dir(obj):
        try:
            attr = getattr(obj, attr_name)
            
            if hasattr(attr, "_vyra_remote_callable"):
                result["callables"].append({
                    "name": getattr(attr, "_vyra_callable_name"),
                    "method": attr,
                    "protocols": attr._vyra_protocols,
                    "kwargs": attr._vyra_kwargs,
                })
            
            if hasattr(attr, "_vyra_remote_speaker"):
                result["speakers"].append({
                    "name": attr._vyra_speaker_name,
                    "method": attr,
                    "protocols": attr._vyra_protocols,
                    "kwargs": attr._vyra_kwargs,
                })
            
            if hasattr(attr, "_vyra_remote_job"):
                result["jobs"].append({
                    "name": attr._vyra_job_name,
                    "method": attr,
                    "protocols": attr._vyra_protocols,
                    "kwargs": attr._vyra_kwargs,
                })
        
        except Exception:
            continue
    
    return result


# Backward compatibility alias
remote_callable_ros2 = remote_callable  # Old decorator name
