"""
Communication Handler Blueprints

Blueprints represent interface definitions that can be registered before their
implementations are available. This enables two-phase initialization:
1. Blueprint registration (during entity creation, from JSON metadata)
2. Callback binding (during component initialization, from decorated methods)

This pattern decouples interface configuration from implementation, enabling:
- Late binding of callbacks
- Dynamic interface registration
- Better testing (mock callbacks easily)
- Clear separation of concerns
"""

from __future__ import annotations

import asyncio
import inspect
import logging
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Dict, List, Optional, Union

from vyra_base.com.core.types import ProtocolType

logger = logging.getLogger(__name__)


class InterfaceType(Enum):
    """Types of communication interfaces"""
    SERVICE = "service"
    PUBLISHER = "publisher"
    SUBSCRIBER = "subscriber"
    ACTION = "action"


@dataclass
class HandlerBlueprint(ABC):
    """
    Base blueprint for all communication interfaces.
    
    A blueprint represents the *definition* of an interface (what it should be),
    separate from its *implementation* (the callback that handles requests).
    
    Attributes:
        name: Unique identifier for this interface
        interface_type: Type of communication pattern
        protocols: Preferred transport protocols (with fallback)
        metadata: Additional configuration (from JSON or decorator)
        _callback: Implementation function (bound during phase 2)
        
    Example:
        >>> blueprint = ServiceBlueprint(
        ...     name="calculate",
        ...     protocols=[ProtocolType.ROS2],
        ...     metadata={"qos": 10}
        ... )
        >>> blueprint.is_bound()
        False
        >>> blueprint.bind_callback(my_calculate_function)
        >>> blueprint.is_bound()
        True
    """
    
    name: str
    interface_type: InterfaceType
    protocols: List[ProtocolType] = field(default_factory=list)
    metadata: Dict[str, Any] = field(default_factory=dict)
    _callback: Optional[Callable] = field(default=None, repr=False)
    
    @property
    def callback(self) -> Optional[Callable]:
        """Get the bound callback (may be None if not yet bound)"""
        return self._callback
    
    def is_bound(self) -> bool:
        """Check if a callback has been bound to this blueprint"""
        return self._callback is not None
    
    def bind_callback(self, callback: Callable) -> None:
        """
        Bind a callback implementation to this blueprint.
        
        Args:
            callback: Function to handle requests/events
            
        Raises:
            ValueError: If callback signature is invalid
            RuntimeError: If already bound
        """
        if self.is_bound():
            raise RuntimeError(
                f"Blueprint '{self.name}' already has a bound callback. "
                f"Unbind first or create a new blueprint."
            )
        
        # Validate signature
        self._validate_callback(callback)
        
        self._callback = callback
        logger.debug(f"✅ Bound callback to blueprint '{self.name}'")
    
    def unbind_callback(self) -> Optional[Callable]:
        """
        Remove the current callback binding.
        
        Returns:
            The previously bound callback, or None
        """
        old_callback = self._callback
        self._callback = None
        if old_callback:
            logger.debug(f"🔓 Unbound callback from blueprint '{self.name}'")
        return old_callback
    
    @abstractmethod
    def _validate_callback(self, callback: Callable) -> None:
        """
        Validate that callback has correct signature for this blueprint type.
        
        Raises:
            ValueError: If signature is invalid
        """
        pass
    
    def get_metadata(self, key: str, default: Any = None) -> Any:
        """Get metadata value by key"""
        return self.metadata.get(key, default)
    
    def update_metadata(self, **kwargs) -> None:
        """Update metadata fields"""
        self.metadata.update(kwargs)
    
    def __repr__(self) -> str:
        bound_status = "✓" if self.is_bound() else "✗"
        protocols_str = ",".join(p.value for p in self.protocols[:2])
        if len(self.protocols) > 2:
            protocols_str += ",..."
        return (
            f"{self.__class__.__name__}(name='{self.name}', "
            f"bound={bound_status}, protocols=[{protocols_str}])"
        )


class ServiceBlueprint(HandlerBlueprint):
    """
    Blueprint for request/response communication (ROS2 Service, gRPC, etc.)
    
    Expected callback signature:
        async def handler(request, response=None) -> Union[dict, response_type]
        or
        def handler(request, response=None) -> Union[dict, response_type]
    
    Example:
        >>> @remote_service
        ... async def calculate(self, request, response=None):
        ...     result = request["x"] + request["y"]
        ...     return {"result": result}
    """
    
    def __init__(
        self,
        name: str,
        protocols: Optional[List[ProtocolType]] = None,
        metadata: Optional[Dict[str, Any]] = None,
        service_type: Optional[Any] = None
    ):
        super().__init__(
            name=name,
            interface_type=InterfaceType.SERVICE,
            protocols=protocols or [],
            metadata=metadata or {}
        )
        self.service_type = service_type
    
    def _validate_callback(self, callback: Callable) -> None:
        """Validate service callback signature"""
        sig = inspect.signature(callback)
        params = list(sig.parameters.keys())
        
        # Remove 'self' if present (for bound methods)
        if params and params[0] == 'self':
            params = params[1:]
        
        # Must accept at least 1 parameter (request)
        # Second parameter (response) is optional for compatibility
        if len(params) < 1:
            raise ValueError(
                f"Service callback '{callback.__name__}' must accept "
                f"at least 1 parameter (request). Got: {params}"
            )
        
        # Check first param is named 'request' (convention)
        if params[0] not in ['request', 'req', 'data']:
            logger.warning(
                f"Service callback '{callback.__name__}' first parameter "
                f"should be named 'request', got '{params[0]}'"
            )


class PublisherBlueprint(HandlerBlueprint):
    """
    Blueprint for pub/sub communication (ROS2 Topic, Redis, MQTT, etc.)
    
    Note: Publishers don't typically have callbacks - they're used to *send* data.
    The callback here is for the publish() method wrapper.
    
    Expected callback signature:
        async def publisher(self, message: dict) -> None
        or
        def publisher(self, message: dict) -> None
    
    Example:
        >>> @remote_publisher
        ... async def publish_status(self, message):
        ...     # Actual publishing handled by transport
        ...     pass
    """
    
    def __init__(
        self,
        name: str,
        protocols: Optional[List[ProtocolType]] = None,
        metadata: Optional[Dict[str, Any]] = None,
        message_type: Optional[Any] = None
    ):
        super().__init__(
            name=name,
            interface_type=InterfaceType.PUBLISHER,
            protocols=protocols or [],
            metadata=metadata or {}
        )
        self.message_type = message_type
    
    def _validate_callback(self, callback: Callable) -> None:
        """Validate publisher callback signature"""
        sig = inspect.signature(callback)
        params = list(sig.parameters.keys())
        
        # Remove 'self' if present
        if params and params[0] == 'self':
            params = params[1:]
        
        # Must accept at least 1 parameter (message)
        if len(params) < 1:
            raise ValueError(
                f"Publisher callback '{callback.__name__}' must accept "
                f"at least 1 parameter (message). Got: {params}"
            )


class SubscriberBlueprint(HandlerBlueprint):
    """
    Blueprint for subscription callbacks (receives published data)
    
    Expected callback signature:
        async def handler(self, message) -> None
        or
        def handler(self, message) -> None
    
    Example:
        >>> @remote_subscriber
        ... async def on_status_update(self, message):
        ...     logger.info(f"Received: {message}")
    """
    
    def __init__(
        self,
        name: str,
        protocols: Optional[List[ProtocolType]] = None,
        metadata: Optional[Dict[str, Any]] = None,
        message_type: Optional[Any] = None
    ):
        super().__init__(
            name=name,
            interface_type=InterfaceType.SUBSCRIBER,
            protocols=protocols or [],
            metadata=metadata or {}
        )
        self.message_type = message_type
    
    def _validate_callback(self, callback: Callable) -> None:
        """Validate subscriber callback signature"""
        sig = inspect.signature(callback)
        params = list(sig.parameters.keys())
        
        # Remove 'self' if present
        if params and params[0] == 'self':
            params = params[1:]
        
        # Must accept at least 1 parameter (message)
        if len(params) < 1:
            raise ValueError(
                f"Subscriber callback '{callback.__name__}' must accept "
                f"at least 1 parameter (message). Got: {params}"
            )


class ActionBlueprint(HandlerBlueprint):
    """
    Blueprint for ActionServer with multiple lifecycle callbacks.
    
    REQUIRED callbacks:
    - execute: Main execution (required)
    - on_goal: Accept/reject goal (optional, default: accept)
    - on_cancel: Accept/reject cancel (optional, default: accept)
    
    The goal_handle provides:
        - goal_handle.goal: The goal request data
        - goal_handle.publish_feedback(feedback): Send progress updates
        - goal_handle.is_cancel_requested(): Check for cancellation
        - goal_handle.succeed()/abort()/canceled(): Set final status
    
    Example:
        >>> @remote_actionServer.on_goal(name="process_batch")
        ... async def accept_goal(self, goal_request):
        ...     return goal_request.count <= 100
        ...
        >>> @remote_actionServer.on_cancel(name="process_batch")
        ... async def cancel_batch(self, goal_handle):
        ...     return True
        ...
        >>> @remote_actionServer.execute(name="process_batch")
        ... async def execute_batch(self, goal_handle):
        ...     for i in range(goal_handle.goal.count):
        ...         if goal_handle.is_cancel_requested():
        ...             goal_handle.canceled()
        ...             return {"processed": i}
        ...         process_item(i)
        ...         goal_handle.publish_feedback({"progress": i+1})
        ...     goal_handle.succeed()
        ...     return {"processed": goal_handle.goal.count}
    """
    
    def __init__(
        self,
        name: str,
        protocols: Optional[List[ProtocolType]] = None,
        metadata: Optional[Dict[str, Any]] = None,
        action_type: Optional[Any] = None
    ):
        # Don't call super().__init__ with _callback, we handle it differently
        self.name = name
        self.interface_type = InterfaceType.ACTION  # Local InterfaceType.ACTION
        self.protocols = protocols or []
        self.metadata = metadata or {}
        self.action_type = action_type
        
        # Multi-callback storage
        self._callbacks: Dict[str, Optional[Callable]] = {
            'on_goal': None,
            'on_cancel': None,
            'execute': None
        }
        
        # Legacy single callback property (for backward compatibility)
        self._callback: Optional[Callable] = None
    
    @property
    def callback(self) -> Optional[Callable]:
        """Returns 'execute' callback for backward compatibility."""
        return self._callbacks.get('execute')
    
    def bind_callback(
        self, 
        callback: Callable, 
        callback_type: str = 'execute'
    ) -> None:
        """
        Bind a specific callback type.
        
        Args:
            callback: Function to bind
            callback_type: One of 'on_goal', 'on_cancel', 'execute'
        
        Raises:
            ValueError: If callback_type is invalid
            RuntimeError: If callback already bound for this type
        """
        if callback_type not in self._callbacks:
            raise ValueError(
                f"Invalid callback_type '{callback_type}'. "
                f"Must be one of: {list(self._callbacks.keys())}"
            )
        
        if self._callbacks[callback_type] is not None:
            raise RuntimeError(
                f"Callback '{callback_type}' already bound for '{self.name}'"
            )
        
        self._validate_callback(callback, callback_type)
        self._callbacks[callback_type] = callback
        logger.debug(f"✅ Bound '{callback_type}' to ActionBlueprint '{self.name}'")
    
    def bind_callbacks(self, **callbacks) -> None:
        """
        Bind multiple callbacks at once.
        
        Example:
            blueprint.bind_callbacks(
                on_goal=handle_goal,
                on_cancel=handle_cancel,
                execute=execute_task
            )
        """
        for callback_type, callback in callbacks.items():
            if callback is not None:
                self.bind_callback(callback, callback_type)
    
    def get_callback(self, callback_type: str) -> Optional[Callable]:
        """Get specific callback by type."""
        return self._callbacks.get(callback_type)
    
    def is_bound(self, callback_type: Optional[str] = None) -> bool:
        """
        Check if callbacks are bound.
        
        Args:
            callback_type: Check specific callback, or None for "required bound"
        
        Returns:
            bool: If callback_type given, checks that one.
                  If None, checks if 'execute' (required) is bound.
        """
        if callback_type:
            return self._callbacks.get(callback_type) is not None
        
        # 'execute' is required
        return self._callbacks['execute'] is not None
    
    def is_fully_bound(self) -> bool:
        """Check if ALL callbacks are bound."""
        return all(cb is not None for cb in self._callbacks.values())
    
    def unbind_callback(self, callback_type: str = 'execute') -> Optional[Callable]:
        """Remove specific callback binding."""
        old_callback = self._callbacks.get(callback_type)
        if callback_type in self._callbacks:
            self._callbacks[callback_type] = None
        return old_callback
    
    def _validate_callback(self, callback: Callable, callback_type: str) -> None:
        """
        Validate callback signature based on type.
        
        All action callbacks require at least 1 parameter:
        - on_goal: goal_request
        - on_cancel: goal_handle  
        - execute: goal_handle
        """
        sig = inspect.signature(callback)
        params = list(sig.parameters.keys())
        
        # Remove 'self' if present
        if params and params[0] == 'self':
            params = params[1:]
        
        # All action callbacks require at least 1 parameter
        if len(params) < 1:
            raise ValueError(
                f"ActionServer '{callback_type}' callback '{callback.__name__}' "
                f"must accept at least 1 parameter. Got: {params}"
            )


# Type alias for any blueprint
AnyBlueprint = Union[
    ServiceBlueprint,
    PublisherBlueprint,
    SubscriberBlueprint,
    ActionBlueprint
]
