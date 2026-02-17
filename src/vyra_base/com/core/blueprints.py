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
    Blueprint for long-running task communication (ROS2 Action, job queues, etc.)
    
    Expected callback signature:
        async def handler(self, goal_handle) -> result
        or
        def handler(self, goal_handle) -> result
    
    The goal_handle provides:
        - goal_handle.goal: The goal request data
        - goal_handle.publish_feedback(feedback): Send progress updates
        - goal_handle.is_cancel_requested(): Check for cancellation
    
    Example:
        >>> @remote_actionServer
        ... async def process_batch(self, goal_handle):
        ...     for i, item in enumerate(goal_handle.goal.items):
        ...         if goal_handle.is_cancel_requested():
        ...             return {"cancelled": True}
        ...         process(item)
        ...         goal_handle.publish_feedback({"progress": i+1})
        ...     return {"processed": len(goal_handle.goal.items)}
    """
    
    def __init__(
        self,
        name: str,
        protocols: Optional[List[ProtocolType]] = None,
        metadata: Optional[Dict[str, Any]] = None,
        action_type: Optional[Any] = None
    ):
        super().__init__(
            name=name,
            interface_type=InterfaceType.ACTION,
            protocols=protocols or [],
            metadata=metadata or {}
        )
        self.action_type = action_type
    
    def _validate_callback(self, callback: Callable) -> None:
        """Validate action callback signature"""
        sig = inspect.signature(callback)
        params = list(sig.parameters.keys())
        
        # Remove 'self' if present
        if params and params[0] == 'self':
            params = params[1:]
        
        # Must accept at least 1 parameter (goal_handle or goal)
        if len(params) < 1:
            raise ValueError(
                f"Action callback '{callback.__name__}' must accept "
                f"at least 1 parameter (goal_handle). Got: {params}"
            )


# Type alias for any blueprint
AnyBlueprint = Union[
    ServiceBlueprint,
    PublisherBlueprint,
    SubscriberBlueprint,
    ActionBlueprint
]
