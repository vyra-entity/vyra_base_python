"""
Interface Endpoints and Endpoint Registry

A unified model that covers the full lifecycle of a communication interface:

    Phase 1 (Definition):  manifest data loaded → manifest_key set
    Phase 2 (Schema):      schema resolved      → schema_ref set
    Phase 3 (Callbacks):   callbacks bound       → is_fully_bound()
    Phase 4 (Transport):   orchestrator wires    → transport_key set

InterfaceEndpoint is the single authoritative record for one named interface.
EndpointRegistry is the thread-safe store that tracks all endpoints and emits
change events so the EndpointOrchestrator can react.
"""

from __future__ import annotations

import asyncio
import inspect
import logging
import threading
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Dict, List, Optional, Set, Union

from vyra_base.com.core.types import ProtocolType

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Enumerations
# ---------------------------------------------------------------------------

class InterfaceType(Enum):
    """Communication pattern for an interface endpoint."""
    SERVICE = "service"
    PUBLISHER = "publisher"
    SUBSCRIBER = "subscriber"
    ACTION = "action"


class EndpointState(Enum):
    """Lifecycle state of an InterfaceEndpoint."""
    UNREGISTERED = "unregistered"     # created with only a callback, no definition yet
    DEFINITION_LOADED = "definition_loaded"  # manifest bound
    SCHEMA_RESOLVED = "schema_resolved"      # manifest + schema present
    CALLBACK_BOUND = "callback_bound"        # all required callbacks bound
    ACTIVE = "active"                        # transport live


# ---------------------------------------------------------------------------
# Base InterfaceEndpoint
# ---------------------------------------------------------------------------

@dataclass
class InterfaceEndpoint(ABC):
    """
    Single authoritative record for one named communication interface.

    Combines what was previously split across HandlerBlueprint (definition)
    and CallbackRegistry (callback store) into one cohesive object.

    Attributes:
        name:           Unique interface name (functionname from metadata).
        interface_type: Communication pattern (service, publisher, …).
        protocols:      Preferred transports, in priority order.
        metadata:       Raw dict from *.meta.json (empty until manifest bound).
        _manifest_key:  Key into ManifestResolver once definition is loaded.
        _schema_ref:    Schema object set by SchemaResolver once resolved.
        _transport_key: Key returned by TransportProviderFactory once active.
    """

    name: str
    interface_type: InterfaceType
    protocols: List[ProtocolType] = field(default_factory=list)
    metadata: Dict[str, Any] = field(default_factory=dict)

    _manifest_key: Optional[str] = field(default=None, repr=False)
    _schema_ref: Optional[Any] = field(default=None, repr=False)
    _transport_key: Optional[str] = field(default=None, repr=False)

    # ---- callback storage -------------------------------------------------

    _callbacks: Dict[str, Optional[Callable]] = field(
        default_factory=dict, repr=False, init=False
    )

    def __post_init__(self) -> None:
        self._callbacks = self._default_callbacks()

    @abstractmethod
    def _default_callbacks(self) -> Dict[str, Optional[Callable]]:
        """Return the initial callback slot map for this endpoint type."""

    @abstractmethod
    def required_callback_slots(self) -> Set[str]:
        """Return the set of callback slot names that *must* be bound."""

    # ---- callback interface -----------------------------------------------

    def bind_callback(
        self, callback: Callable, callback_type: str = "default"
    ) -> None:
        """
        Bind a callable to a callback slot.

        Args:
            callback:      The implementation function.
            callback_type: Slot name. Use 'default' for single-callback types.

        Raises:
            ValueError:   If slot name is invalid for this endpoint type.
            RuntimeError: If the slot is already occupied.
        """
        if callback_type not in self._callbacks:
            raise ValueError(
                f"Unknown callback slot '{callback_type}' for "
                f"{self.__class__.__name__} '{self.name}'. "
                f"Valid slots: {list(self._callbacks.keys())}"
            )
        if self._callbacks[callback_type] is not None:
            raise RuntimeError(
                f"Callback slot '{callback_type}' on '{self.name}' is already bound."
            )
        self._validate_callback(callback, callback_type)
        self._callbacks[callback_type] = callback
        logger.debug(
            "Bound '%s' callback to endpoint '%s'.", callback_type, self.name
        )

    def get_callback(self, callback_type: str = "default") -> Optional[Callable]:
        """Return the callable for a slot, or None if not yet bound."""
        return self._callbacks.get(callback_type)

    def unbind_callback(self, callback_type: str = "default") -> Optional[Callable]:
        """Remove a callback binding and return the old callable."""
        old = self._callbacks.get(callback_type)
        if callback_type in self._callbacks:
            self._callbacks[callback_type] = None
        return old

    def is_bound(self, callback_type: Optional[str] = None) -> bool:
        """
        Check binding status.

        Args:
            callback_type: If given, check that specific slot.
                           If None, check whether *all required* slots are bound.
        """
        if callback_type is not None:
            return self._callbacks.get(callback_type) is not None
        return all(
            self._callbacks.get(slot) is not None
            for slot in self.required_callback_slots()
        )

    def is_fully_bound(self) -> bool:
        """True when every callback slot (including optional ones) is bound."""
        return all(v is not None for v in self._callbacks.values())

    @abstractmethod
    def _validate_callback(self, callback: Callable, callback_type: str) -> None:
        """Validate the callable signature for a given slot."""

    # ---- metadata helpers -------------------------------------------------

    def get_metadata(self, key: str, default: Any = None) -> Any:
        return self.metadata.get(key, default)

    def update_metadata(self, **kwargs: Any) -> None:
        self.metadata.update(kwargs)

    # ---- state helper -----------------------------------------------------

    @property
    def state(self) -> EndpointState:
        if self._transport_key:
            return EndpointState.ACTIVE
        if self._manifest_key and self._schema_ref and self.is_bound():
            return EndpointState.CALLBACK_BOUND
        if self._manifest_key and self._schema_ref:
            return EndpointState.SCHEMA_RESOLVED
        if self._manifest_key:
            return EndpointState.DEFINITION_LOADED
        return EndpointState.UNREGISTERED

    def __repr__(self) -> str:
        bound = "✓" if self.is_bound() else "✗"
        proto_str = ",".join(p.value for p in self.protocols[:2])
        if len(self.protocols) > 2:
            proto_str += ",…"
        return (
            f"{self.__class__.__name__}("
            f"name='{self.name}', "
            f"state={self.state.value}, "
            f"bound={bound}, "
            f"protocols=[{proto_str}])"
        )


# ---------------------------------------------------------------------------
# Concrete endpoint types
# ---------------------------------------------------------------------------

class ServiceEndpoint(InterfaceEndpoint):
    """
    Request / response interface (Zenoh queryable, ROS2 service, gRPC, …).

    Required callback signature::

        async def handler(request, response=None) -> dict | response_type
    """

    def __init__(
        self,
        name: str,
        protocols: Optional[List[ProtocolType]] = None,
        metadata: Optional[Dict[str, Any]] = None,
        service_type: Optional[Any] = None,
    ) -> None:
        super().__init__(
            name=name,
            interface_type=InterfaceType.SERVICE,
            protocols=protocols or [],
            metadata=metadata or {},
        )
        self.service_type = service_type

    def _default_callbacks(self) -> Dict[str, Optional[Callable]]:
        return {"default": None}

    def required_callback_slots(self) -> Set[str]:
        return {"default"}

    def _validate_callback(self, callback: Callable, callback_type: str) -> None:
        sig = inspect.signature(callback)
        params = [
            p for p in sig.parameters.keys() if p != "self"
        ]
        if len(params) < 1:
            raise ValueError(
                f"Service callback '{callback.__name__}' must accept "
                f"at least 1 parameter (request)."
            )
        if params[0] not in ("request", "req", "data"):
            logger.warning(
                "Service callback '%s' first parameter should be 'request', "
                "got '%s'.",
                callback.__name__,
                params[0],
            )


class PublisherEndpoint(InterfaceEndpoint):
    """Publish-only interface (fire-and-forget topic, …)."""

    def __init__(
        self,
        name: str,
        protocols: Optional[List[ProtocolType]] = None,
        metadata: Optional[Dict[str, Any]] = None,
        message_type: Optional[Any] = None,
    ) -> None:
        super().__init__(
            name=name,
            interface_type=InterfaceType.PUBLISHER,
            protocols=protocols or [],
            metadata=metadata or {},
        )
        self.message_type = message_type

    def _default_callbacks(self) -> Dict[str, Optional[Callable]]:
        return {"default": None}

    def required_callback_slots(self) -> Set[str]:
        return {"default"}

    def _validate_callback(self, callback: Callable, callback_type: str) -> None:
        sig = inspect.signature(callback)
        params = [p for p in sig.parameters.keys() if p != "self"]
        if len(params) < 1:
            raise ValueError(
                f"Publisher callback '{callback.__name__}' must accept "
                f"at least 1 parameter (message)."
            )


class SubscriberEndpoint(InterfaceEndpoint):
    """Subscription callback interface (receives published data)."""

    def __init__(
        self,
        name: str,
        protocols: Optional[List[ProtocolType]] = None,
        metadata: Optional[Dict[str, Any]] = None,
        message_type: Optional[Any] = None,
    ) -> None:
        super().__init__(
            name=name,
            interface_type=InterfaceType.SUBSCRIBER,
            protocols=protocols or [],
            metadata=metadata or {},
        )
        self.message_type = message_type

    def _default_callbacks(self) -> Dict[str, Optional[Callable]]:
        return {"default": None}

    def required_callback_slots(self) -> Set[str]:
        return {"default"}

    def _validate_callback(self, callback: Callable, callback_type: str) -> None:
        sig = inspect.signature(callback)
        params = [p for p in sig.parameters.keys() if p != "self"]
        if len(params) < 1:
            raise ValueError(
                f"Subscriber callback '{callback.__name__}' must accept "
                f"at least 1 parameter (message)."
            )


class ActionEndpoint(InterfaceEndpoint):
    """
    Action server interface with three lifecycle callbacks.

    Required:  execute
    Optional:  on_goal, on_cancel
    """

    def __init__(
        self,
        name: str,
        protocols: Optional[List[ProtocolType]] = None,
        metadata: Optional[Dict[str, Any]] = None,
        action_type: Optional[Any] = None,
    ) -> None:
        super().__init__(
            name=name,
            interface_type=InterfaceType.ACTION,
            protocols=protocols or [],
            metadata=metadata or {},
        )
        self.action_type = action_type

    def _default_callbacks(self) -> Dict[str, Optional[Callable]]:
        return {"on_goal": None, "on_cancel": None, "execute": None}

    def required_callback_slots(self) -> Set[str]:
        return {"execute"}

    def _validate_callback(self, callback: Callable, callback_type: str) -> None:
        sig = inspect.signature(callback)
        params = [p for p in sig.parameters.keys() if p != "self"]
        if len(params) < 1:
            raise ValueError(
                f"Action '{callback_type}' callback '{callback.__name__}' "
                f"must accept at least 1 parameter."
            )


# Type alias kept for convenience
AnyEndpoint = Union[ServiceEndpoint, PublisherEndpoint, SubscriberEndpoint, ActionEndpoint]


# ---------------------------------------------------------------------------
# EndpointRegistry
# ---------------------------------------------------------------------------

class EndpointRegistry:
    """
    Thread-safe registry for all InterfaceEndpoints.

    Responsibilities:
    - Store endpoints by name (with optional namespace prefix).
    - Support late binding: a callback may arrive before the manifest
      definition exists; the endpoint is created as a dangling stub.
    - Emit change notifications so EndpointOrchestrator can react.

    Change notification is delivered via an asyncio.Event that is set
    whenever state changes.  The orchestrator obtains the event via
    ``change_event`` and waits on it inside its async loop.
    """

    def __init__(self) -> None:
        self._endpoints: Dict[str, AnyEndpoint] = {}
        self._lock = threading.RLock()
        self._change_event: Optional[asyncio.Event] = None
        logger.debug("EndpointRegistry initialised.")

    # ---- asyncio event ----------------------------------------------------

    def set_event_loop(self) -> None:
        """
        Create the asyncio change event bound to the running loop.
        Call once from within the event loop before starting the orchestrator.
        """
        self._change_event = asyncio.Event()

    @property
    def change_event(self) -> Optional[asyncio.Event]:
        return self._change_event

    def _notify_change(self) -> None:
        if self._change_event is not None:
            try:
                self._change_event.set()
            except RuntimeError:
                pass  # loop not running yet

    # ---- registration helpers ---------------------------------------------

    def _full_name(self, name: str, namespace: Optional[str]) -> str:
        if namespace:
            return f"{namespace.strip('/')}/{name}"
        return name

    # ---- public API -------------------------------------------------------

    def register_endpoint(
        self,
        endpoint: AnyEndpoint,
        namespace: Optional[str] = None,
        override: bool = False,
    ) -> str:
        """
        Register a fully constructed endpoint.

        Returns the fully qualified name used as registry key.
        """
        with self._lock:
            key = self._full_name(endpoint.name, namespace)
            if key in self._endpoints and not override:
                raise ValueError(
                    f"Endpoint '{key}' already registered. Use override=True."
                )
            if override and key in self._endpoints:
                logger.warning("Overriding existing endpoint '%s'.", key)
            self._endpoints[key] = endpoint
            logger.info(
                "Registered endpoint '%s' (type=%s, state=%s).",
                key,
                endpoint.interface_type.value,
                endpoint.state.value,
            )
            self._notify_change()
            return key

    def get_endpoint(
        self, name: str, namespace: Optional[str] = None
    ) -> Optional[AnyEndpoint]:
        with self._lock:
            key = self._full_name(name, namespace)
            ep = self._endpoints.get(key)
            if ep is None:
                ep = self._endpoints.get(name)
            return ep

    # ---- bind_callback ----------------------------------------------------

    def bind_callback(
        self,
        name: str,
        callback: Callable,
        callback_type: str = "default",
        namespace: Optional[str] = None,
    ) -> bool:
        """
        Bind a callback to an endpoint.

        If the endpoint does not yet exist a dangling stub is created with no
        manifest/schema data and a warning is logged.  The orchestrator will
        complete it once the manifest arrives.

        Returns True on success.
        """
        with self._lock:
            key = self._full_name(name, namespace)
            ep = self._endpoints.get(key) or self._endpoints.get(name)

            if ep is None:
                ep = ServiceEndpoint(name=name)
                self._endpoints[key] = ep
                logger.warning(
                    "bind_callback: no endpoint found for '%s' — "
                    "created dangling stub. Definition must arrive later.",
                    key,
                )

            try:
                ep.bind_callback(callback, callback_type)
                logger.info(
                    "Callback bound: endpoint='%s', slot='%s'.",
                    key,
                    callback_type,
                )
                self._notify_change()
                return True
            except (ValueError, RuntimeError) as exc:
                logger.error(
                    "Failed to bind callback to '%s' (slot=%s): %s",
                    key,
                    callback_type,
                    exc,
                )
                raise

    # ---- bind_manifest ----------------------------------------------------

    def bind_manifest(
        self,
        name: str,
        manifest_data: Dict[str, Any],
        namespace: Optional[str] = None,
    ) -> bool:
        """
        Attach manifest (*.meta.json) data to an endpoint.

        If a dangling endpoint exists whose ``name`` matches the incoming
        ``functionname`` in manifest_data, the manifest is merged into that
        stub so that the previously registered callback is preserved.

        Returns True on success.
        """
        with self._lock:
            function_name: str = manifest_data.get("functionname", name)
            key = self._full_name(name, namespace)

            # Look for an existing endpoint under the canonical key or the
            # bare function name (dangling stub created by bind_callback).
            ep = self._endpoints.get(key) or self._endpoints.get(function_name)

            if ep is not None:
                # Re-key if the stub was stored under a different key
                existing_key = next(
                    (k for k, v in self._endpoints.items() if v is ep), key
                )
                if existing_key != key:
                    self._endpoints[key] = ep
                    if existing_key != key:
                        self._endpoints.pop(existing_key, None)
                    logger.info(
                        "bind_manifest: merged manifest into dangling endpoint "
                        "'%s' (previously keyed as '%s').",
                        key,
                        existing_key,
                    )

                ep._manifest_key = function_name
                ep.metadata.update(manifest_data)

                # Update protocols from manifest tags if not yet set
                if not ep.protocols:
                    tags = manifest_data.get("tags", [])
                    ep.protocols = _tags_to_protocols(tags)

                logger.info(
                    "Manifest bound: endpoint='%s', function='%s'.",
                    key,
                    function_name,
                )
                self._notify_change()
                return True

            # No existing endpoint — create one from the manifest data
            interface_kind = manifest_data.get("type", "service")
            protocols = _tags_to_protocols(manifest_data.get("tags", []))
            ep = _endpoint_from_manifest(name, interface_kind, protocols, manifest_data)
            ep._manifest_key = function_name
            self._endpoints[key] = ep
            logger.info(
                "Manifest bound: created new endpoint '%s' (kind=%s).",
                key,
                interface_kind,
            )
            self._notify_change()
            return True

    # ---- bind_schema ------------------------------------------------------

    def bind_schema(
        self,
        name: str,
        schema_ref: Any,
        namespace: Optional[str] = None,
    ) -> bool:
        """
        Attach a resolved schema object to an endpoint.

        Returns True on success, False if the endpoint is not found.
        """
        with self._lock:
            key = self._full_name(name, namespace)
            ep = self._endpoints.get(key) or self._endpoints.get(name)
            if ep is None:
                logger.warning(
                    "bind_schema: endpoint '%s' not found — cannot bind schema.",
                    key,
                )
                return False
            ep._schema_ref = schema_ref
            logger.info(
                "Schema bound: endpoint='%s', schema=%s.",
                key,
                type(schema_ref).__name__,
            )
            self._notify_change()
            return True

    # ---- transport key ----------------------------------------------------

    def set_transport_key(
        self,
        name: str,
        key: str,
        namespace: Optional[str] = None,
    ) -> bool:
        """
        Record the transport key returned by TransportProviderFactory.

        Returns True on success.
        """
        with self._lock:
            full = self._full_name(name, namespace)
            ep = self._endpoints.get(full) or self._endpoints.get(name)
            if ep is None:
                logger.warning(
                    "set_transport_key: endpoint '%s' not found.", full
                )
                return False
            ep._transport_key = key
            logger.info(
                "Transport key set: endpoint='%s', key='%s'.", full, key
            )
            self._notify_change()
            return True

    # ---- query helpers ----------------------------------------------------

    def list_all(self, namespace: Optional[str] = None) -> List[str]:
        with self._lock:
            names = list(self._endpoints.keys())
            if namespace:
                prefix = f"{namespace.strip('/')}/"
                names = [n for n in names if n.startswith(prefix)]
            return sorted(names)

    def list_unbound(self, namespace: Optional[str] = None) -> List[str]:
        with self._lock:
            return [
                k for k in self.list_all(namespace)
                if not self._endpoints[k].is_bound()
            ]

    def list_incomplete(self, namespace: Optional[str] = None) -> List[str]:
        """Endpoints missing manifest or schema."""
        with self._lock:
            return [
                k for k in self.list_all(namespace)
                if not (
                    self._endpoints[k]._manifest_key
                    and self._endpoints[k]._schema_ref
                )
            ]

    def list_ready(self, namespace: Optional[str] = None) -> List[str]:
        """
        Endpoints that have definition + callbacks + schema but no transport yet.
        """
        with self._lock:
            return [
                k for k in self.list_all(namespace)
                if (
                    self._endpoints[k]._manifest_key
                    and self._endpoints[k]._schema_ref
                    and self._endpoints[k].is_bound()
                    and not self._endpoints[k]._transport_key
                )
            ]

    def list_active(self, namespace: Optional[str] = None) -> List[str]:
        """Endpoints with a live transport."""
        with self._lock:
            return [
                k for k in self.list_all(namespace)
                if self._endpoints[k]._transport_key
            ]

    def get_statistics(self, namespace: Optional[str] = None) -> Dict[str, int]:
        with self._lock:
            all_keys = self.list_all(namespace)
            eps = [self._endpoints[k] for k in all_keys]
            stats: Dict[str, int] = {
                "total": len(eps),
                "unbound": sum(1 for e in eps if not e.is_bound()),
                "incomplete": sum(
                    1 for e in eps
                    if not (e._manifest_key and e._schema_ref)
                ),
                "ready": sum(
                    1 for e in eps
                    if e._manifest_key and e._schema_ref
                    and e.is_bound() and not e._transport_key
                ),
                "active": sum(1 for e in eps if e._transport_key),
            }
            for itype in InterfaceType:
                stats[itype.value + "s"] = sum(
                    1 for e in eps if e.interface_type == itype
                )
            return stats

    def clear(self) -> None:
        with self._lock:
            count = len(self._endpoints)
            self._endpoints.clear()
            logger.debug("EndpointRegistry cleared (%d entries removed).", count)


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _tags_to_protocols(tags: List[str]) -> List[ProtocolType]:
    """Convert metadata tag strings to ProtocolType list."""
    mapping = {
        "zenoh": ProtocolType.ZENOH,
        "ros2": ProtocolType.ROS2,
        "redis": ProtocolType.REDIS,
        "uds": ProtocolType.UDS,
    }
    result: List[ProtocolType] = []
    for tag in tags:
        pt = mapping.get(tag.lower())
        if pt and pt not in result:
            result.append(pt)
    return result


def _endpoint_from_manifest(
    name: str,
    interface_kind: str,
    protocols: List[ProtocolType],
    metadata: Dict[str, Any],
) -> AnyEndpoint:
    """Factory: create the right endpoint subclass from a manifest entry."""
    kind = interface_kind.lower()
    if kind == "service":
        return ServiceEndpoint(name=name, protocols=protocols, metadata=metadata)
    if kind in ("message", "publisher"):
        return PublisherEndpoint(name=name, protocols=protocols, metadata=metadata)
    if kind == "subscriber":
        return SubscriberEndpoint(name=name, protocols=protocols, metadata=metadata)
    if kind == "action":
        return ActionEndpoint(name=name, protocols=protocols, metadata=metadata)
    logger.warning(
        "_endpoint_from_manifest: unknown kind '%s' for '%s', defaulting to ServiceEndpoint.",
        interface_kind,
        name,
    )
    return ServiceEndpoint(name=name, protocols=protocols, metadata=metadata)
