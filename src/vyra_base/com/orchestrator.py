"""
Endpoint Orchestrator

The EndpointOrchestrator is a flat, event-driven async task that bridges
the three interface lifecycle phases:

    Phase 1 (Definition)  →  ManifestResolver emits a change event
    Phase 2 (Schema)       →  SchemaResolver emits a change event
    Phase 3 (Callbacks)   →  EndpointRegistry emits a change event

Whenever any of those events fires the orchestrator runs a single
``_process_cycle()`` that iterates over every endpoint that is "ready"
(definition loaded + schema resolved + all required callbacks bound + no
transport yet) and activates it by calling TransportProviderFactory.

The orchestrator is intentionally flat:
- No sub-tasks or nested event loops.
- No polling; it only wakes on events.
- Transport key is written back into EndpointRegistry so downstream code
  can query liveness without touching the factory.

Usage::

    orchestrator = EndpointOrchestrator(
        endpoint_registry=entity.endpoint_registry,
        manifest_resolver=entity.manifest_resolver,
        schema_resolver=entity.schema_resolver,
        factory=entity._factory,
    )
    await orchestrator.start()   # spawns background task

    # Later, to shut down cleanly:
    await orchestrator.stop()
"""

from __future__ import annotations

import asyncio
import logging
from typing import Any, Callable, Dict, List, Optional

from vyra_base.com.endpoint import (
    AnyEndpoint,
    EndpointRegistry,
    InterfaceType,
    ServiceEndpoint,
    PublisherEndpoint,
    SubscriberEndpoint,
    ActionEndpoint,
)
from vyra_base.com.manifest import ManifestResolver
from vyra_base.com.schema import SchemaResolver

logger = logging.getLogger(__name__)


class EndpointOrchestrator:
    """
    Async background task that activates InterfaceEndpoints once they are ready.

    Readiness criteria (all must hold):
    a) ``endpoint._manifest_key`` is set  — definition is loaded
    b) ``endpoint.is_bound()`` is True    — all required callbacks present
    c) ``endpoint._schema_ref`` is set    — schema resolved and bound
    d) ``endpoint._transport_key`` is None — not yet active

    When all criteria are met the orchestrator calls the corresponding
    ``TransportProviderFactory.create_*`` method for every protocol listed
    in ``endpoint.protocols``.  On success the returned transport key is
    stored in the endpoint via ``EndpointRegistry.set_transport_key``.

    Args:
        endpoint_registry: The shared EndpointRegistry instance.
        manifest_resolver: The shared ManifestResolver instance.
        schema_resolver:   The shared SchemaResolver instance.
        factory:           The TransportProviderFactory (renamed from
                           TransportProviderFactory) used to create transports.
    """

    def __init__(
        self,
        endpoint_registry: EndpointRegistry,
        manifest_resolver: ManifestResolver,
        schema_resolver: SchemaResolver,
        factory: Any,
    ) -> None:
        self._registry = endpoint_registry
        self._manifest = manifest_resolver
        self._schema = schema_resolver
        self._factory = factory

        self._task: Optional[asyncio.Task] = None  # type: ignore[type-arg]
        self._stop_event: Optional[asyncio.Event] = None

    # -----------------------------------------------------------------------
    # Lifecycle
    # -----------------------------------------------------------------------

    async def start(self) -> None:
        """
        Initialise asyncio events on all components and start the background task.

        Must be called from within a running event loop.
        """
        if self._task is not None and not self._task.done():
            logger.warning(
                "EndpointOrchestrator.start() called while already running — ignored."
            )
            return

        # Create stop event and inject change events into all components
        self._stop_event = asyncio.Event()
        self._registry.set_event_loop()
        self._manifest.set_event_loop()
        self._schema.set_event_loop()

        self._task = asyncio.create_task(
            self._run(), name="EndpointOrchestrator"
        )
        logger.info("EndpointOrchestrator started.")

    async def stop(self) -> None:
        """Signal the background task to stop and wait for it to finish."""
        if self._stop_event is not None:
            self._stop_event.set()
        if self._task is not None and not self._task.done():
            try:
                await asyncio.wait_for(self._task, timeout=5.0)
            except asyncio.TimeoutError:
                self._task.cancel()
                logger.warning(
                    "EndpointOrchestrator.stop(): task did not finish in 5 s — cancelled."
                )
        logger.info("EndpointOrchestrator stopped.")

    # -----------------------------------------------------------------------
    # External notifications (called by components when their state changes)
    # -----------------------------------------------------------------------

    def notify_manifest_change(self) -> None:
        """Signal that the ManifestResolver has new data."""
        if self._manifest.change_event is not None:
            try:
                self._manifest.change_event.set()
            except RuntimeError:
                pass

    def notify_schema_change(self) -> None:
        """Signal that the SchemaResolver cache has changed."""
        if self._schema.change_event is not None:
            try:
                self._schema.change_event.set()
            except RuntimeError:
                pass

    def notify_endpoint_change(self) -> None:
        """Signal that the EndpointRegistry has been mutated."""
        if self._registry.change_event is not None:
            try:
                self._registry.change_event.set()
            except RuntimeError:
                pass

    # -----------------------------------------------------------------------
    # Internal run loop
    # -----------------------------------------------------------------------

    async def _run(self) -> None:
        """
        Main event loop.  Waits for any change event, then runs one cycle.
        Repeats until the stop event is set.
        """
        assert self._stop_event is not None

        while not self._stop_event.is_set():
            # Build list of all change events (may be None if not yet set up)
            events: List[asyncio.Event] = [e for e in [
                self._registry.change_event,
                self._manifest.change_event,
                self._schema.change_event,
            ] if e is not None]

            if not events:
                await asyncio.sleep(0.1)
                continue

            # Wait for the first event to fire (or stop)
            wait_tasks = [
                asyncio.create_task(e.wait()) for e in events
            ]
            stop_task = asyncio.create_task(self._stop_event.wait())

            done, pending = await asyncio.wait(
                [*wait_tasks, stop_task],
                return_when=asyncio.FIRST_COMPLETED,
            )

            # Cancel remaining waits
            for t in pending:
                t.cancel()

            if self._stop_event.is_set():
                break

            # Clear all events before processing (so new changes during the
            # cycle will re-trigger the loop)
            for event in events:
                event.clear()

            await self._process_cycle()

        logger.debug("EndpointOrchestrator._run exited.")

    # -----------------------------------------------------------------------
    # Processing
    # -----------------------------------------------------------------------

    async def _process_cycle(self) -> None:
        """
        Check every ready endpoint and activate those that qualify.

        Also handles manifest → endpoint propagation: for every unloaded
        metadata entry, creates or updates the corresponding endpoint in the
        registry so that callbacks registered before the manifest arrived are
        matched up.
        """
        # Step 1: propagate newly loaded manifest data to endpoints
        await self._sync_manifest_to_registry()

        # Step 2: resolve schemas for endpoints that have a manifest but no schema
        await self._sync_schema_to_endpoints()

        # Step 3: activate ready endpoints
        ready_keys = self._registry.list_ready()
        for key in ready_keys:
            ep = self._registry.get_endpoint(key)
            if ep is None:
                continue
            try:
                success = await self._try_activate_endpoint(key, ep)
                if not success:
                    logger.debug(
                        "EndpointOrchestrator: '%s' not yet activatable.", key
                    )
            except Exception as exc:
                logger.error(
                    "EndpointOrchestrator: error activating '%s': %s",
                    key,
                    exc,
                    exc_info=True,
                )

    async def _sync_manifest_to_registry(self) -> None:
        """
        Load metadata from ManifestResolver and call bind_manifest for any
        endpoint that is missing its manifest_key.
        """
        try:
            all_meta = self._manifest.load_interface_metadata()
        except Exception as exc:
            logger.error(
                "EndpointOrchestrator._sync_manifest_to_registry: "
                "failed to load metadata: %s",
                exc,
            )
            return

        for fn_name, meta in all_meta.items():
            ep = self._registry.get_endpoint(fn_name)
            if ep is not None and ep._manifest_key is not None:
                continue
            self._registry.bind_manifest(fn_name, meta)

    async def _sync_schema_to_endpoints(self) -> None:
        """
        For every endpoint that has a manifest but no schema_ref, ask the
        SchemaResolver to load the appropriate schema and bind it.
        """
        for key in self._registry.list_all():
            ep = self._registry.get_endpoint(key)
            if ep is None or not ep._manifest_key or ep._schema_ref is not None:
                continue

            tags: List[str] = ep.metadata.get("tags", [])
            protocol = tags[0] if tags else "zenoh"

            schema_ref = self._schema.get_interface_for_function(
                ep._manifest_key, protocol
            )
            if schema_ref is not None:
                self._registry.bind_schema(key, schema_ref)

    async def _try_activate_endpoint(
        self, key: str, endpoint: AnyEndpoint
    ) -> bool:
        """
        Attempt to create a live transport for a ready endpoint.

        Activation criteria (all must hold):
        a) manifest_key is set
        b) is_bound() returns True
        c) schema_ref is set
        d) transport_key is NOT yet set

        Returns True if the transport was successfully created.
        """
        if not (
            endpoint._manifest_key
            and endpoint.is_bound()
            and endpoint._schema_ref is not None
            and endpoint._transport_key is None
        ):
            return False

        itype = endpoint.interface_type
        name = endpoint.name
        protocols = endpoint.protocols or []

        logger.info(
            "EndpointOrchestrator: activating endpoint '%s' "
            "(type=%s, protocols=%s).",
            key,
            itype.value,
            [p.value for p in protocols],
        )

        transport_key: Optional[str] = None

        try:
            if itype == InterfaceType.SERVICE:
                ep_service = endpoint  # type: ignore[assignment]
                callback = ep_service.get_callback("default")
                transport = await self._factory.create_server(
                    name=name,
                    response_callback=callback,
                    protocols=protocols,
                    service_type=getattr(ep_service, "service_type", None),
                )
                if transport is not None:
                    transport_key = getattr(transport, "name", name)

            elif itype == InterfaceType.PUBLISHER:
                ep_pub = endpoint  # type: ignore[assignment]
                transport = await self._factory.create_publisher(
                    name=name,
                    protocols=protocols,
                    message_type=getattr(ep_pub, "message_type", None),
                )
                if transport is not None:
                    transport_key = getattr(transport, "name", name)

            elif itype == InterfaceType.SUBSCRIBER:
                ep_sub = endpoint  # type: ignore[assignment]
                callback = ep_sub.get_callback("default")
                transport = await self._factory.create_subscriber(
                    name=name,
                    subscriber_callback=callback,
                    protocols=protocols,
                    message_type=getattr(ep_sub, "message_type", None),
                )
                if transport is not None:
                    transport_key = getattr(transport, "name", name)

            elif itype == InterfaceType.ACTION:
                ep_action = endpoint  # type: ignore[assignment]
                transport = await self._factory.create_action_server(
                    name=name,
                    handle_goal_request=ep_action.get_callback("on_goal"),
                    handle_cancel_request=ep_action.get_callback("on_cancel"),
                    execution_callback=ep_action.get_callback("execute"),
                    protocols=protocols,
                    action_type=getattr(ep_action, "action_type", None),
                )
                if transport is not None:
                    transport_key = getattr(transport, "name", name)

            else:
                logger.warning(
                    "EndpointOrchestrator: unknown interface type '%s' "
                    "for endpoint '%s'.",
                    itype,
                    key,
                )
                return False

        except Exception as exc:
            logger.error(
                "EndpointOrchestrator: transport creation failed for '%s': %s",
                key,
                exc,
                exc_info=True,
            )
            return False

        if transport_key is None:
            logger.warning(
                "EndpointOrchestrator: factory returned None for '%s' — "
                "endpoint will retry on next cycle.",
                key,
            )
            return False

        self._registry.set_transport_key(key, transport_key)
        logger.info(
            "EndpointOrchestrator: endpoint '%s' is now ACTIVE "
            "(transport_key='%s').",
            key,
            transport_key,
        )
        return True
