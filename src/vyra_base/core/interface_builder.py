"""
InterfaceBuilder — Static helper for creating transport interfaces.

Centralises protocol-dispatch logic that was previously spread throughout
``VyraEntity.set_interfaces``.  Each ``create_*`` classmethod accepts a
:class:`~vyra_base.defaults.entries.FunctionConfigEntry` and creates the
appropriate transport handles for **all** protocols the entry's *tags*
request (or for all available protocols when *tags* is empty).

Tag semantics
-------------
- **Empty tags** → register on every available / applicable protocol
  (backward-compatible default).
- **Non-empty tags** → register *only* on the explicitly listed protocols.

Example::

    # tags=["ros2"]      → ROS2 only
    # tags=["zenoh"]     → Zenoh only
    # tags=["ros2","zenoh"] → both
    # tags=[]            → all applicable (backward compat)
"""
from __future__ import annotations

import logging
from typing import Any

from vyra_base.com import InterfaceFactory, ProtocolType
from vyra_base.com.providers.provider_registry import ProviderRegistry
from vyra_base.defaults.entries import (
    FunctionConfigEntry,
    FunctionConfigTags,
)

logger = logging.getLogger(__name__)


class InterfaceBuilder:
    """
    Static factory that creates transport interfaces for a
    :class:`~vyra_base.defaults.entries.FunctionConfigEntry`.

    All methods are classmethods — no instance state is held.
    """

    # ── Tag helpers ───────────────────────────────────────────────────────────

    @classmethod
    def _wants(cls, tags: list[FunctionConfigTags], protocol: FunctionConfigTags) -> bool:
        """Return True if *tags* is empty (default to all) or contains *protocol*."""
        return not tags or protocol in tags

    @classmethod
    def wants_ros2(cls, tags: list[FunctionConfigTags]) -> bool:
        return cls._wants(tags, FunctionConfigTags.ROS2)

    @classmethod
    def wants_zenoh(cls, tags: list[FunctionConfigTags]) -> bool:
        return cls._wants(tags, FunctionConfigTags.ZENOH)

    @classmethod
    def wants_redis(cls, tags: list[FunctionConfigTags]) -> bool:
        return cls._wants(tags, FunctionConfigTags.REDIS)

    @classmethod
    def wants_uds(cls, tags: list[FunctionConfigTags]) -> bool:
        return cls._wants(tags, FunctionConfigTags.UDS)

    # ── Service (request / response) ─────────────────────────────────────────

    @classmethod
    async def create_service(
        cls,
        setting: FunctionConfigEntry,
        callbacks: dict[str, Any] | None,
        node: Any,
        ros2_available: bool,
        registry: ProviderRegistry | None = None,
    ) -> None:
        """
        Create service servers for all protocols the entry's tags request.

        :param setting: Interface configuration entry.
        :param callbacks: Dict with key ``'response'`` holding the callback.
        :param node: ROS2 node (may be ``None`` when ROS2 is unavailable).
        :param ros2_available: Whether ROS2 is up and a node is present.
        :param registry: Optional pre-created :class:`ProviderRegistry` instance.
                         A new one is created when *None* (avoids repeated look-ups
                         when called for many interfaces in one loop).
        """
        if registry is None:
            registry = ProviderRegistry()

        name = setting.functionname
        response_cb = callbacks.get("response") if callbacks else None
        tags = setting.tags

        # ROS2
        if ros2_available and cls.wants_ros2(tags):
            try:
                await InterfaceFactory.create_server(
                    name=name,
                    response_callback=response_cb,
                    protocols=[ProtocolType.ROS2],
                    service_type=setting.interfacetypes,
                    node=node,
                )
                logger.info(f"✅ ROS2 service created: {name}")
            except Exception as exc:
                logger.warning(f"⚠️ ROS2 service '{name}' failed: {exc}")

        # Zenoh
        if registry.is_available(ProtocolType.ZENOH) and cls.wants_zenoh(tags):
            try:
                result = await InterfaceFactory.create_server(
                    name=name,
                    response_callback=response_cb,
                    protocols=[ProtocolType.ZENOH],
                    service_type=setting.interfacetypes,
                )
                if result is not None:
                    logger.info(f"✅ Zenoh service created: {name}")
                else:
                    logger.debug(
                        f"⏳ Zenoh service '{name}' pending "
                        f"— no callback bound yet (bind callback first)"
                    )
            except Exception as exc:
                logger.warning(f"⚠️ Zenoh service '{name}' failed: {exc}")

        # Redis
        if registry.is_available(ProtocolType.REDIS) and cls.wants_redis(tags):
            try:
                result = await InterfaceFactory.create_server(
                    name=name,
                    response_callback=response_cb,
                    protocols=[ProtocolType.REDIS],
                    service_type=setting.interfacetypes,
                )
                if result is not None:
                    logger.info(f"✅ Redis service created: {name}")
                else:
                    logger.debug(
                        f"⏳ Redis service '{name}' pending "
                        f"— no callback bound yet (bind callback first)"
                    )
            except Exception as exc:
                logger.warning(f"⚠️ Redis service '{name}' failed: {exc}")

        # UDS
        if registry.is_available(ProtocolType.UDS) and cls.wants_uds(tags):
            try:
                result = await InterfaceFactory.create_server(
                    name=name,
                    response_callback=response_cb,
                    protocols=[ProtocolType.UDS],
                    service_type=setting.interfacetypes,
                )
                if result is not None:
                    logger.info(f"✅ UDS service created: {name}")
                else:
                    logger.debug(
                        f"⏳ UDS service '{name}' pending "
                        f"— no callback bound yet (bind callback first)"
                    )
            except Exception as exc:
                logger.warning(f"⚠️ UDS service '{name}' failed: {exc}")

    # ── Action server ─────────────────────────────────────────────────────────

    @classmethod
    async def create_action(
        cls,
        setting: FunctionConfigEntry,
        callbacks: dict[str, Any] | None,
        node: Any,
        ros2_available: bool,
    ) -> None:
        """
        Create action servers for all protocols the entry's tags request.

        :raises ValueError: When callbacks are missing (required for action servers).
        """
        name = setting.functionname

        if not callbacks:
            msg = (
                f"❌ ActionServer '{name}' has no callbacks defined. "
                "Use @remote_actionServer.on_goal/on_cancel/execute decorators."
            )
            logger.error(msg)
            raise ValueError(msg)

        tags = setting.tags

        # ROS2 (currently only ROS2 action servers are supported)
        if ros2_available and cls.wants_ros2(tags):
            logger.debug(
                f"ActionServer '{name}' callbacks — "
                f"on_goal={bool(callbacks.get('on_goal'))}, "
                f"on_cancel={bool(callbacks.get('on_cancel'))}, "
                f"execute={bool(callbacks.get('execute'))}"
            )
            await InterfaceFactory.create_action_server(
                name=name,
                handle_goal_request=callbacks.get("on_goal"),
                handle_cancel_request=callbacks.get("on_cancel"),
                execution_callback=callbacks.get("execute"),
                protocols=[ProtocolType.ROS2],
                action_type=setting.interfacetypes,
                node=node,
            )
        else:
            logger.debug(
                f"⏭️ Skipping action server '{name}': "
                f"ros2_available={ros2_available}, tags={tags}"
            )

    # ── Publisher (one-way message) ───────────────────────────────────────────

    @classmethod
    async def create_publisher(
        cls,
        setting: FunctionConfigEntry,
        node: Any,
        ros2_available: bool,
    ) -> None:
        """
        Create publishers for all protocols the entry's tags request.
        """
        name = setting.functionname
        tags = setting.tags

        # Periodic config
        periodic = setting.periodic is not None

        if setting.periodic is None:
             periodic_caller = None
        else:
            periodic_caller = setting.periodic.caller

        if setting.periodic is None:
            periodic_interval = None
        else:
            periodic_interval = setting.periodic.interval

        # ROS2
        if ros2_available and cls.wants_ros2(tags):
            try:
                await InterfaceFactory.create_publisher(
                    name=name,
                    protocols=[ProtocolType.ROS2],
                    message_type=setting.interfacetypes,
                    node=node,
                    is_publisher=True,
                    qos_profile=setting.qosprofile,
                    description=setting.description,
                    periodic=periodic,
                    interval_time=periodic_interval,
                    periodic_caller=periodic_caller,
                )
                logger.info(f"✅ ROS2 publisher created: {name}")
            except Exception as exc:
                logger.warning(
                    f"⚠️ ROS2 publisher '{name}' could not be created (non-fatal): {exc}"
                )
        else:
            logger.debug(f"⏭️ Skipping ROS2 publisher '{name}': tags={tags}")

    # ── Late-binding upgrade (callback added after initial registration) ───────

    @classmethod
    async def upgrade_service(
        cls,
        setting: FunctionConfigEntry,
        new_callbacks: dict[str, Any],
        registry: ProviderRegistry | None = None,
    ) -> None:
        """
        Upgrade a previously registered service that had no callback with a new one.

        Called when a decorated callback is bound after ``set_interfaces`` ran
        (two-phase / blueprint initialisation pattern).
        """
        if registry is None:
            registry = ProviderRegistry()

        name = setting.functionname
        response_cb = new_callbacks.get("response")

        if registry.is_available(ProtocolType.ZENOH):
            try:
                result = await InterfaceFactory.create_server(
                    name=name,
                    response_callback=response_cb,
                    protocols=[ProtocolType.ZENOH],
                    service_type=setting.interfacetypes,
                )
                if result is not None:
                    logger.info(f"✅ Zenoh server upgraded: {name}")
                else:
                    logger.warning(f"⚠️ Zenoh upgrade returned None for '{name}'")
            except Exception as exc:
                logger.warning(f"⚠️ Failed to upgrade Zenoh server '{name}': {exc}")
